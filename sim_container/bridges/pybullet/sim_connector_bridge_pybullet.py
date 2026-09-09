#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# PyBullet bridge for nepi_app_sim_connector (Phase 4, MULTI_SIMULATOR_INTEGRATION_PLAN.md).
#
# Pure Python, zero ROS/nepi_sdk dependency, zero Gazebo/Webots-style external
# simulator process -- PyBullet is a plain library, so this script's own main
# loop both runs physics (p.stepSimulation()) and is the "bridge," structurally
# the simplest of the four sims done so far (no external engine to launch,
# no controller-launcher convention).
#
# Standalone smoke-test finding worth recording: pushing the bundled r2d2.urdf
# sample robot with p.applyExternalForce() barely moved it (r2d2's own
# wheel-joint friction resisted sliding the chassis around, sensibly enough --
# it is a real physically-modeled wheeled robot, not a frictionless puck).
# Switched to p.resetBaseVelocity() instead -- directly commanding the base's
# linear/angular velocity each step, confirmed to move exactly as commanded
# (0.5 m/s * 1s = 0.494m measured, matching the commanded velocity to within
# integration error). This bridge drives the robot the same way: the goto
# controller computes a body velocity, applied via resetBaseVelocity every
# step, not through force/torque.
#
# Reuses the exact same closed-loop goto controller shape as the other three
# bridges in this plan (proportional gains, turn-in-place gate) -- the control
# law is simulator-agnostic; only the actuator call changes per sim.
#
# RESET is a real, working teleport here (p.resetBasePositionAndOrientation),
# unlike Webots' bridge (not a Supervisor there) -- PyBullet has no such
# restriction, so there is no honest-gap note needed for this one.

import base64
import json
import math
import socket
import threading
import time

import cv2
import numpy as np
import pybullet as p
import pybullet_data

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030

CAMERA_SENSOR_NAME = "pybullet_rover/camera"
CAMERA_WIDTH = 160
CAMERA_HEIGHT = 120

SPAWN_POS = [0.0, 0.0, 0.3]
SPAWN_ORN = [0.0, 0.0, 0.0, 1.0]

SIM_HZ = 240.0
CONTROLLER_HZ = 20.0

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60

GOTO_KP_LIN = 0.5
GOTO_KP_ANG = 1.5
GOTO_TURN_GATE_RAD = math.radians(30.0)
GOTO_TOL_M = 0.1
GOTO_TOL_RAD = math.radians(3.0)
MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADPS = math.radians(60.0)

MOTOR_MAX_LINEAR_MPS = 0.5
MOTOR_WHEEL_BASE_M = 0.4


class PyBulletSimConnectorBridge:

  def __init__(self, app_host, app_port):
    self.app_host = app_host
    self.app_port = app_port

    p.connect(p.DIRECT)  # Headless -- no GUI window needed for this bridge.
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / SIM_HZ)
    p.loadURDF("plane.urdf")
    self.robot = p.loadURDF("r2d2.urdf", SPAWN_POS)

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0

    self.goto_lock = threading.Lock()
    self.goto_target = None
    self.motor_ratios = [0.0, 0.0]

    self.home_lock = threading.Lock()
    self.home_x_m = 0.0
    self.home_y_m = 0.0

    self.frame_lock = threading.Lock()
    self.latest_frame = None
    self.active_image_topic = ""

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.bridgeLoop, daemon = True).start()

    print("sim_connector_bridge_pybullet: started, connecting to %s:%d" %
          (self.app_host, self.app_port), flush = True)

  #**********************
  # Main simulation loop -- physics + control, single thread (PyBullet's C++
  # core is not thread-safe for concurrent stepSimulation calls).

  def run(self):
    steps_per_control = int(SIM_HZ / CONTROLLER_HZ)
    last_image_capture = 0.0
    step_count = 0
    while True:
      p.stepSimulation()
      step_count += 1
      if step_count % steps_per_control == 0:
        self.updatePoseFromSim()
        self.controlTick()
      now = time.time()
      if now - last_image_capture >= 1.0 / IMAGE_RATE_HZ:
        last_image_capture = now
        self.captureFrame()
      time.sleep(1.0 / SIM_HZ)

  def updatePoseFromSim(self):
    pos, orn = p.getBasePositionAndOrientation(self.robot)
    lin_vel, ang_vel = p.getBaseVelocity(self.robot)
    _, _, yaw = p.getEulerFromQuaternion(orn)
    with self.pose_lock:
      self.x_m, self.y_m = pos[0], pos[1]
      self.yaw_rad = yaw
      self.lin_mps = math.hypot(lin_vel[0], lin_vel[1])
      self.ang_radps = ang_vel[2]

  def captureFrame(self):
    try:
      with self.pose_lock:
        x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      # Simple chase-cam: behind and above the robot, looking at it -- there
      # is no onboard camera link on the stock r2d2.urdf model, so this is a
      # scene view rather than a true robot_camera; acceptable for proving the
      # image-relay path, same spirit as Webots' single-camera simplification.
      cam_x = x_m - 0.6 * math.cos(yaw_rad)
      cam_y = y_m - 0.6 * math.sin(yaw_rad)
      view = p.computeViewMatrix([cam_x, cam_y, 0.4], [x_m, y_m, 0.2], [0, 0, 1])
      proj = p.computeProjectionMatrixFOV(60, CAMERA_WIDTH / CAMERA_HEIGHT, 0.05, 10.0)
      _, _, rgba, _, _ = p.getCameraImage(CAMERA_WIDTH, CAMERA_HEIGHT, view, proj,
                                         renderer = p.ER_TINY_RENDERER)
      arr = np.reshape(rgba, (CAMERA_HEIGHT, CAMERA_WIDTH, 4)).astype(np.uint8)
      bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
      ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
      if ok:
        with self.frame_lock:
          self.latest_frame = encoded.tobytes()
    except Exception as e:
      print("sim_connector_bridge_pybullet: bad camera frame: %s" % str(e), flush = True)

  #**********************
  # Closed-loop goto controller -- same shape as the other three bridges.

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def controlTick(self):
    with self.goto_lock:
      target = self.goto_target
      motor_ratios = list(self.motor_ratios)
    with self.pose_lock:
      cur_x, cur_y, cur_yaw = self.x_m, self.y_m, self.yaw_rad

    lin = 0.0
    ang = 0.0
    if target is not None:
      dx = target["x_m"] - cur_x
      dy = target["y_m"] - cur_y
      dist = math.hypot(dx, dy)
      if dist > GOTO_TOL_M:
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw)
        ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(MAX_LINEAR_MPS, GOTO_KP_LIN * dist))
      else:
        yaw_err = 0.0
        if target["yaw_deg"] is not None:
          yaw_err = self.normalizeAngle(math.radians(target["yaw_deg"]) - cur_yaw)
        if abs(yaw_err) > GOTO_TOL_RAD:
          ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * yaw_err))
        else:
          with self.goto_lock:
            self.goto_target = None
          print("sim_connector_bridge_pybullet: goto target reached", flush = True)
          if self.sock is not None:
            self.sendLine(self.sock, {"type": "goto_result", "success": True})
    elif any(motor_ratios):
      lin = (motor_ratios[0] + motor_ratios[1]) / 2.0 * MOTOR_MAX_LINEAR_MPS
      ang = (motor_ratios[1] - motor_ratios[0]) / MOTOR_WHEEL_BASE_M * MOTOR_MAX_LINEAR_MPS

    vx = lin * math.cos(cur_yaw)
    vy = lin * math.sin(cur_yaw)
    p.resetBaseVelocity(self.robot, linearVelocity = [vx, vy, 0], angularVelocity = [0, 0, ang])

  #**********************
  # sim_connector_app_node.py TCP client (background thread)

  def bridgeLoop(self):
    while True:
      sock = None
      try:
        sock = socket.create_connection((self.app_host, self.app_port),
                                        timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception:
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      print("sim_connector_bridge_pybullet: connected to %s:%d" %
            (self.app_host, self.app_port), flush = True)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (sock, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while True:
        try:
          data = sock.recv(4096)
        except socket.timeout:
          continue
        except Exception:
          data = b""
        if not data:
          break
        buf += data
        while b"\n" in buf:
          line, buf = buf.split(b"\n", 1)
          if line.strip():
            self.processLineFromApp(line)

      sender_stop.set()
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      print("sim_connector_bridge_pybullet: connection lost, retrying in %.0fs" %
            RECONNECT_INTERVAL_SEC, flush = True)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    last_image = 0.0
    while not stop_event.is_set():
      now = time.time()
      self.sendLine(sock, self.buildTelemetryLine())

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        self.sendLine(sock, {"type": "sensor_topics", "topics": [
            {"topic_name": CAMERA_SENSOR_NAME, "msg_type": "sensor_msgs/Image"},
        ]})
        self.sendLine(sock, {"type": "environment_options", "options": []})
        last_announce = now

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        with self.frame_lock:
          frame = self.latest_frame
        if frame is not None:
          self.sendLine(sock, {
              "type": "image",
              "topic_name": CAMERA_SENSOR_NAME,
              "data": base64.b64encode(frame).decode("ascii"),
              "stamp": now,
          })
        last_image = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def buildTelemetryLine(self):
    with self.pose_lock:
      x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      lin_mps, ang_radps = self.lin_mps, self.ang_radps
    return {
        "x_m": x_m, "y_m": y_m, "z_m": 0.0,
        "yaw_deg": math.degrees(yaw_rad),
        "x_m_per_sec": lin_mps * math.cos(yaw_rad),
        "y_m_per_sec": lin_mps * math.sin(yaw_rad),
        "yaw_deg_per_sec": math.degrees(ang_radps),
    }

  def sendLine(self, sock, line_dict):
    # Locked around the actual send, not just self.sock's assignment -- the
    # goto-control thread can now send goto_result concurrently with the
    # main sender loop's own sends on the same socket.
    with self.sock_lock:
      try:
        sock.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  #**********************
  # Commands from sim_connector_app_node.py

  def processLineFromApp(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      print("sim_connector_bridge_pybullet: bad line from app: %s" % str(e), flush = True)
      return
    msg_type = msg.get("type")
    if msg_type == "motor_control":
      self.handleMotorControl(msg)
    elif msg_type == "goto_position":
      self.handleGotoPosition(msg)
    elif msg_type == "goto_pose":
      self.handleGotoPose(msg)
    elif msg_type == "go_home":
      self.handleGoHome()
    elif msg_type == "go_stop":
      self.handleGoStop()
    elif msg_type == "setup_action":
      self.handleSetupAction(msg)
    elif msg_type == "set_active_image_topic":
      self.active_image_topic = msg.get("topic_name", "")
    elif msg_type == "camera_settings":
      pass  # Single chase-cam view -- nothing to switch between.
    elif msg_type == "environment_option":
      print("sim_connector_bridge_pybullet: environment_option not supported, ignoring",
            flush = True)
    elif msg_type == "robot_config":
      print("sim_connector_bridge_pybullet: robot_config selected: %s" % msg.get("config"),
            flush = True)

  def handleMotorControl(self, msg):
    ind = int(msg.get("motor_ind", -1))
    ratio = float(msg.get("speed_ratio", 0.0))
    if ind < 0 or ind >= len(self.motor_ratios):
      return
    with self.goto_lock:
      self.motor_ratios[ind] = max(0.0, min(1.0, ratio))

  def handleGotoPosition(self, msg):
    with self.pose_lock:
      cur_x, cur_y = self.x_m, self.y_m
    with self.goto_lock:
      self.goto_target = {
          "x_m": cur_x + float(msg.get("x_meters", 0.0)),
          "y_m": cur_y + float(msg.get("y_meters", 0.0)),
          "yaw_deg": msg.get("yaw_deg"),
      }
      self.motor_ratios = [0.0, 0.0]

  def handleGotoPose(self, msg):
    with self.pose_lock:
      cur_x, cur_y = self.x_m, self.y_m
    with self.goto_lock:
      self.goto_target = {"x_m": cur_x, "y_m": cur_y, "yaw_deg": msg.get("yaw_deg", 0.0)}
      self.motor_ratios = [0.0, 0.0]

  def handleGoHome(self):
    with self.home_lock:
      home_x, home_y = self.home_x_m, self.home_y_m
    with self.goto_lock:
      self.goto_target = {"x_m": home_x, "y_m": home_y, "yaw_deg": None}
      self.motor_ratios = [0.0, 0.0]

  def handleGoStop(self):
    with self.goto_lock:
      self.goto_target = None
      self.motor_ratios = [0.0, 0.0]

  def handleSetupAction(self, msg):
    action = msg.get("action")
    if action == "RESET":
      p.resetBasePositionAndOrientation(self.robot, SPAWN_POS, SPAWN_ORN)
      p.resetBaseVelocity(self.robot, linearVelocity = [0, 0, 0], angularVelocity = [0, 0, 0])
      with self.goto_lock:
        self.goto_target = None
        self.motor_ratios = [0.0, 0.0]
      print("sim_connector_bridge_pybullet: reset to spawn pose", flush = True)
    elif action == "RETURN_HOME":
      self.handleGoHome()


def main():
  import argparse
  parser = argparse.ArgumentParser(description = __doc__)
  parser.add_argument("--host", default = DEFAULT_APP_HOST)
  parser.add_argument("--port", type = int, default = DEFAULT_APP_PORT)
  args, _ = parser.parse_known_args()
  bridge = PyBulletSimConnectorBridge(args.host, args.port)
  bridge.run()


if __name__ == "__main__":
  main()

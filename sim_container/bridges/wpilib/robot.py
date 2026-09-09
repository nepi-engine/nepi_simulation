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

# WPILib HAL Sim bridge for nepi_app_sim_connector (Phase 6, stretch,
# MULTI_SIMULATOR_INTEGRATION_PLAN.md). Run via `python3 robot.py sim`
# (pyfrc's simulation entry point, from robotpy).
#
# This is the one bridge in the plan whose "simulator" has no built-in world
# at all -- see physics.py's docstring for how a NavPose-shaped output gets
# produced anyway. This file owns the WPILib TimedRobot lifecycle plus the
# same TCP-client/closed-loop-controller pattern every other bridge in this
# plan uses, adapted to run its control tick inside a WPILib periodic
# callback instead of its own timer, since WPILib motor outputs only take
# effect on a scheduled periodic tick while the robot is enabled.
#
# Real toolchain finding from this phase (see the plan doc's own write-up):
# the current robotpy release (2024.x) fails to build from source on this
# VM -- it requires C++20 (GCC 10+), and Ubuntu 20.04 ships GCC 9, which
# rejects `-std=c++20` outright and has no `<span>` header. robotpy 2022.4.8
# has prebuilt manylinux wheels for Python 3.8 and installs cleanly with zero
# compilation -- pinned to that instead of chasing a toolchain upgrade for a
# stretch-priority phase.
#
# Deliberately not implemented (see MULTI_SIMULATOR_INTEGRATION_PLAN.md's own
# scoping note for this phase): no camera (no such concept here without a
# separate, unrelated vision-sim add-on) and no environment control (nothing
# to toggle). goto_position IS implemented, not scoped down to
# motor-control-only, because physics.py's integration is a real, trustworthy
# kinematic model (pyfrc's own bundled TwoMotorDrivetrain helper), not a
# guess.

import base64
import json
import math
import socket
import threading
import time

import wpilib
import wpilib.simulation as simlib

import shared_state

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030

LEFT_PWM_CHANNEL = 0
RIGHT_PWM_CHANNEL = 1
TRACK_WIDTH_M = 0.5

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0

GOTO_KP_LIN = 0.5
GOTO_KP_ANG = 1.5
GOTO_TURN_GATE_RAD = math.radians(30.0)
GOTO_TOL_M = 0.1
GOTO_TOL_RAD = math.radians(3.0)
MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADPS = math.radians(60.0)

MOTOR_MAX_LINEAR_MPS = 0.5


class WpilibSimConnectorRobot(wpilib.TimedRobot):

  def robotInit(self):
    self.left_motor = wpilib.PWMVictorSPX(LEFT_PWM_CHANNEL)
    self.right_motor = wpilib.PWMVictorSPX(RIGHT_PWM_CHANNEL)

    # No Driver Station / joystick is attached in this headless bridge use --
    # force the robot enabled in autonomous so periodic motor commands
    # actually reach the (simulated) PWM outputs rather than being zeroed by
    # WPILib's normal disabled-robot safety behavior.
    simlib.DriverStationSim.setDsAttached(True)
    simlib.DriverStationSim.setAutonomous(True)
    simlib.DriverStationSim.setEnabled(True)
    simlib.DriverStationSim.notifyNewData()

    self.home_lock = threading.Lock()
    self.home_x_m = 0.0
    self.home_y_m = 0.0

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.bridgeLoop, daemon = True).start()
    print("sim_connector_bridge_wpilib: robot initialized, connecting to %s:%d" %
          (DEFAULT_APP_HOST, DEFAULT_APP_PORT), flush = True)

  #**********************
  # WPILib periodic loop -- runs at TimedRobot's default 20ms period while
  # enabled. This IS the control tick every other bridge in this plan runs on
  # its own timer/thread; here it has to be a WPILib periodic callback for
  # motor outputs to take effect at all.

  def autonomousPeriodic(self):
    self.controlTick()

  def controlTick(self):
    target = shared_state.get_goto_target()
    motor_ratios = shared_state.get_motor_ratios()
    cur_x, cur_y, cur_yaw, _vx, _vyaw = shared_state.get_pose()

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
          shared_state.set_goto_target(None)
          print("sim_connector_bridge_wpilib: goto target reached", flush = True)
          with self.sock_lock:
            sock = self.sock
          if sock is not None:
            self.sendLine(sock, {"type": "goto_result", "success": True})
    elif any(motor_ratios):
      lin = (motor_ratios[0] + motor_ratios[1]) / 2.0 * MOTOR_MAX_LINEAR_MPS
      ang = (motor_ratios[1] - motor_ratios[0]) / TRACK_WIDTH_M * MOTOR_MAX_LINEAR_MPS

    left_wheel_mps = lin - ang * TRACK_WIDTH_M / 2.0
    right_wheel_mps = lin + ang * TRACK_WIDTH_M / 2.0
    # Matches drivetrains.TwoMotorDrivetrain.calculate()'s own convention
    # (physics.py): l_motor positive is forward, r_motor is negated.
    l_motor = max(-1.0, min(1.0, left_wheel_mps / MAX_LINEAR_MPS))
    r_motor = max(-1.0, min(1.0, -right_wheel_mps / MAX_LINEAR_MPS))
    self.left_motor.set(l_motor)
    self.right_motor.set(r_motor)

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  #**********************
  # sim_connector_app_node.py TCP client (background thread)

  def bridgeLoop(self):
    while True:
      sock = None
      try:
        sock = socket.create_connection((DEFAULT_APP_HOST, DEFAULT_APP_PORT),
                                        timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception:
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      print("sim_connector_bridge_wpilib: connected to %s:%d" %
            (DEFAULT_APP_HOST, DEFAULT_APP_PORT), flush = True)

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
      print("sim_connector_bridge_wpilib: connection lost, retrying in %.0fs" %
            RECONNECT_INTERVAL_SEC, flush = True)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    while not stop_event.is_set():
      now = time.time()
      x_m, y_m, yaw_rad, vx, vyaw = shared_state.get_pose()
      self.sendLine(sock, {
          "x_m": x_m, "y_m": y_m, "z_m": 0.0,
          "yaw_deg": math.degrees(yaw_rad),
          "x_m_per_sec": vx * math.cos(yaw_rad),
          "y_m_per_sec": vx * math.sin(yaw_rad),
          "yaw_deg_per_sec": math.degrees(vyaw),
      })

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        self.sendLine(sock, {"type": "sensor_topics", "topics": []})
        self.sendLine(sock, {"type": "environment_options", "options": []})
        last_announce = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def sendLine(self, sock, line_dict):
    # Locked around the actual send, not just self.sock's assignment -- the
    # goto-convergence check (periodic() -> the drivetrain update path) can
    # now send goto_result concurrently with the main sender loop's own
    # sends on the same socket.
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
      print("sim_connector_bridge_wpilib: bad line from app: %s" % str(e), flush = True)
      return
    msg_type = msg.get("type")
    if msg_type == "motor_control":
      self.handleMotorControl(msg)
    elif msg_type == "goto_position":
      self.handleGotoPosition(msg)
    elif msg_type == "go_home":
      self.handleGoHome()
    elif msg_type == "go_stop":
      self.handleGoStop()
    elif msg_type == "setup_action":
      self.handleSetupAction(msg)
    elif msg_type == "robot_config":
      print("sim_connector_bridge_wpilib: robot_config selected: %s" % msg.get("config"),
            flush = True)
    else:
      print("sim_connector_bridge_wpilib: unhandled command type: %s" % str(msg_type),
            flush = True)

  def handleMotorControl(self, msg):
    ind = int(msg.get("motor_ind", -1))
    ratio = float(msg.get("speed_ratio", 0.0))
    ratios = shared_state.get_motor_ratios()
    if ind < 0 or ind >= len(ratios):
      return
    ratios[ind] = max(0.0, min(1.0, ratio))
    shared_state.set_motor_ratios(ratios)

  def handleGotoPosition(self, msg):
    cur_x, cur_y, _yaw, _vx, _vyaw = shared_state.get_pose()
    shared_state.set_goto_target({
        "x_m": cur_x + float(msg.get("x_meters", 0.0)),
        "y_m": cur_y + float(msg.get("y_meters", 0.0)),
        "yaw_deg": msg.get("yaw_deg"),
    })
    shared_state.set_motor_ratios([0.0, 0.0])

  def handleGoHome(self):
    with self.home_lock:
      home_x, home_y = self.home_x_m, self.home_y_m
    shared_state.set_goto_target({"x_m": home_x, "y_m": home_y, "yaw_deg": None})
    shared_state.set_motor_ratios([0.0, 0.0])

  def handleGoStop(self):
    shared_state.set_goto_target(None)
    shared_state.set_motor_ratios([0.0, 0.0])

  def handleSetupAction(self, msg):
    action = msg.get("action")
    if action == "RESET":
      shared_state.request_reset()
      shared_state.set_goto_target(None)
      shared_state.set_motor_ratios([0.0, 0.0])
      print("sim_connector_bridge_wpilib: reset requested", flush = True)
    elif action == "RETURN_HOME":
      self.handleGoHome()


if __name__ == "__main__":
  wpilib.run(WpilibSimConnectorRobot)

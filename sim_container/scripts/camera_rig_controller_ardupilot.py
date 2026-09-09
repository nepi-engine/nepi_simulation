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

# Camera-rig follow controller, ArduPilot SITL port (Universal Simulator
# Bridge camera feature). New file, not an edit to the rover's
# camera_rig_controller.py: the pose source, the vehicle's motion (full 3D,
# not planar), and the network path are all materially different here, per
# this project's convention of a separate file per distinct
# simulator/workflow (see camera_rig_controller_multi.py for the same
# reasoning applied to the rover's multi-robot port).
#
# Differences from the rover version:
#   - Pose source: /gazebo/model_states (filtered for the "iris_demo" model),
#     not /rover/odom -- ArduPilot SITL has no ROS-native odom topic of its
#     own on this VM; Gazebo's ground-truth model state is the only pose feed
#     available here (confirmed working, including real roll/pitch, by
#     direct test while the SITL vehicle was armed and flying -- see the
#     session summary).
#   - Vehicle motion is full 3D (multirotor), not planar -- the drone's
#     altitude varies, so cam_z now tracks drone_z + offset_z rather than a
#     fixed offset_z as the rover used (rover never left z=0).
#   - Robot view (nose cam) is yaw-only / gimbal-stabilized (camera stays
#     level regardless of airframe roll/pitch), not rigidly slaved to the
#     full airframe attitude. Chosen because NEPI's target use cases
#     (inspection, survey -- the VideoRay/OceanAero/WESMAR field deployments
#     the platform is built around) match real commercial drones' 3-axis
#     gimbals, not FPV racing rigs: a camera that banked/pitched with every
#     stabilization twitch would be a poor default for an inspection data
#     product. It is also the cheapest extension of the rover's own
#     first-person semantic (cam_yaw = vehicle_yaw, pitch/roll = 0) -- same
#     formula, now reading a real quaternion's yaw component instead of
#     assuming pitch/roll were already zero (true for the flat rover, not
#     for a multirotor). The rigidly-slaved alternative is a legitimate
#     convention too (nose-mounted FPV camera) but is not built here -- see
#     the session summary for the full reasoning and how cheaply it could be
#     added later (a per-request stabilized/unstabilized toggle) if ever
#     needed.
#   - Scene view (chase cam)'s look-at is extended to real 3D: pitch is
#     computed from the real altitude difference (drone_z - cam_z), not
#     assumed zero.
#   - Two rigid Gazebo models, camera_rig (robot view) and camera_rig_chase
#     (scene view) -- see iris_arducopter_cmac.world -- both driven and
#     published every control tick, simultaneously, not one model teleported
#     between two poses on a view_mode toggle. Reworked (2026-08-18) after a
#     live report that the single-model design's "third-person view" didn't
#     really exist as an independent thing a client could rely on ("only one
#     instance, and it still glitches") -- switching modes visibly snapped
#     the one existing camera between two unrelated poses, and only whichever
#     mode was currently selected could ever be observed. Matches the same
#     fix already applied to the rover's camera_rig_controller.py (relay both
#     always-live feeds instead of one selected feed), extended here to also
#     need a second physical model since -- unlike the rover's two rigid
#     welded links -- this workflow's cameras were never rigidly attached to
#     the vehicle at all.
#   - Bridge: no separate sim_bridge_node.py exists for this workflow (the
#     ArduPilot driver's only other channel is raw MAVLink, which already
#     carries telemetry/commands and has no camera channel at all) so this
#     node runs its own minimal TCP JSON-lines server directly, combining the
#     roles the rover version split across two processes/files. Settings
#     applied directly to local instance state (no ROS-param handoff needed
#     -- there is no second process here to hand off to). Port 9026 (next
#     free slot in the 902x sim-utility block after the rover's 9021-9025);
#     forwarded by nepi_tunnel in nepi_sitl_dev_env.sh.

import base64
import json
import math
import socket
import threading
import time

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelState, ModelStates
from cv_bridge import CvBridge

import environment_models

PKG_NAME = 'CAMERA_RIG_CONTROLLER_ARDUPILOT'
NODE_NAME = 'camera_rig_controller_ardupilot'

MODEL_STATES_TOPIC = '/gazebo/model_states'
VEHICLE_MODEL_NAME = 'iris_demo'
MODEL_STATE_TOPIC = '/gazebo/set_model_state'

# Two rigs, two models, two topics -- see iris_arducopter_cmac.world's own
# comment and this module's docstring for why both are always driven and
# published simultaneously now, rather than one model teleported between
# poses on a view_mode toggle.
ROBOT_VIEW_IMAGE_TOPIC = '/camera_rig/camera/image_raw'
SCENE_VIEW_IMAGE_TOPIC = '/camera_rig_chase/camera/image_raw'
ROBOT_VIEW_MODEL_NAME = 'camera_rig'
SCENE_VIEW_MODEL_NAME = 'camera_rig_chase'
# camera_rig/camera_rig_chase model.sdf's sensors are both depth cameras now
# (libgazebo_ros_openni_kinect.so) -- these are the raw 32FC1-meters depth
# siblings of the two color topics above.
ROBOT_VIEW_DEPTH_TOPIC = '/camera_rig/camera/depth/image_raw'
SCENE_VIEW_DEPTH_TOPIC = '/camera_rig_chase/camera/depth/image_raw'

# Depth colorization range -- close is blue, far is red (JET), same
# convention and reasoning as camera_rig_controller.py's own constants.
DEPTH_MIN_RANGE_M = 0.3
DEPTH_MAX_RANGE_M = 15.0
# Millimeters, uint16 -- see camera_rig_controller.py's own constant.
DEPTH_MAP_MAX_MM = 65535

# Pose-follow update rate: matches the rover controller's own rationale --
# smooth relative to the ~1-10 Hz MAVLink telemetry rate elsewhere in this
# project, and the camera pose is purely visual so a higher rate costs little.
CONTROL_RATE_HZ = 20.0
# Modest frame rate/quality per the same tunnel-bandwidth convention used by
# every prior phase of this feature.
IMAGE_RATE_HZ = 7.0
# Raw depth map is for later processing, not live viewing (that's what the
# colorized robot_depth/scene_depth feeds are for) -- same reasoning as
# camera_rig_controller.py's own DEPTH_MAP_RATE_HZ for keeping it off the
# tunnel's video-rate budget.
DEPTH_MAP_RATE_HZ = 2.0
JPEG_QUALITY = 60

# Camera bridge TCP server: next free port in the 902x sim-utility block
# (9021 gz_reset, 9022-9025 rover heartbeat/bridge slots). Loopback-only,
# reached from the remote NEPI device solely through nepi_tunnel's reverse
# forward (see nepi_sitl_dev_env.sh).
BRIDGE_PORT = 9026

# Matches rbx_ardupilot_node.py's FACTORY_SETTINGS for these (kept in sync by
# eye -- both sides fall back to the same values before the first
# camera_settings line arrives, e.g. right after a (re)connect).
#
# Delta-from-mount-point convention (added 2026-09-08, requested live: "put
# the robot camera on top of the drone, not under. directly on top. make
# sure those are each the 0 0 0 values, both robot and scene camera views")
# -- matches the SAME convention the rover's own FACTORY_SCENE_OFFSET_X/Y/Z
# and FACTORY_CAMERA_OFFSET_X/Y/Z already use (see rbx_sim_node.py/
# sim_bridge_node.py): DEFAULT_OFFSET_*/DEFAULT_SCENE_OFFSET_* below are now
# the actual mount points (added back in controlCb), while offset_x/y/z and
# scene_offset_x/y/z themselves are a DELTA from that point, so "0" always
# means "stock/factory position" for an operator, never a raw Gazebo-frame
# coordinate. Before this, "0 0 0" for the robot view was the drone's own
# body origin (inside the airframe, not where the camera actually sat) --
# confusing for exactly the same reason the rover's own camera offset was
# before its matching fix.
#
# Robot view (nose cam): directly on top of the body, centered (x=0, y=0) --
# was forward-and-below (a nose/belly mount); moved per the same live
# request above. z=0.15 clears a typical small-quad body's own top plate/
# GPS mast without the render clipping into the airframe mesh.
FACTORY_OFFSET_X = 0.0
FACTORY_OFFSET_Y = 0.0
FACTORY_OFFSET_Z = 0.15
DEFAULT_OFFSET_X = 0.0
DEFAULT_OFFSET_Y = 0.0
DEFAULT_OFFSET_Z = 0.0
# Scene view (chase cam): behind and above the body -- same general chase-cam
# convention as the rover's own scene_offset_* defaults, scaled down since a
# quadcopter's own body/prop footprint is much smaller than the rover's.
# Absolute position unchanged by this fix, only now expressed as the mount
# point a "0" delta resolves to, matching the robot view's own convention.
FACTORY_SCENE_OFFSET_X = -2.0
FACTORY_SCENE_OFFSET_Y = 0.0
FACTORY_SCENE_OFFSET_Z = 1.0
DEFAULT_SCENE_OFFSET_X = 0.0
DEFAULT_SCENE_OFFSET_Y = 0.0
DEFAULT_SCENE_OFFSET_Z = 0.0


class CameraRigControllerArdupilot:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.bridge = CvBridge()

    self.pose_lock = threading.Lock()
    self.drone_x = 0.0
    self.drone_y = 0.0
    self.drone_z = 0.0
    self.drone_yaw = 0.0
    self.have_pose = False

    self.settings_lock = threading.Lock()
    self.offset_x = DEFAULT_OFFSET_X
    self.offset_y = DEFAULT_OFFSET_Y
    self.offset_z = DEFAULT_OFFSET_Z
    self.scene_offset_x = DEFAULT_SCENE_OFFSET_X
    self.scene_offset_y = DEFAULT_SCENE_OFFSET_Y
    self.scene_offset_z = DEFAULT_SCENE_OFFSET_Z

    self.image_lock = threading.Lock()
    self.latest_robot_view_img = None
    self.latest_scene_view_img = None
    self.latest_robot_view_depth = None
    self.latest_scene_view_depth = None

    self.client_lock = threading.Lock()
    self.client_conn = None

    # Live environment model spawn/despawn -- same environment_models.py
    # module and same "type":"environment"/"environment_options" wire
    # messages sim_bridge_node.py already uses for the rover, reusing THIS
    # bridge connection rather than opening a new one (see
    # rbx_ardupilot_node.py's own ENVIRONMENT_SETTING_NAMES comment for the
    # full "why not the same architecture as the rover" answer). Added
    # 2026-09-08, requested live: "changing the environment also doesnt do
    # anything" for the quadcopter.
    self.env_spawner = environment_models.EnvironmentModelSpawner(log_prefix = PKG_NAME)

    self.state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size = 1)

    self.model_states_sub = rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, self.modelStatesCb)
    self.robot_view_sub = rospy.Subscriber(ROBOT_VIEW_IMAGE_TOPIC, Image, self.robotViewImageCb)
    self.scene_view_sub = rospy.Subscriber(SCENE_VIEW_IMAGE_TOPIC, Image, self.sceneViewImageCb)
    self.robot_view_depth_sub = rospy.Subscriber(ROBOT_VIEW_DEPTH_TOPIC, Image,
                                                 self.robotViewDepthCb)
    self.scene_view_depth_sub = rospy.Subscriber(SCENE_VIEW_DEPTH_TOPIC, Image,
                                                 self.sceneViewDepthCb)

    self.control_timer = rospy.Timer(rospy.Duration(1.0 / CONTROL_RATE_HZ), self.controlCb)
    self.image_timer = rospy.Timer(rospy.Duration(1.0 / IMAGE_RATE_HZ), self.imagePublishCb)
    self.depth_map_timer = rospy.Timer(rospy.Duration(1.0 / DEPTH_MAP_RATE_HZ),
                                       self.depthMapPublishCb)

    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    rospy.loginfo(PKG_NAME + ": Camera rig controller initialized")
    rospy.loginfo(PKG_NAME + ": Following " + VEHICLE_MODEL_NAME + " (via " +
                  MODEL_STATES_TOPIC + ") -> " + ROBOT_VIEW_MODEL_NAME + " and " +
                  SCENE_VIEW_MODEL_NAME + " via " + MODEL_STATE_TOPIC)
    rospy.loginfo(PKG_NAME + ": Camera bridge server on 127.0.0.1:" + str(BRIDGE_PORT))

  def run(self):
    """Block until ROS shutdown, servicing the control/image timers and the
    bridge server thread."""
    rospy.spin()

  def modelStatesCb(self, msg):
    try:
      idx = msg.name.index(VEHICLE_MODEL_NAME)
    except ValueError:
      return
    pos = msg.pose[idx].position
    q = msg.pose[idx].orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with self.pose_lock:
      self.drone_x = pos.x
      self.drone_y = pos.y
      self.drone_z = pos.z
      self.drone_yaw = yaw
      self.have_pose = True

  def robotViewImageCb(self, msg):
    self.storeImage(msg, is_scene_view = False)

  def sceneViewImageCb(self, msg):
    self.storeImage(msg, is_scene_view = True)

  def storeImage(self, msg, is_scene_view):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Image conversion failed: " + str(e))
      return
    with self.image_lock:
      if is_scene_view:
        self.latest_scene_view_img = cv_img
      else:
        self.latest_robot_view_img = cv_img

  def robotViewDepthCb(self, msg):
    self.storeDepth(msg, is_scene_view = False)

  def sceneViewDepthCb(self, msg):
    self.storeDepth(msg, is_scene_view = True)

  def storeDepth(self, msg, is_scene_view):
    try:
      depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Depth conversion failed: " + str(e))
      return
    with self.image_lock:
      if is_scene_view:
        self.latest_scene_view_depth = depth_img
      else:
        self.latest_robot_view_depth = depth_img

  def depthToColorImg(self, depth_img):
    """Colorize a 32FC1-meters depth frame: close = blue, far = red (JET)."""
    if depth_img is None:
      return None
    depth_img = np.nan_to_num(depth_img, nan = DEPTH_MAX_RANGE_M,
                               posinf = DEPTH_MAX_RANGE_M, neginf = DEPTH_MAX_RANGE_M)
    clipped = np.clip(depth_img, DEPTH_MIN_RANGE_M, DEPTH_MAX_RANGE_M)
    scaled = ((clipped - DEPTH_MIN_RANGE_M) *
              (255.0 / (DEPTH_MAX_RANGE_M - DEPTH_MIN_RANGE_M))).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_JET)

  def depthToMillimeterPng(self, depth_img):
    """Encode a 32FC1-meters depth frame as a 16-bit PNG in millimeters --
    see camera_rig_controller.py's own version of this method."""
    if depth_img is None:
      return None
    depth_img = np.nan_to_num(depth_img, nan = 0.0, posinf = 0.0, neginf = 0.0)
    depth_mm = np.clip(depth_img * 1000.0, 0, DEPTH_MAP_MAX_MM).astype(np.uint16)
    ok, encoded = cv2.imencode('.png', depth_mm)
    if not ok:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Depth map PNG encode failed")
      return None
    return encoded

  def controlCb(self, timer_event):
    with self.pose_lock:
      if not self.have_pose:
        return
      drone_x = self.drone_x
      drone_y = self.drone_y
      drone_z = self.drone_z
      drone_yaw = self.drone_yaw

    with self.settings_lock:
      # Add the factory mount point back here, the one place that actually
      # needs the real body-frame offset (driveRig's own pose math) --
      # everywhere else (Settings, the RUI) keeps working in the delta. See
      # FACTORY_OFFSET_X/Y/Z's own comment.
      off_x = FACTORY_OFFSET_X + self.offset_x
      off_y = FACTORY_OFFSET_Y + self.offset_y
      off_z = FACTORY_OFFSET_Z + self.offset_z
      scene_off_x = FACTORY_SCENE_OFFSET_X + self.scene_offset_x
      scene_off_y = FACTORY_SCENE_OFFSET_Y + self.scene_offset_y
      scene_off_z = FACTORY_SCENE_OFFSET_Z + self.scene_offset_z

    self.driveRig(ROBOT_VIEW_MODEL_NAME, drone_x, drone_y, drone_z, drone_yaw,
                  off_x, off_y, off_z, is_scene_view = False)
    self.driveRig(SCENE_VIEW_MODEL_NAME, drone_x, drone_y, drone_z, drone_yaw,
                  scene_off_x, scene_off_y, scene_off_z, is_scene_view = True)

  def driveRig(self, model_name, drone_x, drone_y, drone_z, drone_yaw,
              off_x, off_y, off_z, is_scene_view):
    # Rotate the body-frame offset into the drone's current yaw only (not its
    # full 3D attitude) so the rig's position doesn't jitter with small
    # roll/pitch stabilization oscillations -- only the aim direction differs
    # by which rig this is, matching a real gimbal mount's decoupled position.
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    world_dx = off_x * cos_y - off_y * sin_y
    world_dy = off_x * sin_y + off_y * cos_y

    cam_x = drone_x + world_dx
    cam_y = drone_y + world_dy
    cam_z = drone_z + off_z

    if is_scene_view:
      # Chase-cam: real look-at (yaw AND pitch) toward the drone's current
      # position, extended to 3D via the real altitude difference.
      dx = drone_x - cam_x
      dy = drone_y - cam_y
      dz = drone_z - cam_z
      horiz_dist = math.hypot(dx, dy)
      cam_yaw = math.atan2(dy, dx)
      # Negated: eulerToQuat below uses the standard aerospace/sxyz
      # convention where POSITIVE pitch tilts the look axis toward -Z (a
      # Z-down/NED body-frame assumption), but this whole file works in
      # Gazebo's Z-up world frame (see the world file's own <gravity>0 0
      # -9.8</gravity> and every other use of *_z here as ordinary
      # up-positive altitude). Feeding atan2(dz, horiz_dist) straight in
      # therefore pitched the camera AWAY from the drone instead of toward
      # it -- confirmed live and numerically (dz=-1 with the default chase
      # offset produced a look vector tilted up when the target was below)
      # -- reported as "the scene view camera doesn't even have the
      # quadcopter visible there." Negating here converts the Z-up sign
      # into the Z-down convention eulerToQuat expects.
      cam_pitch = -math.atan2(dz, horiz_dist) if horiz_dist > 1e-6 else 0.0
    else:
      # Robot view: yaw-only, gimbal-stabilized -- stays level regardless of
      # the airframe's own roll/pitch (see module docstring for why).
      cam_yaw = drone_yaw
      cam_pitch = 0.0

    qx, qy, qz, qw = self.eulerToQuat(0.0, cam_pitch, cam_yaw)

    state = ModelState()
    state.model_name = model_name
    state.pose.position.x = cam_x
    state.pose.position.y = cam_y
    state.pose.position.z = cam_z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.reference_frame = 'world'
    self.state_pub.publish(state)

  def eulerToQuat(self, roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

  def imagePublishCb(self, timer_event):
    with self.image_lock:
      robot_img = self.latest_robot_view_img
      scene_img = self.latest_scene_view_img
      robot_depth = self.latest_robot_view_depth
      scene_depth = self.latest_scene_view_depth
    self.encodeAndSend(robot_img, 'robot_color', '.jpg', [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY], 'jpeg')
    self.encodeAndSend(scene_img, 'scene_color', '.jpg', [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY], 'jpeg')
    self.encodeAndSend(self.depthToColorImg(robot_depth), 'robot_depth', '.jpg',
                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY], 'jpeg')
    self.encodeAndSend(self.depthToColorImg(scene_depth), 'scene_depth', '.jpg',
                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY], 'jpeg')

  def depthMapPublishCb(self, timer_event):
    with self.image_lock:
      robot_depth = self.latest_robot_view_depth
      scene_depth = self.latest_scene_view_depth
    self.sendEncoded(self.depthToMillimeterPng(robot_depth), 'robot_depth_map', 'png16')
    self.sendEncoded(self.depthToMillimeterPng(scene_depth), 'scene_depth_map', 'png16')

  def encodeAndSend(self, cv_img, camera, ext, params, fmt):
    if cv_img is None:
      return
    ok, encoded = cv2.imencode(ext, cv_img, params)
    if not ok:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Image encode failed")
      return
    self.sendEncoded(encoded, camera, fmt)

  def sendEncoded(self, encoded, camera, fmt):
    if encoded is None:
      return
    line = {
      'type': 'image',
      'camera': camera,
      'format': fmt,
      'data': base64.b64encode(encoded.tobytes()).decode('ascii'),
      'stamp': rospy.Time.now().to_sec(),
    }
    self.sendLineToClient(line)

  def applyCameraSettings(self, cmd):
    with self.settings_lock:
      self.offset_x = float(cmd.get('offset_x', DEFAULT_OFFSET_X))
      self.offset_y = float(cmd.get('offset_y', DEFAULT_OFFSET_Y))
      self.offset_z = float(cmd.get('offset_z', DEFAULT_OFFSET_Z))
      self.scene_offset_x = float(cmd.get('scene_offset_x', DEFAULT_SCENE_OFFSET_X))
      self.scene_offset_y = float(cmd.get('scene_offset_y', DEFAULT_SCENE_OFFSET_Y))
      self.scene_offset_z = float(cmd.get('scene_offset_z', DEFAULT_SCENE_OFFSET_Z))

  def sendLineToClient(self, line_dict):
    with self.client_lock:
      conn = self.client_conn
    if conn is None:
      return
    try:
      conn.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Failed to send line to client: " + str(e))
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      # shutdown() before close(): serveClient (a DIFFERENT thread) is
      # almost certainly blocked in a timeout=None recv() on this exact
      # socket -- closing a fd out from under a thread blocked in recv() on
      # it does not reliably unblock that recv() on Linux, so without the
      # shutdown() that thread can stay wedged, leaving bridgeServerLoop's
      # single accept() slot (listen(1)) unable to take the client's next
      # reconnect attempt. Confirmed as the mechanism behind "quadcopter
      # image flickers in and out / won't stay" -- the client (rbx_ardupilot_
      # node.py's cameraBridgeLoop) retries every 3s after any hiccup (tunnel
      # blip, etc.), but each retry silently failed/refused until whatever
      # eventually broke the wedged recv() on its own.
      try:
        conn.shutdown(socket.SHUT_RDWR)
      except Exception:
        pass
      try:
        conn.close()
      except Exception:
        pass

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy sets a process-global socket.setdefaulttimeout(60) on
    # init_node(), which accept() applies to every accepted connection. The
    # settings side of this channel is legitimately idle for long stretches
    # (settings change rarely), so a recv timeout here must not be treated as
    # client death -- clear it and block instead; a real disconnect still
    # unblocks recv with EOF, and the 7 Hz image send loop independently
    # detects a dead client via its own sendall failure.
    srv.settimeout(None)
    srv.bind(('0.0.0.0', BRIDGE_PORT))  # 0.0.0.0: direct-LAN reachable, see sim_bridge_node.py's own bind comment
    srv.listen(1)
    while not rospy.is_shutdown():
      try:
        conn, _ = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      rospy.loginfo(PKG_NAME + ": Bridge client connected")
      with self.client_lock:
        self.client_conn = conn
      # Tell the device which environment models exist on this VM right
      # away, same "push state on connect" instinct as sim_bridge_node.py's
      # own copy of this exact line.
      self.sendLineToClient({'type': 'environment_options',
                            'options': environment_models.list_environment_models()})
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      rospy.loginfo(PKG_NAME + ": Bridge client disconnected")

  def serveClient(self, conn):
    buf = b''
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Bridge client recv error: " + repr(e))
        return
      if not data:
        rospy.loginfo(PKG_NAME + ": Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          cmd = json.loads(line)
        except Exception as e:
          rospy.logwarn(PKG_NAME + ": Bad bridge command line: " + str(e))
          continue
        if cmd.get('type') == 'camera_settings':
          self.applyCameraSettings(cmd)
        elif cmd.get('type') == 'environment':
          self.env_spawner.set_active_model(cmd.get('model_name'))
        else:
          rospy.logwarn_throttle(5.0, PKG_NAME + ": Unrecognized bridge line type: " +
                                 str(cmd.get('type')))


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = CameraRigControllerArdupilot()
  node.run()

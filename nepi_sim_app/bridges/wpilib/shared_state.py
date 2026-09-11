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

# Shared state between robot.py (WPILib TimedRobot lifecycle + TCP bridge
# thread) and physics.py (pyfrc PhysicsEngine, called by the sim framework's
# own timer) for the WPILib HAL Sim bridge (Phase 6, stretch,
# MULTI_SIMULATOR_INTEGRATION_PLAN.md). A plain shared module-level object is
# the simplest way to cross that boundary -- both files load in the same
# process, and pyfrc's own convention (publish physics state via
# NetworkTables, read it back robot-side) is overkill for a single-process
# bridge that never needs to look like real robot code on real hardware.

import threading

_lock = threading.Lock()

# Written by physics.py's update_sim, read by robot.py's periodic loop and the
# bridge's telemetry sender.
pose_x_m = 0.0
pose_y_m = 0.0
pose_yaw_rad = 0.0
vel_x_mps = 0.0
vel_yaw_radps = 0.0

# Written by robot.py's bridge thread (from sim_connector_app_node.py
# commands), read by robot.py's own periodic loop to drive the motors.
goto_target = None       # dict(x_m, y_m, yaw_deg or None) or None
motor_ratios = [0.0, 0.0]

# Written by robot.py on a RESET setup_action, consumed (and cleared) by
# physics.py's update_sim -- the physics engine owns the only reference to
# pyfrc's field object, so the actual teleport has to happen there.
reset_requested = False


def set_pose(x_m, y_m, yaw_rad, vel_x_mps_, vel_yaw_radps_):
  global pose_x_m, pose_y_m, pose_yaw_rad, vel_x_mps, vel_yaw_radps
  with _lock:
    pose_x_m, pose_y_m, pose_yaw_rad = x_m, y_m, yaw_rad
    vel_x_mps, vel_yaw_radps = vel_x_mps_, vel_yaw_radps_


def get_pose():
  with _lock:
    return pose_x_m, pose_y_m, pose_yaw_rad, vel_x_mps, vel_yaw_radps


def set_goto_target(target):
  global goto_target
  with _lock:
    goto_target = target


def get_goto_target():
  with _lock:
    return goto_target


def set_motor_ratios(ratios):
  global motor_ratios
  with _lock:
    motor_ratios = list(ratios)


def get_motor_ratios():
  with _lock:
    return list(motor_ratios)


def request_reset():
  global reset_requested
  with _lock:
    reset_requested = True


def consume_reset_request():
  global reset_requested
  with _lock:
    was_requested = reset_requested
    reset_requested = False
  return was_requested

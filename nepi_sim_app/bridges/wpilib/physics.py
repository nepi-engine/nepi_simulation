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

# pyfrc PhysicsEngine for the WPILib HAL Sim bridge (Phase 6, stretch,
# MULTI_SIMULATOR_INTEGRATION_PLAN.md). WPILib's simulation stack has no
# built-in "world" -- it simulates a robot's control system (motor
# controllers, encoders), not a renderable space a vehicle moves through. This
# file is what makes it produce a NavPose-shaped output at all: it reads the
# two drive motors' simulated PWM output values every physics tick and
# integrates a 2D pose from them via pyfrc's own bundled differential-drive
# kinematics helper (pyfrc.physics.drivetrains.TwoMotorDrivetrain) -- reused
# rather than hand-derived, matching how every other bridge in this plan
# reuses an already-proven closed-loop/kinematics implementation instead of
# re-deriving one from scratch.

import hal.simulation
from pyfrc.physics import drivetrains
from pyfrc.physics.units import units
from wpimath.geometry import Pose2d, Rotation2d

import shared_state

LEFT_PWM_CHANNEL = 0
RIGHT_PWM_CHANNEL = 1

TRACK_WIDTH_M = 0.5
MAX_SPEED_MPS = 0.5


class PhysicsEngine:

  def __init__(self, physics_controller):
    self.physics_controller = physics_controller
    self.drivetrain = drivetrains.TwoMotorDrivetrain(
        x_wheelbase = TRACK_WIDTH_M * units.meters, speed = MAX_SPEED_MPS * units.mps)

  def update_sim(self, now, tm_diff):
    if shared_state.consume_reset_request():
      self.physics_controller.field.setRobotPose(Pose2d(0, 0, Rotation2d(0)))

    l_motor = hal.simulation.getPWMSpeed(LEFT_PWM_CHANNEL)
    r_motor = hal.simulation.getPWMSpeed(RIGHT_PWM_CHANNEL)

    chassis_speeds = self.drivetrain.calculate(l_motor, r_motor)
    pose = self.physics_controller.drive(chassis_speeds, tm_diff)

    shared_state.set_pose(
        pose.X(), pose.Y(), pose.rotation().radians(),
        chassis_speeds.vx, chassis_speeds.omega)

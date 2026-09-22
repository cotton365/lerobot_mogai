#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import math
import time
from functools import cached_property

import jkrc

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_jaka_follower import JakaFollowerConfig

logger = logging.getLogger(__name__)

# JAKA 6-axis joint names
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# SO-101 leader action keys in order, matching JAKA joint_1..joint_6
LEADER_ACTION_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class JakaFollower(Robot):
    """
    JAKA 6-axis industrial robot as a LeRobot follower.

    Connects via TCP/IP using the jkrc SDK. The controller runs an internal
    8ms servo loop; ``servo_j`` feeds target positions into that loop.
    """

    config_class = JakaFollowerConfig
    name = "jaka_follower"

    def __init__(self, config: JakaFollowerConfig):
        super().__init__(config)
        self.config = config
        self._robot = None
        self._cached_obs: RobotObservation = {}
        self._warmup_count = 0
        self._shoulder_pan_zero = -149.0
        self._shoulder_lift_zero = -149.0
        self._elbow_flex_zero = -149.0
        self._wrist_flex_zero = -149.0
        self._wrist_roll_zero = -149.0
        self._joint6_zero = -149.0
        self._previous_speed_1 = 0.0
        self._last_target_j1 = None
        self._previous_speed_2 = 0.0
        self._last_target_j2 = None
        self._previous_speed_3 = 0.0
        self._last_target_j3 = None
        self._previous_speed_4 = 0.0
        self._last_target_j4 = None
        self._previous_speed_5 = 0.0
        self._last_target_j5 = None
        self._previous_speed = 0.0
        self._last_target_j6 = None

    # ── features ──────────────────────────────────────────────────────

    @cached_property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    # ── connection state ──────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._robot is not None

    @property
    def is_calibrated(self) -> bool:
        return True  # industrial arm — factory calibrated

    # ── lifecycle ─────────────────────────────────────────────────────

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        logger.info(f"Connecting to JAKA at {self.config.robot_ip} ...")
        self._robot = jkrc.RC(self.config.robot_ip)
        self._robot.login()
        time.sleep(1)
        self._robot.power_on()
        time.sleep(7)
        self._robot.enable_robot()
        time.sleep(2)

        #每次拉回初始位置
        reserve_jog = [0, 1.5, 0.01, 1.6, 0.0, 1.5]
        self._robot.joint_move(reserve_jog, 0, True, 1.0)
        time.sleep(1)

        self.configure()
        logger.info(f"{self} connected.")

    def configure(self) -> None:
        self._robot.servo_move_use_joint_NLF(max_vr=120, max_ar=180, max_jr=720)
        self._robot.servo_move_enable(True)
        time.sleep(1)

        # Seed the observation cache with real initial positions
        ret = self._robot.get_joint_position()
        if ret[0] != 0 or len(ret) < 2 or len(ret[1]) != 6:
            raise ConnectionError(
                f"Failed to read joint positions from JAKA: ret={ret}. "
                "Check that the robot is powered on and reachable."
            )
        joints_rad = ret[1]
        self._jaka_j1_zero = joints_rad[0]  # actual JAKA joint_1 (rad)
        self._jaka_j2_zero = joints_rad[1]  # actual JAKA joint_2 (rad)
        self._jaka_j3_zero = joints_rad[2]  # actual JAKA joint_3 (rad)
        self._jaka_j4_zero = joints_rad[3]  # actual JAKA joint_4 (rad)
        self._jaka_j5_zero = joints_rad[4]  # actual JAKA joint_5 (rad)
        self._jaka_j6_zero = joints_rad[5]  # actual JAKA joint_6 (rad)
        self._jaka_init_joints = list(joints_rad)  # full warm-up reference
        self._cached_obs = {
            f"{name}.pos": math.degrees(val) for name, val in zip(JOINT_NAMES, joints_rad)
        }
        self._previous_speed_1 = 0.0
        self._last_target_j1 = joints_rad[0]
        self._previous_speed_2 = 0.0
        self._last_target_j2 = joints_rad[1]
        self._previous_speed_3 = 0.0
        self._last_target_j3 = joints_rad[2]
        self._previous_speed_4 = 0.0
        self._last_target_j4 = joints_rad[3]
        self._previous_speed_5 = 0.0
        self._last_target_j5 = joints_rad[4]
        self._previous_speed = 0.0
        self._last_target_j6 = joints_rad[5]

        # No warm-up — servo_move_enable already initializes the servo loop.

    def calibrate(self) -> None:
        pass  # industrial arm does not need LeRobot-style calibration

    # ── calibration constants (test mode) ────────────────────────────────
    # _jaka_j6_zero is set from actual JAKA position in configure()

    # ── runtime ───────────────────────────────────────────────────────

    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Return cached observation to avoid a TCP round-trip.
        # The teleop loop calls this every iteration, but we only need
        # the single TCP call in send_action() to hit the 8ms window.
        return self._cached_obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._warmup_count += 1

        if self._warmup_count == 1:
            logger.info(">>> Warm-up started (300 iterations, ~5s) <<<")

        if self._warmup_count <= 300:
            jog = self._jaka_init_joints
        else:
            leader_key_1 = f"{self.config.leader_joint_1}.pos"
            leader_key_2 = f"{self.config.leader_joint_2}.pos"
            leader_key_3 = f"{self.config.leader_joint_3}.pos"
            leader_key_4 = f"{self.config.leader_joint_4}.pos"
            leader_key_5 = f"{self.config.leader_joint_5}.pos"
            leader_key_6 = f"{self.config.leader_joint}.pos"
            missing_keys = [
                key for key in (leader_key_1, leader_key_2, leader_key_3, leader_key_4, leader_key_5, leader_key_6)
                if key not in action
            ]
            if missing_keys:
                raise KeyError(
                    f"Action is missing {missing_keys}. Got keys: {sorted(action)}"
                )

            leader_deg_1 = action[leader_key_1]
            leader_deg_2 = action[leader_key_2]
            leader_deg_3 = action[leader_key_3]
            leader_deg_4 = action[leader_key_4]
            leader_deg_5 = action[leader_key_5]
            leader_deg_6 = action[leader_key_6]

            if self._warmup_count == 301:
                self._shoulder_pan_zero = leader_deg_1
                self._shoulder_lift_zero = leader_deg_2
                self._elbow_flex_zero = leader_deg_3
                self._wrist_flex_zero = leader_deg_4
                self._wrist_roll_zero = leader_deg_5
                self._joint6_zero = leader_deg_6
                logger.info(
                    f">>> Warm-up done, live mapping active. "
                    f"shoulder_pan_zero={self._shoulder_pan_zero:.2f}° "
                    f"shoulder_lift_zero={self._shoulder_lift_zero:.2f}° "
                    f"elbow_flex_zero={self._elbow_flex_zero:.2f}° "
                    f"wrist_flex_zero={self._wrist_flex_zero:.2f}° "
                    f"wrist_roll_zero={self._wrist_roll_zero:.2f}° "
                    f"joint6_zero={self._joint6_zero:.2f}° <<<"
                )

            delta_deg_1 = leader_deg_1 - self._shoulder_pan_zero
            delta_deg_2 = leader_deg_2 - self._shoulder_lift_zero
            delta_deg_3 = leader_deg_3 - self._elbow_flex_zero
            delta_deg_4 = leader_deg_4 - self._wrist_flex_zero
            delta_deg_5 = leader_deg_5 - self._wrist_roll_zero
            delta_deg_6 = leader_deg_6 - self._joint6_zero

            # 默认所有关节反向 1:1 映射。实测 joint_2 / joint_4 因物理翻转需要正向，
            # 因此这两个关节使用 + ，其余保持 -。
            raw_target_j1 = self._jaka_j1_zero - math.radians(delta_deg_1)
            raw_target_j2 = self._jaka_j2_zero + math.radians(delta_deg_2)
            raw_target_j3 = self._jaka_j3_zero - math.radians(delta_deg_3)
            raw_target_j4 = self._jaka_j4_zero + math.radians(delta_deg_4)
            raw_target_j5 = self._jaka_j5_zero - math.radians(delta_deg_5)
            raw_target_j6 = self._jaka_j6_zero - math.radians(delta_deg_6)

            dt = self.config.control_period_s
            max_accel = self.config.max_accel_rad_s2
            max_speed = self.config.max_speed_rad_s

            # Joint_1 speed/acceleration limiting (same algorithm as below).
            reference_j1 = self._last_target_j1
            diff_j1 = raw_target_j1 - reference_j1
            desired_speed_j1 = diff_j1 / dt
            accel_j1 = (desired_speed_j1 - self._previous_speed_1) / dt
            accel_j1 = max(-max_accel, min(max_accel, accel_j1))
            speed_j1 = self._previous_speed_1 + accel_j1 * dt
            speed_j1 = max(-max_speed, min(max_speed, speed_j1))
            cmd_step_j1 = speed_j1 * dt
            self._previous_speed_1 = speed_j1
            jaka_j1_rad = reference_j1 + cmd_step_j1
            self._last_target_j1 = jaka_j1_rad

            # Joint_2 speed/acceleration limiting (same algorithm as below).
            reference_j2 = self._last_target_j2
            diff_j2 = raw_target_j2 - reference_j2
            desired_speed_j2 = diff_j2 / dt
            accel_j2 = (desired_speed_j2 - self._previous_speed_2) / dt
            accel_j2 = max(-max_accel, min(max_accel, accel_j2))
            speed_j2 = self._previous_speed_2 + accel_j2 * dt
            speed_j2 = max(-max_speed, min(max_speed, speed_j2))
            cmd_step_j2 = speed_j2 * dt
            self._previous_speed_2 = speed_j2
            jaka_j2_rad = reference_j2 + cmd_step_j2
            self._last_target_j2 = jaka_j2_rad

            # Joint_3 speed/acceleration limiting (same algorithm as below).
            reference_j3 = self._last_target_j3
            diff_j3 = raw_target_j3 - reference_j3
            desired_speed_j3 = diff_j3 / dt
            accel_j3 = (desired_speed_j3 - self._previous_speed_3) / dt
            accel_j3 = max(-max_accel, min(max_accel, accel_j3))
            speed_j3 = self._previous_speed_3 + accel_j3 * dt
            speed_j3 = max(-max_speed, min(max_speed, speed_j3))
            cmd_step_j3 = speed_j3 * dt
            self._previous_speed_3 = speed_j3
            jaka_j3_rad = reference_j3 + cmd_step_j3
            self._last_target_j3 = jaka_j3_rad

            # Joint_4 speed/acceleration limiting (same algorithm as joint_5/6).
            reference_j4 = self._last_target_j4
            diff_j4 = raw_target_j4 - reference_j4
            desired_speed_j4 = diff_j4 / dt
            accel_j4 = (desired_speed_j4 - self._previous_speed_4) / dt
            accel_j4 = max(-max_accel, min(max_accel, accel_j4))
            speed_j4 = self._previous_speed_4 + accel_j4 * dt
            speed_j4 = max(-max_speed, min(max_speed, speed_j4))
            cmd_step_j4 = speed_j4 * dt
            self._previous_speed_4 = speed_j4
            jaka_j4_rad = reference_j4 + cmd_step_j4
            self._last_target_j4 = jaka_j4_rad

            # Joint_5 speed/acceleration limiting (same algorithm as joint_6).
            reference_j5 = self._last_target_j5
            diff_j5 = raw_target_j5 - reference_j5
            desired_speed_j5 = diff_j5 / dt
            accel_j5 = (desired_speed_j5 - self._previous_speed_5) / dt
            accel_j5 = max(-max_accel, min(max_accel, accel_j5))
            speed_j5 = self._previous_speed_5 + accel_j5 * dt
            speed_j5 = max(-max_speed, min(max_speed, speed_j5))
            cmd_step_j5 = speed_j5 * dt
            self._previous_speed_5 = speed_j5
            jaka_j5_rad = reference_j5 + cmd_step_j5
            self._last_target_j5 = jaka_j5_rad

            # Joint_6 speed/acceleration limiting (existing, unchanged).
            reference_j6 = self._last_target_j6
            diff_j6 = raw_target_j6 - reference_j6
            desired_speed_j6 = diff_j6 / dt
            accel_j6 = (desired_speed_j6 - self._previous_speed) / dt
            accel_j6 = max(-max_accel, min(max_accel, accel_j6))
            speed_j6 = self._previous_speed + accel_j6 * dt
            speed_j6 = max(-max_speed, min(max_speed, speed_j6))
            cmd_step_j6 = speed_j6 * dt
            self._previous_speed = speed_j6
            jaka_j6_rad = reference_j6 + cmd_step_j6
            self._last_target_j6 = jaka_j6_rad

            jog = [jaka_j1_rad, jaka_j2_rad, jaka_j3_rad, jaka_j4_rad, jaka_j5_rad, jaka_j6_rad]

            # Periodic status every ~2s (120 iters), non-intrusive
            if self._warmup_count % 120 == 0:
                logger.info(
                    f"shoulder_pan={leader_deg_1:.2f}° delta1={delta_deg_1:.2f}° "
                    f"shoulder_lift={leader_deg_2:.2f}° delta2={delta_deg_2:.2f}° "
                    f"elbow_flex={leader_deg_3:.2f}° delta3={delta_deg_3:.2f}° "
                    f"wrist_flex={leader_deg_4:.2f}° delta4={delta_deg_4:.2f}° "
                    f"wrist_roll={leader_deg_5:.2f}° delta5={delta_deg_5:.2f}° "
                    f"joint6={leader_deg_6:.2f}° delta6={delta_deg_6:.2f}° "
                    f"target_j1={jaka_j1_rad:.3f}rad "
                    f"target_j2={jaka_j2_rad:.3f}rad "
                    f"target_j3={jaka_j3_rad:.3f}rad "
                    f"target_j4={jaka_j4_rad:.3f}rad "
                    f"target_j5={jaka_j5_rad:.3f}rad "
                    f"target_j6={jaka_j6_rad:.3f}rad"
                )

        ret = self._robot.servo_j(jog, 0, 1)
        if ret[0] != 0:
            logger.warning(f"servo_j error code: {ret[0]}")

        # Update cache in degrees (jog is in radians)
        self._cached_obs = {
            f"{name}.pos": math.degrees(v_rad)
            for name, v_rad in zip(JOINT_NAMES, jog)
        }

        return action

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._robot.servo_move_enable(False)
        self._robot.disable_robot()
        self._robot.power_off()
        self._robot.logout()
        self._robot = None
        logger.info(f"{self} disconnected.")

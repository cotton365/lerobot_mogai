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

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_mycobot_leader import MyCobotLeaderConfig

logger = logging.getLogger(__name__)

# Joint names in order. The first 5 match MyCobotFollower (so101-style names),
# joint6 is the extra wrist axis of the MyCobot arm and gripper the claw motor.
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "joint6",
    "gripper",
]


class MyCobotLeader(Robot):
    """
    MyCobot leader arm driven by Feetech STS3215 motors.

    Mechanically identical to the MyCobot 280 follower, but wired with the
    same Feetech motors bus as the SO-101. The arm is used as a teleoperation
    leader: its joint positions are read and fed to a follower (e.g.
    MyCobotFollower.send_action). Arm joints are reported in degrees, which
    is what MyCobotFollower expects from its leader.

    Calibration follows the same interactive Feetech flow as the SO leader:
    run ``lerobot-calibrate --robot.type=mycobot_leader`` or pass
    ``calibrate=True`` to :meth:`connect`.
    """

    config_class = MyCobotLeaderConfig
    name = "mycobot_leader"

    def __init__(self, config: MyCobotLeaderConfig):
        super().__init__(config)
        self.config = config

        if len(config.motor_ids) != len(JOINT_NAMES):
            raise ValueError(
                f"Expected {len(JOINT_NAMES)} motor_ids (one per joint in {JOINT_NAMES}), got {len(config.motor_ids)}"
            )

        motors = {}
        for joint_name, motor_id in zip(JOINT_NAMES, config.motor_ids):
            if joint_name == "gripper":
                norm_mode = MotorNormMode.RANGE_0_100
            else:
                norm_mode = MotorNormMode.DEGREES
            motors[joint_name] = Motor(motor_id, "sts3215", norm_mode)

        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
        )

    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        # On the MyCobot leader both wrist_roll and joint6 are typically
        # continuous-turn axes. Treat them as full-turn motors and record the
        # remaining joints (shoulder/elbow/wrist/gripper) interactively.
        full_turn_motors = ["wrist_roll", "joint6"]
        unknown_range_motors = [motor for motor in self.bus.motors if motor not in full_turn_motors]
        print(
            f"Move all joints except {full_turn_motors} sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        for motor in full_turn_motors:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs = self.bus.sync_read("Present_Position")
        obs = {f"{motor}.pos": val for motor, val in obs.items()}
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # The leader arm does not receive commands; return the action unchanged.
        logger.debug(f"{self} ignores action {action}")
        return action

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")

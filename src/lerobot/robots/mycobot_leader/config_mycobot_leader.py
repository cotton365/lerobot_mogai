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

from dataclasses import dataclass, field

from ..config import RobotConfig


@RobotConfig.register_subclass("mycobot_leader")
@dataclass
class MyCobotLeaderConfig(RobotConfig):
    """Configuration for the MyCobot leader arm driven by Feetech STS3215 motors."""

    # Serial port of the Feetech motors bus
    port: str = "/dev/ttyUSB0"

    # Motor ID for each joint, in JOINT_NAMES order
    # (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, joint6, gripper)
    motor_ids: tuple = field(default=(1, 2, 3, 4, 5, 6, 7))

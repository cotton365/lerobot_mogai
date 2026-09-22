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

from dataclasses import dataclass

from ..config import RobotConfig


@RobotConfig.register_subclass("jaka_follower")
@dataclass
class JakaFollowerConfig(RobotConfig):
    """Configuration for JAKA 6-axis industrial robot via TCP/IP (jkrc SDK)."""

    # TCP/IP address of the JAKA robot controller
    robot_ip: str = "10.5.5.100"

    # Which leader joint controls JAKA joint_1. MyCobot's 1st joint is shoulder_pan.
    leader_joint_1: str = "shoulder_pan"

    # Which leader joint controls JAKA joint_2. MyCobot's 2nd joint is shoulder_lift.
    leader_joint_2: str = "shoulder_lift"

    # Which leader joint controls JAKA joint_3. MyCobot's 3rd joint is elbow_flex.
    leader_joint_3: str = "elbow_flex"

    # Which leader joint controls JAKA joint_4. MyCobot's 4th joint is wrist_flex.
    leader_joint_4: str = "wrist_flex"

    # Which leader joint controls JAKA joint_5. MyCobot's 5th joint is wrist_roll.
    leader_joint_5: str = "wrist_roll"

    # Which leader joint controls JAKA joint_6. MyCobot's 6th joint is joint6.
    leader_joint: str = "joint6"

    # Speed/acceleration limiting for the mapped JAKA joints. The controller
    # computes the instantaneous velocity as diff / control_period_s, then
    # applies both acceleration and velocity clamps before converting back to
    # a step.
    control_period_s: float = 0.008
    max_speed_rad_s: float = 1.25
    max_accel_rad_s2: float = 9.0

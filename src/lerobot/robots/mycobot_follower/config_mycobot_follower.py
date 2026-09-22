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


@RobotConfig.register_subclass("mycobot_follower")
@dataclass
class MyCobotFollowerConfig(RobotConfig):
    """Configuration for Elephant Robotics MyCobot 280 follower via pymycobot."""

    # Serial port
    port: str = "/dev/ttyUSB0"

    # Baud rate
    baudrate: str = "115200"

    # Default speed for teleop (1-100)
    default_speed: int = 50

    # Speed during sync phase (1-100, slower = safer)
    sync_speed: int = 25

    # Number of interpolation steps during sync
    sync_steps: int = 100

    # Step interval during sync (seconds)
    sync_interval: float = 0.03

    # Maximum allowed leader offset during sync (degrees).
    # If any joint exceeds this, sync is aborted — user needs to reposition.
    max_sync_offset: float = 15.0

    # Direction sign per joint: +1 or -1.
    # Flip to -1 if that joint moves opposite to the leader.
    joint_directions: tuple = field(default=(1, 1, 1, 1, 1))

    # --- Joint range parameters (degrees) ---
    # Leader (SO-101) max positive excursion per joint
    leader_max_pos: tuple = field(default=(110.0, 100.0, 97.0, 95.0, 163.0))
    # Leader (SO-101) max negative excursion per joint (absolute value)
    leader_max_neg: tuple = field(default=(110.0, 100.0, 97.0, 95.0, 157.0))
    # Follower (MyCobot 280) max positive excursion per joint
    follower_max_pos: tuple = field(default=(168.0, 140.0, 150.0, 150.0, 160.0))
    # Follower (MyCobot 280) max negative excursion per joint (absolute value)
    follower_max_neg: tuple = field(default=(168.0, 140.0, 150.0, 150.0, 155.0))

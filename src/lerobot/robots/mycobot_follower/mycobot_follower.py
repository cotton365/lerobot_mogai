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
import time
from functools import cached_property

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_mycobot_follower import MyCobotFollowerConfig

logger = logging.getLogger(__name__)

# ── Joint mapping: SO-101 name → MyCobot joint ID (1-indexed) ──────
JOINT_MAP = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
}
JOINT_NAMES = list(JOINT_MAP.keys())  # ordered: shoulder_pan .. wrist_roll
NUM_ARM_JOINTS = len(JOINT_NAMES)      # 5
NUM_MYCOBOT_JOINTS = 6                 # MyCobot has 6 joints total

# Encoder resolution (12-bit)
ENCODER_MAX = 4095
ENCODER_STEPS_PER_REV = 4096  # steps per 360°


class MyCobotFollower(Robot):
    """
    Elephant Robotics MyCobot 280 as a LeRobot follower.

    Controlled by an SO-101 leader via normalized delta mapping.
    Joint ranges of leader and follower are normalized so that each
    joint moves to the same *percentage* of its physical range.

    Phase 1 (sync): On first ``send_action``, the MyCobot slowly moves
    to match the leader's current (slightly-off-zero) position.

    Phase 2 (teleop): Delta-based following with range normalization.
    """

    config_class = MyCobotFollowerConfig
    name = "mycobot_follower"

    def __init__(self, config: MyCobotFollowerConfig):
        super().__init__(config)
        self.config = config
        self._robot = None               # pymycobot.MyCobot280 instance
        self._synced = False
        self._leader_ref: dict[str, float] = {}   # leader angles at sync point
        self._baseline: list[int] = []            # MyCobot encoders at sync point
        self._cached_obs: RobotObservation = {}

        # Unpack tuple configs into indexed lists for fast access
        self._leader_max_pos = list(config.leader_max_pos)
        self._leader_max_neg = list(config.leader_max_neg)
        self._follower_max_pos = list(config.follower_max_pos)
        self._follower_max_neg = list(config.follower_max_neg)
        self._directions = list(config.joint_directions)

    # ── Robot interface properties ──────────────────────────────────

    @cached_property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._robot is not None

    @property
    def is_calibrated(self) -> bool:
        return True  # no LeRobot-style calibration needed

    # ── Lifecycle ───────────────────────────────────────────────────

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        from pymycobot import MyCobot280

        logger.info(f"Connecting to MyCobot 280 on {self.config.port} @ {self.config.baudrate} ...")
        self._robot = MyCobot280(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=0.1,
            debug=False,
        )

        # Power on and enable servos
        self._robot.power_on()
        time.sleep(0.5)
        self._robot.focus_all_servos()
        time.sleep(0.5)

        # Verify we can read encoders
        try:
            encs = self._robot.get_encoders()
            if not isinstance(encs, list) or len(encs) < NUM_MYCOBOT_JOINTS:
                raise ConnectionError(
                    f"Failed to read encoders from MyCobot 280: got {encs}. "
                    "Check power and serial connection."
                )
        except Exception as e:
            self._robot = None
            raise ConnectionError(
                f"Could not communicate with MyCobot 280 on {self.config.port}: {e}"
            ) from e

        self._synced = False
        self._leader_ref = {}
        self._baseline = []
        self.configure()
        logger.info(f"{self} connected. Encoders: {encs}")

    def configure(self) -> None:
        pass  # MyCobot 280 needs no extra configuration

    def calibrate(self) -> None:
        pass  # factory calibrated

    # ── Phase 1: Sync ───────────────────────────────────────────────

    def _sync_phase(self, action: RobotAction) -> None:
        """
        Move MyCobot to match the leader's current (slightly-off-zero) position.

        Called once on the first ``send_action``. Blocks for ~3 seconds while
        the MyCobot slowly interpolates to the target.
        """
        logger.info(">>> Phase 1: Syncing MyCobot to leader position <<<")

        # ── Read current MyCobot encoders ───────────────────────────
        current_encs = self._robot.get_encoders()
        if len(current_encs) < NUM_MYCOBOT_JOINTS:
            raise ConnectionError("Failed to read MyCobot encoders during sync.")

        # ── Compute target encoders from leader offset ──────────────
        targets = list(current_encs)  # copy
        leader_offsets = {}

        for idx, joint_name in enumerate(JOINT_NAMES):
            leader_key = f"{joint_name}.pos"
            if leader_key not in action:
                continue

            leader_deg = float(action[leader_key])
            leader_offsets[joint_name] = leader_deg

            # Safety check: leader offset must be within reasonable range
            if abs(leader_deg) > self.config.max_sync_offset:
                logger.warning(
                    f"Leader {joint_name} offset {leader_deg:.1f}° exceeds "
                    f"max_sync_offset {self.config.max_sync_offset}°. "
                    "Please reposition the leader closer to zero and restart."
                )
                # Clamp instead of aborting
                leader_deg = max(-self.config.max_sync_offset,
                                 min(self.config.max_sync_offset, leader_deg))

            delta_enc = self._deg_to_enc_delta(leader_deg, idx)
            ele_id = JOINT_MAP[joint_name]
            targets[ele_id - 1] = int(current_encs[ele_id - 1] + delta_enc)
            # Clamp to valid encoder range
            targets[ele_id - 1] = max(0, min(ENCODER_MAX, targets[ele_id - 1]))

        # Joint 6 stays put
        targets[5] = current_encs[5]

        logger.info(f"Leader offsets: {leader_offsets}")
        logger.info(f"MyCobot current encoders: {current_encs}")
        logger.info(f"MyCobot target  encoders: {targets}")

        # ── Interpolate slowly to targets ───────────────────────────
        steps = self.config.sync_steps
        interval = self.config.sync_interval
        speed = self.config.sync_speed

        print(f"\n  Syncing MyCobot to leader position ({steps} steps, speed={speed})...")
        print(f"  Please keep the leader still during sync.\n")

        try:
            for i in range(1, steps + 1):
                t = i / steps
                interp = [
                    int(current_encs[j] + (targets[j] - current_encs[j]) * t)
                    for j in range(NUM_MYCOBOT_JOINTS)
                ]
                self._robot.set_encoders(interp, speed)
                time.sleep(interval)

                if i % 20 == 1 or i == steps:
                    pct = i / steps * 100
                    print(f"\r  Sync progress: {pct:.0f}%", end="", flush=True)

            print()  # newline after progress
        except KeyboardInterrupt:
            logger.info("Sync interrupted by user.")
            raise

        # ── Record reference ────────────────────────────────────────
        self._leader_ref = {
            joint_name: leader_offsets.get(joint_name, 0.0)
            for joint_name in JOINT_NAMES
        }
        self._baseline = targets
        self._synced = True

        logger.info(f">>> Sync complete. leader_ref={self._leader_ref} <<<")
        logger.info(f">>> baseline={self._baseline} <<<")

    # ── Normalization helpers ───────────────────────────────────────

    def _deg_to_enc_delta(self, leader_deg: float, joint_idx: int) -> float:
        """
        Convert a leader degree offset to a follower encoder delta
        using asymmetric range normalization.
        """
        if leader_deg >= 0:
            leader_norm = leader_deg / self._leader_max_pos[joint_idx]
            follower_deg = leader_norm * self._follower_max_pos[joint_idx]
        else:
            leader_norm = leader_deg / self._leader_max_neg[joint_idx]
            follower_deg = leader_norm * self._follower_max_neg[joint_idx]

        direction = self._directions[joint_idx]
        return direction * (follower_deg / 360.0 * ENCODER_STEPS_PER_REV)

    # ── Phase 2: Runtime ────────────────────────────────────────────

    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self._cached_obs:
            return self._cached_obs

        # Fallback: read actual encoders and convert to approximate degrees
        try:
            encs = self._robot.get_encoders()
            obs = {}
            for idx, joint_name in enumerate(JOINT_NAMES):
                ele_id = JOINT_MAP[joint_name]
                enc = encs[ele_id - 1]
                # Convert encoder to approximate degrees relative to baseline
                if self._synced and len(self._baseline) > ele_id - 1:
                    delta = enc - self._baseline[ele_id - 1]
                    deg = delta / ENCODER_STEPS_PER_REV * 360.0
                else:
                    deg = float(enc)
                obs[f"{joint_name}.pos"] = deg
            return obs
        except Exception:
            return {f"{j}.pos": 0.0 for j in JOINT_NAMES}

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # ── Phase 1: Sync on first call ─────────────────────────────
        if not self._synced:
            self._sync_phase(action)
            # After sync, the follower is at baseline — return the action as-is
            return action

        # ── Phase 2: Normal teleop ──────────────────────────────────
        targets = list(self._baseline)  # start from baseline
        # Ensure targets has 6 elements (pad with baseline or current)
        if len(targets) < NUM_MYCOBOT_JOINTS:
            # Shouldn't happen, but be safe
            current = self._robot.get_encoders()
            targets = list(current)

        for idx, joint_name in enumerate(JOINT_NAMES):
            leader_key = f"{joint_name}.pos"
            if leader_key not in action:
                continue

            leader_current = float(action[leader_key])
            leader_ref = self._leader_ref.get(joint_name, 0.0)
            leader_delta = leader_current - leader_ref
            delta_enc = self._deg_to_enc_delta(leader_delta, idx)

            ele_id = JOINT_MAP[joint_name]
            raw_target = self._baseline[ele_id - 1] + delta_enc
            targets[ele_id - 1] = max(0, min(ENCODER_MAX, int(raw_target)))

        # Joint 6: keep at baseline (don't move)
        if len(self._baseline) >= NUM_MYCOBOT_JOINTS:
            targets[5] = self._baseline[5]

        self._robot.set_encoders(targets, self.config.default_speed)

        # Update cached observation
        self._cached_obs = {
            f"{joint_name}.pos": float(action.get(f"{joint_name}.pos", 0.0))
            for joint_name in JOINT_NAMES
        }

        return action

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info(f"Disconnecting {self} ...")
        self._robot = None
        self._synced = False
        self._leader_ref = {}
        self._baseline = []
        self._cached_obs = {}
        logger.info(f"{self} disconnected.")

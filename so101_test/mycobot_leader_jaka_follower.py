#!/usr/bin/env python

"""
MyCobotLeader → JakaFollower 遥操测试（全 6 轴映射）。

映射关系:
    MyCobot shoulder_pan  -> JAKA joint_1
    MyCobot shoulder_lift -> JAKA joint_2
    MyCobot elbow_flex    -> JAKA joint_3
    MyCobot wrist_flex    -> JAKA joint_4
    MyCobot wrist_roll    -> JAKA joint_5
    MyCobot joint6        -> JAKA joint_6

用法:
    python so101_test/mycobot_leader_jaka_follower.py \
        --leader-port /dev/ttyUSB0 \
        --robot-ip 10.5.5.100 \
        --id jaka \
        --fps 125

如果 MyCobot leader 还没有校准文件，先运行:
    lerobot-calibrate --robot.type=mycobot_leader --robot.port=/dev/ttyUSB0 --robot.id=leader
或者在本脚本加 --calibrate 在连接时执行交互校准。
方向：芯片在右手
"""

import argparse
import logging
import signal
import sys
import time
import math

from lerobot.robots.jaka_follower import JakaFollower, JakaFollowerConfig
from lerobot.robots.mycobot_leader import MyCobotLeader, MyCobotLeaderConfig
from lerobot.utils.robot_utils import precise_sleep

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("mycobot_leader_jaka_follower")

_running = True

# 固定映射：MyCobot 六轴 -> JAKA 六轴
LEADER_JOINT_1 = "shoulder_pan"
LEADER_JOINT_2 = "shoulder_lift"
LEADER_JOINT_3 = "elbow_flex"
LEADER_JOINT_4 = "wrist_flex"
LEADER_JOINT_5 = "wrist_roll"
LEADER_JOINT_6 = "joint6"


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n\n正在停止...")


def main():
    global _running
    parser = argparse.ArgumentParser(description="MyCobotLeader -> JakaFollower teleop (6-axis)")
    parser.add_argument("--leader-port", required=True, help="MyCobot leader serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--robot-ip", default="10.5.5.100", help="JAKA controller IP")
    parser.add_argument("--id", default="jaka", help="Robot id used for config/calibration paths")
    parser.add_argument("--leader-id", default="leader", help="MyCobot leader id used for calibration path")
    parser.add_argument("--fps", type=float, default=125, help="Control loop frequency (Hz). "
                        "JAKA servo uses step=1 (8ms per cycle), so 1/0.008 = 125 Hz.")
    parser.add_argument("--calibrate", action="store_true", help="Run interactive calibration if not calibrated")
    parser.add_argument("--max-speed-rad-s", type=float, default=None,
                        help="Optional override for max joint speed in rad/s (defaults to JakaFollowerConfig)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    # ── Connect MyCobot leader ──────────────────────────────────────
    print(f"[1/3] Connecting MyCobot leader on {args.leader_port} ...")
    leader_cfg = MyCobotLeaderConfig(port=args.leader_port, id=args.leader_id)
    leader = MyCobotLeader(leader_cfg)
    leader.connect(calibrate=args.calibrate)
    print(f"  ✅ MyCobot leader ready. calibrated={leader.is_calibrated}")

    if not leader.is_calibrated:
        print(
            "\n错误: MyCobot leader 没有可用的校准文件。\n"
            f"  当前使用的 calibration id = '{args.leader_id}'\n"
            "  请确认校准时的 id 是否一致，例如:\n"
            "    lerobot-calibrate --robot.type=mycobot_leader --robot.port=/dev/ttyUSB0 --robot.id=leader\n"
            "  或者在运行本脚本时指定正确的 --leader-id。\n"
            "  也可以加 --calibrate 重新执行交互校准。"
        )
        leader.disconnect()
        sys.exit(1)

    # ── Connect JAKA follower ───────────────────────────────────────
    print(f"[2/3] Connecting JAKA follower at {args.robot_ip} ...")
    follower_cfg_kwargs = {
        "robot_ip": args.robot_ip,
        "id": args.id,
        "leader_joint_1": LEADER_JOINT_1,
        "leader_joint_2": LEADER_JOINT_2,
        "leader_joint_3": LEADER_JOINT_3,
        "leader_joint_4": LEADER_JOINT_4,
        "leader_joint_5": LEADER_JOINT_5,
        "leader_joint": LEADER_JOINT_6,
    }
    if args.max_speed_rad_s is not None:
        follower_cfg_kwargs["max_speed_rad_s"] = args.max_speed_rad_s
    follower_cfg = JakaFollowerConfig(**follower_cfg_kwargs)
    follower = JakaFollower(follower_cfg)
    follower.connect()
    print(f"  ✅ JAKA follower ready.")

    print(
        f"[3/3] Teleop started. Move leader '{LEADER_JOINT_1}' -> JAKA joint_1, "
        f"'{LEADER_JOINT_2}' -> JAKA joint_2, "
        f"'{LEADER_JOINT_3}' -> JAKA joint_3, "
        f"'{LEADER_JOINT_4}' -> JAKA joint_4, "
        f"'{LEADER_JOINT_5}' -> JAKA joint_5, "
        f"'{LEADER_JOINT_6}' -> JAKA joint_6. Ctrl+C to exit.\n"
    )

    frame_period = 1.0 / args.fps
    frame = 0

    try:
        while _running:
            loop_start = time.perf_counter()

            obs = leader.get_observation()
            action = {
                f"{LEADER_JOINT_1}.pos": obs[f"{LEADER_JOINT_1}.pos"],
                f"{LEADER_JOINT_2}.pos": obs[f"{LEADER_JOINT_2}.pos"],
                f"{LEADER_JOINT_3}.pos": obs[f"{LEADER_JOINT_3}.pos"],
                f"{LEADER_JOINT_4}.pos": obs[f"{LEADER_JOINT_4}.pos"],
                f"{LEADER_JOINT_5}.pos": obs[f"{LEADER_JOINT_5}.pos"],
                f"{LEADER_JOINT_6}.pos": obs[f"{LEADER_JOINT_6}.pos"],
            }
            follower.send_action(action)

            frame += 1
            if frame % 120 == 0:
                print(
                    f"  1 {LEADER_JOINT_1}={action[f'{LEADER_JOINT_1}.pos']:.2f}°  "
                    f"JAKA target j1={follower._cached_obs['joint_1.pos']:.2f}°  "
                    f"speed1={follower._previous_speed_1:.3f}rad/s"
                )
                print(
                    f"  2 {LEADER_JOINT_2}={action[f'{LEADER_JOINT_2}.pos']:.2f}°  "
                    f"JAKA target j2={follower._cached_obs['joint_2.pos']:.2f}°  "
                    f"speed2={follower._previous_speed_2:.3f}rad/s"
                )
                print(
                    f"  3 {LEADER_JOINT_3}={action[f'{LEADER_JOINT_3}.pos']:.2f}°  "
                    f"JAKA target j3={follower._cached_obs['joint_3.pos']:.2f}°  "
                    f"speed3={follower._previous_speed_3:.3f}rad/s"
                )
                print(
                    f"  4 {LEADER_JOINT_4}={action[f'{LEADER_JOINT_4}.pos']:.2f}°  "
                    f"JAKA target j4={follower._cached_obs['joint_4.pos']:.2f}°  "
                    f"speed4={follower._previous_speed_4:.3f}rad/s"
                )
                print(
                    f"  5 {LEADER_JOINT_5}={action[f'{LEADER_JOINT_5}.pos']:.2f}°  "
                    f"JAKA target j5={follower._cached_obs['joint_5.pos']:.2f}°  "
                    f"speed5={follower._previous_speed_5:.3f}rad/s"
                )
                print(
                    f"  6 {LEADER_JOINT_6}={action[f'{LEADER_JOINT_6}.pos']:.2f}°  "
                    f"JAKA target j6={follower._cached_obs['joint_6.pos']:.2f}°  "
                    f"speed6={follower._previous_speed:.3f}rad/s"
                )

            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(frame_period - dt_s, 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        print("\n断开...")
        follower.disconnect()
        leader.disconnect()
        print("退出.")


if __name__ == "__main__":
    main()

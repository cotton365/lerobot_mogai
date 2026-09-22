#!/usr/bin/env python

"""
只读诊断：查看 MyCobotLeader 各关节当前 Homing_Offset 是否会在校准时超限。

用法:
    python so101_test/check_mycobot_leader_homing.py --port /dev/ttyACM0 --id leader

说明:
    只读取电机位置，不会写任何校准数据。
"""

import argparse

from lerobot.robots.mycobot_leader import MyCobotLeader, MyCobotLeaderConfig

# STS3215 12-bit 编码器，半圈位置
HALF_TURN = 2047


def main():
    parser = argparse.ArgumentParser(description="Check MyCobotLeader homing offset limits (read-only)")
    parser.add_argument("--port", required=True, help="MyCobot leader serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", default="leader", help="MyCobot leader id used for calibration path")
    args = parser.parse_args()

    config = MyCobotLeaderConfig(port=args.port, id=args.id)
    leader = MyCobotLeader(config)
    leader.connect(calibrate=False)

    try:
        positions = leader.bus.sync_read("Present_Position", normalize=False)

        print("\n当前各关节 Homing 偏移检查（只读）")
        print(f"{'NAME':<16} {'ID':>3} {'POS':>8} {'OFFSET':>8} {'STATUS'}")
        print("-" * 60)

        has_error = False
        for name, motor in leader.bus.motors.items():
            pos = positions[name]
            offset = pos - HALF_TURN
            ok = abs(offset) <= HALF_TURN
            status = "OK" if ok else "OVER LIMIT"
            if not ok:
                has_error = True
            print(f"{name:<16} {motor.id:>3} {pos:>8} {offset:>8}  {status}")

        print()
        if has_error:
            print("⚠ 存在超限关节。重新校准前请把这些关节摆到中间位置附近。")
        else:
            print("✅ 所有关节的 Homing Offset 都在可写范围内，可以重新校准。")
    finally:
        leader.disconnect()


if __name__ == "__main__":
    main()

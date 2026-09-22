#!/usr/bin/env python

"""
实时读取 SO-101 机械臂关节角度。

连接 SO-101 后关闭扭矩，你可以手动拖动机械臂，
终端会实时刷新显示每个关节的当前角度。

用法:
    python read_joints.py --port /dev/ttyUSB0
    python read_joints.py --port /dev/ttyACM0 --fps 20
"""

import argparse
import logging
import signal
import sys
import time

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# 关节显示名称
JOINT_LABELS = {
    "shoulder_pan":   "肩部旋转",
    "shoulder_lift":  "肩部抬升",
    "elbow_flex":     "肘部弯曲",
    "wrist_flex":     "腕部弯曲",
    "wrist_roll":     "腕部旋转",
    "gripper":        "夹爪",
}

# 停止标志
_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n正在退出...")


def _move_cursor_up(n: int) -> None:
    """光标上移 n 行."""
    sys.stdout.write(f"\033[{n}A")


def _clear_line() -> None:
    """清除当前行."""
    sys.stdout.write("\033[2K")


def _format_angle(deg: float) -> str:
    """格式化角度值，带颜色提示."""
    if abs(deg) < 0.5:
        deg = 0.0  # 忽略微小抖动
    return f"{deg:+8.2f}°"


def read_joints(port: str, fps: int = 15, use_degrees: bool = True) -> None:
    """
    连接 SO-101 并实时显示关节角度。

    Args:
        port: 串口路径，如 /dev/ttyUSB0
        fps: 刷新率
        use_degrees: True 返回角度值，False 返回 [-100, 100] 归一化值
    """
    config = SO101FollowerConfig(
        port=port,
        id="so101_test",
        use_degrees=use_degrees,
    )

    print(f"正在连接 SO-101 (端口: {port})...")
    robot = SO101Follower(config)
    robot.connect()

    # 关闭扭矩，使机械臂可手动拖动（仍可读取位置）
    robot.bus.disable_torque()
    for motor in robot.bus.motors:
        robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    print("扭矩已关闭，可以手动拖动机械臂。")
    print("按 Ctrl+C 退出。\n")
    time.sleep(0.5)

    # 记录行数，用于首次显示后的光标定位
    num_lines = len(robot.bus.motors) + 2  # joints + header + blank
    first_render = True

    try:
        while _running:
            # 读取所有关节位置
            positions = robot.bus.sync_read("Present_Position")

            if not first_render:
                _move_cursor_up(num_lines)

            # 清屏并重绘
            unit = "度" if use_degrees else "(归一化)"
            print(f"\r{'─' * 55}")
            print(f"\r  SO-101 关节角度 {unit}    [按 Ctrl+C 退出]")
            print(f"\r{'─' * 55}")

            for motor_name, label in JOINT_LABELS.items():
                angle = positions[motor_name]
                _clear_line()
                print(f"\r  {label:<12} ({motor_name:<14}) {_format_angle(angle)}")

            _clear_line()
            print(f"\r{'─' * 55}")

            first_render = False
            time.sleep(1.0 / fps)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n正在断开连接...")
        robot.disconnect()
        print("已断开。")

    print("退出。")


def main():
    parser = argparse.ArgumentParser(
        description="实时读取 SO-101 机械臂关节角度"
    )
    parser.add_argument(
        "--port", "-p",
        type=str,
        required=True,
        help="串口路径，如 /dev/ttyUSB0 或 /dev/ttyACM0",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="刷新率 (默认 15)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="显示原始归一化值 (-100~100) 而非角度",
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    read_joints(
        port=args.port,
        fps=args.fps,
        use_degrees=not args.raw,
    )


if __name__ == "__main__":
    main()

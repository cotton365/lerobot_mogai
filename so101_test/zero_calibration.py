#!/usr/bin/env python

"""
SO-101 一键置零校准脚本。

将机械臂摆到你想要的 Zero Pose（所有关节自然伸直），
运行此脚本，所有**臂关节**的读数就会全部置零（0°）。
夹爪不参与置零，保持原有校准不变。

原理：
    Present_Position = Actual_Position - Homing_Offset
    degrees = (Present_Position - mid) * 360 / 4095

    1. 读取当前原始编码器位置 raw_pos
    2. 设置 homing_offset = raw_pos - 2047，使当前位置的 Present_Position ≈ 2047
    3. 设置 range_min=0, range_max=4095，使 mid ≈ 2047.5
    4. 结果：当前位置 degrees ≈ (2047 - 2047.5) * 360 / 4095 ≈ 0°

用法：
    python so101_test/zero_calibration.py --port /dev/ttyACM0
    python so101_test/zero_calibration.py --port /dev/ttyACM0 --id my_robot
"""

import argparse
import logging
import sys

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# 编码器满量程 (12-bit)
MAX_RES = 4095
HALF_TURN = 2047

# 参与置零的臂关节（夹爪不参与）
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 关节中文标签
JOINT_LABELS = {
    "shoulder_pan":   "肩部旋转",
    "shoulder_lift":  "肩部抬升",
    "elbow_flex":     "肘部弯曲",
    "wrist_flex":     "腕部弯曲",
    "wrist_roll":     "腕部旋转",
    "gripper":        "夹爪",
}


def zero_calibration(port: str, robot_id: str = "so101_test") -> None:
    """
    一键置零校准（仅臂关节，夹爪不动）。

    Args:
        port: 串口路径
        robot_id: 校准文件 ID
    """
    config = SO101FollowerConfig(
        port=port,
        id=robot_id,
        use_degrees=True,
    )

    print(f"正在连接 SO-101 (端口: {port})...")
    robot = SO101Follower(config)
    robot.bus.connect()

    # 加载已有校准（夹爪需要它）
    if robot.calibration:
        robot.bus.write_calibration(robot.calibration)

    # 关闭扭矩以读取位置
    robot.bus.disable_torque()
    for motor in robot.bus.motors:
        robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    # --- Step 1: 重置臂关节 homing_offset，读取原始位置 ---
    # 只重置臂关节，夹爪保持原校准不动！
    print("\n[1/3] 重置臂关节 homing_offset → 0（夹爪保持原校准）...")
    robot.bus.reset_calibration(ARM_JOINTS)

    raw_positions = robot.bus.sync_read("Present_Position", normalize=False)
    print("\n  关节               raw_pos   →   homing_offset")
    print(f"  {'─' * 45}")

    # --- Step 2: 为臂关节构建零位校准 ---
    print(f"\n[2/3] 构建臂关节零位校准...")

    # 保留夹爪原有校准
    gripper_cal = robot.calibration.get("gripper") if robot.calibration else None

    robot.calibration = {}
    for motor_name, m in robot.bus.motors.items():
        if motor_name == "gripper":
            # 夹爪：恢复原有校准（保持不变）
            if gripper_cal:
                robot.calibration["gripper"] = gripper_cal
                print(f"  {'gripper':<16} {'(保持原校准)':>5}   →   {'(不变)':>6}")
            else:
                # 没有原校准，给一个安全的默认值（range 0-4095，当前位置为 mid）
                raw_pos = raw_positions[motor_name]
                homing_offset = raw_pos - HALF_TURN
                robot.calibration[motor_name] = MotorCalibration(
                    id=m.id,
                    drive_mode=0,
                    homing_offset=homing_offset,
                    range_min=0,
                    range_max=MAX_RES,
                )
                print(f"  {'gripper':<16} {raw_pos:>5}   →   {homing_offset:+6d}  (无原校准，使用默认)")
            continue

        raw_pos = raw_positions[motor_name]
        homing_offset = raw_pos - HALF_TURN

        print(f"  {motor_name:<16} {raw_pos:>5}   →   {homing_offset:+6d}")

        robot.calibration[motor_name] = MotorCalibration(
            id=m.id,
            drive_mode=0,
            homing_offset=homing_offset,
            range_min=0,
            range_max=MAX_RES,
        )

    # --- Step 3: 写入并验证 ---
    print(f"\n[3/3] 写入校准参数并验证...")
    robot.bus.write_calibration(robot.calibration)
    robot._save_calibration()
    print(f"校准文件已保存: {robot.calibration_fpath}")

    print(f"\n{'─' * 55}")
    print(f"  验证 - 臂关节应为 0°，夹爪为原值:")
    print(f"{'─' * 55}")
    positions = robot.bus.sync_read("Present_Position")
    all_zero = True
    for name in robot.bus.motors:
        val = positions[name]
        if name == "gripper":
            # 夹爪显示百分比，不参与置零检查
            print(f"  · {JOINT_LABELS.get(name, name):<12} ({name:<14}) {val:>6.1f} %  [保持不变]")
        else:
            status = "✓" if abs(val) < 0.5 else "✗"
            if abs(val) >= 0.5:
                all_zero = False
            print(f"  {status} {JOINT_LABELS.get(name, name):<12} ({name:<14}) {val:+8.2f}°")
    print(f"{'─' * 55}")

    if all_zero:
        print("\n✅ 所有臂关节已成功置零！")
    else:
        print("\n⚠ 部分臂关节仍有偏差，请检查后重试。")

    robot.disconnect()
    print("已断开连接。")


def main():
    parser = argparse.ArgumentParser(
        description="SO-101 一键置零校准 - 臂关节置零，夹爪不动"
    )
    parser.add_argument(
        "--port", "-p",
        type=str,
        required=True,
        help="串口路径，如 /dev/ttyUSB0 或 /dev/ttyACM0",
    )
    parser.add_argument(
        "--id",
        type=str,
        default="so101_test",
        help="机器人 ID，用于校准文件名 (默认: so101_test)",
    )
    args = parser.parse_args()

    try:
        zero_calibration(port=args.port, robot_id=args.id)
    except KeyboardInterrupt:
        print("\n用户中断。")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

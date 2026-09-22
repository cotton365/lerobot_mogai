#!/usr/bin/env python

"""
SO-101 缓慢归零脚本。

将所有**臂关节**从当前位置缓慢、轻柔地移动回 0°（零位）。
夹爪不参与归零，保持在原位不动。

适用于 zero_calibration.py 置零后，机械臂有偏差需要自动修正的场景。

特点：
- 小步长 + 低速，不会用力过猛
- 实时显示各关节当前角度和距离目标的偏差
- 按 Ctrl+C 可随时急停（立即断开扭矩）
- 到达容差范围内自动停止
- 夹爪完全不动，保持原位

用法：
    python so101_test/slow_return_zero.py --port /dev/ttyACM0
    python so101_test/slow_return_zero.py --port /dev/ttyACM0 --step 0.3 --tolerance 0.2
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

# ── 默认参数 ──────────────────────────────────────────────
DEFAULT_STEP_DEG = 0.3        # 每步移动度数（越小越慢越安全）
DEFAULT_STEP_INTERVAL = 0.04  # 步间间隔（秒）
DEFAULT_TOLERANCE = 0.3       # 到达容差（度）
DEFAULT_GOAL_VELOCITY = 300   # 电机转速（0=最快, 值越大越快）
DEFAULT_TORQUE_LIMIT = 1000    # 扭矩限制（0-1000）

# ── 参与归零的臂关节（夹爪不参与）──────────────────────
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# ── 中文标签 ──────────────────────────────────────────────
JOINT_LABELS = {
    "shoulder_pan":   "肩部旋转",
    "shoulder_lift":  "肩部抬升",
    "elbow_flex":     "肘部弯曲",
    "wrist_flex":     "腕部弯曲",
    "wrist_roll":     "腕部旋转",
    "gripper":        "夹爪",
}

_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n\n⚠ 收到停止信号，正在急停...")


def slow_return_to_zero(
    port: str,
    robot_id: str = "so101_test",
    step_deg: float = DEFAULT_STEP_DEG,
    step_interval: float = DEFAULT_STEP_INTERVAL,
    tolerance: float = DEFAULT_TOLERANCE,
    goal_velocity: int = DEFAULT_GOAL_VELOCITY,
    torque_limit: int = DEFAULT_TORQUE_LIMIT,
) -> None:
    """
    缓慢归零：仅臂关节逐步移动到 0°，夹爪不动。

    Args:
        port: 串口路径
        robot_id: 校准文件 ID
        step_deg: 每步最大移动度数
        step_interval: 步间间隔（秒）
        tolerance: 到达容差（度）
        goal_velocity: 电机目标转速
        torque_limit: 扭矩限制
    """
    global _running
    _running = True

    config = SO101FollowerConfig(
        port=port,
        id=robot_id,
        use_degrees=True,
    )

    # ── 连接 ──────────────────────────────────────────────
    print(f"正在连接 SO-101 (端口: {port})...")
    robot = SO101Follower(config)
    robot.bus.connect()

    if robot.calibration:
        robot.bus.write_calibration(robot.calibration)
    else:
        print("❌ 未找到校准文件，请先运行 zero_calibration.py")
        robot.bus.disconnect()
        return

    # ── 配置电机 ──────────────────────────────────────────
    robot.bus.disable_torque()

    # 臂关节：低速 + 限扭矩
    for motor in ARM_JOINTS:
        robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        robot.bus.write("P_Coefficient", motor, 16)
        robot.bus.write("I_Coefficient", motor, 0)
        robot.bus.write("D_Coefficient", motor, 32)
        robot.bus.write("Acceleration", motor, 254)
        robot.bus.write("Goal_Velocity", motor, goal_velocity)
        robot.bus.write("Torque_Limit", motor, torque_limit)

    # 夹爪：只配安全参数，不设 Goal_Velocity（防止意外移动）
    robot.bus.write("Operating_Mode", "gripper", OperatingMode.POSITION.value)
    robot.bus.write("P_Coefficient", "gripper", 16)
    robot.bus.write("I_Coefficient", "gripper", 0)
    robot.bus.write("D_Coefficient", "gripper", 32)
    robot.bus.write("Max_Torque_Limit", "gripper", 500)
    robot.bus.write("Protection_Current", "gripper", 250)
    robot.bus.write("Overload_Torque", "gripper", 25)

    robot.bus.enable_torque()

    # ── 读取初始位置 ──────────────────────────────────────
    positions = robot.bus.sync_read("Present_Position")
    gripper_val = positions.get("gripper", 0)

    print(f"\n当前状态:")
    print(f"{'─' * 55}")
    for name in robot.bus.motors:
        label = JOINT_LABELS.get(name, name)
        if name == "gripper":
            print(f"  {label:<12} ({name:<14}) {positions[name]:>6.1f} %  [保持不动]")
        else:
            deviation = abs(positions[name])
            flag = "  ← 已到位" if deviation <= tolerance else ""
            print(f"  {label:<12} ({name:<14}) {positions[name]:+8.2f}°  偏差: {deviation:.1f}°{flag}")
    print(f"{'─' * 55}")

    # 检查是否需要移动
    max_deviation = max(abs(positions[name]) for name in ARM_JOINTS)
    if max_deviation <= tolerance:
        print("\n✅ 所有臂关节已在零位，无需移动。")
        robot.bus.disable_torque()
        robot.bus.disconnect()
        return

    print(f"\n开始缓慢归零... (步长={step_deg}°, 间隔={step_interval}s,"
          f" 容差={tolerance}°, 夹爪不动)")
    print(f"按 Ctrl+C 急停\n")

    # ── 逐步移动 ──────────────────────────────────────────
    step_count = 0
    try:
        while _running:
            positions = robot.bus.sync_read("Present_Position")

            all_done = True
            goal = {}

            for name in robot.bus.motors:
                current = positions[name]

                if name == "gripper":
                    # 夹爪：保持当前位置，绝不移动
                    goal[name] = current
                    continue

                diff = 0.0 - current  # 目标是 0°

                if abs(diff) <= tolerance:
                    goal[name] = current  # 已到达
                else:
                    all_done = False
                    move = step_deg if diff > 0 else -step_deg
                    if abs(move) > abs(diff):
                        move = diff
                    goal[name] = current + move

            # 发送目标位置到电机
            robot.bus.sync_write("Goal_Position", goal)

            step_count += 1

            # 实时显示（每 5 步刷新）
            if step_count % 5 == 1 or all_done:
                _move_cursor_up(len(ARM_JOINTS) + 2)
                print(f"\r  {'─' * 50}")
                for name in ARM_JOINTS:
                    label = JOINT_LABELS.get(name, name)
                    current = positions[name]
                    deviation = abs(current)
                    bar = _progress_bar(deviation, max_deviation)
                    _clear_line()
                    print(f"\r  {label:<12} {current:+7.2f}° {bar} {deviation:5.1f}°")
                _clear_line()
                status = "✅ 到达!" if all_done else f"⏳ 移动中... (步数: {step_count})"
                print(f"\r  {'─' * 50} {status}")
                _clear_line()
                print(f"\r  夹爪: {gripper_val:.1f}% [保持不动]")

            if all_done:
                break

            time.sleep(step_interval)

    except KeyboardInterrupt:
        _running = False
        print("\n用户中断。")

    finally:
        print("\n正在安全停止...")
        robot.bus.disable_torque()
        robot.bus.disconnect()
        print("已断开连接。")

    if _running:
        print("\n✅ 归零完成！所有臂关节已到达零位。")


# ── 显示辅助 ──────────────────────────────────────────────

def _move_cursor_up(n: int) -> None:
    sys.stdout.write(f"\033[{n}A")
    sys.stdout.flush()


def _clear_line() -> None:
    sys.stdout.write("\033[2K")
    sys.stdout.flush()


def _progress_bar(value: float, max_val: float, width: int = 20) -> str:
    """偏差越小，进度条越满。"""
    ratio = min(value / max(max_val, 0.01), 1.0)
    filled = int((1 - ratio) * width)
    empty = width - filled
    return f"[{'█' * filled}{'░' * empty}]"


# ── 入口 ──────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SO-101 缓慢归零 - 臂关节轻柔归零，夹爪不动"
    )
    parser.add_argument("--port", "-p", type=str, required=True,
                        help="串口路径，如 /dev/ttyACM0")
    parser.add_argument("--id", type=str, default="so101_test",
                        help="机器人 ID (默认: so101_test)")
    parser.add_argument("--step", "-s", type=float, default=DEFAULT_STEP_DEG,
                        help=f"每步移动度数 (默认: {DEFAULT_STEP_DEG}°)")
    parser.add_argument("--interval", "-i", type=float, default=DEFAULT_STEP_INTERVAL,
                        help=f"步间间隔秒数 (默认: {DEFAULT_STEP_INTERVAL}s)")
    parser.add_argument("--tolerance", "-t", type=float, default=DEFAULT_TOLERANCE,
                        help=f"到达容差度数 (默认: {DEFAULT_TOLERANCE}°)")
    parser.add_argument("--velocity", "-v", type=int, default=DEFAULT_GOAL_VELOCITY,
                        help=f"电机转速 (默认: {DEFAULT_GOAL_VELOCITY}, 越小越慢)")
    parser.add_argument("--torque-limit", type=int, default=DEFAULT_TORQUE_LIMIT,
                        help=f"扭矩限制 0-1000 (默认: {DEFAULT_TORQUE_LIMIT})")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    try:
        slow_return_to_zero(
            port=args.port,
            robot_id=args.id,
            step_deg=args.step,
            step_interval=args.interval,
            tolerance=args.tolerance,
            goal_velocity=args.velocity,
            torque_limit=args.torque_limit,
        )
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

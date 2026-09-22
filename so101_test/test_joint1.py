#!/usr/bin/env python

"""
SO-101 → MyCobot 280 单关节测试：仅 shoulder_pan → J1。

用法：
    python so101_test/zero_calibration.py --port /dev/ttyACM0   # 先置零
    python so101_test/test_joint1.py --leader-port /dev/ttyACM0 --mycobot-port /dev/ttyACM1
"""

import argparse
import signal
import sys
import time

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# ── 归一化参数 ──────────────────────────────────────────────
LEADER_MAX = 110.0       # SO-101 shoulder_pan 正向最大
FOLLOWER_MAX = 168.0     # MyCobot J1 正向最大
DIRECTION = 1

_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n\n正在停止...")


def main():
    global _running
    parser = argparse.ArgumentParser(description="SO-101 → MyCobot 280 J1 单关节测试")
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--mycobot-port", required=True)
    parser.add_argument("--speed", type=int, default=80)
    parser.add_argument("--fps", type=int, default=120)
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    # ── 连接 SO-101 Leader ─────────────────────────────────
    print(f"连接 SO-101 ({args.leader_port})...")
    leader_config = SO101FollowerConfig(port=args.leader_port, id="so101_test", use_degrees=True)
    leader = SO101Follower(leader_config)
    leader.bus.connect()
    if leader.calibration:
        leader.bus.write_calibration(leader.calibration)
    else:
        print("❌ 无校准，先运行 zero_calibration.py")
        leader.bus.disconnect(); return
    leader.bus.disable_torque()
    for m in leader.bus.motors:
        leader.bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

    # ── 连接 MyCobot 280 ───────────────────────────────────
    print(f"连接 MyCobot 280 ({args.mycobot_port})...")
    from pymycobot import MyCobot280
    mc = MyCobot280(port=args.mycobot_port, baudrate="115200", timeout=0.1, debug=False)

    j1_now = mc.get_angles()[0]
    print(f"  MyCobot J1 当前: {j1_now:.2f}°")

    # ── Phase 1: 快速同步 ──────────────────────────────────
    leader_vals = leader.bus.sync_read("Present_Position")
    leader_deg = leader_vals.get("shoulder_pan", 0.0)
    print(f"  SO-101 shoulder_pan: {leader_deg:+.2f}°")

    if abs(leader_deg) > 20:
        print(f"  ⚠ 偏移过大，请重新 zero_calibration")
        leader.disconnect(); return

    leader_norm = leader_deg / LEADER_MAX
    follower_target = leader_norm * FOLLOWER_MAX * DIRECTION
    print(f"  归一化: {leader_deg:+.2f}° → {follower_target:+.2f}°")
    print(f"  J1 同步: {j1_now:.2f}° → {follower_target:+.2f}°")

    # 快速同步（减少步数和等待）
    steps = 20
    for i in range(1, steps + 1):
        t = i / steps
        interp = j1_now + (follower_target - j1_now) * t
        mc.send_angle(1, interp, args.speed)
        time.sleep(0.02)
    print("  同步完成")

    leader_ref = leader_deg
    j1_baseline = follower_target

    # ── Phase 2: 高速遥操循环 ──────────────────────────────
    print(f"\n  遥操中 (J1 only, speed={args.speed}). Ctrl+C 退出.\n")

    frame = 0
    fps_t = time.perf_counter()
    fps_n = 0
    t_read_sum = 0.0
    t_send_sum = 0.0
    try:
        while _running:
            t0 = time.perf_counter()

            # 读 leader
            t1 = time.perf_counter()
            leader_cur = leader.bus.sync_read("Present_Position").get("shoulder_pan", 0.0)
            t_read = time.perf_counter() - t1

            # 归一化 → J1 目标
            leader_delta = leader_cur - leader_ref
            target = j1_baseline + (leader_delta / LEADER_MAX) * FOLLOWER_MAX * DIRECTION

            # 发送
            t2 = time.perf_counter()
            mc.send_angle(1, target, args.speed)
            t_send = time.perf_counter() - t2

            frame += 1
            fps_n += 1
            t_read_sum += t_read
            t_send_sum += t_send

            # 每秒显示
            now = time.perf_counter()
            if now - fps_t >= 1.0:
                fps = fps_n / (now - fps_t)
                dt_ms = (now - t0) * 1000
                t_read_avg = t_read_sum / fps_n * 1000
                t_send_avg = t_send_sum / fps_n * 1000
                print(f"\r  FPS:{fps:.0f}  read:{t_read_avg:.1f}ms  send:{t_send_avg:.1f}ms  "
                      f"leader:{leader_cur:+6.1f}°  J1:{target:+7.2f}°  ", end="")
                print(f"\r  FPS:{fps:.0f}  dt:{dt_ms:.1f}ms  "
                      f"leader:{leader_cur:+6.1f}°  J1:{target:+7.2f}°  ", end="")
                fps_t = now
                fps_n = 0
                t_read_sum = 0.0
                t_send_sum = 0.0

    except KeyboardInterrupt:
        pass
    finally:
        print("\n断开...")
        mc = None
        leader.disconnect()
        print("退出.")


if __name__ == "__main__":
    main()

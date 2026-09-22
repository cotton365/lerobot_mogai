#!/usr/bin/env python

"""
SO-101 → MyCobot 280 全关节遥操测试（5 个臂关节）。

启动流程:
  1. SO-101 置零（先运行 zero_calibration.py，然后把臂摆到零位）
  2. MyCobot 自动归零（send_angles([0,0,0,0,0,0])）
  3. 双方以 0° 为基准开始遥操

归一化映射：leader 偏移 % → follower 偏移 %。

用法:
    python so101_test/zero_calibration.py --port /dev/ttyACM0   # SO-101 置零
    python so101_test/test_all_joints.py --leader-port /dev/ttyACM0 --mycobot-port /dev/ttyACM1
"""

import argparse
import collections
import signal
import sys
import threading
import time

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

# ── 关节映射：SO-101 → MyCobot ────────────────────────────
# (so101_key, ele_id, leader_max_pos, leader_max_neg, follower_max_pos, follower_max_neg, direction)
JOINT_CONFIG = [
    # name            ele_id  ldr_max+  ldr_max-  fol_max+  fol_max-  dir
    ("shoulder_pan",      1,     110.0,    110.0,    168.0,    168.0,   1),
    ("shoulder_lift",     2,     100.0,    100.0,    140.0,    140.0,   1),
    ("elbow_flex",        3,      97.0,     97.0,    150.0,    150.0,   1),
    ("wrist_flex",        4,      95.0,     95.0,    150.0,    150.0,   1),
    ("wrist_roll",        5,     163.0,    157.0,    160.0,    155.0,   1),
]

_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n\n正在停止...")


def main():
    global _running
    parser = argparse.ArgumentParser(description="SO-101 → MyCobot 280 全关节遥操")
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--mycobot-port", required=True)
    parser.add_argument("--speed", type=int, default=80)
    parser.add_argument("--fps", type=int, default=60, help="目标控制频率 (Hz)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    # ── Step 1: 连接 SO-101 Leader ──────────────────────────
    print(f"[1/4] 连接 SO-101 ({args.leader_port}) ...")
    cfg = SO101FollowerConfig(port=args.leader_port, id="so101_test", use_degrees=True)
    leader = SO101Follower(cfg)
    leader.bus.connect()
    if not leader.calibration:
        print("❌ 无校准，先运行 zero_calibration.py")
        leader.bus.disconnect(); return
    leader.bus.write_calibration(leader.calibration)
    leader.bus.disable_torque()
    for m in leader.bus.motors:
        leader.bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
    print(f"  ✅ SO-101 就绪 (已校准，请将臂摆到零位)")

    # ── Step 2: 连接 MyCobot 280 ────────────────────────────
    print(f"[2/4] 连接 MyCobot 280 ({args.mycobot_port}) ...")
    from pymycobot import MyCobot280
    mc = MyCobot280(port=args.mycobot_port, baudrate="115200", timeout=0.1, debug=False)
    mc.set_fresh_mode(1)  # 刷新模式：始终执行最新命令
    print(f"  ✅ MyCobot 就绪 (fresh_mode=1)")

    # ── Step 3: 双方归零 ────────────────────────────────────
    print(f"[3/4] 双方归零 ...")
    print(f"  SO-101: 请确认臂已摆到零位 (读数应接近 0°)")
    ldr_zero = leader.bus.sync_read("Present_Position")
    for name, *_ in JOINT_CONFIG:
        val = ldr_zero.get(name, 0.0)
        flag = "⚠" if abs(val) > 10 else "✅"
        print(f"    {name:<16} {val:+7.2f}°  {flag}")

    print(f"  MyCobot: 正在归零 ...")
    mc.send_angles([0, 0, 0, 0, 0, 0], 30)
    time.sleep(3.0)  # 等待归零动作完成

    # 读取归零后位置（带重试）
    fol_pos = mc.get_angles()
    for retry in range(5):
        if isinstance(fol_pos, list) and len(fol_pos) >= 6:
            break
        time.sleep(0.5)
        fol_pos = mc.get_angles()
    if not isinstance(fol_pos, list) or len(fol_pos) < 6:
        print(f"  ⚠ get_angles 返回异常: {fol_pos}, 使用默认 [0,0,0,0,0,0]")
        fol_pos = [0.0] * 6
    print(f"    当前位置: {[f'{a:.1f}' for a in fol_pos]}")

    # 基准 = 双方都是 0°
    leader_ref = {name: 0.0 for name, *_ in JOINT_CONFIG}
    follower_ref = [0.0, 0.0, 0.0, 0.0, 0.0, fol_pos[5]]  # J6 保持当前位置

    print(f"\n  ✅ 基准: Leader=0°, Follower=0° (J6={follower_ref[5]:.1f}° hold)")
    print(f"[4/4] 开始遥操! Ctrl+C 退出\n")

    # ── 遥操循环（双线程 + 队列 + 固定读取频率）─────────────
    frame = 0
    t_read_acc = 0.0
    t_calc_acc = 0.0
    send_count = [0]
    stat_t0 = time.perf_counter()
    stat_n = 0
    fps_target = args.fps
    frame_period = 1.0 / fps_target

    # 命令队列：主线程 60Hz 写入，发送线程按序消费
    QUEUE_MAX = 10000
    _target_queue = collections.deque(maxlen=QUEUE_MAX)
    _queue_lock = threading.Lock()
    _latest_ldr = {}
    _send_latency = [0.0]
    _queue_latency = [0.0]

    def _sender():
        """发送线程：按 FIFO 顺序消费队列 → send_angles（非阻塞）"""
        while _running:
            with _queue_lock:
                if _target_queue:
                    target, read_time = _target_queue.popleft()
                else:
                    target, read_time = None, None

            if target is None:
                time.sleep(0.001)  # 队列空，稍等
                continue

            send_start = time.perf_counter()
            _queue_latency[0] = send_start - read_time
            mc.send_angles(target, args.speed, _async=True)
            _send_latency[0] = time.perf_counter() - send_start
            send_count[0] += 1

    sender = threading.Thread(target=_sender, daemon=True)
    sender.start()

    try:
        while _running:
            loop_start = time.perf_counter()

            # ── 读 leader ──
            t1 = time.perf_counter()
            ldr = leader.bus.sync_read("Present_Position")
            read_time = t1
            t_read = time.perf_counter() - t1

            # ── 归一化计算目标角度 ──
            t2 = time.perf_counter()
            target = [0.0] * 6
            target[5] = follower_ref[5]  # J6 不动
            for name, ele_id, ldr_p, ldr_n, fol_p, fol_n, _dir in JOINT_CONFIG:
                ldr_delta = ldr.get(name, 0.0) - leader_ref[name]
                if ldr_delta >= 0:
                    ldr_norm = ldr_delta / ldr_p
                    fol_delta = ldr_norm * fol_p
                else:
                    ldr_norm = ldr_delta / ldr_n
                    fol_delta = ldr_norm * fol_n
                target[ele_id - 1] = follower_ref[ele_id - 1] + fol_delta * _dir
            t_calc = time.perf_counter() - t2

            # ── 入队 ──
            with _queue_lock:
                _target_queue.append((target, read_time))
                _latest_ldr = dict(ldr)
                qsize = len(_target_queue)

            frame += 1
            stat_n += 1
            t_read_acc += t_read
            t_calc_acc += t_calc

            # ── 固定读取频率 ──
            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(frame_period - dt_s, 0.0))

            # ── 每秒输出详细延时 ──
            now = time.perf_counter()
            if now - stat_t0 >= 1.0:
                elapsed = now - stat_t0
                read_fps = stat_n / elapsed
                t_read_avg = t_read_acc / stat_n * 1000
                t_calc_avg = t_calc_acc / stat_n * 1000
                send_lat = _send_latency[0] * 1000
                queue_lat = _queue_latency[0] * 1000
                send_n = send_count[0]

                print(f"\r  read_FPS:{read_fps:6.1f} (target:{fps_target}) | "
                      f"read:{t_read_avg:5.1f}ms "
                      f"calc:{t_calc_avg:5.2f}ms | "
                      f"queue:{queue_lat:5.0f}ms send:{send_lat:5.0f}ms ({send_n}/s) "
                      f"backlog:{qsize} | ", end="")
                for name, ele_id, *_ in JOINT_CONFIG:
                    ldr_val = _latest_ldr.get(name, 0.0)
                    fol_tgt = target[ele_id - 1]
                    print(f"{name[-5:]}:{ldr_val:+5.1f}→{fol_tgt:+6.1f} ", end="")
                print()

                stat_t0 = now
                stat_n = 0
                t_read_acc = 0.0
                t_calc_acc = 0.0
                send_count[0] = 0

    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        sender.join(timeout=3.0)
        print("\n断开...")
        mc = None
        leader.disconnect()
        print("退出.")


if __name__ == "__main__":
    main()

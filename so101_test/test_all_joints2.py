#!/usr/bin/env python

"""
MyCobotLeader → MyCobotFollower 全关节遥操测试（5 个臂关节）。

与 test_all_joints.py 的区别:
  - Leader 换成 MyCobotLeader (Feetech STS3215, 360° 自由旋转)
  - Follower 用 MyCobotFollower 类 (连接/上电), 但发命令走 pymycobot
  - 不需要归零, 也不需要主臂校准: 启动时把两臂摆成完全相同姿态,
    按 Enter 记录基准, 之后纯 delta 跟随 (主臂 360° 角度自动解回绕)

主臂读的是原始编码器步数 (normalize=False), 自己换算度数 —
没有校准文件也能跑 (绝对零位无关, 只用 delta)。

主臂范围远超从臂: 从臂目标 clamp 到自身机械极限 (follower_max)。

用法:
    python so101_test/test_all_joints2.py --leader-port /dev/ttyACM0 --mycobot-port /dev/ttyUSB0

注意:
    MyCobotFollowerConfig 里 leader_max_pos/neg 是主臂实际最大行程
    (主臂与从臂同构, 一般等于 follower_max, 如不同请在 config 里改)
"""

import argparse
import collections
import signal
import sys
import threading
import time

from lerobot.robots.mycobot_follower import MyCobotFollower, MyCobotFollowerConfig
from lerobot.robots.mycobot_follower.mycobot_follower import JOINT_MAP
from lerobot.robots.mycobot_leader import MyCobotLeader, MyCobotLeaderConfig
from lerobot.utils.robot_utils import precise_sleep

# 12-bit 编码器分辨率
ENCODER_RES = 4095  # 与 DEGREES 归一化一致: max_res = 4096 - 1
HALF_TURN_STEPS = 2048  # 回绕判断阈值 (半圈)


# 关节映射表: (名称, 从臂电机ID(1索引), 主臂max+, 主臂max-, 从臂max+, 从臂max-, 方向)
# 数据全部来自 MyCobotFollowerConfig + JOINT_MAP, 单点维护
def build_joint_config(follower_cfg: MyCobotFollowerConfig):
    cfg = follower_cfg
    return [
        (name, ele_id, ldr_p, ldr_n, fol_p, fol_n, direction)
        for (name, ele_id), ldr_p, ldr_n, fol_p, fol_n, direction in zip(
            JOINT_MAP.items(),  # (name, ele_id) 有序
            cfg.leader_max_pos,
            cfg.leader_max_neg,
            cfg.follower_max_pos,
            cfg.follower_max_neg,
            cfg.joint_directions,
        )
    ]


_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False
    print("\n\n正在停止...")


def main():
    global _running
    parser = argparse.ArgumentParser(description="MyCobotLeader → MyCobotFollower 全关节遥操")
    parser.add_argument("--leader-port", required=True, help="主臂 (Feetech) 串口")
    parser.add_argument("--mycobot-port", required=True, help="从臂 (MyCobot 280) 串口")
    parser.add_argument("--speed", type=int, default=80, help="从臂速度 (1-100)")
    parser.add_argument("--fps", type=int, default=60, help="目标控制频率 (Hz)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)

    # ── Step 1: 连接 MyCobot Leader (无需校准) ──────────────────
    print(f"[1/4] 连接 MyCobot Leader ({args.leader_port}) ...")
    leader_cfg = MyCobotLeaderConfig(port=args.leader_port)
    leader = MyCobotLeader(leader_cfg)
    leader.connect(calibrate=False)  # connect 关扭矩, 主臂可自由拖动
    print(f"  ✅ MyCobot Leader 就绪 (扭矩已关, 读原始编码器, 无需校准)")

    # ── Step 2: 连接 MyCobot Follower ───────────────────────────
    print(f"[2/4] 连接 MyCobot Follower ({args.mycobot_port}) ...")
    follower_cfg = MyCobotFollowerConfig(port=args.mycobot_port)
    follower = MyCobotFollower(follower_cfg)
    follower.connect()  # 上电 + focus_all_servos + 编码器校验
    mc = follower._robot
    mc.set_fresh_mode(1)  # 刷新模式: 始终执行最新命令
    print(f"  ✅ MyCobot Follower 就绪 (fresh_mode=1)")

    # 关节映射表 (5 个臂关节)
    JOINT_CONFIG = build_joint_config(follower_cfg)

    # ── Step 3: 记录基准 (两臂相同姿态, 免校准) ────────────────
    print(f"[3/4] 基准采集 ...")
    print(f"  ⚠ 请把主臂和从臂摆成【完全相同的姿态】, 然后按 Enter")
    input()

    ldr_raw = leader.bus.sync_read("Present_Position", normalize=False)  # 原始步数
    for name, *_ in JOINT_CONFIG:
        val = ldr_raw.get(name, 0)
        print(f"    主臂 {name:<16} raw={val:5d}  ({val * 360.0 / ENCODER_RES - 180.0:+7.2f}°)")

    fol_pos = mc.get_angles()
    for retry in range(5):
        if isinstance(fol_pos, list) and len(fol_pos) >= 6:
            break
        time.sleep(0.5)
        fol_pos = mc.get_angles()
    if not isinstance(fol_pos, list) or len(fol_pos) < 6:
        print(f"  ⚠ get_angles 返回异常: {fol_pos}, 使用 [0,0,0,0,0,0]")
        fol_pos = [0.0] * 6
    print(f"    从臂角度: {[f'{a:.1f}' for a in fol_pos]}")

    follower_ref = fol_pos  # 6 关节, J6 保持当前位置
    leader_ref = {name: ldr_raw[name] for name, *_ in JOINT_CONFIG}
    prev_raw = dict(leader_ref)
    cont_step = dict(leader_ref)  # 连续步数 (解回绕)

    print(f"\n  ✅ 基准已记录. 开始遥操! Ctrl+C 退出\n")
    print(f"[4/4] 遥操中 ...")

    # ── 遥操循环 (双线程 + 队列 + 固定读取频率) ─────────────────
    frame = 0
    t_read_acc = 0.0
    t_calc_acc = 0.0
    send_count = [0]
    stat_t0 = time.perf_counter()
    stat_n = 0
    fps_target = args.fps
    frame_period = 1.0 / fps_target

    QUEUE_MAX = 10000
    _target_queue = collections.deque(maxlen=QUEUE_MAX)
    _queue_lock = threading.Lock()
    _latest_ldr = {}
    _send_latency = [0.0]
    _queue_latency = [0.0]

    def _sender():
        """发送线程: 按 FIFO 顺序消费队列 → send_angles (非阻塞)"""
        while _running:
            with _queue_lock:
                if _target_queue:
                    target, read_time = _target_queue.popleft()
                else:
                    target, read_time = None, None

            if target is None:
                time.sleep(0.001)  # 队列空, 稍等
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

            # ── 读 leader 原始步数 (无需校准) ──
            t1 = time.perf_counter()
            ldr = leader.bus.sync_read("Present_Position", normalize=False)
            read_time = t1
            t_read = time.perf_counter() - t1

            # ── 解回绕: 主臂 360° 自由转, 半圈处步数跳变 ──
            t2 = time.perf_counter()
            target = [0.0] * 6
            target[5] = follower_ref[5]  # J6 不动
            for name, ele_id, ldr_p, ldr_n, fol_p, fol_n, _dir in JOINT_CONFIG:
                cur = ldr.get(name, prev_raw[name])

                # 帧间移动 < 半圈假设: 跨回绕点时补回 4096 步
                delta = cur - prev_raw[name]
                if delta > HALF_TURN_STEPS:
                    delta -= 4096
                elif delta < -HALF_TURN_STEPS:
                    delta += 4096
                prev_raw[name] = cur
                cont_step[name] += delta

                # 相对基准的 leader 偏移 (步数 → 度数, 零位无关)
                ldr_delta_deg = (cont_step[name] - leader_ref[name]) * 360.0 / ENCODER_RES

                # → 从臂偏移 (非对称归一化)
                if ldr_delta_deg >= 0:
                    ldr_norm = ldr_delta_deg / ldr_p
                    fol_delta = ldr_norm * fol_p
                else:
                    ldr_norm = ldr_delta_deg / ldr_n
                    fol_delta = ldr_norm * fol_n

                # 主臂超范围: clamp 到从臂机械极限
                fol_delta = max(-fol_n, min(fol_p, fol_delta))

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
                    ldr_delta = (cont_step[name] - leader_ref[name]) * 360.0 / ENCODER_RES
                    fol_tgt = target[ele_id - 1]
                    print(f"{name[-5:]}:{ldr_delta:+5.1f}→{fol_tgt:+6.1f} ", end="")
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
        follower.disconnect()
        leader.disconnect()
        print("退出.")


if __name__ == "__main__":
    main()

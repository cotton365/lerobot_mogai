# LeRobot JAKA Follower Integration — Handoff

## Summary

Integrating a JAKA 6-axis industrial robot as a **follower** in LeRobot's teleoperation framework, controlled by an SO-101 leader arm. JAKA uses TCP/IP via the closed-source `jkrc` SDK (.so files). IP: `10.5.5.100`.

## What Works

- JAKA connection/login/power/servo enable all successful
- `servo_j(jog, 0, 1)` called every loop iteration, no error codes after warm-up
- SO-101 leader reads joints via USB (`/dev/ttyACM0`)
- 300-iteration warm-up (~5s) + dynamic zero capture prevents timing violations at transition
- Loop stable at 16.7ms (60 FPS) after warm-up

## Critical Issue

**JAKA joint 6 does not physically move** despite `servo_j` receiving varying target positions. `target_j6` changes (e.g. 5.4 rad) but `actual_j6` stays fixed (6.0 rad). `servo_j` returns 0 (success), JAKA silently ignores position changes.

User's original `jk.py` with tight-loop `servo_j(jog, 0, 1)` + gradually decrementing `jog[5]` WORKS — JAKA moves. Our code also uses step=1 but JAKA doesn't move. Suspected: warm-up "hold state" lock-in, or cadence mismatch.

## Key Finding: step parameter

- step>=5: JAKA ignores ALL position changes, holds initial position (actual_j6 never changes)
- step=1: JAKA SHOULD track (works in jk.py), but still not working in our setup

## Files

**New:** `src/lerobot/robots/jaka_follower/` (__init__.py, config_jaka_follower.py, jaka_follower.py ~188 lines)
**Modified:** `robots/utils.py`, `scripts/lerobot_teleoperate.py`, `scripts/lerobot_record.py`

## JAKA API

```
get_joint_position() -> (0, (j1..j6)) radians
servo_j(joints, 0, 1) -> (error_code, _), non-blocking, 1=step*8ms
```

## Current Mapping (test mode)

Only wrist_roll -> JAKA joint_6. Scale: SO-101 10deg -> JAKA 1deg (inverse).
Dynamic zero captured at warmup->live transition.

## Run Command

```
lerobot-teleoperate --robot.type=jaka_follower --robot.robot_ip=10.5.5.100 --robot.id=jaka --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.use_degrees=true --teleop.id=leader --fps=125
```

## Environment

Conda env: `lerobot` (`/home/zh/miniconda3/envs/lerobot`)
May need `pip install numpy` if import errors.

## Next Steps

1. Strip down: no warm-up, hard-coded gradually-changing jog (like jk.py: -0.005 rad/iter), step=1, verify JAKA moves
2. If works: add gradual ramp transition instead of abrupt switch
3. If doesn't work: investigate JAKA servo state after servo_move_enable

## Suggested Skills

- `diagnosing-bugs` — JAKA not moving despite valid commands

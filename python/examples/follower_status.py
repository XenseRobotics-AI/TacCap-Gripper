#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""读从爪的状态:一次性读、开流读、以及状态里每个字段的意思。

从爪**只流电机状态**这一种数据。固件在这个角色上把 IMU / 编码器 / 电子皮肤的
流编译掉了,所以 `start_streaming()` 只有 `motor_hz` 一个参数。

两种读法,各有各的用途:

    g.motor.read_status()        一次命令往返,想什么时候读就什么时候读
    g.motor.on_status(cb)        注册回调,按固件的节奏推过来

**控制期间不要用 read_status()。** 命令的 ACK 会和控制帧在串口上撞车;控制器
自己持有状态流,`snapshot()` 拿到的就是最新一帧,不需要额外轮询 —— 见
control_and_read.py。

一个容易踩的坑:**状态可能是旧的而一切看着正常。** 固件曾经因为向电机请求了
主动上报(而这台电机不支持)就停掉了自己的轮询,于是空闲时状态帧永久冻结,
版本号、数据流、计数全都正常,唯一症状是读到的数悄悄是旧的。所以这个脚本会
打印 `status_timestamp_ms` 的推进量:它必须跟着墙钟走。

用法
    python python/examples/follower_status.py right
    python python/examples/follower_status.py --seconds 5 --hz 100
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import MotorStopReason, log


def describe_status(s) -> str:
    """状态位 + 停止原因,翻成人能读的。"""
    try:
        reason = str(MotorStopReason(s.stop_reason)).split(".")[-1]
    except ValueError:
        reason = f"0x{s.stop_reason:02X}"
    return f"status=0x{s.status:04X} stop={reason}"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument("--seconds", type=float, default=3.0, help="开流读多久")
    ap.add_argument("--hz", type=int, default=100, help="电机状态流频率(固件上限 100)")
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # ── 一次性读 ──────────────────────────────────────────────────────────────
    s = g.motor.read_status()
    print("\n[一次性读] g.motor.read_status()")
    print(f"  位置    {s.actual_pos:+.5f} rad     (电机轴原始角,零点由上电标定确定)")
    print(f"  速度    {s.actual_vel:+.5f} rad/s")
    print(f"  力矩    {s.actual_torque:+.4f} Nm    (反馈值,不是命令值)")
    print(f"  温度    {s.motor_temp_c:.1f} °C")
    print(f"  {describe_status(s)}")

    # 归一化开度要标定过才有意义:0=闭合 1=张开,由 gripper_config 的行程定义。
    cfg = g.get_gripper_config()
    print(f"\n[归一化] g.position() = {g.position():.4f}")
    print(
        f"  行程 min_open={cfg.min_open_rad:.5f}  max_open={cfg.max_open_rad:.5f} rad"
    )

    # ── 故障 ──────────────────────────────────────────────────────────────────
    fr = g.motor.fault_report()
    print(
        f"\n[故障] 电机 0x{fr.motor_fault_code:08X}  锁存 0x{fr.motor_latched_fault_code:08X}"
        f"  固件 0x{fr.firmware_fault_code:08X}"
    )
    print("  锁存位要显式清除才会消失 —— 当前为 0 不代表从没出过故障。")

    # ── 开流读 ────────────────────────────────────────────────────────────────
    frames: list = []
    sub = g.motor.on_status(lambda x: frames.append((time.perf_counter(), x)))
    # 固件侧最后一帧状态的 tick。新鲜度必须用它,不能用 sample.host_time ——
    # 后者是**主机收到的时刻**,只要流还在推就一直在涨,哪怕固件送的是同一份
    # 旧数据。这正是当初状态冻结 45 秒却毫无症状的那个洞。
    fw_ms_before = g.motor.fault_report().status_timestamp_ms
    t0 = time.perf_counter()
    g.start_streaming(motor_hz=args.hz)
    try:
        time.sleep(args.seconds)
    finally:
        g.stop_streaming()
        g.motor.off(sub)
    elapsed = time.perf_counter() - t0

    print(f"\n[开流读] g.start_streaming(motor_hz={args.hz}) + on_status()")
    if not frames:
        print("  **一帧都没收到** —— 流没起来,或者回调注册晚于开流。")
        return 1
    print(f"  {len(frames)} 帧 / {elapsed:.2f} s = {len(frames) / elapsed:.1f} Hz")

    # 新鲜度:固件的状态时间戳必须跟着墙钟推进。冻住 = 上面说的那个坑。
    fw_ms_after = g.motor.fault_report().status_timestamp_ms
    advance = fw_ms_after - fw_ms_before
    wall_ms = elapsed * 1000.0
    fresh = advance > wall_ms * 0.5
    print(
        f"  固件状态时间戳推进 {advance} ms,墙钟 {wall_ms:.0f} ms"
        f"  {'OK' if fresh else '**状态是旧的 —— 流在推,内容没更新**'}"
    )

    lo = min(f[1].actual_pos for f in frames)
    hi = max(f[1].actual_pos for f in frames)
    print(f"  位置区间 [{lo:+.5f}, {hi:+.5f}] rad  (静止时这两个数应当几乎相等)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

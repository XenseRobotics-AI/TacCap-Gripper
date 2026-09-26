#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""边控制边读状态:控制器跑着的时候,状态从哪里拿。

**控制期间不要调 `motor.read_status()`。** 那是一次命令往返,它的 ACK 会和控制帧
在同一条串口上撞车。控制器已经持有电机状态流,所以状态是白拿的:

    s = c.snapshot()          # 非阻塞,不发任何命令
    s.observation.position    # 归一化开度 [0,1]
    s.observation.torque      # 反馈力矩 Nm
    s.state / s.arrived / s.holding

`snapshot()` 在**一把锁下**取出整份视图,所以 state、observation、命令力矩描述的
是同一时刻。分字段去读会拼出一个从没存在过的状态 —— 位置来自这一帧、状态来自
下一帧。

`observation.age_ms` 是这份观测的年龄。它是判断「流还活着」的正确依据:帧还在推
不等于内容在更新(见 follower_status.py 里状态冻结那个坑)。

控制回路和读回路**频率不同是常态**:控制按状态流相位走 100 Hz,而策略/日志往往
只要 10~30 Hz。两边不需要对齐,snapshot() 随便什么时候调都给最新的一致视图。

用法
    python python/examples/control_and_read.py right
    python python/examples/control_and_read.py --read-hz 20 --seconds 8
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import (
    ForcePositionConfig,
    ForcePositionController,
    ForcePositionState,
    log,
)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument("--read-hz", type=float, default=10.0, help="读状态的频率")
    ap.add_argument("--seconds", type=float, default=8.0, help="总时长")
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=None,
        help="力矩预算 Nm。默认取电机的连续堵转额定(EL05 1.1 / RS00 3.6),不能超过它",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # 按设备实际装的电机取默认值 —— EL05 和 RS00 的持续夹持差 3.3 倍
    cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
    if args.grasp_torque is not None:
        cfg.grasp_torque_nm = args.grasp_torque
    c = ForcePositionController(g, cfg)
    c.start()

    # 控制侧:每 2 秒换一个目标。读侧:按 --read-hz 独立采样,不和控制对齐。
    targets = [1.0, 0.0, 1.0, 0.0]
    period = args.seconds / len(targets)
    interval = 1.0 / args.read_hz

    print(
        f"\n控制 {args.seconds:.0f}s,每 {period:.1f}s 换一次目标;"
        f"读侧独立跑 {args.read_hz:g} Hz\n"
    )
    print(
        f"  {'t':>5} {'目标':>5} {'位置':>7} {'力矩':>8} {'命令':>7} "
        f"{'年龄':>6} {'状态':>17} {'到位':>5}"
    )

    stale = 0
    t0 = time.perf_counter()
    try:
        for i, tgt in enumerate(targets):
            c.set_target(tgt)
            deadline = t0 + (i + 1) * period
            while time.perf_counter() < deadline:
                s = c.snapshot()  # 非阻塞,一致视图
                o = s.observation
                if s.state == ForcePositionState.FAULT:
                    print(f"  FAULT: {s.fault_reason}")
                    return 1
                # 观测太旧说明流断了,而不是爪子不动 —— 两者的处理完全不同。
                if o.valid and o.age_ms > 100:
                    stale += 1
                print(
                    f"  {time.perf_counter() - t0:5.1f} {tgt:5.2f} {o.position:7.4f} "
                    f"{o.torque:+8.4f} {s.commanded_torque_nm:7.4f} "
                    f"{o.age_ms:5.0f}ms {str(s.state).split('.')[-1]:>17} "
                    f"{str(s.arrived):>5}"
                )
                time.sleep(interval)
    finally:
        c.stop()

    print(f"\n观测过期(age>100ms)次数: {stale}")
    if stale:
        print("  **流有断续** —— 控制器还在发帧,但它看到的状态是旧的。")
        return 1
    print("  状态全程新鲜:控制和读互不干扰,读侧不需要发任何命令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

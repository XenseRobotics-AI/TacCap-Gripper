#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""量行进段的速度纹波 —— 控制器的**质量**,不是能不能到位。

impedance_control.py / force_position_control.py 验的是「到位」和「到位与被挡住
可分」,那是功能。功能对了控制仍然可以很难看:爪子一边走一边抖,平均速度只有命令
的七成。那正是重构前的实测状态。

指标和 `docs/CONTROL_REFACTOR.md` §1.2 的基线表一致,好直接对比:

    相对速度纹波 = σ(|v|) / 均值(|v|)     行进段,掐掉加减速两头
    峰峰         = max(|v|) - min(|v|)
    均速/命令    = 均值(|v|) / close_speed_radps
    极限环频率   = 去均值后的过零次数 / 2 / 时长
    逆向帧       = 速度方向与行进方向相反的帧数(区分**脉动**与**振动**)

最后一条是当初用来定性的:旧设计逆向帧 0/300 —— 爪子从不倒退,所以那是速度脉动
不是机械振动,再加上「纹波跟 kd 走、不跟速度走」的四组对照,排除了机械共振。

采样走状态流回调,满 100Hz,不额外发任何命令 —— 行进期间轮询会和控制帧撞车。

用法
    python python/examples/control_ripple.py right
    python python/examples/control_ripple.py --rounds 10 --controller both
"""

from __future__ import annotations

import argparse
import statistics as st
import time

import _calib_flow

from xense.taccap import (
    ForcePositionConfig, ForcePositionController,
    ImpedanceConfig, ImpedanceController, log,
)


def summarize(seg, commanded: float) -> dict:
    """seg: [(t, pos, vel, torque)],已经是行进段。"""
    n = len(seg)
    # 掐掉两头各 20%:加速段和减速段不是稳态,算进去等于把控制器的正常行为
    # 记成纹波。
    core = seg[int(n * 0.2):int(n * 0.8)]
    v = [abs(r[2]) for r in core]
    mu, sd = st.mean(v), st.pstdev(v)
    dur = core[-1][0] - core[0][0]

    # 极限环频率:去均值后数过零。比 FFT 简单,而且在这种低频单峰的情况下
    # 给出的是同一个数。
    dev = [x - mu for x in v]
    crossings = sum(1 for a, b in zip(dev, dev[1:]) if a * b < 0)
    freq = crossings / 2.0 / dur if dur > 0 else 0.0

    # 逆向帧:整段的行进方向由位移决定,再数有多少帧的速度和它反号。
    direction = 1.0 if core[-1][1] > core[0][1] else -1.0
    reversals = sum(1 for r in core if r[2] * direction < -0.02)

    return {
        "n": len(core), "mean": mu, "sd": sd,
        "ripple": sd / mu * 100 if mu else float("nan"),
        "pp": max(v) - min(v), "freq": freq,
        "ratio": mu / commanded * 100 if commanded else float("nan"),
        "reversals": reversals,
    }


def run(name, make_ctrl, commanded, gripper, rounds: int) -> int:
    rows: list = []
    sub = gripper.motor.on_status(
        lambda s: rows.append((time.perf_counter(), s.actual_pos,
                               s.actual_vel, s.actual_torque)))
    c = make_ctrl()
    c.start()

    def settle(budget=8.0):
        # 先等 arrived 变假(命令被采纳)再等变真 —— set_target() 不会立刻清掉
        # 上一次运动留下的 arrived,直接等真会在爪子还没动时就返回。
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 1.2 and _arrived(c):
            time.sleep(0.02)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < budget and not _arrived(c):
            time.sleep(0.02)

    def _arrived(ctrl):
        s = ctrl.snapshot()
        a = getattr(s, "arrived", None)
        if a is not None:
            return a
        # 阻抗控制器没有 arrived,用「速度落下来」代替。
        return s.observation.valid and abs(s.observation.velocity) < 0.02

    per_dir: dict[str, list] = {"闭合": [], "张开": []}
    try:
        c.set_target(1.0); settle(); time.sleep(0.4)
        for _ in range(rounds):
            for label, tgt in (("闭合", 0.0), ("张开", 1.0)):
                rows.clear()
                c.set_target(tgt)
                settle()
                time.sleep(0.1)
                seg = [r for r in rows if abs(r[2]) > 0.05]
                if len(seg) >= 30:
                    per_dir[label].append(summarize(seg, commanded))
                time.sleep(0.2)
    finally:
        c.stop()
        gripper.motor.off(sub)

    print(f"\n=== {name} ===  命令速度 {commanded:.2f} rad/s,{rounds} 轮")
    print(f"  {'方向':>4} {'轮':>3} {'相对纹波':>18} {'峰峰':>16} "
          f"{'极限环':>8} {'均速/命令':>10} {'逆向帧':>8}")
    bad = 0
    for label, xs in per_dir.items():
        if not xs:
            print(f"  {label:>4}  —— 没有有效样本")
            bad += 1
            continue
        rip = [x["ripple"] for x in xs]
        pp = [x["pp"] for x in xs]
        rev = sum(x["reversals"] for x in xs)
        tot = sum(x["n"] for x in xs)
        print(f"  {label:>4} {len(xs):>3} "
              f"{st.median(rip):>6.1f}% [{min(rip):.1f}-{max(rip):.1f}] "
              f"{st.median(pp):>8.3f} [{min(pp):.3f}-{max(pp):.3f}] "
              f"{st.median([x['freq'] for x in xs]):>6.1f}Hz "
              f"{st.median([x['ratio'] for x in xs]):>8.0f}% "
              f"{rev:>4}/{tot}")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    _calib_flow.add_target_argument(ap)
    ap.add_argument("--rounds", type=int, default=10, help="每个方向测多少轮")
    ap.add_argument("--controller", choices=("force-position", "impedance", "both"),
                    default="both")
    ap.add_argument("--close-speed", type=float, default=0.5, help="行进速度 rad/s")
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _calib_flow.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")
    env = g.get_envelope()
    print(f"[envelope] {env}")

    bad = 0
    if args.controller in ("force-position", "both"):
        def mk_fp():
            cfg = ForcePositionConfig()
            cfg.close_speed_radps = args.close_speed
            return ForcePositionController(g, cfg)
        bad += run("ForcePositionController", mk_fp, args.close_speed, g, args.rounds)

    if args.controller in ("impedance", "both"):
        def mk_imp():
            cfg = ImpedanceConfig()
            return ImpedanceController(g, cfg)
        # 阻抗没有「命令速度」这个量:接近速度由 peak/kd 决定。用实测均速自比,
        # 所以 ratio 一列对它没有意义。
        bad += run("ImpedanceController", mk_imp, args.close_speed, g, args.rounds)

    print("\n基线(docs/CONTROL_REFACTOR.md §1.2,重构前,10 轮):")
    print("    闭合 纹波 37%  峰峰 0.59  12-13Hz  均速/命令 77%")
    print("    张开 纹波 24%  峰峰 0.51  20Hz     均速/命令 94%")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())

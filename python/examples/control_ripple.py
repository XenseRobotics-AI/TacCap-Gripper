#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Measure velocity ripple over the travel segment -- the controller's
**quality**, not whether it arrives.

impedance_control.py / force_position_control.py check "arrives" and
"arrived vs. blocked are distinguishable"; that is function. With function
correct, control can still look bad: the gripper shakes as it travels and the
mean speed is only 70% of the command. That was exactly the measured state
before the refactor.

The metrics match the pre-refactor baseline table, for direct comparison:

    relative ripple  = σ(|v|) / mean(|v|)     travel segment, ramp up/down trimmed
    peak-to-peak     = max(|v|) - min(|v|)
    mean/command     = mean(|v|) / commanded speed
                       ForcePosition: the config's actual close_speed_radps
                       Impedance: design approach speed budget/kd (it has no ramp speed)
    limit-cycle freq = zero crossings after mean removal / 2 / duration
    reversal frames  = frames whose velocity opposes the travel direction
                       (tells **pulsation** from **vibration**)

The last one is what was used to characterize it back then: the old design had
0/300 reversal frames -- the gripper never moved backward, so it was velocity
pulsation, not mechanical vibration; together with four control comparisons
showing "ripple follows kd, not speed", that ruled out mechanical resonance.

Sampling uses the status stream callback, full 100Hz, sending no extra
commands -- polling during travel collides with control frames.

Usage
    python python/examples/control_ripple.py right
    python python/examples/control_ripple.py --rounds 10 --controller both
"""

from __future__ import annotations

import argparse
import statistics as st
import time

import _target
from xense.taccap import (
    ForcePositionConfig,
    ForcePositionController,
    ImpedanceConfig,
    ImpedanceController,
    log,
)


def summarize(seg, commanded: float) -> dict:
    """seg: [(t, pos, vel, torque)], already the travel segment."""
    n = len(seg)
    # Trim 20% off each end: the acceleration and deceleration phases are not
    # steady state; including them would count normal controller behavior as
    # ripple.
    core = seg[int(n * 0.2) : int(n * 0.8)]
    v = [abs(r[2]) for r in core]
    mu, sd = st.mean(v), st.pstdev(v)
    dur = core[-1][0] - core[0][0]

    # Limit-cycle frequency: count zero crossings after mean removal. Simpler
    # than an FFT, and in this low-frequency single-peak case it gives the same
    # number.
    dev = [x - mu for x in v]
    crossings = sum(1 for a, b in zip(dev, dev[1:]) if a * b < 0)
    freq = crossings / 2.0 / dur if dur > 0 else 0.0

    # Reversal frames: the segment's travel direction comes from displacement,
    # then count frames whose velocity has the opposite sign.
    direction = 1.0 if core[-1][1] > core[0][1] else -1.0
    reversals = sum(1 for r in core if r[2] * direction < -0.02)

    return {
        "n": len(core),
        "mean": mu,
        "sd": sd,
        "ripple": sd / mu * 100 if mu else float("nan"),
        "pp": max(v) - min(v),
        "freq": freq,
        "ratio": mu / commanded * 100 if commanded else float("nan"),
        "reversals": reversals,
    }


# "Stopped" criterion: raw position range within this window stays within
# this much.
STILL_WINDOW_S = 0.3
STILL_RANGE_RAD = 0.0015


def run(name, make_ctrl, commanded, gripper, rounds: int) -> int:
    rows: list = []
    sub = gripper.motor.on_status(
        lambda s: rows.append(
            (time.perf_counter(), s.actual_pos, s.actual_vel, s.actual_torque)
        )
    )
    c = make_ctrl()
    c.start()

    def settle(budget=8.0):
        # First wait for arrived to go false (command taken), then true --
        # set_target() does not immediately clear the arrived left by the
        # previous move; waiting for true directly would return before the
        # gripper has moved.
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
        # The impedance controller has no arrived; use "position stopped
        # moving" instead, not velocity: MIT status frame velocity has about
        # ±0.07 rad/s quantization noise when the motor is stopped, so
        # |v| < 0.02 almost never holds and every step would wait out the full
        # budget (found by DVT testing on 0086s). One position count is about
        # 0.00038 rad; a range <= 0.0015 rad within 0.3 s is about 4 counts.
        now = time.perf_counter()
        win = [r[1] for r in rows[-60:] if now - r[0] <= STILL_WINDOW_S]
        if len(win) < 20:
            return False
        return max(win) - min(win) <= STILL_RANGE_RAD

    per_dir: dict[str, list] = {"close": [], "open": []}
    try:
        c.set_target(1.0)
        settle()
        time.sleep(0.4)
        for _ in range(rounds):
            for label, tgt in (("close", 0.0), ("open", 1.0)):
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

    print(f"\n=== {name} ===  commanded speed {commanded:.2f} rad/s, {rounds} rounds")
    print(
        f"  {'dir':>4} {'n':>3} {'rel ripple':>18} {'pk-pk':>16} "
        f"{'cycle':>8} {'mean/cmd':>10} {'reversal':>8}"
    )
    bad = 0
    for label, xs in per_dir.items():
        if not xs:
            print(f"  {label:>4}  -- no valid samples")
            bad += 1
            continue
        rip = [x["ripple"] for x in xs]
        pp = [x["pp"] for x in xs]
        rev = sum(x["reversals"] for x in xs)
        tot = sum(x["n"] for x in xs)
        print(
            f"  {label:>4} {len(xs):>3} "
            f"{st.median(rip):>6.1f}% [{min(rip):.1f}-{max(rip):.1f}] "
            f"{st.median(pp):>8.3f} [{min(pp):.3f}-{max(pp):.3f}] "
            f"{st.median([x['freq'] for x in xs]):>6.1f}Hz "
            f"{st.median([x['ratio'] for x in xs]):>8.0f}% "
            f"{rev:>4}/{tot}"
        )
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--rounds", type=int, default=10, help="rounds to measure per direction"
    )
    ap.add_argument(
        "--controller", choices=("force-position", "impedance", "both"), default="both"
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=None,
        help="ForcePosition ramp speed rad/s, default 1.1 from for_spec; "
        "impedance does not use it",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective "
        f"{a.effective if a.effective else '*** firmware enforces nothing ***'}"
    )
    if not a.ok:
        print(f"[warn] {a.detail}")

    bad = 0
    if args.controller in ("force-position", "both"):
        fp_cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
        if args.close_speed is not None:
            fp_cfg.close_speed_radps = args.close_speed

        def mk_fp():
            return ForcePositionController(g, fp_cfg)

        # The denominator is the ramp speed actually in effect in the config; without
        # --close-speed that is the for_spec value.
        bad += run(
            "ForcePositionController", mk_fp, fp_cfg.close_speed_radps, g, args.rounds
        )

    if args.controller in ("impedance", "both"):
        imp_cfg = ImpedanceConfig.for_spec(g.motor.get_spec())

        def mk_imp():
            return ImpedanceController(g, imp_cfg)

        # Impedance has no "commanded speed": position error is clamped within
        # budget/kp, drive torque is capped at the budget, and the gripper
        # accelerates until damping kd·v balances it -- the design approach
        # speed is budget/kd. Only with that as the denominator is
        # "mean/command" meaningful.
        #
        # --close-speed was once used as the denominator; that is the
        # ForcePosition ramp speed, which impedance does not use at all: RS00
        # reported 187% because of it, which looked like overspeed but was
        # actually ~87% of the 1.1 rad/s design value.
        imp_speed = imp_cfg.max_position_torque_nm / imp_cfg.kd
        bad += run("ImpedanceController", mk_imp, imp_speed, g, args.rounds)

    print("\nbaseline (pre-refactor, 10 rounds):")
    print("    close ripple 37%  pk-pk 0.59  12-13Hz  mean/cmd 77%")
    print("    open  ripple 24%  pk-pk 0.51  20Hz     mean/cmd 94%")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())

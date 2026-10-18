#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ImpedanceController usage: supervised position-impedance control.

A background thread submits MIT frames phase-locked to the status stream; the
policy side only touches two non-blocking calls:

    c.set_target(p)     # p in [0,1], 0=closed 1=open
    s = c.snapshot()    # state + observation + both guard flags, one-lock consistent view

This script is also the **on-hardware acceptance test**: it runs a target
sequence and checks the invariant that separates "arrived" from "blocked" --

    When blocked short of the target, the correct behavior is the error clamp
    saturating at max_position_torque_nm and holding -- that is the grip
    force, not a fault. There is no stall state to check: contact does not
    need detecting, saturation itself is contact.

A guard still firing at the commanded position means the guard has mistaken
the normal tracking steady state for a stall. Any step failing -> non-zero
exit code.

Motion safety envelope
----------------------
When blocked, kp x position error has no upper bound; the firmware-side
envelope clamps it. The envelope is **disabled** by default; write it once
with --set-envelope (persists across power-off, once per device is enough).

Usage
    python python/examples/impedance_control.py --show-envelope
    python python/examples/impedance_control.py --set-envelope
    python python/examples/impedance_control.py right
    python python/examples/impedance_control.py --targets 1.0,0.5,0.0

Safety: real motion; the exit path always sends zero torque and disables.
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import (
    ImpedanceConfig,
    ImpedanceController,
    ImpedanceState,
    log,
)

# Normalized error counted as "arrived". Impedance tracking stops where
# kp x error balances friction, not at the mathematical zero, so this is a bit
# wider than the controller's internal criterion, to avoid reporting a passing
# step as failed.
ARRIVED = 0.03


def settle(c: ImpedanceController, budget: float = 3.0):
    """Wait for the controller to settle; return (state name, snapshot, elapsed).

    Waits for "velocity drops" rather than a fixed 2 s sleep: a fixed wait can
    both cut off a slow move and waste time on a fast one.
    """
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        s = c.snapshot()
        if s.state == ImpedanceState.FAULT:
            break
        if s.observation.valid and abs(s.observation.velocity) < 0.02:
            # Count it only when two frames in a row are stopped, so the instant of a
            # zero crossing is not taken as stopped.
            time.sleep(0.05)
            s2 = c.snapshot()
            if s2.observation.valid and abs(s2.observation.velocity) < 0.02:
                return s2.state.name, s2, time.perf_counter() - t0
        time.sleep(0.01)
    s = c.snapshot()
    return s.state.name, s, time.perf_counter() - t0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    # All defaults are None and derived from the device's motor spec -- a
    # hard-coded kd would override the value for_spec() computed, and kd is
    # exactly the term that keeps "grip tighter" from becoming "hit faster".
    ap.add_argument(
        "--kp", type=float, default=None, help="impedance stiffness Nm/rad (default 20)"
    )
    ap.add_argument(
        "--kd",
        type=float,
        default=None,
        help="impedance damping Nm·s/rad. **Also sets the approach speed** "
        "(≈ budget/kd). Default derived from the motor: budget / "
        "MOTOR_APPROACH_SPEED_RADPS, 1.00 on EL05 and 3.27 on RS00, 1.1 rad/s on both",
    )
    ap.add_argument(
        "--show-envelope",
        action="store_true",
        help="print the envelope (stored/effective) and exit",
    )
    ap.add_argument(
        "--set-envelope",
        action="store_true",
        help="fix the envelope to the motor rating the device reports, then continue "
        "(writes MCU flash; no write if already correct)",
    )
    ap.add_argument(
        "--targets",
        default="1.0,0.6,0.3,0.6,1.0",
        help="comma-separated normalized target sequence",
    )
    args = ap.parse_args()

    try:
        targets = [float(x) for x in args.targets.split(",") if x.strip()]
    except ValueError:
        ap.error(f"--targets failed to parse: {args.targets!r}")
    if not targets or any(not 0.0 <= t <= 1.0 for t in targets):
        ap.error("--targets must be normalized positions in [0,1]")

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # What goes in the envelope is not the caller's problem: the device knows
    # the rating of the motor it carries, and the firmware clamps to that
    # rating whatever is written. The SDK reads the numbers from the device
    # (0x56).
    #
    # stored and effective can differ: the firmware logs clamping cont down
    # only on a UART not wired to USB, while the readback is the record in
    # flash.
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective "
        f"{a.effective if a.effective else '*** firmware enforces nothing ***'}"
    )
    if not a.ok:
        print(f"[warn] {a.detail}")
    if args.set_envelope:
        w = g.ensure_envelope()
        print(
            f"[envelope] written {w.written}"
            if w.wrote
            else "[envelope] already correct, not written"
        )
    if args.show_envelope:
        return 0

    # Defaults come from the motor actually fitted: budget = continuous stall
    # rating, ceiling = rotating rating, kd computed from the budget so the
    # approach speed stays at MOTOR_APPROACH_SPEED_RADPS
    spec = g.motor.get_spec()
    cfg = ImpedanceConfig.for_spec(spec)
    if args.kp is not None:
        cfg.kp = args.kp
    if args.kd is not None:
        cfg.kd = args.kd
    print(
        f"[motor] {spec.name}  budget={cfg.max_position_torque_nm:.2f}Nm "
        f"ceiling={cfg.rated_torque_nm:.2f}Nm kp={cfg.kp:.1f} kd={cfg.kd:.3f} "
        f"-> approach {cfg.max_position_torque_nm / cfg.kd:.2f} rad/s"
    )

    c = ImpedanceController(g, cfg)
    failures = 0
    try:
        g.motor.clear_fault()
        c.start()  # initial target is the current position, no jump
        g.motor.enable()

        for target in targets:
            c.set_target(target)
            name, s, dt = settle(c)
            o = s.observation
            err = target - o.position
            # All from the status stream, no bus traffic during control --
            # ACK-based calls like read_status() collide with control frames,
            # snapshot() does not.
            print(
                f"  target={target:.2f} -> {name:14s} pos={o.position:.4f} "
                f"err={err:+.4f} tq={o.torque:+.3f} Nm cmd={s.commanded_torque_nm:.3f} "
                f"temp={o.motor_temp_c:.0f}°C took {dt:.2f}s"
            )

            if name == "FAULT":
                print(f"    FAIL  fault: {s.fault_reason}")
                failures += 1
                break

            budget = cfg.max_position_torque_nm
            if abs(err) <= ARRIVED:
                if s.torque_capped:
                    # Arrived yet hitting the ceiling means the ceiling took the
                    # normal tracking steady state for a stall.
                    print("    FAIL  arrived but hit the torque ceiling")
                    failures += 1
                else:
                    print("    ok    arrived")
            elif s.commanded_torque_nm >= budget - 0.05:
                # Stopped short of the target with commanded torque saturated
                # at the budget -- exactly what blocked should look like, and
                # it holds; that is the grip force.
                print(
                    f"    ok    blocked short of target by {abs(err):.4f}, command "
                    f"saturated at budget {s.commanded_torque_nm:.3f}/{budget:.3f} "
                    f"Nm and holding"
                )
            else:
                # Neither arrived nor saturated: either the gains are too soft to
                # push, or the budget is not being used at all.
                print(
                    f"    FAIL  stopped short of target by {abs(err):.4f}, but command "
                    f"is only {s.commanded_torque_nm:.3f} Nm, not saturated at "
                    f"budget {budget:.3f}"
                )
                failures += 1

            if o.torque > cfg.rated_torque_nm + 0.15:
                print(
                    f"    FAIL  measured torque {o.torque:.3f} Nm exceeded the ceiling "
                    f"{cfg.rated_torque_nm:.3f} Nm"
                )
                failures += 1

        final = c.snapshot()
        print(
            f"\n[guards] torque ceiling fired {final.torque_caps} times "
            f"(the error clamp is not a state; it is active every frame, no count)"
        )
    finally:
        try:
            c.stop()  # stop() sends zero torque first
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)

    if failures:
        print(f"\n{failures} step(s) failed")
        return 1
    print(f"\nall {len(targets)} steps passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ForcePositionController usage: force-position hybrid control.

**There is only one control law throughout**, with no contact detection:

    command(target, kp, kd, torque_budget = grasp_torque_nm)

The PD request is error-clamped to the budget: free travel pays only friction,
and when blocked it clamps at exactly the budget and holds there. Contact is not
detected — **saturation itself is contact**. SDK 0.2.0 removed the host-side
contact state machine: the MCU already runs the same stall criterion at 500Hz and
is authoritative, and running a second copy on the host is worse than running it
on either side alone, because two copies of the same physical event drift apart.

(This docstring once described the removed scheme "kp=0 close -> contact
detection -> kp=kd=0 pure tau_ff hold". The script kept running, but it taught a
model that does not exist — now the state it runs into is HOLDING_POSITION, and
it does not enter HoldingForce unless actually blocked.)

Travel speed is set by a **time-based ramp**, not "current position + error
limit": the latter keeps the position error permanently saturated, which is a
constant-force push rather than speed tracking, and measured it runs at twice the
target speed.

The closed endpoint additionally carries a **bounded feedforward preload**
(close_preload_nm, default 0.25 Nm) that seats the jaw firmly against the
mechanical stop. Without it, kp*(target-actual) goes to exactly zero at the
target, the jaw stops applying force once it arrives, and the backlash stays
there — measured holding torque at the closed position was only 0.001 Nm. The
preload is **reserved** out of the budget, not added on top, so the invariant
that the total request never exceeds grasp_torque_nm still holds.

Like ImpedanceController, the interface is just two non-blocking calls:

    c.set_target(p)      # p in [0,1], 0=closed 1=open
    s = c.snapshot()     # state + observation + commanded torque, consistent
                         # view under one lock

Note that snapshot().grasp_torque_nm is the **setpoint**; the firmware's I2t and
temperature wall bring the actual output down, so the grip force is not constant.

This script is also the **on-hardware acceptance test**: it runs a target
sequence that includes intermediate positions and checks the invariant —

    when holding is reported, the jaw must clearly **not have reached** the
    commanded position.

The closed endpoint (target 0.0) is the exception: the stop and the finger pads
themselves block the jaw, so reporting holding there means clamping at the
budget, which is a normal final state.

holding is an observation: the command has used up the torque budget and the jaw
still is not moving. Reporting holding at the commanded position means the budget
was used up by a move that should have arrived, which points to a calibration or
mapping problem.

Any failed step -> non-zero exit code.

Usage
    python python/examples/force_position_control.py right
    python python/examples/force_position_control.py --grasp-torque 1.1
    python python/examples/force_position_control.py --targets 1.0,0.7,0.35,0.0

Safety: real motion + grip force; the exit path always sends zero torque and
disables.
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import (
    ForcePositionConfig,
    ForcePositionController,
    log,
)

TERMINAL = ("HOLDING_FORCE", "HOLDING_POSITION", "FAULT")

# detail::ForcePositionTuning::arrival_eps_rad — the radius within which the
# controller considers the jaw "arrived". It is widened a bit here before use, so
# that measurement noise does not report a passing step as a failure.
ARRIVAL_BAND_RAD = 0.010
ARRIVAL_SLACK = 2.5
# Matches the SDK's kClosedEndpointEps: a normalized target no greater than this
# is the closed endpoint.
CLOSED_ENDPOINT = 1e-3


def await_command(
    c: ForcePositionController, target: float, budget: float = 1.0
) -> bool:
    """Wait until the controller has actually taken this target.

    set_target() is **queued**: the command is applied only on the next frame's
    phase of the status stream (so every write lands in the MCU's idle window).
    Reading snapshot() right after sending returns the final state of the
    **previous** target — so "has not started moving" is taken as "already
    converged", and the whole acceptance run passes with the jaw never moving.
    Do not remove this step.
    """
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        if abs(c.snapshot().target_position - target) <= 1e-4:
            return True
        time.sleep(0.005)
    return False


def settle(c: ForcePositionController, budget: float = 8.0):
    """Wait for the controller to settle in a final state; return (state name,
    snapshot, elapsed)."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        s = c.snapshot()
        name = s.state.name
        if name in TERMINAL:
            return name, s, time.perf_counter() - t0
        time.sleep(0.01)
    s = c.snapshot()
    return s.state.name, s, time.perf_counter() - t0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=None,
        help="torque budget Nm — not used during free travel; when blocked the "
        "jaw stops at this value. Defaults to the motor's continuous stall rating "
        "(EL05 1.1 / RS00 3.6) and cannot exceed it",
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=None,
        help="close speed rad/s. Default MOTOR_APPROACH_SPEED_RADPS (1.1)",
    )
    ap.add_argument(
        "--targets",
        default="1.0,0.5,0.0,0.5,1.0",
        help="comma-separated sequence of normalized targets. The default includes "
        "intermediate positions — running only the endpoints cannot reveal a "
        'wrong "arrived/contact" judgment',
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
    # audit rather than get_envelope: looking only at the Enforce flag is blind to
    # both "the stored value is not the effective value" and a layout mismatch,
    # and either one leaves the hold without thermal protection.
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective "
        f"{a.effective if a.effective else '*** firmware enforces nothing ***'}"
    )
    if not a.ok:
        print(
            f"[warn] {a.detail}\n"
            "       Thermal protection for long holds depends on this. "
            "Fix: impedance_control.py --set-envelope"
        )

    # Take defaults from the motor the device actually carries (grasp / hold /
    # motion / close_speed all follow it)
    cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
    if args.grasp_torque is not None:
        cfg.grasp_torque_nm = args.grasp_torque  # stops here when blocked = grip force
    if args.close_speed is not None:
        cfg.close_speed_radps = args.close_speed
    # The rest comes from for_spec: hold = rotating rating, motion = torque range
    # (EL05 1.8/6.0, RS00 5.0/14.0), status_timeout_ms=350 (stream loss -> FAULT +
    # zero torque), motor_stream_hz=100.
    print(
        f"[config] grasp={cfg.grasp_torque_nm:.2f}Nm close_speed={cfg.close_speed_radps:.2f}rad/s "
        f"hold={cfg.hold_torque_limit_nm:.2f}Nm motion={cfg.motion_torque_limit_nm:.2f}Nm"
    )

    c = ForcePositionController(g, cfg)
    failures = 0
    try:
        g.motor.clear_fault()
        c.start()  # checks the device's persisted 0x700B limit at startup, seeds a
        # position hold
        g.motor.enable()

        # Convert the arrival band to normalized units: the controller judges in
        # rad space, while positions here are normalized.
        travel = g.position_map().max_open_rad
        band = ARRIVAL_BAND_RAD * ARRIVAL_SLACK / travel
        print(
            f"[band] travel {travel:.4f} rad, arrived if |err| <= {band:.4f} (normalized)"
        )

        for target in targets:
            before = c.snapshot().state.name
            c.set_target(target)
            if not await_command(c, target):
                print(
                    f"  target={target:.2f} -> FAIL  controller did not take this target"
                )
                failures += 1
                continue
            name, s, dt = settle(c)
            pos = s.observation.position
            err = abs(pos - target)
            # Position/torque/temperature all come from the status stream's
            # snapshot; nothing touches the bus while controlling.
            print(
                f"  target={target:.2f} -> {name:16s} pos={pos:.4f} err={err:.4f} "
                f"cmd={s.commanded_torque_nm:.3f} meas={s.observation.torque:+.3f} Nm "
                f"temp={s.observation.motor_temp_c:.0f}°C took {dt:.2f}s"
            )

            if name == "FAULT":
                print(f"    FAIL  fault: {s.fault_reason}")
                failures += 1
                break

            # Once the grip is stable, a deeper close target is **deliberately**
            # swallowed: the jaw already cannot push further at this force, and
            # restarting would only shake loose a good grip. It counts as neither
            # pass nor fail — no motion happened in this step at all, so it must
            # not be used to judge "is the protection correct".
            if before == "HOLDING_FORCE" and name == "HOLDING_FORCE" and dt < 0.05:
                print(
                    "    skip  already holding force, deeper close target ignored "
                    "(by design); only an open target leaves it"
                )
                continue
            if name not in ("HOLDING_POSITION", "HOLDING_FORCE"):
                print("    FAIL  did not settle in a final state within budget")
                failures += 1
                continue

            # This is the core check: holding is valid only when the jaw is
            # blocked **short of** the commanded position. Reporting holding after
            # arriving means a move that should have arrived used up the torque
            # budget — a calibration or normalized-mapping problem.
            # The closed endpoint is the exception (since SDK 0.3.7): target 0.0
            # is the mechanical stop, and the stop and finger pads themselves
            # block the jaw — blocked at full close and clamped at the budget,
            # HOLDING_FORCE is a normal final state, just like arrival. Measured on
            # 0094s (RS00): the jaw touches the finger pads ~12mrad before the
            # stop, the controller fills up to the budget and reports
            # HOLDING_FORCE, and presses to the stop within 0.2 s (3.2Nm). The old
            # check failed this as "holding reported after arrival", which
            # contradicts the controller's design.
            if s.holding and target <= CLOSED_ENDPOINT:
                print(
                    f"    ok    closed end blocked by stop/finger pads, clamped at "
                    f"budget (err {err:.4f})"
                )
            elif s.holding and err <= band:
                print(
                    "    FAIL  holding reported at the commanded position — a move "
                    "that should have arrived used up the torque budget"
                )
                failures += 1
            elif s.holding:
                print(
                    f"    ok    blocked {err:.4f} short of target (normalized), "
                    "holding correct"
                )
            elif s.arrived:
                print("    ok    arrived")
            else:
                # Neither arrived nor holding, yet already settled — nothing
                # explains why it stopped here.
                print(
                    f"    FAIL  stopped {err:.4f} short of target, but neither arrived"
                    f" nor holding — did the motion actually happen?"
                )
                failures += 1
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
    print(f"\n{len(targets)} steps completed, no failures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

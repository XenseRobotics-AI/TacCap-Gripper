#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Read status while controlling: where to get state while a controller runs.

**Do not call `motor.read_status()` during control.** It is a command round
trip, and its ACK collides with control frames on the same serial port. The
controller already owns the motor status stream, so state comes for free:

    s = c.snapshot()          # non-blocking, sends no commands
    s.observation.position    # normalized opening [0,1]
    s.observation.torque      # feedback torque Nm
    s.state / s.arrived / s.holding

`snapshot()` takes the whole view **under one lock**, so state, observation
and commanded torque describe the same instant. Reading field by field stitches
together a state that never existed -- position from this frame, state from
the next.

`observation.age_ms` is the age of this observation. It is the correct basis
for judging "the stream is alive": frames still being pushed does not mean
the content is updating (see the status-freeze trap in follower_status.py).

The control loop and the read loop **normally run at different rates**:
control runs at 100 Hz phase-locked to the status stream, while policy/logging
often needs only 10~30 Hz. The two need no alignment; snapshot() called at any
time gives the latest consistent view.

Usage
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
    ap.add_argument("--read-hz", type=float, default=10.0, help="status read rate")
    ap.add_argument("--seconds", type=float, default=8.0, help="total duration")
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=None,
        help="torque budget Nm. Defaults to the motor's continuous stall rating "
        "(EL05 1.1 / RS00 3.6); must not exceed it",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # Defaults come from the motor actually fitted -- EL05 and RS00 sustained
    # grip differ by 3.3x
    cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
    if args.grasp_torque is not None:
        cfg.grasp_torque_nm = args.grasp_torque
    c = ForcePositionController(g, cfg)
    c.start()

    # Control side: a new target every 2 s. Read side: samples independently at
    # --read-hz, not aligned with control.
    targets = [1.0, 0.0, 1.0, 0.0]
    period = args.seconds / len(targets)
    interval = 1.0 / args.read_hz

    print(
        f"\ncontrol {args.seconds:.0f}s, new target every {period:.1f}s; "
        f"read side runs independently at {args.read_hz:g} Hz\n"
    )
    print(
        f"  {'t':>5} {'tgt':>5} {'pos':>7} {'torque':>8} {'cmd':>7} "
        f"{'age':>6} {'state':>17} {'arr':>5}"
    )

    stale = 0
    t0 = time.perf_counter()
    try:
        for i, tgt in enumerate(targets):
            c.set_target(tgt)
            deadline = t0 + (i + 1) * period
            while time.perf_counter() < deadline:
                s = c.snapshot()  # non-blocking, consistent view
                o = s.observation
                if s.state == ForcePositionState.FAULT:
                    print(f"  FAULT: {s.fault_reason}")
                    return 1
                # A too-old observation means the stream dropped, not that the gripper
                # is not moving -- the two are handled completely differently.
                if o.valid and o.age_ms > 100:
                    stale += 1
                print(
                    f"  {time.perf_counter() - t0:5.1f} {tgt:5.2f} {o.position:7.4f} "
                    f"{o.torque:+8.4f} {s.commanded_torque_nm:7.4f} "
                    f"{o.age_ms:5.0f}ms {s.state.name:>17} "
                    f"{str(s.arrived):>5}"
                )
                time.sleep(interval)
    finally:
        c.stop()

    print(f"\nstale observations (age>100ms): {stale}")
    if stale:
        print(
            "  **stream is intermittent** -- the controller is still sending frames, "
            "but the state it sees is stale."
        )
        return 1
    print(
        "  state fresh throughout: control and reads do not interfere, the read "
        "side needs to send no commands."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

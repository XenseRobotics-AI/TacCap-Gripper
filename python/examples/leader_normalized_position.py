#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Stream a leader gripper's opening as a normalized 0..1 value.

`normalize_position=True` makes the SDK read the encoder-max calibration
(Cmd::EncoderMaxCal 0x2C, firmware >= V2.1) at open() time and install the
resulting converter on the Encoder. Every sample — one-shot and streamed —
then carries `.position`:

    0.0 = fully closed   (the encoder zero)
    1.0 = fully open     (the calibrated max travel angle)

`.position_rad` keeps reporting raw radians either way; normalization adds a
field, it does not repurpose one. Without the flag, `.position` is `nan`.

Calibrate first if this raises "never calibrated":
    python python/examples/fisheye_cal.py measure-encoder-max

Usage:
    python python/examples/leader_normalized_position.py
    python python/examples/leader_normalized_position.py right --hz 50
    # Bypass the firmware read (e.g. pre-V2.1 firmware):
    python python/examples/leader_normalized_position.py --encoder-max-rad 1.30
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import _calib_flow
import _target
from xense.taccap import LeaderGripper, ProtocolError


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _target.add_target_argument(p)
    p.add_argument("--hz", type=int, default=100, help="encoder stream rate")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="stop after N seconds (0 = run until Ctrl-C)")
    p.add_argument("--encoder-max-rad", type=float, default=0.0,
                   help="use this travel span instead of reading it from the "
                        "firmware (rad)")
    args = p.parse_args()

    eps, _by_side, _all = _target.resolve_target(args.target)
    device = eps.mcu_device
    try:
        gripper = LeaderGripper(
            mcu_device=device,
            normalize_position=True,
            encoder_max_rad=args.encoder_max_rad,
        )
    except ProtocolError as e:
        # The SDK deliberately refuses rather than guessing — but this is an
        # interactive tool, so walk the user through fixing it instead of just
        # printing the error. (The library stays non-interactive: anything
        # headless keeps getting the exception.)
        print(f"\n{e}\n", file=sys.stderr)
        try:
            plain = LeaderGripper(mcu_device=device)
        except Exception:
            return 1
        with plain as g:
            try:
                stored = _calib_flow.offer_calibration(g)
            except (EOFError, KeyboardInterrupt):
                print("\naborted.", file=sys.stderr)
                return 1
        if not stored:
            return 1
        _calib_flow.restart_notice()
        return 0

    with gripper as g:
        span = g.position_map.max_open_rad
        print(f"normalization on: 0 = closed, 1 = open at {span:.4f} rad "
              f"({math.degrees(span):.1f}°)")

        # One-shot read, straight off the converter.
        print(f"position() now: {g.position():.3f}")

        g.encoder.on_data(_printer())
        g.start_streaming(imu_hz=0, encoder_hz=args.hz)
        print(f"streaming at {args.hz} Hz — Ctrl-C to stop\n")

        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        try:
            while deadline is None or time.monotonic() < deadline:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    print()
    return 0


def _printer():
    """Throttled one-line bar so a 100 Hz stream stays readable."""
    state = {"next": 0.0}

    def on_sample(s):
        now = time.monotonic()
        if now < state["next"]:
            return
        state["next"] = now + 0.05          # ~20 Hz of console output
        filled = int(round(s.position * 40))
        bar = "#" * filled + "-" * (40 - filled)
        sys.stdout.write(
            f"\r[{bar}] position={s.position:5.3f}  "
            f"({s.position_rad:6.3f} rad)"
        )
        sys.stdout.flush()

    return on_sample


if __name__ == "__main__":
    raise SystemExit(main())

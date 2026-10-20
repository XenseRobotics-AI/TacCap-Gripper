#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Read the wrist-camera intrinsics and print them as JSON. Read-only — no flash
writes, no motor motion.

Goes through `Calibration.resolve_fisheye()`, not `read_fisheye()`. The
difference matters: **an uncalibrated unit answers the read request with an
all-zero record, not a NACK**, and handing that to FisheyeUndistorter remaps
every frame to solid black. resolve_fisheye() keeps the "read → check usable →
fall back to the SDK reference values" policy in one place, and it is the same
decision the SDK makes internally.

The `source` field in the output is the result of that decision:

    "device"     intrinsics come from the calibration record flashed on this unit
    "reference"  the unit is not calibrated (or the record is unusable); the SDK's
                 shared reference values are used. The reference values are
                 approximate — lens assembly varies unit to unit, so think twice
                 before using them for precise measurement. --require-device
                 makes this case fail outright.

JSON goes to stdout, diagnostics go to the log (stderr), so it can be piped
directly:

    python python/examples/read_intrinsics.py left > left_cal.json
    python python/examples/read_intrinsics.py left | jq .K

Usage
    python python/examples/read_intrinsics.py                # omit when only one is plugged in
    python python/examples/read_intrinsics.py left
    python python/examples/read_intrinsics.py right --out right_cal.json
    python python/examples/read_intrinsics.py left --require-device
"""

from __future__ import annotations

import argparse
import json
import sys

import _target
from xense.taccap import FollowerGripper, LeaderGripper, log

# The calibration record **does not store the image size**. The SDK always uses these
# intrinsics at the 640x480 they were calibrated at; scaling them to another resolution
# is a guess — so the assumption is written explicitly into the output here, rather
# than pretending to know.
ASSUMED_IMAGE_SIZE = [640, 480]


def _enum_name(v) -> str:
    return str(v).split(".")[-1].lower()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--follower",
        action="store_true",
        help="open as FollowerGripper (default LeaderGripper). "
        "Both classes can read the intrinsics; leader is the default to get around "
        "the follower's firmware version gate",
    )
    ap.add_argument(
        "--out", metavar="FILE", help="write to a file; if omitted, print to stdout"
    )
    ap.add_argument(
        "--require-device",
        action="store_true",
        help="exit with an error if the intrinsics fall back to the SDK reference "
        "values, instead of using them silently",
    )
    ap.add_argument(
        "--indent", type=int, default=2, help="JSON indent, 0 = single line"
    )
    args = ap.parse_args()

    ep, _by_side, _all = _target.resolve_target(args.target)
    cls = FollowerGripper if args.follower else LeaderGripper
    g = cls(mcu_device=ep.mcu_device)

    try:
        cal, is_reference, reason = g.calibration.resolve_fisheye()

        if is_reference:
            log.warning(
                f"intrinsics fell back to the SDK reference values ({reason}) — this "
                "unit has no usable calibration record. The reference values are "
                "approximate; lens assembly varies unit to unit."
            )
            if args.require_device:
                sys.exit(
                    "error: --require-device requires the unit's own calibration, but it "
                    f"fell back to the reference values: {reason}"
                )

        doc = {
            "sn": ep.firmware_sn,
            "side": _enum_name(ep.side),
            "role": _enum_name(ep.role),
            "mcu_device": ep.mcu_device,
            "firmware": _target.firmware_version(g),
            "source": "reference" if is_reference else "device",
            "reason": reason,
            "model": "fisheye_equidistant",
            "assumed_image_size": ASSUMED_IMAGE_SIZE,
            "fx": cal.fx,
            "fy": cal.fy,
            "cx": cal.cx,
            "cy": cal.cy,
            "k1": cal.k1,
            "k2": cal.k2,
            "k3": cal.k3,
            "k4": cal.k4,
            "K": [[float(v) for v in row] for row in cal.K],
            "D": [float(v) for v in cal.D],
        }
    finally:
        g.transport.stop()

    text = json.dumps(doc, indent=args.indent or None, ensure_ascii=False)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        log.info(f"intrinsics written to {args.out} (source={doc['source']})")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

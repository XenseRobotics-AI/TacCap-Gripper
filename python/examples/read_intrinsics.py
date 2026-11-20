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
from xense.taccap import FollowerGripper, LeaderGripper, ProtocolError, log

# The intrinsics hold only at the resolution they were calibrated at — 640x480 and
# 1920x1080 are different sensor crops, so they cannot be rescaled between the two.
# `image_size` says which one, and `image_size_source` how the SDK knows:
#   "recorded"   the firmware stored it with the params (firmware 1.3.0-rc1+ / 1.3.1)
#   "inferred"   the record carries no size; inferred from the principal point
#   "reference"  the SDK reference values, which are a 640x480 calibration
# `assumed_image_size` is kept, with the same value, for consumers written before
# the size was recorded.
#
# `slots` reports one entry per capture resolution ("640x480", "1920x1080"), each
# from Calibration.resolve_fisheye_for(w, h) and shaped like the top level
# (source / reason / image_size / image_size_source / fx..k4 / K / D). Firmware
# 1.3.1 (or 1.3.0-rc4+) keeps a separate calibration per resolution
# (`dual_slot_firmware`: true); on older firmware the single record fills the
# entry for its own size and the other falls back to the reference. A
# "reference" entry for 1920x1080 is NOT usable on 1080p frames — the reference
# values are a 640x480 calibration. The top-level fields are unchanged: they are
# still resolve_fisheye(), i.e. the 640x480 record on dual-slot firmware.

SLOT_SIZES = ((640, 480), (1920, 1080))


def _enum_name(v) -> str:
    return str(v).split(".")[-1].lower()


def _intrinsics(resolved) -> dict:
    """The size + params part of the document, from one ResolvedFisheyeCal."""
    cal, is_reference, reason = resolved
    if is_reference:
        size_source = "reference"
    elif resolved.size_inferred:
        size_source = "inferred"
    else:
        size_source = "recorded"
    return {
        "source": "reference" if is_reference else "device",
        "reason": reason,
        "image_size": list(resolved.calibrated_size),
        "image_size_source": size_source,
        # The unit's own record IS the reference values: the factory stored
        # FISHEYE_FALLBACK_CAL instead of calibrating it.
        "matches_reference": resolved.matches_reference,
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


def _dual_slot(cal) -> bool:
    try:
        return cal.supports_fisheye_slots()
    except ProtocolError:  # firmware without Cmd 0x2B at all
        return False


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
        resolved = g.calibration.resolve_fisheye()
        _cal, is_reference, reason = resolved
        top = _intrinsics(resolved)

        if resolved.matches_reference:
            log.warning(
                "this unit's stored fisheye calibration IS the SDK reference values "
                "(FISHEYE_FALLBACK_CAL) — the factory did not calibrate it"
            )
            if args.require_device:
                sys.exit(
                    "error: --require-device requires the unit's own calibration, but "
                    "the stored one is the SDK reference values"
                )
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
            "source": top["source"],
            "reason": reason,
            "model": "fisheye_equidistant",
            "image_size": top["image_size"],
            "image_size_source": top["image_size_source"],
            "assumed_image_size": top["image_size"],
            **{k: top[k] for k in ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4")},
            "K": top["K"],
            "D": top["D"],
            "dual_slot_firmware": _dual_slot(g.calibration),
            "slots": {
                f"{w}x{h}": _intrinsics(g.calibration.resolve_fisheye_for(w, h))
                for w, h in SLOT_SIZES
            },
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

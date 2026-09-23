#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Read / write the fisheye-camera and encoder-max calibration records that the
gripper firmware persists in its internal flash (firmware V2.0 / V2.1).

Two independent records live behind these commands:

  fisheye     Cmd::CameraFisheyeCal (0x2B) — fx, fy, cx, cy, k1..k4 for the
              middle wrist camera. Available on BOTH leader and follower.
  encoder-max Cmd::EncoderMaxCal (0x2C) — the encoder shaft angle (rad) at
              which the gripper is fully open, measured from the encoder zero
              (fully closed). LEADER ONLY.

The firmware is a dumb store for both: it persists the bytes, does no unit
conversion and no clamping, and rejects only NaN/Inf (plus max_rad <= 0). A
record that was never written reads back as `None` rather than zeros, so you
can tell "never calibrated" from "calibrated to exactly 0".

The encoder-max value is what LeaderGripper(normalize_position=True) uses to
turn raw encoder radians into a 0..1 opening — see
`leader_normalized_position.py`.

Usage:
    # Show both records
    python python/examples/fisheye_cal.py show
    python python/examples/fisheye_cal.py show right

    # Write fisheye intrinsics + distortion
    python python/examples/fisheye_cal.py set-fisheye \\
        --fx 320.5 --fy 321.0 --cx 319.5 --cy 240.2 \\
        --k1 -0.031 --k2 0.0072 --k3 -0.0013 --k4 0.0002

    # Load them from an OpenCV-style .npz holding K (3x3) and D (4,)
    python python/examples/fisheye_cal.py set-fisheye --from-npz cam_left.npz

    # Measure the full-open angle interactively and store it
    python python/examples/fisheye_cal.py measure-encoder-max

    # Or write a known value directly
    python python/examples/fisheye_cal.py set-encoder-max --max-rad 1.30

选择器和其它示例一致:位置参数 left / right(或直接给固件 SN),
省略则要求只插了一台。列出连着的夹爪:
    python -c "from xense.taccap import scan_grippers; \\
               [print(g.firmware_sn, g.mcu_device) for g in scan_grippers()]"
"""

from __future__ import annotations

import argparse
import sys

import _calib_flow
import _target
from xense.taccap import (
    CameraFisheyeCal,
    FollowerGripper,
    LeaderGripper,
    ProtocolError,
)

_TTY = sys.stdout.isatty()


def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s


def _bold(s):
    return _c("1", s)


def _yellow(s):
    return _c("33", s)


def _green(s):
    return _c("32", s)


def open_gripper(args):
    """Open as a follower when asked, else as a leader.

    Fisheye calibration works on either; encoder-max is leader-only. The two
    classes share the same discovery path, so this is purely about which
    command surface the caller wants.
    """
    eps, _by_side, _all = _target.resolve_target(args.target)
    device = eps.mcu_device
    cls = FollowerGripper if getattr(args, "follower", False) else LeaderGripper
    return cls(mcu_device=device)


# ---- show -------------------------------------------------------------------


def cmd_show(args) -> int:
    with open_gripper(args) as g:
        cal = g.calibration
        fw = _target.firmware_version(g)
        print(_bold(f"\nFirmware {fw}") +
              "  (fisheye needs cmd set >= V2.0, encoder-max >= V2.1: "
              "leader >= 1.2.0; followers are gated at >= 1.2.5 on open)")

        print(_bold("\nFisheye camera calibration (Cmd 0x2B)"))
        try:
            fisheye = cal.read_fisheye()
        except ProtocolError as e:
            # InvalidCmd here = firmware predates V2.0 (0x2B isn't in its
            # command table at all).
            print(f"  {_yellow('unavailable')} — {e}")
            fisheye = None
        else:
            if fisheye is None:
                print("  " + _yellow("not calibrated") +
                      " — firmware returned CalNotSet")
        if fisheye is not None:
            print(f"  fx={fisheye.fx:.4f}  fy={fisheye.fy:.4f}")
            print(f"  cx={fisheye.cx:.4f}  cy={fisheye.cy:.4f}")
            print(f"  k1={fisheye.k1:.6f}  k2={fisheye.k2:.6f}  "
                  f"k3={fisheye.k3:.6f}  k4={fisheye.k4:.6f}")
            print("  OpenCV K:")
            for row in fisheye.K:
                print("    [" + "  ".join(f"{v:10.4f}" for v in row) + "]")
            print("  OpenCV D: [" +
                  "  ".join(f"{v:.6f}" for v in fisheye.D) + "]")

        print(_bold("\nEncoder max travel angle (Cmd 0x2C, leader only)"))
        try:
            max_rad = cal.read_encoder_max_rad()
        except ProtocolError as e:
            # InvalidCmd here = follower hardware, or firmware older than V2.1.
            print(f"  {_yellow('unavailable')} — {e}")
        else:
            if max_rad is None:
                print("  " + _yellow("not calibrated") +
                      " — firmware returned CalNotSet")
                print("  Run: python python/examples/fisheye_cal.py "
                      "measure-encoder-max")
            else:
                import math
                print(f"  max_rad = {max_rad:.4f} rad "
                      f"({math.degrees(max_rad):.1f}°)")
        print()
    return 0


# ---- set-fisheye ------------------------------------------------------------


def cmd_set_fisheye(args) -> int:
    if args.from_npz:
        import numpy as np

        data = np.load(args.from_npz)
        try:
            K = np.asarray(data["K"], dtype=float).reshape(3, 3)
            D = np.asarray(data["D"], dtype=float).reshape(-1)
        except KeyError as e:
            raise SystemExit(
                f"{args.from_npz}: expected arrays named 'K' and 'D', "
                f"missing {e}"
            ) from None
        if D.size < 4:
            raise SystemExit(
                f"{args.from_npz}: D has {D.size} coefficients, fisheye needs 4"
            )
        cal = CameraFisheyeCal(
            fx=float(K[0, 0]), fy=float(K[1, 1]),
            cx=float(K[0, 2]), cy=float(K[1, 2]),
            k1=float(D[0]), k2=float(D[1]), k3=float(D[2]), k4=float(D[3]),
        )
    else:
        missing = [n for n in ("fx", "fy", "cx", "cy")
                   if getattr(args, n) is None]
        if missing:
            raise SystemExit(
                "set-fisheye needs --fx/--fy/--cx/--cy (or --from-npz); "
                f"missing: {', '.join('--' + m for m in missing)}"
            )
        cal = CameraFisheyeCal(
            fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy,
            k1=args.k1, k2=args.k2, k3=args.k3, k4=args.k4,
        )

    print(f"Writing {cal}")
    with open_gripper(args) as g:
        g.calibration.write_fisheye(cal)
        readback = g.calibration.read_fisheye()
    print(_green("written to MCU flash. Read back:"))
    print(f"  {readback}")
    return 0


# ---- encoder max ------------------------------------------------------------


def cmd_set_encoder_max(args) -> int:
    with open_gripper(args) as g:
        g.calibration.write_encoder_max_rad(args.max_rad)
        readback = g.calibration.read_encoder_max_rad()
    print(_green(f"written: max_rad = {readback:.4f} rad"))
    return 0


def cmd_measure_encoder_max(args) -> int:
    if getattr(args, "follower", False):
        raise SystemExit("measure-encoder-max is leader-only")

    with open_gripper(args) as g:
        stored = _calib_flow.guided_calibration(
            g, assume_yes=args.yes)
        if stored is None:
            return 1

    print("\nNormalized position is now available:")
    print("  g = LeaderGripper(mcu_device=..., normalize_position=True)")
    print("  g.encoder.read_once().position   # 0 = closed, 1 = open")
    _calib_flow.restart_notice()
    return 0


def main() -> int:
    # Device selection lives on a parent parser so every subcommand takes the
    # same positional selector (`show right`, `set-fisheye left ...`) — the one
    # form used across all the examples.
    common = argparse.ArgumentParser(add_help=False)
    _target.add_target_argument(common)
    common.add_argument("--follower", action="store_true",
                        help="open as a FollowerGripper (fisheye only; the "
                             "encoder-max commands are leader-only)")

    # `common` goes on the SUBcommands only. It used to be on the top-level
    # parser too, so `--sn X show` and `show --sn X` both worked; a positional
    # selector cannot do that (the first word would be eaten as the target),
    # and having --follower on both parsers meant the subparser's default
    # silently overwrote a top-level `--follower` anyway.
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("show", parents=[common],
                   help="print both calibration records").set_defaults(
        func=cmd_show)

    sf = sub.add_parser("set-fisheye", parents=[common],
                        help="write fisheye parameters")
    sf.add_argument("--from-npz", metavar="FILE",
                    help="load K (3x3) and D (4,) from an OpenCV-style .npz")
    for name in ("fx", "fy", "cx", "cy"):
        sf.add_argument(f"--{name}", type=float)
    for name in ("k1", "k2", "k3", "k4"):
        sf.add_argument(f"--{name}", type=float, default=0.0)
    sf.set_defaults(func=cmd_set_fisheye)

    se = sub.add_parser("set-encoder-max", parents=[common],
                        help="write a known full-open angle (rad)")
    se.add_argument("--max-rad", type=float, required=True)
    se.set_defaults(func=cmd_set_encoder_max)

    me = sub.add_parser("measure-encoder-max", parents=[common],
                        help="measure the full-open angle interactively, "
                             "then store it")
    me.add_argument("-y", "--yes", action="store_true",
                    help="write without the confirmation prompt")
    me.set_defaults(func=cmd_measure_encoder_max)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

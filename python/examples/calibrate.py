#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Encoder calibration for a TacCap leader gripper — zero + travel span.

Pick which gripper to calibrate by side — ``left`` / ``right`` — or, if
you prefer to be explicit, by its firmware SN. Either way exactly one
gripper is selected, so with both sides plugged in you cannot
accidentally zero the wrong one. The script:

  1. Resolves the side (or SN) to one endpoint and prints the firmware SN
     it picked, so what got calibrated is on the record.
  2. Opens the LeaderGripper and prints the current encoder reading.
  3. Asks you to hold the gripper FULLY CLOSED, then latches that pose
     as the new zero via Encoder::set_zero (wire Cmd::SetEncoderZero).
  4. Re-reads the encoder to confirm post-zero ≈ 0.
  5. Asks you to OPEN the gripper to its mechanical limit and STORES the
     measured angle as the travel span (Cmd::EncoderMaxCal 0x2C). That
     span is what LeaderGripper(normalize_position=True) divides by, so
     until it is stored, normalized position is unavailable.

Both records live in MCU flash and survive power cycles. Step 5 needs
command set >= V2.1 (leader >= 1.2.0); the script checks that up front, before
step 3 writes anything, so an old gripper is never left half-calibrated.

Usage:
    python python/examples/calibrate.py left
    python python/examples/calibrate.py right --skip-open-probe
    python python/examples/calibrate.py TCGU01A28Z0023m   # explicit SN

Side comes from the firmware-burned SN read over the wire (Cmd::GetSn),
not from the CH343 USB chip serial — the same rule the rest of the stack
uses, so ``left`` here is the same gripper ``left`` means everywhere else.

Tip: list available SNs with
    python -c "from xense.taccap import scan_grippers, Side; \\
               [print(f'{\"L\" if g.side==Side.Left else \"R\"} fw={g.firmware_sn} mcu={g.mcu_serial}') for g in scan_grippers()]"
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from xense.taccap import LeaderGripper, Side, scan_grippers

import _target
import _calib_flow

# No expected full-open angle is defined on purpose. The measured span IS
# the calibration — it is what the mechanism does and what the SDK normalizes
# against, so there is nothing to compare it to. The previous 1.7 rad "design
# baseline" was stale and only produced false alarms: two units measured
# 1.1582 and 1.1589 rad (66.4°), agreeing to 0.04°, both outside its band.

# Tolerance for the "is the new zero actually zero" post-latch check.
# Firmware latches what it sees the moment it processes the command, so
# any residual is mostly hand-jitter between read and latch.
POST_ZERO_TOLERANCE_RAD = 0.01

# Mild ANSI colors for the human-facing prompts. Skipped if stdout
# isn't a tty (piped to a log file, etc.) so the log stays grep-clean.
_TTY = sys.stdout.isatty()
def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s
def _cyan(s):   return _c("36", s)
def _yellow(s): return _c("33", s)
def _green(s):  return _c("32", s)
def _red(s):    return _c("31", s)
def _bold(s):   return _c("1",  s)


def _rad_to_deg(r: float) -> float:
    return r * 180.0 / math.pi


def _open_gripper(eps) -> LeaderGripper:
    # Calibration only talks to the MCU (encoder); cameras stay off.
    return LeaderGripper(eps.mcu_device)


def _read_positions_rad(g: LeaderGripper) -> tuple[float, float]:
    """Return (raw, cooked) position in rad.

    `cooked` is what SDK consumers see (clamped to >= 0 by Encoder's
    post-zero normalisation). `raw` is the firmware-side value before
    clamping — calibration UX must show this, otherwise pre-latch drift
    (e.g. 0.09 rad) would silently display as 0.00 and the user would
    think calibration isn't needed.
    """
    s = g.encoder.read_once()
    return float(s.raw_position_rad), float(s.position_rad)


def _prompt(msg: str) -> None:
    """Block until the user hits Enter; surface ANSI emphasis on the prompt."""
    try:
        input(f"  {msg} ")
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(_red("aborted."))


def calibrate(target: str, *, skip_open_probe: bool) -> int:
    eps, by_side, all_eps = _target.resolve_target(target)

    print()
    print(_cyan("=" * 64))
    print(_cyan(f"  TacCap leader-gripper encoder calibration"))
    print(_cyan("=" * 64))
    print(f"  requested    : {_bold(target)}"
          f"{'  (resolved by side)' if by_side else ''}")
    print(f"  firmware SN  : {_bold(eps.firmware_sn)}")
    print(f"  side         : {_bold(_target.side_str(eps.side))}")
    print(f"  mcu serial   : {eps.mcu_serial}")
    print(f"  mcu device   : {eps.mcu_device}")
    if by_side:
        # Show the whole scan when the SN was chosen for the user — it is
        # the only way they can tell the pick was the one they meant.
        print(f"  visible      : {_target.listing(all_eps)}")
    print()

    g = _open_gripper(eps)

    # ---- 0. Pre-flight: can this firmware store a travel span? -------------
    # Checked BEFORE step 2 latches a new zero into flash, so a pre-V2.1
    # gripper is refused while it is still untouched rather than left with a
    # new zero and no span. Skipped when the caller only wants the zero.
    if not skip_open_probe:
        _calib_flow.require_support(g)
        existing = _calib_flow.read_encoder_max(g)
        if existing is not None:
            print(f"  existing span: {_bold(f'{existing:.4f} rad')} "
                  f"({_rad_to_deg(existing):.2f}°) — will be overwritten")
            print()

    # ---- 1. Current reading ------------------------------------------------
    cur_raw, cur_cooked = _read_positions_rad(g)
    print(f"  current encoder: "
          f"{_bold(f'raw={cur_raw:+.4f} rad')}  ({_rad_to_deg(cur_raw):+.2f}°)"
          f"   cooked={cur_cooked:+.4f} rad")
    print()

    # ---- 2. Latch zero -----------------------------------------------------
    print(_yellow("Step 1/2: hold the gripper FULLY CLOSED."))
    print(_yellow(f"          (Ctrl+C any time to abort without changing zero.)"))
    _prompt(_yellow("→ press [Enter] when held closed:"))

    pre_raw, pre_cooked = _read_positions_rad(g)
    print(f"  pre-latch reading : raw={pre_raw:+.4f} rad "
          f"({_rad_to_deg(pre_raw):+.2f}°)   cooked={pre_cooked:+.4f}")

    try:
        g.encoder.set_zero()
    except Exception as e:
        print(_red(f"  ✗ set_zero failed: {type(e).__name__}: {e}"))
        return 1
    time.sleep(0.05)  # let firmware settle one streaming tick

    post_raw, post_cooked = _read_positions_rad(g)
    print(f"  post-latch reading: raw={post_raw:+.4f} rad "
          f"({_rad_to_deg(post_raw):+.2f}°)   cooked={post_cooked:+.4f}")

    # Validate against the RAW value — cooked is clamped at 0 so it
    # would always look "perfect" even for residual drift up to +inf
    # on the negative side. raw tells the truth.
    if abs(post_raw) <= POST_ZERO_TOLERANCE_RAD:
        print(_green(f"  ✓ zero latched OK (|raw post-zero| ≤ {POST_ZERO_TOLERANCE_RAD:.3f} rad)"))
    else:
        print(_yellow(
            f"  ⚠ raw post-zero is {post_raw:+.4f} rad. Firmware latched what "
            "it saw the instant the cmd arrived — most likely you moved the "
            "gripper between read and latch. Re-run if you want it tighter."
        ))
    print()

    # ---- 3. Measure and STORE the full-open travel span --------------------
    #
    # There is no expected value to compare against. The measured span is the
    # answer — it is what the mechanism actually does, and it is what the SDK
    # normalizes against. (The old ±tolerance check against a 1.7 rad design
    # figure only produced false alarms: two units measure ~1.158 rad.)
    if skip_open_probe:
        print(_cyan("  --skip-open-probe set, encoder zero done."))
        print(_yellow(
            "  Note: the travel span was NOT measured, so normalized position "
            "stays unavailable. Re-run without --skip-open-probe to store it."))
        return 0

    print(_yellow("Step 2/2: open the gripper to its MECHANICAL LIMIT."))
    _prompt(_yellow("→ press [Enter] when fully open:"))

    open_raw, _ = _read_positions_rad(g)
    print(f"  fully-open reading: "
          f"{_bold(f'{open_raw:+.4f} rad')}  ({_rad_to_deg(open_raw):+.2f}°)")

    if open_raw <= 0.0:
        print(_red(
            f"  ✗ span is {open_raw:+.4f} rad — the encoder did not move in "
            "the positive direction while opening. Either the zero did not "
            "take or the gripper was not opened. Nothing stored."))
        return 1

    readback = _calib_flow.store_max(g, open_raw)
    print(_green(f"  ✓ stored: max_rad = {readback:.4f} rad "
                 f"({_rad_to_deg(readback):.2f}°)"))
    print(_cyan("    normalized position is now available: "
                "LeaderGripper(..., normalize_position=True)"))
    _calib_flow.restart_notice()
    print()

    # ---- 4. Optional live readout for visual confirmation ------------------
    print(_cyan("  Live encoder readout (10 Hz, raw | cooked, position 0..1; "
                "Ctrl+C to exit):"))
    try:
        while True:
            raw, cooked = _read_positions_rad(g)
            # Normalize against the span we just stored, so the readout shows
            # exactly what the SDK will report after a restart.
            pos = min(max(cooked / readback, 0.0), 1.0)
            sys.stdout.write(
                f"\r    raw={raw:+.4f} rad ({_rad_to_deg(raw):+6.2f}°) | "
                f"cooked={cooked:+.4f} rad | position={pos:5.3f}     "
            )
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
        print(_green("  done."))
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _target.add_target_argument(p, required=True)
    p.add_argument(
        "--skip-open-probe",
        action="store_true",
        help="Only latch the encoder zero; do not measure or store the "
             "travel span (leaves normalized position unavailable).",
    )
    args = p.parse_args()
    return calibrate(args.target, skip_open_probe=args.skip_open_probe)


if __name__ == "__main__":
    sys.exit(main())

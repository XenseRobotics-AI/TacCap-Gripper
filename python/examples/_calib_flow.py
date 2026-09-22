# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Guided encoder-max calibration, shared by the example CLIs.

This lives in examples/ and NOT in the ``xense.taccap`` package on purpose:
it prompts on stdin, and a library that blocks on stdin breaks every headless
consumer — ROS 2 nodes, background services, unit tests. The SDK keeps raising
``ProtocolError`` when the calibration is missing; deciding to walk a human
through fixing it is an application-level choice, so it belongs here.

There is no "expected" full-open angle anywhere in this flow. The measured
value *is* the answer: the gripper's travel is whatever the mechanism does,
and comparing it against a hardcoded design figure only ever produced false
alarms (two units measured 1.1582 and 1.1589 rad against a stale 1.7 baseline).
The only checks are the ones that catch a genuinely broken measurement —
a non-positive span, or a zero that did not take.
"""

from __future__ import annotations

import math
import os
import sys

from xense.taccap import ProtocolError, Side, scan_grippers

# ---- device selection, shared by every example CLI --------------------------
#
# One selector across the repo: a positional ``left`` / ``right`` (or a literal
# firmware SN). Deliberately NOT a ``--sn`` / ``--device`` flag — a rig has a
# left and a right of everything (gripper, wrist camera, tactile pair), and the
# side is the only handle that means the same thing for all of them. Wrist
# cameras carry the *same* sequence number as the gripper they sit on, so
# ``left`` picks out one physical half of the rig everywhere it appears.
#
# Side always comes from the firmware SN read over the wire (Cmd::GetSn), never
# from the CH343 USB-chip SN — that one is burned independently of which hand
# the gripper is.

# Accepted spellings of the side selector. Anything else on the command line is
# treated as a literal firmware SN, so the two forms can never collide (Xense
# SNs start with "TCGU01…").
SIDE_ALIASES = {
    "left": Side.Left,
    "l": Side.Left,
    "right": Side.Right,
    "r": Side.Right,
}


def side_str(side) -> str:
    return {Side.Left: "Left", Side.Right: "Right"}.get(side, "Unknown")


def listing(all_eps) -> str:
    return (", ".join(f"{e.firmware_sn} ({side_str(e.side)})" for e in all_eps)
            or "(none)")


def resolve_target(target: str | None):
    """``left`` / ``right`` / a firmware SN / ``None`` -> one endpoint.

    ``None`` means "the only gripper plugged in" and fails loudly when that is
    not what is plugged in — an example must never guess which half of a
    bilateral rig the user meant.

    Returns ``(endpoint, by_side, all_endpoints)``.
    """
    all_eps = scan_grippers()

    if target is None:
        if len(all_eps) == 1:
            return all_eps[0], False, all_eps
        sys.exit(
            f"error: {len(all_eps)} grippers plugged in — say which one: "
            "'left', 'right', or a firmware SN.\n"
            f"       currently visible: {listing(all_eps)}"
        )

    side = SIDE_ALIASES.get(target.strip().lower())
    by_side = side is not None
    if by_side:
        matches = [e for e in all_eps if e.side == side]
        what = f"side={side_str(side)}"
    else:
        matches = [e for e in all_eps if e.firmware_sn == target]
        what = f"firmware SN={target!r}"

    if not matches:
        hint = "" if by_side else "\n       (you can also just pass 'left' or 'right')"
        sys.exit(
            f"error: no gripper matching {what} is plugged in.\n"
            f"       currently visible: {listing(all_eps)}{hint}"
        )
    if len(matches) > 1:
        if by_side:
            sys.exit(
                f"error: {len(matches)} plugged-in grippers report {what} — "
                f"their firmware SNs are "
                f"{', '.join(e.firmware_sn for e in matches)}. Fix the burned "
                "SNs, or pass the SN you want explicitly."
            )
        sys.exit(
            f"error: {len(matches)} grippers report SN={target!r} — "
            "firmware-SN collision, check firmware burning."
        )
    return matches[0], by_side, all_eps


def add_target_argument(parser, *, required: bool = False) -> None:
    """Attach the repo-wide positional selector to an argparse parser."""
    parser.add_argument(
        "target", nargs=None if required else "?", metavar="left|right|SN",
        help="Which gripper: 'left' / 'right' (side comes from the firmware SN), "
             "or an explicit SN such as TCGU01A28Z0116m."
             + ("" if required else " Omit when exactly one is plugged in."),
    )


def open_follower(target):
    """``resolve_target()`` + ``FollowerGripper`` -> ``(gripper, endpoint)``.

    Three examples had a byte-identical copy of this, each with its own
    ``--side`` flag, which contradicted the single positional selector the rest
    of the examples use (see docs/EXAMPLES.md). One home, one convention.

    The endpoint comes back with the gripper because callers want its SN and
    side for display, and re-deriving them would mean a second ``scan_grippers()``
    -- a full USB walk, and a chance for the answer to change in between.
    """
    from xense.taccap import FollowerGripper

    endpoint, _by_side, _all = resolve_target(target)
    print(f"[discovery] {endpoint.firmware_sn}  {endpoint.mcu_device}")
    return FollowerGripper(mcu_device=endpoint.mcu_device), endpoint


# Tolerance for the "did the new zero actually take" check. The firmware
# latches whatever it sees the instant it processes the command, so any
# residual is hand-jitter between the read and the latch.
POST_ZERO_TOLERANCE_RAD = 0.01

_TTY = sys.stdout.isatty()


def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s


def bold(s):
    return _c("1", s)


def yellow(s):
    return _c("33", s)


def green(s):
    return _c("32", s)


def red(s):
    return _c("31", s)


def _ota_script_path() -> str:
    """Path to ota_update.py that the reader can paste as-is.

    Relative to their cwd, because this repo is usually vendored as a
    submodule: someone running calibrate.py from a parent repo's root needs
    third_party/taccap-gripper/python/examples/..., and a hardcoded
    python/examples/... would send them looking for a directory they are not
    standing in. Falls back to the absolute path when relpath would climb out
    of the tree (different drive, or a cwd above nothing in common).
    """
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "ota_update.py")
    try:
        rel = os.path.relpath(p)
    except ValueError:
        return p
    return rel if not rel.startswith(os.pardir + os.sep) else p


def deg(rad: float) -> str:
    return f"{math.degrees(rad):.1f}°"


def format_version(major, minor, patch, build=None) -> str:
    """Render a firmware version for humans: MAJOR.MINOR.PATCH.

    The wire carries a fourth "build" byte, but it is pinned to 0 and means
    nothing to users, so it is deliberately not shown — printing "1.2.1.0"
    only invited people to type the trailing zero into version comparisons.
    `build` is accepted so callers can pass all four through without
    unpacking, and ignored on purpose. Mirrors C++
    protocol::version_string().
    """
    return f"{major}.{minor}.{patch}"


def firmware_version(gripper) -> str:
    """Best-effort MAJOR.MINOR.PATCH, for capability error messages.

    Cmd::GetVersion returns the compiled-in constant, not the OTA bank
    metadata, so this is the authoritative "what build am I talking to".
    """
    from xense.taccap import Cmd

    try:
        ack = gripper.transport.send_cmd(Cmd.GetVersion, b"", 500)
        if len(ack.data) >= 4:
            return format_version(*ack.data[:4])
    except Exception:
        pass
    return "<unknown>"


class CalibrationUnsupported(RuntimeError):
    """Firmware predates V2.1 and has no Cmd 0x2C at all."""


def read_encoder_max(gripper):
    """Return the stored travel span, or None when never calibrated.

    Raises CalibrationUnsupported when the firmware does not implement
    Cmd::EncoderMaxCal (0x2C) — distinct from "supported but unset", which
    the firmware reports as CalNotSet and the SDK surfaces as None.
    """
    try:
        return gripper.calibration.read_encoder_max_rad()
    except ProtocolError as e:
        raise CalibrationUnsupported(str(e)) from None


def require_support(gripper, fw_version: str | None = None) -> None:
    """Fail before anything is written if the firmware is too old.

    Call this BEFORE latch_zero(): step 1 persists a new encoder zero to
    flash, so finding out at the final write that the firmware cannot store
    the span would leave the gripper half-calibrated — new zero, no span.
    """
    try:
        read_encoder_max(gripper)
    except CalibrationUnsupported as e:
        if fw_version is None:
            fw_version = firmware_version(gripper)
        raise SystemExit(
            f"{red('✗')} encoder-max calibration needs command set >= V2.1 "
            f"(leader >= 1.2.0); this gripper reports {fw_version}.\n"
            f"  {e}\n"
            f"  Nothing was changed. Flash it first:\n"
            f"      python {_ota_script_path()} master \\\n"
            f"          <left|right> --target-version 1.2.1\n"
            f"  (that image ships in this SDK under firmware/; pick it by the\n"
            f"   gripper's ROLE — the last character of its firmware SN, 'm'\n"
            f"   for master and 's' for slave — not by which hand it is on.)"
        ) from None


def latch_zero(gripper) -> float:
    """Step 1: prompt, latch the fully-closed pose as zero, verify it took."""
    print(bold("\nStep 1 — hold the gripper FULLY CLOSED"))
    input("  press Enter to latch that pose as the encoder zero... ")
    gripper.encoder.set_zero()
    after = gripper.encoder.read_once().position_rad
    print(f"  post-zero reading: {after:+.4f} rad")
    if abs(after) > POST_ZERO_TOLERANCE_RAD:
        print("  " + yellow(
            f"warning: {abs(after):.4f} rad off zero — the pose moved between "
            f"the read and the latch. Re-run if this looks wrong."))
    return after


def measure_max(gripper) -> float:
    """Step 2: prompt, sample the fully-open angle. Raises on a bad reading."""
    print(bold("\nStep 2 — open the gripper to its mechanical limit"))
    input("  press Enter to sample the full-open angle... ")
    max_rad = gripper.encoder.read_once().position_rad
    print(f"  full-open reading: {max_rad:.4f} rad ({deg(max_rad)})")

    if max_rad <= 0.0:
        raise SystemExit(
            f"{red('✗')} measured span is {max_rad:.4f} rad — the encoder did "
            f"not move in the positive direction while opening. Either the "
            f"zero step did not take, or the gripper was not actually opened. "
            f"Nothing was written."
        )
    return max_rad


def store_max(gripper, max_rad: float) -> float:
    """Persist the span and read it straight back for confirmation."""
    gripper.calibration.write_encoder_max_rad(max_rad)
    readback = gripper.calibration.read_encoder_max_rad()
    if readback is None:
        raise SystemExit(
            f"{red('✗')} wrote {max_rad:.4f} rad but read back "
            f"'not calibrated' — the flash write did not stick.")
    return readback


def guided_calibration(gripper, *, fw_version: str | None = None,
                       assume_yes: bool = False) -> float | None:
    """Full two-step flow: support check → zero → measure → confirm → store.

    Returns the stored span, or None if the user declined at the confirmation
    prompt (in which case nothing was written).
    """
    require_support(gripper, fw_version)
    latch_zero(gripper)
    max_rad = measure_max(gripper)

    if not assume_yes:
        reply = input(f"\nWrite {max_rad:.4f} rad ({deg(max_rad)}) "
                      f"to MCU flash? [y/N] ")
        if reply.strip().lower() not in ("y", "yes"):
            print("Aborted; nothing written. The new encoder zero from step 1 "
                  "does remain in effect.")
            return None

    readback = store_max(gripper, max_rad)
    print(green(f"\n✓ stored: max_rad = {readback:.4f} rad ({deg(readback)})"))
    return readback


def restart_notice() -> None:
    """Both records live in MCU flash, so only this process needs restarting.

    set_encoder_zero persists via storage_write_encoder_calibration and the
    span via Cmd 0x2C, and both take effect on the firmware immediately — no
    power cycle. The restart is only so this process re-reads them at startup.
    """
    print(bold("\nPlease restart this program") +
          " to pick up the new calibration.")
    print("  (No power cycle needed — both values are already live in MCU "
          "flash.)")


def offer_calibration(gripper, *, fw_version: str | None = None,
                      assume_yes: bool = False) -> bool:
    """Entry point for apps that find the calibration missing at startup.

    Returns True if a span was stored (caller should tell the user to
    restart), False if the user declined.
    """
    # Check support before asking — offering a calibration the firmware
    # cannot store would be a question we can't honour.
    require_support(gripper, fw_version)

    print(yellow("✗ this gripper has no encoder-max calibration") +
          " — normalized position is unavailable until it is measured.")
    if not assume_yes:
        reply = input("  Calibrate now? [Y/n] ")
        if reply.strip().lower() in ("n", "no"):
            print("  Skipped. Run this later:\n"
                  "      python python/examples/calibrate.py <left|right|SN>")
            return False
    return guided_calibration(gripper, fw_version=fw_version,
                              assume_yes=assume_yes) is not None

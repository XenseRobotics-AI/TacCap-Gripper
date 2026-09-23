# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Guided encoder-max calibration: walk a human through zero + travel span.

Imported by the three examples that can hit a missing calibration —
``calibrate.py``, ``fisheye_cal.py`` and ``leader_normalized_position.py``.
Device selection and printing live in ``_target.py``; this module is only the
walkthrough.

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

import os

from _target import bold, deg, firmware_version, green, red, yellow
from xense.taccap import ProtocolError

# Tolerance for the "did the new zero actually take" check. The firmware
# latches whatever it sees the instant it processes the command, so any
# residual is hand-jitter between the read and the latch.
POST_ZERO_TOLERANCE_RAD = 0.01


def _ota_script_path() -> str:
    """Path to ota_update.py that the reader can paste as-is.

    Relative to their cwd, because this repo is usually vendored as a
    submodule: someone running calibrate.py from a parent repo's root needs
    third_party/taccap-gripper/python/examples/..., and a hardcoded
    python/examples/... would send them looking for a directory they are not
    standing in. Falls back to the absolute path when relpath would climb out
    of the tree (different drive, or a cwd above nothing in common).
    """
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ota_update.py")
    try:
        rel = os.path.relpath(p)
    except ValueError:
        return p
    return rel if not rel.startswith(os.pardir + os.sep) else p


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
        print(
            "  "
            + yellow(
                f"warning: {abs(after):.4f} rad off zero — the pose moved between "
                f"the read and the latch. Re-run if this looks wrong."
            )
        )
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
            f"'not calibrated' — the flash write did not stick."
        )
    return readback


def guided_calibration(
    gripper, *, fw_version: str | None = None, assume_yes: bool = False
) -> float | None:
    """Full two-step flow: support check → zero → measure → confirm → store.

    Returns the stored span, or None if the user declined at the confirmation
    prompt (in which case nothing was written).
    """
    require_support(gripper, fw_version)
    latch_zero(gripper)
    max_rad = measure_max(gripper)

    if not assume_yes:
        reply = input(
            f"\nWrite {max_rad:.4f} rad ({deg(max_rad)}) to MCU flash? [y/N] "
        )
        if reply.strip().lower() not in ("y", "yes"):
            print(
                "Aborted; nothing written. The new encoder zero from step 1 "
                "does remain in effect."
            )
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
    print(bold("\nPlease restart this program") + " to pick up the new calibration.")
    print("  (No power cycle needed — both values are already live in MCU flash.)")


def offer_calibration(
    gripper, *, fw_version: str | None = None, assume_yes: bool = False
) -> bool:
    """Entry point for apps that find the calibration missing at startup.

    Returns True if a span was stored (caller should tell the user to
    restart), False if the user declined.
    """
    # Check support before asking — offering a calibration the firmware
    # cannot store would be a question we can't honour.
    require_support(gripper, fw_version)

    print(
        yellow("✗ this gripper has no encoder-max calibration")
        + " — normalized position is unavailable until it is measured."
    )
    if not assume_yes:
        reply = input("  Calibrate now? [Y/n] ")
        if reply.strip().lower() in ("n", "no"):
            print(
                "  Skipped. Run this later:\n"
                "      python python/examples/calibrate.py <left|right|SN>"
            )
            return False
    return (
        guided_calibration(gripper, fw_version=fw_version, assume_yes=assume_yes)
        is not None
    )

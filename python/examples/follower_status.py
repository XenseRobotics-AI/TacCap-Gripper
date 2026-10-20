#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Read follower state: one-shot read, streaming read, and what each status
field means.

It first prints three versions: gripper firmware, motor model, and the
motor's own firmware (checked against the minimum). When troubleshooting, send
us this script's full output; it contains all the version info we need.

The follower **streams motor status only**. In this role the firmware
compiles out the IMU / encoder / e-skin streams, so `start_streaming()` takes
only one parameter, `motor_hz`.

Two ways to read, each with its own use:

    g.motor.read_status()        one command round trip, read whenever you want
    g.motor.on_status(cb)        register a callback, pushed at the firmware's pace

**Do not use read_status() during control.** The command's ACK collides with
control frames on the serial port; the controller owns the status stream
itself and `snapshot()` returns the latest frame, no extra polling needed --
see control_and_read.py.

An easy trap: **status can be stale while everything looks normal.** The
firmware once stopped its own polling because it requested active reporting
from the motor (which this motor does not support), so the status frame froze
permanently while idle; version, data stream and counters were all normal,
and the only symptom was that the values read were silently stale. So this
script prints how far `status_timestamp_ms` advances: it must track the wall
clock.

Usage
    python python/examples/follower_status.py right
    python python/examples/follower_status.py --seconds 5 --hz 100
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import MotorStopReason, log


def describe_status(s) -> str:
    """Status bits + stop reason, rendered human-readable."""
    try:
        reason = str(MotorStopReason(s.stop_reason)).split(".")[-1]
    except ValueError:
        reason = f"0x{s.stop_reason:02X}"
    return f"status=0x{s.status:04X} stop={reason}"


# Minimum motor firmware, compared on the raw four bytes of
# MotorVersion.version (most significant first). The two product lines are
# numbered independently and are not comparable. Motors below it fail factory
# test: EL05 1.0.5.0.2 has constant velocity feedback, RS00 0.0.3.22 jitters
# in motion.
MOTOR_FW_MIN = {
    "EL05": ((10, 5, 0, 4), "1.0.5.0.4"),
    "RS00": ((0, 0, 3, 32), "0.0.3.32"),
}


def print_motor_versions(g) -> None:
    """Motor model + the motor's own firmware version (not the gripper firmware),
    checked against the minimum."""
    model = g.motor.get_model()
    print(f"[motor] {model!r}")
    if not model.from_flash:
        print(
            "  model is the compile-time default, not this unit's record -- see "
            'firmware/README.md "Follower motor setup"'
        )

    try:
        v = g.motor.motor_version()
    except Exception as e:  # follower firmware < 1.2.6 has no 0x58
        print(f"[motor fw] unreadable ({e}) -- needs follower firmware >= 1.2.6")
        return
    if not v.valid:
        print(
            "[motor fw] unreadable -- the motor does not answer version on the MIT "
            "protocol, and there is no record in flash (needs follower firmware "
            ">= 1.2.14); record it from the nameplate with "
            "g.motor.set_motor_fw_version()"
        )
        return
    src = "flash record" if v.from_flash else "live read from motor"
    line = f"[motor fw] {v.vendor_str}  ({src})"
    floor = MOTOR_FW_MIN.get(model.name)
    if floor is not None:
        ok = tuple(v.version) >= floor[0]
        line += f"  {'OK' if ok else '**below minimum ' + floor[1] + ', motor firmware update needed**'}"
    print(line)
    if v.motor_stopped:
        print(
            "  this read made the motor execute a stop (RobStride comm type 4); "
            "re-enable before moving"
        )


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--seconds", type=float, default=3.0, help="how long to read the stream"
    )
    ap.add_argument(
        "--hz",
        type=int,
        default=100,
        help="motor status stream rate (firmware max 100)",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")
    print_motor_versions(g)

    # ── One-shot read ─────────────────────────────────────────────────────────
    s = g.motor.read_status()
    print("\n[one-shot] g.motor.read_status()")
    print(
        f"  position  {s.actual_pos:+.5f} rad     "
        "(raw motor shaft angle, zero set by power-on calibration)"
    )
    print(f"  velocity  {s.actual_vel:+.5f} rad/s")
    print(f"  torque    {s.actual_torque:+.4f} Nm    (feedback, not the command)")
    print(f"  temp      {s.motor_temp_c:.1f} °C")
    print(f"  {describe_status(s)}")

    # Normalized opening is meaningful only after calibration: 0=closed 1=open,
    # defined by the travel in gripper_config.
    cfg = g.get_gripper_config()
    print(f"\n[normalized] g.position() = {g.position():.4f}")
    print(
        f"  travel min_open={cfg.min_open_rad:.5f}  max_open={cfg.max_open_rad:.5f} rad"
    )

    # ── Faults ────────────────────────────────────────────────────────────────
    fr = g.motor.fault_report()
    print(
        f"\n[faults] motor 0x{fr.motor_fault_code:08X}  "
        f"latched 0x{fr.motor_latched_fault_code:08X}"
        f"  firmware 0x{fr.firmware_fault_code:08X}"
    )
    print(
        "  latched bits clear only when explicitly cleared -- 0 now does not mean "
        "there was never a fault."
    )

    # ── Streaming read ────────────────────────────────────────────────────────
    frames: list = []
    sub = g.motor.on_status(lambda x: frames.append((time.perf_counter(), x)))
    # Firmware-side tick of the last status frame. Freshness must use this, not
    # sample.host_time -- the latter is **when the host received it**, and keeps
    # rising as long as the stream is pushing, even if the firmware sends the
    # same stale data. That is exactly the hole through which status froze for
    # 45 s with no symptom.
    fw_ms_before = g.motor.fault_report().status_timestamp_ms
    t0 = time.perf_counter()
    g.start_streaming(motor_hz=args.hz)
    try:
        time.sleep(args.seconds)
    finally:
        g.stop_streaming()
        g.motor.off(sub)
    elapsed = time.perf_counter() - t0

    print(f"\n[streaming] g.start_streaming(motor_hz={args.hz}) + on_status()")
    if not frames:
        print(
            "  **not a single frame received** -- the stream did not start, or the "
            "callback was registered after streaming started."
        )
        return 1
    print(f"  {len(frames)} frames / {elapsed:.2f} s = {len(frames) / elapsed:.1f} Hz")

    # Freshness: the firmware status timestamp must advance with the wall clock.
    # Frozen = the trap described above.
    fw_ms_after = g.motor.fault_report().status_timestamp_ms
    advance = fw_ms_after - fw_ms_before
    wall_ms = elapsed * 1000.0
    fresh = advance > wall_ms * 0.5
    print(
        f"  firmware status timestamp advanced {advance} ms, wall clock "
        f"{wall_ms:.0f} ms"
        f"  {'OK' if fresh else '**status is stale -- stream pushing, content not updating**'}"
    )

    lo = min(f[1].actual_pos for f in frames)
    hi = max(f[1].actual_pos for f in frames)
    print(
        f"  position range [{lo:+.5f}, {hi:+.5f}] rad  "
        "(at rest these two should be nearly equal)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

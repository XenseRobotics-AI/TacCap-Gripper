#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Show everything a TacCap-Gripper produces in one Rerun viewer, on one timeline.

For each selected gripper:

    gripper     leader:   encoder opening (0..1 and rad), IMU accel / gyro, button
                follower: motor status — opening, velocity, torque, temperature,
                          enabled / fault / stalled bits
    wrist       the XC wrist camera, through this SDK's Camera
    tactile     both visuotactile sensors, through xensesdk — the gel image, the
                force field drawn over it, the resultant force, optionally depth

Read-only: the motor is never enabled, nothing is written to flash. Use it to
check a rig before recording. To drive the jaw while watching all of this, use
`gripper_console.py --rerun` instead (same viewer, plus target and commanded
torque).

rerun-sdk and xensesdk come with environment.yml.

Usage:
    python python/examples/rerun_viewer.py                    # every gripper plugged in
    python python/examples/rerun_viewer.py left
    python python/examples/rerun_viewer.py left right --depth
    python python/examples/rerun_viewer.py right --tactile-image difference
    python python/examples/rerun_viewer.py left --no-tactile --no-wrist   # MCU only
    python python/examples/rerun_viewer.py left --save rig.rrd --duration 20
    python python/examples/rerun_viewer.py left --connect rerun+http://HOST:9876/proxy

How the streams line up. Every SDK sample (IMU, encoder, motor status, camera
frame) carries `host_time`, seconds on the host's monotonic clock. xensesdk's
`OutputType.TimeStamp` is wall-clock (time.time()) instead, taken ~3 ms before
selectSensorInfo() returns; it is moved onto the monotonic clock as it arrives,
so all of it shares the `host_time` timeline. Each stream is logged from its
own thread, and each thread sets that timeline itself (Rerun's time is
per-thread).

The entity layout is listed in _rerun.py.

Things that bite:

  * The wrist camera and the tactile sensors share the gripper's USB 2.0 link.
    Every camera must stream MJPEG (the SDK's Camera does by default); see the
    README's host setup. Compare the measured rates in the viewer against the
    nominal ones.
  * Opening a UVC device that another process already holds fails or steals it.
    If a camera service owns the wrist camera, pass --no-wrist.
  * Open the tactile sensors with nothing touching the gel: the no-contact
    reference (what the difference image and force are measured against) is
    taken when they open.
  * The tactile sensors run at 120 fps. Every frame is read (the printed rate
    is the real one) but only --tactile-hz of them are logged, and depth is
    shown at quarter resolution: at the full rate, two sensors fill a GB of
    viewer memory in a few seconds. Images are JPEG-compressed for the same
    reason (--jpeg-quality, 0 = raw).
"""

from __future__ import annotations

import argparse
import sys
import time

import _rerun
import _target
import numpy as np
from _rerun import rr, stamp
from xense.taccap import (
    FollowerGripper,
    GripperPosition,
    LeaderGripper,
    ProtocolError,
    Role,
    Side,
    log,
    scan_grippers,
)


def open_gripper(ep):
    """The right class for the endpoint's role, plus its opening converter.

    A missing travel calibration is not fatal: the viewer still shows the raw
    angle and says the 0..1 opening is unavailable. The follower's converter is
    kept even then, because its ``reverse`` flag is what turns motor-frame
    velocity and torque into the grip frame.
    """
    if ep.role == Role.Follower:
        g = FollowerGripper(mcu_device=ep.mcu_device)
        pm = GripperPosition(g.get_gripper_config())
    else:
        try:
            # Fills EncoderSample.position on every streamed sample.
            g = LeaderGripper(mcu_device=ep.mcu_device, normalize_position=True)
            pm = g.position_map
        except ProtocolError:
            g, pm = LeaderGripper(mcu_device=ep.mcu_device), None
    if pm is None or not pm.valid:
        log.warning(
            f"{ep.firmware_sn}: travel not calibrated — no 0..1 opening, raw rad only"
        )
    return g, pm


def stream_leader(g, side: str, args, pm) -> list:
    root = f"{side}/gripper"
    normalized = pm is not None and pm.valid

    def on_encoder(s):
        stamp(s.host_time)
        rr.log(f"{root}/encoder_rad", rr.Scalars(s.position_rad))
        if normalized:
            rr.log(f"{root}/position", rr.Scalars(s.position))

    def on_imu(s):
        stamp(s.host_time)
        rr.log(f"{side}/imu/accel", rr.Scalars(np.asarray(s.accel_mps2, dtype=float)))
        rr.log(f"{side}/imu/gyro", rr.Scalars(np.asarray(s.gyro_radps, dtype=float)))

    def on_key(s):
        stamp(s.host_time)
        rr.log(f"{side}/button", rr.TextLog(f"key {s.key_id} -> {s.key_state}"))

    for axis in ("accel", "gyro"):
        rr.log(f"{side}/imu/{axis}", rr.SeriesLines(names=["x", "y", "z"]), static=True)
    subs = [
        (g.encoder, g.encoder.on_data(on_encoder)),
        (g.imu, g.imu.on_data(on_imu)),
        (g.key, g.key.on_event(on_key)),
    ]
    g.start_streaming(imu_hz=args.imu_hz, encoder_hz=args.encoder_hz)
    return subs


def stream_follower(g, side: str, args, pm) -> list:
    """Raw motor status, converted the way GripperObservation does it.

    Velocity and torque are flipped into the grip frame (+ = closing), so these
    plots read the same as gripper_console.py --rerun, which logs the
    controller's observation.
    """
    grip = 1.0 if pm.reverse else -1.0  # GripperPosition::to_grip_frame
    calibrated = pm.valid

    def on_status(s):
        _rerun.log_follower(
            side,
            s.host_time,
            position=pm.to_position(s.actual_pos) if calibrated else None,
            velocity=grip * s.actual_vel,
            torque=grip * s.actual_torque,
            temperature=s.motor_temp_c,
            status=s.status,
        )
        rr.log(f"{side}/gripper/motor_rad", rr.Scalars(s.actual_pos))

    _rerun.init_follower(side)
    subs = [(g.motor, g.motor.on_status(on_status))]
    g.start_streaming(motor_hz=args.motor_hz)
    return subs


def resolve_endpoints(targets: list[str]) -> list:
    if not targets:
        eps = [e for e in scan_grippers() if e.side in (Side.Left, Side.Right)]
        if not eps:
            sys.exit("error: no gripper with a burned firmware SN is plugged in")
        return eps
    return [_target.resolve_target(t)[0] for t in targets]


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "targets",
        nargs="*",
        metavar="left|right|SN",
        help="grippers to show; omit for every gripper plugged in",
    )
    p.add_argument("--imu-hz", type=int, default=100)
    p.add_argument("--encoder-hz", type=int, default=100)
    p.add_argument("--motor-hz", type=int, default=100)
    p.add_argument(
        "--duration", type=float, default=0.0, help="stop after N s (0 = Ctrl-C)"
    )
    _rerun.add_arguments(p)
    args = p.parse_args()

    _rerun.require()
    log.set_level("warn")
    endpoints = resolve_endpoints(args.targets)
    Sensor, tactile = _rerun.tactile_by_side(args, endpoints)

    rr.init("taccap_gripper")
    # Filled as devices open, so the finally block releases exactly what was opened.
    opened: list[tuple] = []  # (rig, gripper, converter, subscriptions)
    try:
        for ep in endpoints:
            side = "left" if ep.side == Side.Left else "right"
            if any(rig.side == side for rig, *_ in opened):
                sys.exit(
                    f"error: two grippers on the {side} side — pass the one you want by SN"
                )
            g, pm = open_gripper(ep)
            rig = _rerun.Rig(side, ep.role, ep.firmware_sn)
            opened.append((rig, g, pm, []))
            print(
                f"[gripper] {side}: {ep.firmware_sn}  {ep.role.name}  fw {g.firmware_version}"
            )
            rig.open(g, args, Sensor, tactile.get(side, []))

        _rerun.connect([rig for rig, *_ in opened], args)
        for rig, g, pm, subs in opened:
            stream = stream_follower if rig.role == Role.Follower else stream_leader
            subs += stream(g, rig.side, args, pm)
            rig.start(args)

        print("streaming — Ctrl-C to stop")
        t0 = time.monotonic()
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for rig, g, _pm, subs in opened:
            for line in rig.close():
                print(line)
            for source, sub in subs:
                source.off(sub)
            g.close()
        rr.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())

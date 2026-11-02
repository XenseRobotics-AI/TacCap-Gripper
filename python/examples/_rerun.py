# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Shared Rerun plumbing for rerun_viewer.py and gripper_console.py --rerun.

Not part of ``xense.taccap``: Rerun is a viewer choice, not an SDK dependency.

What is here:
  - ``add_arguments()`` — the wrist / tactile / output flags both scripts take
  - ``Rig`` — opens one gripper's wrist camera and tactile sensors, starts and
    stops their streams, and releases them
  - ``log_follower()`` — one follower state sample, from whichever source
  - ``connect()`` — open the viewer (or a file, or a remote viewer) with the
    layout from ``blueprint()``

Every stream goes on one timeline, ``host_time``: seconds on the host's
monotonic clock, which is what every SDK sample's ``host_time`` already is.
Rerun's current time is per-thread, so each callback sets it itself (``stamp``).

Entity layout, per side (``left`` / ``right``):

    {side}/gripper/position            0..1, 0 = closed
    {side}/gripper/target              0..1, what the controller is told (console)
    {side}/gripper/{velocity,torque}   grip frame: + = closing
    {side}/gripper/commanded_torque    what the controller asks for (console)
    {side}/gripper/{temperature,status,state}
    {side}/gripper/encoder_rad, {side}/imu/{accel,gyro}, {side}/button  (leader)
    {side}/wrist/image
    {side}/tactile/{finger}/image      + marker_flow (arrows) on the same image
    {side}/tactile/{finger}/depth
    {side}/tactile/{finger}/force_resultant   Fx Fy Fz Mx My Mz
"""

from __future__ import annotations

import os
import sys
import threading
import time

# Read by the Rerun SDK at init: smaller batches = lower latency in the viewer.
os.environ.setdefault("RERUN_FLUSH_TICK_SECS", "0.02")

import _tactile  # noqa: E402
import numpy as np  # noqa: E402
from xense.taccap import Camera, ColorMode, FisheyeUndistorter, Role, log  # noqa: E402

# Not fatal at import: gripper_console.py imports this module for its flags and
# must keep working without Rerun. Anything that logs calls require() first.
try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError:
    rr = rrb = None

TIMELINE = "host_time"
# MotorStatusBit — the masks stay C++-side, see MotorStatusSample.status.
MOTOR_ENABLED, MOTOR_FAULT, MOTOR_STALLED = 0x0001, 0x0002, 0x0004


def require() -> None:
    if rr is None:
        sys.exit(
            "error: rerun is not installed — mamba env update -f environment.yml "
            "(or: uv pip install rerun-sdk)"
        )


def stamp(t: float) -> None:
    rr.set_time(TIMELINE, duration=t)


def log_image(path: str, rgb: np.ndarray, quality: int) -> None:
    img = rr.Image(rgb)
    rr.log(path, img.compress(jpeg_quality=quality) if quality > 0 else img)


def add_arguments(parser) -> None:
    g = parser.add_argument_group(
        "rerun (rerun_viewer.py; gripper_console.py only with --rerun)"
    )
    g.add_argument("--no-wrist", action="store_true", help="skip the wrist camera")
    g.add_argument("--undistort", action="store_true", help="rectify the wrist fisheye")
    g.add_argument(
        "--no-tactile", action="store_true", help="skip the visuotactile sensors"
    )
    g.add_argument(
        "--tactile-image",
        choices=("rectify", "difference"),
        default="rectify",
        help="which tactile image to show (difference highlights contact)",
    )
    g.add_argument(
        "--no-force",
        dest="force",
        action="store_false",
        help="tactile images only: no marker flow, no force, no inference engine",
    )
    g.add_argument("--depth", action="store_true", help="also show the tactile depth")
    g.add_argument(
        "--tactile-hz",
        type=float,
        default=30.0,
        help="tactile frames logged per second (the sensor runs at 120)",
    )
    g.add_argument(
        "--jpeg-quality", type=int, default=80, help="0 = log images uncompressed"
    )
    g.add_argument(
        "--window", type=float, default=10.0, help="seconds shown in the plots"
    )
    out = g.add_mutually_exclusive_group()
    out.add_argument(
        "--save", metavar="FILE.rrd", help="write a recording instead of a viewer"
    )
    out.add_argument(
        "--connect",
        metavar="URL",
        help="stream to a running viewer, e.g. rerun+http://HOST:9876/proxy",
    )


# ---- gripper -----------------------------------------------------------------


def init_follower(side: str) -> None:
    rr.log(
        f"{side}/gripper/status",
        rr.SeriesLines(names=["enabled", "fault", "stalled"]),
        static=True,
    )


def log_follower(
    side: str,
    host_time: float,
    *,
    position: float | None,
    velocity: float,
    torque: float,
    temperature: float,
    status: int,
) -> None:
    """One follower state sample. velocity / torque in the grip frame (+ = closing)."""
    root = f"{side}/gripper"
    stamp(host_time)
    if position is not None:
        rr.log(f"{root}/position", rr.Scalars(position))
    rr.log(f"{root}/velocity", rr.Scalars(velocity))
    rr.log(f"{root}/torque", rr.Scalars(torque))
    rr.log(f"{root}/temperature", rr.Scalars(temperature))
    rr.log(
        f"{root}/status",
        rr.Scalars(
            [
                float(bool(status & b))
                for b in (MOTOR_ENABLED, MOTOR_FAULT, MOTOR_STALLED)
            ]
        ),
    )


# ---- wrist camera ------------------------------------------------------------


def open_wrist(g, role, side: str, args):
    """Find this gripper's XC camera by serial and stream it; None if absent.

    Reads the fisheye calibration over the MCU link when --undistort is on, so
    call it before a controller takes the bus.
    """
    from wrist_camera import scan_video_devices  # the XC serial grammar lives there

    role = "follower" if role == Role.Follower else "leader"
    cams = [
        d
        for d in scan_video_devices()["wrist"]
        if d["side"] == side and d["role"] == role
    ]
    if not cams:
        log.warning(
            f"{side}: no {role} XC wrist camera under /dev/v4l/by-id — skipping it"
        )
        return None
    cam = Camera(
        cams[0]["path"], width=640, height=480, fps=30.0, color_mode=ColorMode.RGB
    )
    if args.undistort:
        cal, is_reference, reason = g.calibration.resolve_fisheye()
        if is_reference:
            log.warning(
                f"{side}: wrist undistortion uses the SDK reference intrinsics ({reason})"
            )
        cam.set_undistorter(FisheyeUndistorter(cal))
    print(f"[wrist]   {side}: {cams[0]['serial']}")
    return cam


# ---- tactile (xensesdk) ------------------------------------------------------


class TactileStream:
    """One visuotactile sensor, read on its own thread and logged to Rerun.

    selectSensorInfo() blocks until the next processed frame, so the loop needs
    no pacing of its own. The sensor runs at 120 fps; every frame is read (so
    ``frames`` is the real rate) but only --tactile-hz of them are logged —
    at the full rate, two sensors fill a GB of viewer memory in seconds.
    """

    def __init__(self, Sensor, info, side: str, args):
        self.info, self.args = info, args
        self.root = f"{side}/tactile/{info.finger}"
        self.names = [args.tactile_image]
        if args.force:
            self.names += ["marker", "marker_init", "force_resultant"]
        if args.depth:
            self.names.append("depth")
        # TimeStamp rides last, so zip() with self.names never sees it.
        self.types = _tactile.output_types(Sensor, self.names) + [
            Sensor.OutputType.TimeStamp
        ]
        self.sensor = _tactile.open_sensor(
            Sensor, info.serial, disable_infer=not _tactile.needs_inference(self.names)
        )
        self.viz_shape = _tactile.viz_marker_shape(self.sensor)
        self.frames = 0
        self._next_log = 0.0
        self._stop = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name=f"tactile-{info.serial}", daemon=True
        )

    def start(self):
        rr.log(
            f"{self.root}/force_resultant",
            rr.SeriesLines(names=["Fx", "Fy", "Fz", "Mx", "My", "Mz"]),
            static=True,
        )
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread.is_alive():  # never started when opening failed midway
            self.thread.join(timeout=2.0)
        _tactile.release_sensor(self.sensor)

    def _run(self):
        while not self._stop.is_set():
            try:
                out = self.sensor.selectSensorInfo(*self.types)
            except Exception as e:
                log.warning(f"{self.info.serial}: read failed ({e})")
                time.sleep(0.1)
                continue
            self.frames += 1
            now = time.monotonic()
            if now < self._next_log:
                continue
            self._next_log = now + 1.0 / self.args.tactile_hz
            # xensesdk's TimeStamp is wall clock, taken ~3 ms before the call
            # returns. Moved onto the monotonic clock per frame, so an NTP step
            # cannot shift the tactile streams against the others.
            stamp(now - (time.time() - float(np.asarray(out[-1]).reshape(-1)[0])))
            frame = dict(zip(self.names, out))
            # Same orientation and channel order as the LeRobot dataset — see
            # the display conventions in _tactile.py.
            image = _tactile.to_display(frame[self.args.tactile_image])
            log_image(
                f"{self.root}/image", _tactile.to_rgb(image), self.args.jpeg_quality
            )
            if "marker" in frame:
                # The SDK's own marker-flow view (Sensor.drawMarkerMove): rest
                # -> current position, 1:1, on its thinned grid, same green.
                starts, ends = _tactile.marker_flow(
                    frame["marker_init"], frame["marker"], self.viz_shape
                )
                rr.log(
                    f"{self.root}/marker_flow",
                    rr.Arrows2D(
                        origins=starts,
                        vectors=ends - starts,
                        colors=[3, 253, 3],
                        radii=1.0,
                    ),
                )
                rr.log(
                    f"{self.root}/force_resultant",
                    rr.Scalars(np.asarray(frame["force_resultant"], dtype=float)),
                )
            if "depth" in frame:
                depth = np.asarray(frame["depth"], dtype=np.float32)[::4, ::4]
                rr.log(
                    f"{self.root}/depth",
                    rr.DepthImage(
                        _tactile.to_display(depth), meter=1000.0, colormap="viridis"
                    ),
                )


# ---- one gripper's cameras ---------------------------------------------------


class Rig:
    """The wrist camera and tactile sensors of one gripper.

    Opening is split from starting so that everything slow (sensor open, the
    viewer spawn) is done before any stream logs, and so a caller can open the
    cameras before a controller takes the MCU bus.
    """

    def __init__(self, side: str, role, firmware_sn: str):
        self.side, self.role, self.firmware_sn = side, role, firmware_sn
        self.wrist = None
        self.tactile: list[TactileStream] = []
        self._t0 = None

    def open(self, g, args, Sensor, tactile_infos) -> None:
        """Open the cameras. Sensor is None when tactile is off."""
        if not args.no_wrist:
            self.wrist = open_wrist(g, self.role, self.side, args)
        for info in sorted(tactile_infos, key=lambda t: t.finger):
            print(
                f"[tactile] {self.side}: {info.serial} ({info.finger} finger) "
                "— opening, keep the gel untouched"
            )
            # Sequential on purpose: see _tactile.open_sensor().
            self.tactile.append(TactileStream(Sensor, info, self.side, args))
        if Sensor is not None and not self.tactile:
            log.warning(
                f"{self.side}: no visuotactile sensor shares this gripper's USB hub"
            )

    def start(self, args) -> None:
        self._t0 = time.monotonic()
        if self.wrist is not None:
            path, quality = f"{self.side}/wrist/image", args.jpeg_quality

            def on_frame(f):
                stamp(f.host_time)
                log_image(path, f.image, quality)

            self.wrist.start(on_frame)
        for t in self.tactile:
            t.start()

    def close(self) -> list[str]:
        """Stop and release everything; returns the measured rates to print."""
        elapsed = time.monotonic() - self._t0 if self._t0 is not None else 0.0
        report = []
        for t in self.tactile:
            t.stop()
            if elapsed > 0:
                report.append(
                    f"[rate] {self.side} tactile {t.info.serial}: "
                    f"{t.frames / elapsed:.1f} Hz"
                )
        if self.wrist is not None:
            self.wrist.stop()
            if elapsed > 0:
                report.append(
                    f"[rate] {self.side} wrist: {self.wrist.actual_fps:.1f} Hz, "
                    f"{self.wrist.dropped_frames} dropped"
                )
        return report


def tactile_by_side(args, endpoints):
    """``(Sensor, {side: [TactileSensor]})``; Sensor is None with --no-tactile.

    ``endpoints`` are the grippers being shown — see
    ``_tactile.scan_tactile_sensors()`` for why they are passed in, not rescanned.
    """
    if args.no_tactile:
        return None, {}
    Sensor = _tactile.import_sensor_class()
    by_side = {}
    for t in _tactile.scan_tactile_sensors(endpoints):
        by_side.setdefault(t.side, []).append(t)
    return Sensor, by_side


# ---- viewer ------------------------------------------------------------------


def blueprint(rigs: list[Rig], args):
    """One column per gripper: cameras on top, time series below."""
    recent = rrb.VisibleTimeRange(
        TIMELINE,
        start=rrb.TimeRangeBoundary.cursor_relative(seconds=-args.window),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )

    def series(name, *paths):
        return rrb.TimeSeriesView(
            origin="/", contents=list(paths), name=name, time_ranges=recent
        )

    columns = []
    for rig in rigs:
        side = rig.side
        g = f"{side}/gripper"
        images = []
        if rig.wrist is not None:
            images.append(
                rrb.Spatial2DView(origin=f"{side}/wrist", name=f"{side} wrist")
            )
        for t in rig.tactile:
            images.append(
                rrb.Spatial2DView(
                    origin=t.root,
                    contents=[f"{t.root}/image", f"{t.root}/marker_flow"],
                    name=f"{side} tactile {t.info.finger}",
                )
            )
            if args.depth:
                images.append(
                    rrb.Spatial2DView(
                        origin=t.root,
                        contents=f"{t.root}/depth",
                        name=f"{side} depth {t.info.finger}",
                    )
                )
        plots = [series(f"{side} opening (0 = closed)", f"{g}/position", f"{g}/target")]
        if rig.role == Role.Follower:
            plots += [
                series(
                    f"{side} torque (Nm, + = closing)",
                    f"{g}/torque",
                    f"{g}/commanded_torque",
                ),
                series(f"{side} velocity (rad/s, + = closing)", f"{g}/velocity"),
                series(f"{side} motor status", f"{g}/status"),
            ]
        else:
            plots += [
                series(f"{side} accel (m/s²)", f"{side}/imu/accel"),
                series(f"{side} gyro (rad/s)", f"{side}/imu/gyro"),
            ]
        if args.force:
            plots += [
                series(f"{side} {t.info.finger} resultant", f"{t.root}/force_resultant")
                for t in rig.tactile
            ]
        parts = ([rrb.Horizontal(*images)] if images else []) + [rrb.Grid(*plots)]
        columns.append(
            rrb.Vertical(*parts, row_shares=[3, 2] if images else None, name=side)
        )
    return rrb.Blueprint(rrb.Horizontal(*columns), collapse_panels=True)


def connect(rigs: list[Rig], args) -> None:
    """Send everything logged from here on to a viewer, a remote viewer or a file."""
    bp = blueprint(rigs, args)
    if args.save:
        rr.save(args.save, default_blueprint=bp)
    elif args.connect:
        rr.connect_grpc(args.connect, default_blueprint=bp)
    else:
        rr.spawn(memory_limit="25%", default_blueprint=bp)
    for rig in rigs:
        rr.log(
            f"{rig.side}/info",
            rr.TextDocument(f"{rig.firmware_sn}  {rig.role.name}"),
            static=True,
        )

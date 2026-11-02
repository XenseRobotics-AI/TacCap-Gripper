#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Read the gripper's visuotactile (GSPS / OG) sensors through ``xensesdk``.

Each TacCap-Gripper jaw carries a visuotactile sensor: a camera looking at the
back of a marked gel. The **xensesdk** wheel turns its frames into a rectified
image, a contact-difference image, a depth map and a force field. This SDK
(`xense.taccap`) does not read these sensors; this script shows the two working
side by side.

    install         comes with environment.yml (uv pip install xensesdk)
    import          from xensesdk import Sensor

Which sensor is which:

    side      the gripper it is mounted on — from the gripper sharing its USB
              hub, so the gripper must be plugged in for `left` / `right` to work
    finger    which jaw — the GSPS serial's last digit, odd = left, even = right

The API in three calls (what this script does for every sensor):

    sensor = Sensor.create("GSPS01A31Z0049", disable_infer=False)
    rectify, force, fr = sensor.selectSensorInfo(
        Sensor.OutputType.Rectify,          # (700, 400, 3) **BGR** uint8
        Sensor.OutputType.Force,            # (35, 20, 3) force field
        Sensor.OutputType.ForceResultant,   # (6,) Fx Fy Fz Mx My Mz
    )                                       # one result per type, same order
    sensor.release()

Before showing an image anywhere but OpenCV, flip it to RGB (`img[..., ::-1]`);
the LeRobot integration also transposes it to (400, 700), and so do these
viewers, so a frame looks the same here as in a recorded dataset.

selectSensorInfo() blocks until the next processed frame, so its call rate
is the sensor rate — 120 fps on the GSPS01, a little less per sensor when one
loop reads two. `--outputs` picks the types:

    rectify          rectified gel image                       no inference
    difference       contact vs. the no-contact reference      no inference
    depth            indentation map, mm                       inference
    marker           (35, 20, 2) tangential marker shift       inference
    force            (35, 20, 3) force distribution            inference
    force_resultant  (6,) net force / torque                   inference

When only the first two are asked for, the inference engine is not loaded
(`disable_infer=True`), so the sensor opens faster and uses no GPU.

The reference frame (what "no contact" looks like) is taken when the sensor
opens. **Open it with nothing touching the gel**, or press `c` later to
take the reference again (`sensor.calibrateSensor()`), again with nothing
touching it.

Usage:
    python python/examples/tactile_sensor.py --list
    python python/examples/tactile_sensor.py left                  # both fingers
    python python/examples/tactile_sensor.py left --finger right
    python python/examples/tactile_sensor.py GSPS01A31Z0049
    python python/examples/tactile_sensor.py right --outputs rectify,difference
    python python/examples/tactile_sensor.py left --no-display --duration 10

The window shows each sensor as one column, outputs stacked, in the
orientation the LeRobot integration records: the SDK's portrait (700, 400)
image transposed to (400, 700). Force arrows sit on the markers they belong to.

Keys (viewer window):
    q / ESC   quit        c   take the no-contact reference again
    s         save a PNG of the current view

To see the tactile sensors next to the gripper state and the wrist camera,
on one timeline, use rerun_viewer.py.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import _tactile
import numpy as np

# Gel indentation rarely exceeds a couple of millimetres; a fixed scale keeps
# the depth colours comparable between frames (auto-scaling would blow up the
# noise when nothing is touching).
DEFAULT_DEPTH_MAX_MM = 2.0


def render(cv2, name: str, data, size: tuple[int, int], args) -> np.ndarray:
    """One output -> a BGR tile of ``size`` (w, h), in the LeRobot orientation.

    Images stay BGR (that is what OpenCV wants); everything is transposed to
    landscape the way the dataset stores it — see _tactile.py.
    """
    w, h = size
    if name in ("rectify", "difference"):
        tile = _tactile.to_display(data)
    elif name == "depth":
        d = np.clip(np.asarray(data, dtype=np.float32) / args.depth_max_mm, 0.0, 1.0)
        d = _tactile.to_display((d * 255).astype(np.uint8))
        tile = cv2.applyColorMap(d, cv2.COLORMAP_VIRIDIS)
    elif name in ("force", "marker"):
        tile = np.zeros((h, w, 3), np.uint8)
        if name == "force":
            draw_field(cv2, tile, data, args.force_scale * w / 700)
        else:  # positions, not displacements: show where each marker is
            draw_markers(cv2, tile, data)
    else:  # force_resultant has no image; it is printed onto the first tile
        return None
    if tile.shape[:2] != (h, w):
        tile = cv2.resize(tile, (w, h))
    return label(cv2, tile, name)


def draw_field(cv2, canvas: np.ndarray, field, scale: float) -> None:
    """In-plane force as arrows on the markers; the dot size is the normal part."""
    origins, vectors = _tactile.field_arrows(field, canvas.shape[:2], scale)
    normal = np.abs(np.asarray(field, dtype=np.float32)[..., 2]).reshape(-1)
    for (x0, y0), (dx, dy), fz in zip(origins, vectors, normal):
        p0 = (int(x0), int(y0))
        cv2.arrowedLine(
            canvas, p0, (int(x0 + dx), int(y0 + dy)), (80, 220, 80), 1, tipLength=0.3
        )
        r = int(min(8.0, fz * scale * 0.5))
        if r > 0:
            cv2.circle(canvas, p0, r, (60, 120, 255), -1)


def draw_markers(cv2, canvas: np.ndarray, markers) -> None:
    """Marker2D: (x, y) positions in the portrait image, transposed to fit."""
    h, w = canvas.shape[:2]
    pts = np.asarray(markers, dtype=np.float32).reshape(-1, 2)
    for x, y in pts * (w / 700.0):  # portrait 700 tall -> display width w
        cv2.circle(canvas, (int(y), int(x)), 2, (80, 220, 80), -1)


def label(cv2, tile: np.ndarray, text: str, row: int = 0) -> np.ndarray:
    tile = np.ascontiguousarray(tile)
    org = (8, 20 + 18 * row)
    cv2.putText(
        tile, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA
    )
    cv2.putText(
        tile, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
    )
    return tile


def format_resultant(fr) -> str:
    fx, fy, fz, mx, my, mz = (float(v) for v in np.asarray(fr).reshape(-1)[:6])
    return f"F=({fx:+6.2f},{fy:+6.2f},{fz:+6.2f})  M=({mx:+6.3f},{my:+6.3f},{mz:+6.3f})"


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "target",
        nargs="?",
        metavar="left|right|GSPS-SN",
        help="'left' / 'right' = both sensors on that gripper (the gripper must "
        "be plugged in), or one GSPS serial. Omit to open every sensor found.",
    )
    p.add_argument("--list", action="store_true", help="list the sensors and exit")
    p.add_argument("--finger", choices=("left", "right"), help="only this jaw's sensor")
    p.add_argument(
        "--outputs",
        default="rectify,difference,depth,force,force_resultant",
        help=f"comma-separated, from: {', '.join(_tactile.OUTPUTS)}",
    )
    p.add_argument(
        "--depth-max-mm",
        type=float,
        default=DEFAULT_DEPTH_MAX_MM,
        help="depth that maps to the top of the colour scale",
    )
    p.add_argument(
        "--force-scale",
        type=float,
        default=20.0,
        help="pixels per force unit for the force arrows",
    )
    p.add_argument("--tile-width", type=int, default=420, help="viewer tile width, px")
    p.add_argument("--no-display", action="store_true", help="print, no window")
    p.add_argument(
        "--duration", type=float, default=0.0, help="stop after N s (0 = Ctrl-C)"
    )
    p.add_argument("--save-dir", default="tactile_shots", help="where `s` saves a PNG")
    args = p.parse_args()

    if args.list:
        _tactile.print_sensors(_tactile.scan_tactile_sensors())
        return 0

    names = [n.strip() for n in args.outputs.split(",") if n.strip()]
    Sensor = _tactile.import_sensor_class()
    types = _tactile.output_types(Sensor, names)
    disable_infer = not _tactile.needs_inference(names)
    targets = _tactile.select_tactile(args.target, args.finger)

    sensors = []  # (info, Sensor)
    try:
        for info in targets:  # sequentially — see _tactile.open_sensor()
            print(
                f"[open] {info.serial}  gripper={info.side or '?'}  finger={info.finger}"
                f"  (disable_infer={disable_infer})"
            )
            t0 = time.monotonic()
            sensors.append(
                (
                    info,
                    _tactile.open_sensor(
                        Sensor, info.serial, disable_infer=disable_infer
                    ),
                )
            )
            print(f"       ready in {time.monotonic() - t0:.1f} s")
        return run(args, names, types, sensors)
    except KeyboardInterrupt:
        return 0
    finally:
        for info, sensor in sensors:
            _tactile.release_sensor(sensor)


def run(args, names, types, sensors) -> int:
    cv2 = None
    if not args.no_display:
        import cv2
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    counts = {info.serial: 0 for info, _ in sensors}
    t_start = last_print = time.monotonic()
    shots = 0

    while deadline is None or time.monotonic() < deadline:
        columns, lines = [], []
        for info, sensor in sensors:
            out = sensor.selectSensorInfo(*types)
            out = out if isinstance(out, tuple) else (out,)
            frame = dict(zip(names, out))
            counts[info.serial] += 1
            if "force_resultant" in frame:
                lines.append(
                    f"{info.serial} ({info.finger}): {format_resultant(frame['force_resultant'])}"
                )
            if cv2 is not None:
                columns.append(view_column(cv2, args, info, frame))

        now = time.monotonic()
        if cv2 is None:
            if now - last_print >= 0.5:
                last_print = now
                rates = "  ".join(
                    f"{sn}: {n / (now - t_start):.1f} Hz" for sn, n in counts.items()
                )
                print(rates + ("\n  " + "\n  ".join(lines) if lines else ""))
            continue

        canvas = columns[0] if len(columns) == 1 else side_by_side(columns)
        cv2.imshow("tactile — q quit, c re-reference, s save", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("c"):
            print(
                "[reference] taking a new no-contact reference — keep the gel untouched"
            )
            for _info, sensor in sensors:
                sensor.calibrateSensor()
        if key == ord("s"):
            os.makedirs(args.save_dir, exist_ok=True)
            path = os.path.join(args.save_dir, f"tactile_{shots:04d}.png")
            cv2.imwrite(path, canvas)
            shots += 1
            print(f"  saved {path}")

    if cv2 is not None:
        cv2.destroyAllWindows()
    elapsed = time.monotonic() - t_start
    for sn, n in counts.items():
        print(
            f"[rate] {sn}: {n} frames in {elapsed:.1f} s = {n / max(elapsed, 1e-9):.1f} Hz"
        )
    return 0


def view_column(cv2, args, info, frame: dict) -> np.ndarray:
    """One sensor's outputs stacked top to bottom, at the gel's aspect ratio."""
    ref = next(
        (frame[n] for n in ("rectify", "difference", "depth") if n in frame), None
    )
    gh, gw = ref.shape[:2] if ref is not None else (700, 400)  # portrait
    w = args.tile_width
    size = (w, max(1, int(round(w * gw / gh))))  # landscape after the transpose
    tiles = [
        t for n, d in frame.items() if (t := render(cv2, n, d, size, args)) is not None
    ]
    if not tiles:
        tiles = [np.zeros((size[1], w, 3), np.uint8)]
    tiles[0] = label(cv2, tiles[0], f"{info.serial}  {info.finger} finger", row=1)
    if "force_resultant" in frame:
        fr = np.asarray(frame["force_resultant"]).reshape(-1)
        tiles[0] = label(cv2, tiles[0], f"Fz {float(fr[2]):+.2f}", row=2)
    return np.vstack(tiles)


def side_by_side(columns: list[np.ndarray]) -> np.ndarray:
    height = max(c.shape[0] for c in columns)
    padded = [np.pad(c, ((0, height - c.shape[0]), (0, 4), (0, 0))) for c in columns]
    return np.hstack(padded)


if __name__ == "__main__":
    sys.exit(main())

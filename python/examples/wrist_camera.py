#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Stand-alone wrist-camera viewer, with fisheye undistortion on a switch.

Pick the gripper the same way every other example does — `left` / `right`
(or an explicit serial):

    python python/examples/wrist_camera.py left
    python python/examples/wrist_camera.py right --undistort
    python python/examples/wrist_camera.py XCA24Z0003m --compare

**This opens the XC wrist camera only.** Xense visuotactile (OG/GSPS) sensors
sit on `/dev/video*` right beside it, and they are NOT usable here: their
capture and rectification live in the `xensesdk` wheel, not in this SDK.
Selection is therefore by serial grammar, never by device index — a GSPS serial
or a raw `/dev/videoN` path is refused rather than opened.

    Wrist camera   XC<batch><line><seq><m|s>    e.g. XCA24Z0003m
    Visuotactile   GSPS01<batch><line><seq>     e.g. GSPS01A24Z0001   (refused here)

Side comes from the sequence's last digit — odd = left, even = right — the same
rule the firmware SN uses, so a wrist camera resolves to the same side as the
gripper it is mounted on. The `m` / `s` patch letter is leader / follower.

The wrist UVC device is normally owned by an external camera service, so
`LeaderGripper.open()` does not touch it (`open_cameras` defaults to False).
This script opens it directly as a bare `Camera` — the same C++ class the
gripper would use — and installs a `FisheyeUndistorter` on it, so rectified
frames come out of the capture path itself rather than being remapped at each
call site.

Where the intrinsics come from (in order):

  --from-npz FILE   an OpenCV-style .npz holding K (3x3) and D (4,), i.e. what
                    `fisheye_cal.py set-fisheye --from-npz` writes to flash.
  --no-mcu          skip the gripper entirely and use FISHEYE_FALLBACK_CAL,
                    the SDK's shared reference calibration.
  (default)         ask the gripper **on the same side** over the MCU link via
                    `Calibration.resolve_fisheye()`, which applies the
                    read-and-fall-back policy for you and reports whether the
                    unit supplied its own numbers or the reference ones stood
                    in. Reference intrinsics are approximate — lens placement
                    varies per assembly — so this warns loudly when it uses
                    them, and so does this script.

Undistortion switch:

    (default)         raw fisheye frames — the SDK's own default, where neither
                      a bare `Camera` nor a gripper (`undistort_wrist=False`)
                      rectifies anything until asked to
    --undistort       rectify inside the capture path
    --compare         raw | rectified side by side
    u                 cycle those three live, in the viewer window

Note `undistort_wrist=True` does not "degrade to raw" when a unit has no stored
calibration: undistortion is always installed, with the SDK's reference
intrinsics standing in and a warning. Off-by-default is about the switch, not
about whether a switched-on path might silently skip the remap.

Note the undistorter is built for the calibrated 640x480 only: the firmware
record carries no image size, so rescaling the intrinsics would be a guess and
is refused rather than silently wrong. Ask for another size and undistortion
is unavailable at that size (the script says so and stays on raw frames).

Usage:
    # what is plugged in, and which of it this script can open
    python python/examples/wrist_camera.py --list

    python python/examples/wrist_camera.py left                # raw fisheye
    python python/examples/wrist_camera.py left --undistort    # rectified
    python python/examples/wrist_camera.py left --compare --balance 1.0
    python python/examples/wrist_camera.py left --undistort --no-mcu

    # headless: no window, just rate stats (plus one PNG/s with --save-dir)
    python python/examples/wrist_camera.py left --no-display --duration 10

Keys (viewer window):
    q / ESC   quit            u   cycle raw / rectified / compare
    s         save a PNG      [ ] balance -/+ 0.1 (rebuilds the remap tables)

This uses the synchronous `Camera.read()` loop so the window lives on the main
thread. For a callback-driven stream instead:

    cam.start(lambda f: ...)   # background capture thread
    cam.stop()

Both paths honour the installed undistorter identically.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import time

import numpy as np
from xense.taccap import (
    FISHEYE_FALLBACK_CAL,
    Camera,
    CameraFisheyeCal,
    ColorMode,
    FisheyeUndistorter,
    FollowerGripper,
    LeaderGripper,
    ProtocolError,
    Role,
    Side,
    log,
    scan_grippers,
)

# The one resolution the firmware calibration record is defined at.
CALIB_W, CALIB_H = 640, 480

MODE_RAW = "raw"
MODE_RECT = "rectified"
MODE_COMPARE = "compare"
MODE_CYCLE = (MODE_RAW, MODE_RECT, MODE_COMPARE)

_TTY = sys.stdout.isatty()


def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s


def _bold(s):
    return _c("1", s)


def _yellow(s):
    return _c("33", s)


# ---- device selection -------------------------------------------------------
#
# Serial grammar, shared with the firmware SN rule (see discovery.hpp) and with
# the sibling lerobot integration's serial_discovery.py — keep the three in step.
#
#   wrist camera   XC<batch><line><seq><m|s>   XCA24Z0003m
#   visuotactile   GSPS01<batch><line><seq>    GSPS01A24Z0001
#
# The /dev/v4l/by-id name embeds the serial, which is why selection goes through
# it rather than through /dev/videoN: the index moves with plug order, and the
# node right next to the wrist camera is usually a visuotactile sensor.
BYID_DIR = "/dev/v4l/by-id"
CAMERA_SN_RE = re.compile(r"(XC[A-Z]\d{2}[ZA]\d{4}[ms])")
TACTILE_SN_RE = re.compile(r"(GSPS01[A-Z]\d{2}[ZA]\d{4})")

SIDES = ("left", "right")
PATCH_ROLE = {"m": "leader", "s": "follower"}


def side_of_serial(sn: str) -> str:
    """Odd trailing sequence digit → left, even → right (单左双右)."""
    seq = sn[-5:-1] if sn[-1] in PATCH_ROLE else sn[-4:]
    return "left" if int(seq[-1]) % 2 == 1 else "right"


def role_of_serial(sn: str) -> str:
    return PATCH_ROLE.get(sn[-1], "unknown")


def scan_video_devices() -> dict[str, list[dict]]:
    """Classify the /dev/v4l/by-id capture nodes into wrist / tactile / other."""
    out = {"wrist": [], "tactile": [], "other": []}
    for path in sorted(glob.glob(f"{BYID_DIR}/*-video-index0")):
        cam = CAMERA_SN_RE.search(path)
        tac = TACTILE_SN_RE.search(path)
        if cam:
            sn = cam.group(1)
            out["wrist"].append(
                {
                    "serial": sn,
                    "path": path,
                    "side": side_of_serial(sn),
                    "role": role_of_serial(sn),
                }
            )
        elif tac:
            out["tactile"].append(
                {
                    "serial": tac.group(1),
                    "path": path,
                    "side": side_of_serial(tac.group(1)),
                }
            )
        else:
            out["other"].append({"serial": "", "path": path})
    return out


def print_devices() -> None:
    devs = scan_video_devices()
    print(_bold("\nXC wrist cameras — what this script can open"))
    if devs["wrist"]:
        for d in devs["wrist"]:
            print(f"  {d['serial']:<14} {d['side']:<5} {d['role']:<9} {d['path']}")
    else:
        print("  " + _yellow("none found") + f" — no XC serial under {BYID_DIR}/")

    if devs["tactile"]:
        print(_bold("\nGSPS visuotactile sensors — NOT openable here"))
        for d in devs["tactile"]:
            print(f"  {d['serial']:<14} {d['side']:<5} {'':<9} {d['path']}")
        print(
            "  这些走 xensesdk,不是本 SDK: Sensor.create(<serial>) — 见 docs/USAGE.md"
        )

    if devs["other"]:
        print(_bold("\nOther V4L2 capture nodes (not Xense devices)"))
        for d in devs["other"]:
            print(f"  {'':<14} {'':<5} {'':<9} {d['path']}")
    print()


def resolve_wrist_camera(target: str) -> dict:
    """Resolve `left` / `right` / an explicit XC serial to one wrist camera.

    Refuses anything that is not an XC device — a GSPS serial and a raw
    /dev/videoN path are both rejected with an explanation rather than opened,
    because pointing this script at a visuotactile sensor is the one mistake
    the device layout invites.
    """
    key = target.strip()
    devs = scan_video_devices()

    if TACTILE_SN_RE.search(key):
        raise SystemExit(
            f"{key} 是视触觉(GSPS)传感器,不是腕相机。它的采集与矫正在 xensesdk 里,"
            "不走本 SDK —— 见 docs/USAGE.md 的触觉一节。"
        )
    if key.startswith("/dev/"):
        raise SystemExit(
            f"不接受设备路径({key})。/dev/videoN 的编号随插拔顺序变,而且紧挨着腕相机的"
            "那个节点通常是视触觉传感器 —— 请用 left / right 或 XC 序列号。"
        )

    if key.lower() in SIDES:
        side = key.lower()
        matches = [d for d in devs["wrist"] if d["side"] == side]
        if not matches:
            print_devices()
            raise SystemExit(f"没有找到 {side} 侧的 XC 腕相机(见上表)。")
        if len(matches) > 1:
            names = ", ".join(f"{d['serial']}({d['role']})" for d in matches)
            raise SystemExit(
                f"{side} 侧有 {len(matches)} 个腕相机:{names}。"
                "leader 和 follower 各带一个时请直接给序列号。"
            )
        return matches[0]

    if CAMERA_SN_RE.fullmatch(key):
        for d in devs["wrist"]:
            if d["serial"] == key:
                return d
        found = ", ".join(d["serial"] for d in devs["wrist"]) or "无"
        raise SystemExit(f"序列号 {key} 没插在这台机器上。已连接的腕相机:{found}")

    raise SystemExit(
        f"无法识别的目标 {target!r} —— 用 left / right,或形如 XCA24Z0003m 的腕相机序列号"
        "(--list 可以列出连着的设备)。"
    )


# ---- calibration source -----------------------------------------------------


def cal_from_npz(path: str) -> CameraFisheyeCal:
    data = np.load(path)
    try:
        K = np.asarray(data["K"], dtype=float).reshape(3, 3)
        D = np.asarray(data["D"], dtype=float).reshape(-1)
    except KeyError as e:
        raise SystemExit(
            f"{path}: expected arrays named 'K' and 'D', missing {e}"
        ) from None
    if D.size < 4:
        raise SystemExit(f"{path}: D has {D.size} coefficients, fisheye needs 4")
    return CameraFisheyeCal(
        fx=float(K[0, 0]),
        fy=float(K[1, 1]),
        cx=float(K[0, 2]),
        cy=float(K[1, 2]),
        k1=float(D[0]),
        k2=float(D[1]),
        k3=float(D[2]),
        k4=float(D[3]),
    )


def cal_from_gripper(cam: dict) -> tuple[CameraFisheyeCal, str]:
    """Read the intrinsics off the MCU of the gripper on the camera's side.

    The wrist camera's own serial already says which side (and which role) it
    belongs to, so there is nothing extra to select here — that is the point of
    keeping one `left` / `right` selector for the whole rig.

    This deliberately does NOT use `_target.resolve_target()`, even though both
    end up scanning for a gripper by side. Two differences make the shared
    helper the wrong tool: it has no role preference, and it reports failure
    with `sys.exit()`. Here the side is already known (it came from the camera,
    not from the user), the role is a preference worth expressing, and failure
    must stay a catchable exception — the caller degrades to the reference
    intrinsics rather than aborting, because the viewer is useful without the
    MCU link. Do not "unify" these two without changing that.
    """
    grippers = scan_grippers()
    if not grippers:
        raise RuntimeError("no TacCap gripper found")

    want_side = Side.Left if cam["side"] == "left" else Side.Right
    want_role = {"leader": Role.Leader, "follower": Role.Follower}.get(cam["role"])
    matches = [g for g in grippers if g.side == want_side]
    if want_role is not None and any(g.role == want_role for g in matches):
        matches = [g for g in matches if g.role == want_role]
    if not matches:
        found = ", ".join(f"{g.firmware_sn or '<no SN>'}" for g in grippers) or "无"
        raise RuntimeError(
            f"no {cam['side']} gripper on the bus to read the calibration from "
            f"(connected: {found})"
        )
    eps = matches[0]

    # The fisheye record lives on both roles; the class only decides which
    # command surface we get, so default to leader when the SN can't say.
    cls = FollowerGripper if eps.role == Role.Follower else LeaderGripper
    with cls(mcu_device=eps.mcu_device) as g:
        # resolve_fisheye() — not read_fisheye() — because an uncalibrated unit
        # answers with an all-zero record rather than a NACK, and remapping with
        # fx = fy = 0 yields a uniformly black frame with nothing raised.
        cal, is_reference, reason = g.calibration.resolve_fisheye()
    if is_reference:
        return cal, f"SDK reference intrinsics ({reason})"
    return cal, f"read from {eps.firmware_sn or eps.mcu_device}"


def resolve_calibration(args, cam: dict) -> tuple[CameraFisheyeCal, str, bool]:
    """Returns (cal, provenance, is_reference)."""
    if args.from_npz:
        return cal_from_npz(args.from_npz), f"loaded from {args.from_npz}", False
    if args.no_mcu:
        return (FISHEYE_FALLBACK_CAL, "SDK reference intrinsics (--no-mcu)", True)
    try:
        cal, provenance = cal_from_gripper(cam)
    except (RuntimeError, ProtocolError, OSError) as e:
        # The wrist camera is usable without the MCU link, so a failure here
        # degrades to the reference calibration rather than aborting — same
        # policy the SDK applies internally. It is approximate, hence the shout.
        log.warning(
            f"could not read the calibration from the gripper ({e}); "
            "falling back to the SDK reference intrinsics"
        )
        return FISHEYE_FALLBACK_CAL, f"SDK reference intrinsics ({e})", True
    return cal, provenance, provenance.startswith("SDK reference")


# ---- rendering --------------------------------------------------------------


def to_bgr(image: np.ndarray, color_mode) -> np.ndarray:
    """cv2 wants BGR; the camera hands out whatever --color-mode asked for."""
    return image[:, :, ::-1] if color_mode == ColorMode.RGB else image


def save_png(
    cv2,
    canvas: np.ndarray,
    save_dir: str,
    mode: str,
    index: int,
    announce: bool = False,
) -> int:
    """Write one PNG; returns 1 so callers can keep a count."""
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, f"wrist_{mode}_{index:06d}.png")
    cv2.imwrite(path, canvas)
    if announce:
        print(f"  saved {path}")
    return 1


def annotate(cv2, canvas: np.ndarray, lines: list[str]) -> np.ndarray:
    canvas = np.ascontiguousarray(canvas)
    for i, text in enumerate(lines):
        org = (8, 20 + 18 * i)
        cv2.putText(
            canvas, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA
        )
        cv2.putText(
            canvas,
            text,
            org,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return canvas


# ---- main -------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "target",
        nargs="?",
        metavar="left|right|XC-SN",
        help="Which wrist camera to open: 'left' / 'right' (resolved "
        "by the serial's sequence parity, same rule as the "
        "gripper SN), or an explicit XC serial such as "
        "XCA24Z0003m. Visuotactile (GSPS) sensors are refused — "
        "they belong to xensesdk, not to this SDK.",
    )
    p.add_argument(
        "--list",
        action="store_true",
        help="List the connected Xense video devices and exit.",
    )

    p.add_argument(
        "--undistort",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Rectify frames in the capture path. Off by default, "
        "matching the SDK itself: neither a bare Camera nor a "
        "gripper (undistort_wrist=False) rectifies unless asked.",
    )
    p.add_argument(
        "--compare",
        action="store_true",
        help="Start showing raw | rectified side by side.",
    )
    p.add_argument(
        "--balance",
        type=float,
        default=0.0,
        help="0 = calibrated focal length (natural view, default); "
        "1 = 0.70x focal length for the widest field of view, "
        "with more black border. Clamped to [0,1].",
    )

    src = p.add_argument_group("calibration source")
    src.add_argument(
        "--from-npz",
        metavar="FILE",
        default=None,
        help="Read K (3x3) and D (4,) from an OpenCV .npz instead of from the gripper.",
    )
    src.add_argument(
        "--no-mcu",
        action="store_true",
        help="Do not talk to a gripper; use the SDK reference "
        "intrinsics (approximate — see --help).",
    )

    cam = p.add_argument_group("capture")
    cam.add_argument("--width", type=int, default=CALIB_W)
    cam.add_argument("--height", type=int, default=CALIB_H)
    cam.add_argument("--fps", type=float, default=30.0)
    cam.add_argument(
        "--no-mjpg", action="store_true", help="Request YUYV instead of MJPEG."
    )
    cam.add_argument(
        "--color-mode",
        choices=("bgr", "rgb"),
        default="bgr",
        help="Channel order the frames come out in. bgr (default) "
        "matches the bare Camera and OpenCV; rgb matches what "
        "the gripper's wrist_camera hands out.",
    )

    out = p.add_argument_group("output")
    out.add_argument(
        "--no-display",
        action="store_true",
        help="Headless: no window, just rate stats.",
    )
    out.add_argument(
        "--save-dir",
        metavar="DIR",
        default=None,
        help="Where snapshots go: what 's' writes in the viewer, "
        "and one PNG per second when headless.",
    )
    out.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Run for N seconds, then exit. 0 = until Ctrl+C / q.",
    )
    out.add_argument(
        "--log-level", default="info", help="trace|debug|info|warn|error|critical|off"
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    log.set_level(args.log_level)

    if args.list:
        print_devices()
        return 0
    if not args.target:
        raise SystemExit(
            "缺少目标 —— 用 left / right,或腕相机序列号(如 XCA24Z0003m);"
            "--list 可以列出连着的设备。"
        )

    cam_info = resolve_wrist_camera(args.target)
    device = cam_info["path"]
    color_mode = ColorMode.RGB if args.color_mode == "rgb" else ColorMode.BGR

    # The remap tables are defined at the calibrated resolution only. Say so
    # once, up front, instead of letting the constructor throw mid-stream.
    calibrated_size = (args.width, args.height) == (CALIB_W, CALIB_H)
    if not calibrated_size and (args.undistort or args.compare):
        print(
            _yellow(
                f"undistortion needs {CALIB_W}x{CALIB_H} (the calibrated size) — "
                f"you asked for {args.width}x{args.height}, so frames stay raw.\n"
                "The firmware record stores no image size; rescaling the "
                "intrinsics would be a guess."
            )
        )

    # Resolve the intrinsics up front whenever rectification is reachable — that
    # includes staying raw in the viewer, where `u` can switch to rectified at
    # any moment and an MCU round-trip mid-loop would stall the capture.
    # Headless AND raw is the one case that can never rectify, so it skips the
    # gripper entirely rather than opening a serial link for nothing.
    wants_undistort = args.undistort or args.compare
    reachable = calibrated_size and (wants_undistort or not args.no_display)

    undist = None
    cal = provenance = None
    is_reference = False
    if reachable:
        cal, provenance, is_reference = resolve_calibration(args, cam_info)
        undist = FisheyeUndistorter(cal, CALIB_W, CALIB_H, args.balance)

    mode = MODE_RAW
    if undist is not None:
        mode = (
            MODE_COMPARE
            if args.compare
            else (MODE_RECT if args.undistort else MODE_RAW)
        )

    cam = Camera(
        device=device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        use_mjpg=not args.no_mjpg,
        color_mode=color_mode,
    )

    print(_bold("\nWrist camera"))
    print(
        f"  serial     : {cam_info['serial']}  ({cam_info['side']}, {cam_info['role']})"
    )
    print(f"  device     : {device}")
    print(
        f"  format     : {args.width}x{args.height} @ {args.fps:g} "
        f"{'YUYV' if args.no_mjpg else 'MJPG'}, {args.color_mode.upper()}"
    )
    if undist is not None:
        print(_bold("\nFisheye calibration"))
        print(f"  source     : {provenance}")
        print(f"  fx={cal.fx:.4f}  fy={cal.fy:.4f}  cx={cal.cx:.4f}  cy={cal.cy:.4f}")
        print(f"  k1={cal.k1:.6f}  k2={cal.k2:.6f}  k3={cal.k3:.6f}  k4={cal.k4:.6f}")
        print(
            f"  balance    : {undist.balance:.2f} "
            f"(focal scale {undist.focal_scale:.3f})"
        )
        print("  rectified K:")
        for row in undist.new_camera_matrix:
            print("    [" + "  ".join(f"{v:10.4f}" for v in row) + "]")
        if is_reference:
            print(
                _yellow(
                    "\n  These are the SDK's shared reference values, not this "
                    "unit's.\n  Lens placement varies per assembly (the principal "
                    "point especially),\n  so do not measure in pixels off these "
                    "frames. Calibrate with:\n"
                    "    python python/examples/fisheye_cal.py set-fisheye "
                    "--from-npz cam.npz"
                )
            )
    print(f"\n  mode       : {mode}\n")

    # cv2 is needed for the window AND for writing snapshots, so import it
    # whenever either is asked for; `gui` tracks only whether we show a window.
    cv2 = None
    if not args.no_display or args.save_dir:
        try:
            import cv2 as _cv2

            cv2 = _cv2
        except ImportError:
            log.warning("opencv-python not importable — no window, no snapshots")
    gui = cv2 is not None and not args.no_display

    def apply_mode(m: str) -> None:
        """MODE_RECT rectifies inside the capture path; MODE_COMPARE needs the
        raw frame too, so it remaps in Python and leaves the camera alone."""
        cam.set_undistorter(undist if m == MODE_RECT else None)

    apply_mode(mode)

    def build_canvas(frame) -> np.ndarray:
        """The frame as cv2 wants it (BGR), side by side with its raw form in
        compare mode."""
        image = to_bgr(frame.image, color_mode)
        if mode == MODE_COMPARE and undist is not None:
            return np.hstack([image, to_bgr(undist.apply(frame.image), color_mode)])
        return image

    win = "TacCap wrist camera"
    save_dir = args.save_dir or "."
    saved = 0
    t0 = time.monotonic()
    last_stat = last_save = t0
    read_failures = 0
    # Camera.actual_fps is only updated by the streaming (start()) path, so in
    # this read() loop it would read a flat 0.0 — measure the rate here instead.
    fps, fps_t0, fps_n = 0.0, t0, 0

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - t0 >= args.duration:
                break

            frame = cam.read()
            if frame is None:
                read_failures += 1
                if read_failures in (1, 10, 100):
                    log.warning(f"camera read timed out ({read_failures}x)")
                continue

            fps_n += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                fps, fps_t0, fps_n = fps_n / (now - fps_t0), now, 0

            if not gui:
                if now - last_stat >= 2.0:
                    last_stat = now
                    print(
                        f"  {cam.total_frames} frames | {fps:5.1f} fps | "
                        f"{cam.dropped_frames} dropped | mode={mode}"
                    )
                if args.save_dir and cv2 is not None and now - last_save >= 1.0:
                    last_save = now
                    saved += save_png(
                        cv2, build_canvas(frame), save_dir, mode, frame.frame_index
                    )
                continue

            canvas = annotate(
                cv2,
                build_canvas(frame),
                [
                    f"{mode}  balance={undist.balance:.2f}"
                    if undist is not None
                    else f"{mode}  (no calibration)",
                    f"#{frame.frame_index}  {fps:.1f} fps  "
                    f"dropped={cam.dropped_frames}",
                    "q quit   u mode   s save   [ ] balance",
                ],
            )
            if mode == MODE_COMPARE:
                h, w = canvas.shape[:2]
                cv2.line(canvas, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)

            try:
                cv2.imshow(win, canvas)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as e:
                # No GUI backend (headless box, no DISPLAY, or a headless
                # opencv-python build). Keep streaming — and keep saving.
                log.warning(f"cannot open a window ({e}) — running headless")
                gui = False
                continue

            if key in (ord("q"), 27):
                break
            elif key == ord("u"):
                if undist is None:
                    log.warning(
                        "no calibration at this resolution — undistortion unavailable"
                    )
                else:
                    mode = MODE_CYCLE[(MODE_CYCLE.index(mode) + 1) % len(MODE_CYCLE)]
                    apply_mode(mode)
                    log.info(f"mode: {mode}")
            elif key == ord("s"):
                saved += save_png(
                    cv2, canvas, save_dir, mode, frame.frame_index, announce=True
                )
            elif key in (ord("["), ord("]")) and undist is not None:
                step = -0.1 if key == ord("[") else 0.1
                balance = min(1.0, max(0.0, undist.balance + step))
                # The tables are built once per undistorter, so a new balance
                # means a new one — cheap enough at human key-press rates.
                undist = FisheyeUndistorter(cal, CALIB_W, CALIB_H, balance)
                apply_mode(mode)
                log.info(
                    f"balance: {undist.balance:.2f} "
                    f"(focal scale {undist.focal_scale:.3f})"
                )
    except KeyboardInterrupt:
        print()
    finally:
        cam.stop()
        if gui:
            # Never let teardown be the thing that raises — a headless
            # opencv-python build throws here just as imshow() does.
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    elapsed = time.monotonic() - t0
    print(_bold("\nSummary"))
    rate = cam.total_frames / elapsed if elapsed > 0 else 0.0
    print(
        f"  {cam.total_frames} frames in {elapsed:.1f}s ({rate:.1f} fps "
        f"average), {cam.dropped_frames} dropped, "
        f"{read_failures} read timeouts"
    )
    if saved:
        print(f"  {saved} snapshot(s) under {os.path.abspath(save_dir)}")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

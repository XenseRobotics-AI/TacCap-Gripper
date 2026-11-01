# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Shared plumbing for the examples that read the visuotactile (GSPS / OG) sensors.

Not part of ``xense.taccap``, and deliberately so: tactile capture,
rectification and force/depth inference live in the ``xensesdk`` wheel. This SDK
owns the gripper MCU and the wrist camera; the two meet only in the examples,
which is why the glue sits here next to ``_target.py``.

What is here:
  - discovery — ``scan_tactile_sensors()`` lists the GSPS sensors under
    /dev/v4l/by-id and pairs each with a gripper *side* and a *finger*
  - selection — ``select_tactile()``, the ``left`` / ``right`` / serial selector
  - ``xensesdk`` lifecycle — ``import_sensor_class()``, ``open_sensor()``,
    ``release_sensor()``, with the work-arounds every caller otherwise rediscovers
  - ``OUTPUTS`` — the output names the examples accept, mapped to
    ``Sensor.OutputType``
  - display conventions — ``to_rgb()``, ``to_display()``, ``field_arrows()``,
    the same orientation and channel order the LeRobot integration records

Two numbers identify a tactile sensor, and they mean different things:

    side    which gripper it is mounted on. NOT in the GSPS serial: it comes from
            the gripper sharing the sensor's USB hub, whose side comes from its
            firmware SN (odd sequence = left, even = right).
    finger  which jaw of that gripper it sits in — the GSPS serial's last digit,
            odd = left finger, even = right finger.

So ``GSPS01A31Z0049`` is "the left-finger sensor" of whichever gripper it is
cabled to. Reading its parity as the gripper side is the easy mistake.

The pairing rule (USB hub + last digit) is the one the LeRobot integration
uses for recording — ``lerobot/robots/taccap_gripper/serial_discovery.py`` in
xense-taccap-lerobot. Keep the two in step.
"""

from __future__ import annotations

import glob
import os
import re
import sys
import time
from dataclasses import dataclass

import numpy as np

BYID_DIR = "/dev/v4l/by-id"
V4L_BYPATH_DIR = "/dev/v4l/by-path"
SERIAL_BYPATH_DIR = "/dev/serial/by-path"

TACTILE_SN_RE = re.compile(r"(GSPS01[A-Z]\d{2}[ZA]\d{4})")
# A by-path name embeds ``usb-<bus>:<port>.<port>…:<config>``; the hub a device
# hangs off is that port path minus the device's own (last) port.
_USB_PORT_RE = re.compile(r"^(.*)-usb-(\d+):([\d.]+):")

# Output name (CLI) -> (Sensor.OutputType attribute, needs the inference engine).
# Shapes as the SDK returns them, portrait. Image outputs are BGR uint8 — checked:
# Raw is byte-for-byte what OpenCV decodes from the same sensor, and Rectify
# keeps that channel order. Depth is float32 mm; the rest are float arrays.
OUTPUTS = {
    "rectify": ("Rectify", False),  # (700, 400, 3) BGR rectified gel image
    "difference": ("AugDifference", False),  # (700, 400, 3) BGR, vs. reference
    "depth": ("Depth", True),  # (700, 400) indentation, mm
    "marker": ("Marker2D", True),  # (35, 20, 2) marker positions (x, y), px
    "force": ("Force", True),  # (35, 20, 3) force distribution
    "force_resultant": ("ForceResultant", True),  # (6,) Fx Fy Fz Mx My Mz
}


@dataclass
class TactileSensor:
    serial: str
    path: str  # /dev/v4l/by-id/... (stable across replugs, unlike /dev/videoN)
    finger: str  # "left" / "right", from the serial's last digit
    side: str | None  # gripper side, from the hub; None when no gripper shares it
    gripper_sn: str | None


def finger_of_serial(sn: str) -> str:
    return "left" if int(sn[-1]) % 2 == 1 else "right"


def _hub_of_bypath(name: str) -> str | None:
    """``pci-0000:00:14.0-usb-0:6.4:1.0-video-index0`` -> ``pci-0000:00:14.0-usb-0:6``.

    The controller prefix stays in the key: the port path alone is only unique
    within one host controller.
    """
    m = _USB_PORT_RE.search(name)
    if not m:
        return None
    controller, bus, ports = m.groups()
    parent = ports.rsplit(".", 1)[0] if "." in ports else ports
    return f"{controller}-usb-{bus}:{parent}"


def _device_hub(node: str, bypath_dir: str) -> str | None:
    real = os.path.realpath(node)
    for link in glob.glob(f"{bypath_dir}/*"):
        if "usbv2" in os.path.basename(link):  # USB-2 mirror of the same device
            continue
        if os.path.realpath(link) == real:
            return _hub_of_bypath(os.path.basename(link))
    return None


def _gripper_hubs(endpoints=None) -> dict[str, tuple[str, str]]:
    """``{usb-hub: (side, firmware_sn)}`` for the given grippers, or every one
    ``scan_grippers()`` sees."""
    from xense.taccap import Side, scan_grippers

    hubs = {}
    for ep in scan_grippers() if endpoints is None else endpoints:
        if ep.side not in (Side.Left, Side.Right):
            continue
        hub = _device_hub(ep.mcu_device, SERIAL_BYPATH_DIR)
        if hub is not None:
            hubs[hub] = ("left" if ep.side == Side.Left else "right", ep.firmware_sn)
    return hubs


def scan_tactile_sensors(endpoints=None) -> list[TactileSensor]:
    """Every GSPS sensor under /dev/v4l/by-id, with its finger and gripper side.

    The sides come from ``endpoints`` (``GripperEndpoints``) when given, else
    from ``scan_grippers()``. Pass them once a gripper is open: the scan cannot
    read the firmware SN of a port this process already holds, so every sensor
    on that gripper's hub would come back without a side.

    A sensor on a hub with no gripper keeps ``side=None`` rather than failing
    here — listing is for looking, and an unpowered gripper is exactly what one
    looks for.
    """
    hubs = _gripper_hubs(endpoints)
    seen, out = set(), []
    for path in sorted(glob.glob(f"{BYID_DIR}/*-video-index0")):
        m = TACTILE_SN_RE.search(os.path.basename(path))
        if not m or m.group(1) in seen:
            continue
        sn = m.group(1)
        seen.add(sn)
        side, gripper_sn = hubs.get(_device_hub(path, V4L_BYPATH_DIR), (None, None))
        out.append(TactileSensor(sn, path, finger_of_serial(sn), side, gripper_sn))
    return out


def print_sensors(sensors: list[TactileSensor]) -> None:
    if not sensors:
        print(f"no GSPS visuotactile sensor under {BYID_DIR}/")
        return
    print(f"{'serial':<16}{'gripper':<8}{'finger':<8}{'gripper SN':<18}device")
    for s in sensors:
        print(
            f"{s.serial:<16}{s.side or '?':<8}{s.finger:<8}"
            f"{s.gripper_sn or '(no gripper on hub)':<18}{s.path}"
        )


def select_tactile(
    target: str | None, finger: str | None = None
) -> list[TactileSensor]:
    """``left`` / ``right`` / a GSPS serial / ``None`` -> the sensors to open.

    A side selects both fingers of that gripper; ``finger`` narrows it to one.
    ``None`` means every sensor plugged in. Exits with the table on a miss, the
    same way ``_target.resolve_target()`` does for grippers.
    """
    sensors = scan_tactile_sensors()
    key = (target or "").strip()
    if not key:
        picked = sensors
    elif key.lower() in ("left", "l", "right", "r"):
        side = "left" if key.lower().startswith("l") else "right"
        picked = [s for s in sensors if s.side == side]
    elif TACTILE_SN_RE.fullmatch(key):
        picked = [s for s in sensors if s.serial == key]
    else:
        sys.exit(
            f"error: unrecognized tactile target {target!r} — use left / right, "
            "or a GSPS serial such as GSPS01A31Z0049."
        )
    if finger:
        picked = [s for s in picked if s.finger == finger]
    if not picked:
        print_sensors(sensors)
        what = f"{target or 'any gripper'}" + (f", {finger} finger" if finger else "")
        sys.exit(
            f"error: no visuotactile sensor for {what} (table above). A side is "
            "only known when the gripper sharing the sensor's USB hub is plugged "
            "in and has a burned firmware SN."
        )
    return picked


# ---- xensesdk ---------------------------------------------------------------


def _patch_find_library_udev() -> None:
    """Make ``ctypes.util.find_library("udev")`` return a file, not a directory.

    Inside a conda env it can resolve to ``$CONDA_PREFIX/lib/udev/`` (a
    directory), and ``pyudev`` — which ``xensesdk`` uses to enumerate sensors —
    then dies with ``OSError: ... Is a directory``.
    """
    import ctypes.util

    orig = ctypes.util.find_library
    if getattr(orig, "_taccap_patched", False):
        return

    def find_library(name):
        found = orig(name)
        if name != "udev" or (found and not os.path.isdir(found)):
            return found
        for candidate in (
            "/lib/x86_64-linux-gnu/libudev.so.1",
            "/usr/lib/x86_64-linux-gnu/libudev.so.1",
            "/lib/aarch64-linux-gnu/libudev.so.1",
            "/usr/lib/aarch64-linux-gnu/libudev.so.1",
        ):
            if os.path.exists(candidate):
                return candidate
        return found

    find_library._taccap_patched = True
    ctypes.util.find_library = find_library


def import_sensor_class():
    """``xensesdk.Sensor``, or exit with how to install it."""
    _patch_find_library_udev()
    try:
        from xensesdk import Sensor
    except ImportError:
        sys.exit(
            "error: xensesdk is not installed. The visuotactile sensors are read "
            "through it, not through this SDK:\n"
            "       mamba env update -f environment.yml   (or: uv pip install xensesdk)\n"
            "       (see the tactile section of docs/USAGE.md)"
        )
    return Sensor


def output_types(Sensor, names: list[str]) -> list:
    unknown = [n for n in names if n not in OUTPUTS]
    if unknown:
        sys.exit(
            f"error: unknown tactile output(s) {unknown}; choose from {list(OUTPUTS)}"
        )
    return [getattr(Sensor.OutputType, OUTPUTS[n][0]) for n in names]


def needs_inference(names: list[str]) -> bool:
    return any(OUTPUTS[n][1] for n in names)


# Linear gain the SDK applies to the difference image. The sensors ship at 1.5,
# which clipped and was noisy; the LeRobot integration records with 1.0
# (TacCapGripperConfig.tactile_diff_gain), and the examples match it so the
# difference view here looks like the one there.
DIFF_GAIN = 1.0
WARMUP_S = 0.5


def open_sensor(Sensor, serial: str, *, disable_infer: bool):
    """``Sensor.create()`` the way the LeRobot integration does it.

    Open sensors **one at a time**. The first open of a serial reads the
    sensor's flash to build ``~/.xensesdk/config/<serial>``, and that read
    resets and re-enumerates the device; two cold opens racing each other move
    camera nodes mid-open. Warm opens are cheap.

    ``disable_infer=True`` skips loading the inference engine, which Rectify
    and Difference never need. ``process.diff_gain`` is set to ``DIFF_GAIN``.
    The first ``WARMUP_S`` of frames are read and dropped, so the caller's
    first frame comes from a settled stream.
    """
    sensor = Sensor.create(
        serial,
        disable_infer=disable_infer,
        overrides={"process.diff_gain": DIFF_GAIN},
    )
    t_end = time.monotonic() + WARMUP_S
    while time.monotonic() < t_end:
        try:
            sensor.selectSensorInfo(Sensor.OutputType.Rectify)
        except Exception:
            time.sleep(0.05)  # errors are expected while the stream settles
    return sensor


# ---- display conventions ----------------------------------------------------
#
# What the viewers show is what the LeRobot integration records, so a frame
# looks the same live and in the dataset (lerobot/cameras/xense/camera_xense.py):
#
#   orientation   the SDK's portrait (700, 400) is transposed to (400, 700) —
#                 a transpose, i.e. a mirror about the diagonal, not a rotation
#   channels      BGR -> RGB for anything but OpenCV
#
# Force and marker fields go through the same transpose, so arrows stay on the
# markers they belong to.


def to_display(arr: np.ndarray) -> np.ndarray:
    """Swap the first two axes: (700, 400[, C]) -> (400, 700[, C])."""
    return np.ascontiguousarray(np.swapaxes(arr, 0, 1))


def to_rgb(bgr: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(bgr[..., ::-1])


def field_arrows(field, display_hw: tuple[int, int], scale: float):
    """``(origins, vectors)``, (N, 2) each, for a (rows, cols, >=2) field.

    In the coordinates of a ``to_display()`` image of size ``display_hw``. The
    field's components are (x, y) in the SDK's portrait image, and cell (i, j)
    sits on marker (i, j), whose rest position is
    ((j + 1) * W / (cols + 1), (i + 1) * H / (rows + 1)) — measured from
    Marker2D, which puts a 35 x 20 grid 19.05 px apart in x, 19.44 in y. Both
    the positions and the components are then swapped, the same transpose the
    image went through.
    """
    field = np.asarray(field, dtype=np.float32)
    rows, cols = field.shape[:2]
    h, w = display_hw[1], display_hw[0]  # portrait size, before the transpose
    jj, ii = np.meshgrid(np.arange(cols), np.arange(rows))
    x = (jj + 1) * w / (cols + 1)
    y = (ii + 1) * h / (rows + 1)
    origins = np.stack([y, x], axis=-1).reshape(-1, 2)  # transposed: (x, y) -> (y, x)
    vectors = field[..., [1, 0]].reshape(-1, 2) * scale
    return origins, vectors


def release_sensor(sensor) -> None:
    """``Sensor.release()``, stopping the SDK's own capture thread first.

    xensesdk 2.1.2 / 2.1.3 can close its OpenCV backend while that thread is
    still reading from it. Suspending the stream joins the thread before
    release() closes the device. The handle is SDK-internal, hence its own try:
    a version without it must still get released.
    """
    try:
        cam = sensor.ctx.get_handle("cam")
        if cam is not None:
            cam.suspend_stream()
    except Exception:
        pass
    sensor.release()

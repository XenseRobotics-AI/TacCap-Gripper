# taccap-gripper

[中文](README-zh.md)

C++17 / Python SDK for the **TacCap-Gripper**, XenseRobotics' multimodal tactile
data-collection gripper. One namespace: `xense::taccap::` in C++,
`xense.taccap` in Python.

The SDK is written once in C++; the Python package is a pybind11 binding over
the same code, so both languages behave identically. The host talks to the
gripper's MCU over the TC-GU-01 serial protocol, and the MCU drives the motor —
real-time safety (motion envelope, stall and thermal protection) runs in
firmware and cannot be bypassed from the host.

## Features

- **Leader & follower** — `LeaderGripper` (hand-held master: encoder, IMU,
  button) and `FollowerGripper` (actuated gripper with a RobStride motor).
- **Controllers** — `ImpedanceController` follows a position,
  `ForcePositionController` grasps with a bounded force. Both take the same two
  non-blocking calls: `set_target(0..1)` and `snapshot()`.
- **Normalized position** — `0` = closed, `1` = open, on one-shot reads and on
  every streamed sample.
- **Sensors** — IMU, encoder, LEDs, button, power-on auto-calibration.
- **Wrist camera** — V4L2 capture with fisheye undistortion from the
  intrinsics stored on the unit itself.
- **Zero-config discovery** — side (left/right) and role (leader/follower) come
  from the firmware-burned serial number; nothing is configured per unit.
- **Firmware OTA** for the gripper MCU and for the follower's motor, over USB.

Out of scope: visuotactile (OG) capture is provided by the `xensesdk` wheel;
teleoperation, policies and recording belong in the consuming application
(e.g. the LeRobot integration).

---

## Requirements

| | |
| --- | --- |
| OS | Linux (Ubuntu 22.04+). The camera path is V4L2; macOS / Windows are not supported |
| Python | 3.12 |
| Build | [Mamba](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install) via Miniforge — `environment.yml` pins the full toolchain |
| Firmware | follower **≥ 1.2.5** (enforced); see [Firmware](#firmware) |

## Install

**Step 0 — Mamba.** Install Miniforge, which ships `mamba` (much faster than
plain conda, and conda-forge by default). Skip if `mamba --version` already
works.

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
# accept the prompts, then open a new terminal
```

**Step 1 — System packages.** `v4l-utils` provides `v4l2-ctl`, which you need
to set and check the camera format ([Host setup → 3](#3-wrist-camera-always-mjpeg)).
Run `apt update` first so you get the current version:

```bash
sudo apt update
sudo apt install -y git curl v4l-utils
sudo apt install -y --only-upgrade libusb-1.0-0   # keep the USB runtime current after kernel updates
```

> [!NOTE]
> After an `apt upgrade` that brings a new kernel, upgrade the USB packages as
> above and reboot, so the running kernel and the userspace USB stack come from
> the same update. A stale USB stack after a kernel bump is a common reason
> cameras that worked yesterday stop opening today.

**Step 2 — Clone.**

```bash
git clone https://github.com/XenseRobotics-AI/TacCap-Gripper.git
cd TacCap-Gripper
```

**Step 3 — Create and activate the environment.** The env is named `taccap`
and installs the compiler, CMake, OpenCV, pybind11 and Python 3.12 from
conda-forge:

```bash
mamba env create -f environment.yml
mamba activate taccap
```

To refresh an existing env after pulling a new release:
`mamba env update -f environment.yml --prune`.

**Step 4 — Install the SDK** (from the activated env):

```bash
uv pip install -e . --no-build-isolation
```

Keep `--no-build-isolation`: it builds against the `pybind11` pinned in the env.
After pulling a new release, run this again — an editable install keeps
serving the previously compiled extension until you reinstall.

**Step 5 — Verify.**

```bash
python -c "import xense.taccap as t; print(t.__version__)"
v4l2-ctl --version
```

C++-only builds, tests and environment traps: [docs/INSTALL.md](docs/INSTALL.md).

## Host setup (one-time)

Do all three steps before plugging in a gripper for the first time.

### 1. Serial-port and camera permissions

The gripper MCU enumerates as `/dev/ttyACM*` (group `dialout`), the wrist
camera as `/dev/video*` (group `video`). Without `dialout` the SDK can list a
gripper but cannot open it to read its firmware SN, so `scan_grippers()` reports
`role=Unknown` / an empty `firmware_sn`, and opening fails with
`Permission denied`.

```bash
sudo usermod -aG dialout,video "$USER"
# log out and back in (or reboot), then replug the gripper
```

Verify the MCU is fully readable — `role` must be `Leader` / `Follower` (not
`Unknown`) and `firmware_sn` non-empty:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side.name, g.role.name, repr(g.firmware_sn))"
```

### 2. Keep ModemManager off the gripper serial port

The MCU is a CH343 USB-serial bridge (`1a86:55d2`). On every hot-plug,
**ModemManager** (installed by default on Ubuntu) probes the new port with AT
commands and holds it for a few seconds, so opening the gripper in that window
fails with:

```
IoError: SerialBus: open(/dev/serial/by-id/usb-1a86_USB_Dual_Serial_...-if02): Device or resource busy
```

Typical symptom: the first launch works, but replug → relaunch immediately is
*busy*. Tell ModemManager to ignore these devices (real modems are unaffected):

```bash
sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
# TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) — keep ModemManager off them
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify, with a gripper plugged in:

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE   # -> ID_MM_DEVICE_IGNORE=1
```

On a dedicated robot PC with no cellular modem you can instead run
`sudo systemctl disable --now ModemManager`. If `brltty` (braille driver) is
installed it grabs `1a86` devices the same way — remove it with
`sudo apt remove brltty`.

### 3. Wrist camera: always MJPEG

> [!IMPORTANT]
> The wrist camera **must** stream in **MJPEG** (`MJPG`). Uncompressed YUYV at
> 640×480 @ 30 fps is ~147 Mbit/s of reserved isochronous bandwidth; the
> gripper's cameras share a **USB 2.0** link, and a couple of YUYV streams
> exhaust it. The result is `VIDIOC_STREAMON: No space left on device`, a
> camera that fails to open, or silently dropped frames.

The SDK's `Camera` requests MJPEG by default (`use_mjpg=True`) — do not turn it
off (`use_mjpg=False`, `wrist_camera.py --no-mjpg`) on a full rig. Any other
program that opens the camera (OpenCV, ffmpeg, GStreamer, a camera service)
must request MJPEG itself; in OpenCV, set `CAP_PROP_FOURCC` to `MJPG` *before*
width/height.

Check and set the format with `v4l2-ctl` (package `v4l-utils`, installed in
[Install → Step 1](#install)):

```bash
CAM=/dev/v4l/by-id/<wrist-camera>-video-index0      # list with: ls /dev/v4l/by-id/

v4l2-ctl -d $CAM --list-formats-ext                 # 'MJPG' must be listed
v4l2-ctl -d $CAM --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d $CAM --set-parm=30
v4l2-ctl -d $CAM --get-fmt-video                    # -> Pixel Format : 'MJPG'
```

The V4L2 format is driver state: it resets on replug or reboot, and an
application may set its own when it opens the device. Treat `v4l2-ctl` as the
way to set and verify the format, and make sure every capturing application
asks for MJPEG.

---

## Quick start

Every example takes one positional selector: `left`, `right`, or a firmware
SN. Omit it when exactly one gripper is attached.

```bash
# 1. What is attached? (read-only)
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn, g.mcu_device)"

# 2. Read its state (read-only)
python python/examples/follower_status.py left

# 3. Write the firmware motion-safety envelope (once per follower; writes flash)
python python/examples/impedance_control.py left --set-envelope

# 4. First motion, interactively (j/k/o/c keys)
python python/examples/gripper_console.py left

# 5. Grasp
python python/examples/force_position_control.py left
```

### Leader — read it

```python
import xense.taccap as t

g = t.LeaderGripper.open(normalize_position=True)   # the one attached gripper
g.encoder.on_data(lambda s: print(s.position_rad, s.position))   # position: 0..1
g.imu.on_data(lambda s: print(s))
g.start_streaming(imu_hz=100, encoder_hz=100)
# ...
g.stop_streaming()
```

### Follower — drive it

Always drive the motor through a controller, and build its config with
`for_spec()` so the limits match the installed motor (EL05 or RS00).

```python
import xense.taccap as t

g = t.FollowerGripper(t.find_follower().mcu_device)
cfg = t.ForcePositionConfig.for_spec(g.motor.get_spec())
c = t.ForcePositionController(g, cfg)

g.motor.clear_fault()
c.start()                       # start the controller before enabling the motor
g.motor.enable()
try:
    c.set_target(0.0)           # 0 = closed, 1 = open; non-blocking
    while True:
        s = c.snapshot()        # consistent state, no bus traffic
        if s.holding or s.arrived:
            break
finally:
    c.stop()                    # zero torque, then disables the motor
```

Use `t.ImpedanceController(g, t.ImpedanceConfig.for_spec(g.motor.get_spec()))`
— same two calls — to follow a position instead of grasping. While a
controller runs, read state from `snapshot()`, never `motor.read_status()`.

| `ForcePositionConfig` | default | meaning |
| --- | --- | --- |
| `grasp_torque_nm` | motor's continuous stall rating (EL05 1.1 N·m, RS00 3.6 N·m) | grip force a blocked jaw settles at |
| `close_speed_radps` | 1.1 rad/s | travel speed |

### Wrist camera

```python
from xense.taccap import Camera

cam = Camera(device="/dev/v4l/by-id/<wrist-camera>-video-index0",
             width=640, height=480, fps=30.0)       # MJPEG by default
cam.start(lambda f: print(f.frame_index, f.image.shape))
```

Or attach it to a gripper with `open_cameras=True, wrist_video=...`
(and `undistort_wrist=True` for rectified frames). Gripper objects do not open
the camera unless asked, because a camera service usually owns it.

### Both sides in one process

```python
from xense.taccap import FollowerGripper, scan_grippers, Side

eps = {e.side: e for e in scan_grippers()}
left, right = FollowerGripper(eps[Side.Left].mcu_device), FollowerGripper(eps[Side.Right].mcu_device)
```

Each gripper owns its own serial link and threads; give each its own controller.

### Serial numbers

```
TCGU01 A24 Z 0001 m
└─┬──┘ └┬┘ │ └┬─┘ └ role: m = leader (master), s = follower (slave)
product batch line seq  last digit: odd = Left, even = Right
```

The wrist camera (`XC…`) carries the same sequence number as its gripper, so
`left` / `right` selects both.

## Examples

| Script | What it does | Effect on the device |
| --- | --- | --- |
| `follower_status.py` | read follower state | read-only |
| `leader_normalized_position.py` | stream leader position 0..1 | read-only |
| `wrist_camera.py` | view / list wrist cameras (`--undistort`, `--list`) | read-only |
| `read_intrinsics.py` | dump calibration as JSON | read-only |
| `impedance_control.py` | follow a position; `--show-envelope` / `--set-envelope` | moves motor / writes flash |
| `force_position_control.py` | grasp with bounded force | moves motor |
| `gripper_console.py` | interactive keyboard control | moves motor |
| `control_and_read.py` | read state while controlling | moves motor |
| `control_ripple.py` | measure tracking quality | moves motor |
| `calibrate.py` | encoder zero + travel span | writes flash |
| `fisheye_cal.py` | show / set fisheye and encoder-max records | read-only / writes flash |
| `ota_update.py` | flash gripper firmware | **flashes firmware** |
| `full_ota_update.py` | guided follower MCU + motor update and verification | **flashes firmware; operator power cycles required** |
| `motor_firmware_update.py` | guided motor-only update to a specified version | **flashes firmware; operator power cycles required** |
| `motor_ota_update.py` | flash the follower's motor firmware | **flashes firmware** |

Details: [docs/EXAMPLES.md](docs/EXAMPLES.md). C++ examples build into
`build/cpp/examples/`.

## Firmware

Prebuilt images ship in [`firmware/`](firmware/). Pick the image by **role**
(last character of the firmware SN: `m` leader, `s` follower) — flashing the
wrong role's image bricks the MCU.

```bash
python python/examples/ota_update.py slave left     # or: --all
```

**Power-cycle after every flash.** Follower: unplug the **24 V** cable for
~2 s (USB can stay in). Leader: replug USB. Skipping this leaves the device
looking healthy while it drops status frames.

`FollowerGripper` refuses followers below firmware 1.2.5. Leader and follower
firmware are versioned independently. Required motor firmware: EL05 ≥ 1.0.5.0.4,
RS00 ≥ 0.0.3.32. Image table, motor model, RS00 setup and motor OTA:
[firmware/README.md](firmware/README.md). Step-by-step motor firmware update:
[docs/MOTOR_OTA.md](docs/MOTOR_OTA.md) ([中文](docs/MOTOR_OTA-zh.md)).

For the standalone Chinese/English maintenance GUI, supported PC/Jetson packages,
and reviewed upgrade/downgrade targets, see [Firmware updater](docs/FIRMWARE_UPDATER.md).
It bundles its runtime; users do not need the SDK development environment above.
The combined CLI is `python python/examples/full_ota_update.py firmware/motor/rs00-0.0.3.32.bin left`.

### Checking versions

`follower_status.py` prints the gripper firmware, the motor model and the
motor's own firmware, checked against the required minimum. It is read-only:

```bash
python python/examples/follower_status.py left
# [fw] FirmwareVersion(1.2.16.0)
# [motor] MotorModel(RS00, id=1, from flash, t_max=14.0Nm, v_max=33.0rad/s)
# [motor fw] 0.0.3.32  (...)  OK
```

## Troubleshooting

When reporting a problem, include the output of `follower_status.py` for each
follower and the latest log from `~/.taccaplogs/`.

| Symptom | Fix |
| --- | --- |
| `role=Unknown`, empty `firmware_sn`, `Permission denied` | add user to `dialout` ([step 1](#1-serial-port-and-camera-permissions)) |
| `Device or resource busy` right after plugging in | ModemManager rule ([step 2](#2-keep-modemmanager-off-the-gripper-serial-port)) |
| `No space left on device`, camera won't open, dropped frames | camera not in MJPEG ([step 3](#3-wrist-camera-always-mjpeg)) |
| Status frames lost after a firmware flash | power-cycle (follower: cut 24 V) |
| C++ / bindings change has no effect | reinstall in the consuming env |
| Normalized position is NaN or raises | calibrate first: `calibrate.py`, see [docs/CALIBRATION.md](docs/CALIBRATION.md) |

Logs: every process writes a DEBUG log to `~/.taccaplogs/session_*.log`
(override with `$TACCAP_LOG_DIR`). Read it first when something goes wrong.

## Documentation

- [docs/USAGE.md](docs/USAGE.md) ([中文](docs/USAGE-zh.md)) — end-to-end bring-up of gripper, wrist camera and tactile sensor
- [docs/INSTALL.md](docs/INSTALL.md) — build options, C++-only builds, environment traps
- [docs/FIRMWARE_UPDATER.md](docs/FIRMWARE_UPDATER.md) — standalone PC/Jetson firmware maintenance and packaging
- [docs/MOTOR_OTA.md](docs/MOTOR_OTA.md) ([中文](docs/MOTOR_OTA-zh.md)) — updating the follower's motor firmware over USB-C
- [docs/CALIBRATION.md](docs/CALIBRATION.md) — encoder zero, travel span, fisheye intrinsics
- [docs/EXAMPLES.md](docs/EXAMPLES.md) — every example script and controller parameter
- [firmware/README.md](firmware/README.md) — firmware images, flashing, motor setup
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — layers, threading, module map
- [CHANGELOG.md](CHANGELOG.md)

## Citation

```bibtex
@software{taccap_gripper,
  title   = {taccap-gripper: an SDK for the TacCap multimodal tactile gripper},
  author  = {{XenseRobotics Co., Ltd.}},
  year    = {2026},
  url     = {https://github.com/XenseRobotics-AI/TacCap-Gripper},
  version = {VERSION},
  license = {Apache-2.0}
}
```

Replace `VERSION` with the installed release (`xense.taccap.__version__`), and
report the gripper firmware version (`g.firmware_version`) alongside it.

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.

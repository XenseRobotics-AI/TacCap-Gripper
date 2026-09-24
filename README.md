# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** — XenseRobotics' multimodal
tactile data-collection gripper. One namespace, `xense::taccap::` in C++ and
`xense.taccap` in Python.

## How it is built

**One implementation, two languages.** The whole SDK is C++17; the Python
package is a pybind11 binding over that same code, not a second port. A fix
lands once and both sides get it, and a behaviour you measure from Python is
the behaviour C++ has.

The host never touches the motor directly: it speaks the TC-GU-01 serial
protocol to the gripper's MCU, and the MCU relays to the RobStride motor over
FDCAN.
Every `Motor` call in this SDK is therefore a command to the MCU, not a CAN
write — which is why the MCU can enforce limits the host cannot bypass.

**Four layers, each usable on its own.** L1 is the TC-GU-01 wire protocol —
byte-stuffed framing, CRC, typed payload codecs. L2 is the async transport that
owns the serial port, matches ACKs to commands by sequence number, and fans
DATA frames out to per-command subscribers. L3 is the typed components
(`Motor`, `Encoder`, `IMU`, `Camera`, `Led`, `Calibration`, `Diagnostics`). L4
aggregates them into `LeaderGripper` / `FollowerGripper` and adds the
background controllers. You can open a `Transport` and talk frames, or open a
gripper and never see one. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

**Two threads, and user code runs on neither of the critical ones.** The
transport's reader thread does nothing but read, parse and hand off; a separate
dispatcher thread runs subscriber callbacks. That split is load-bearing: a
Python callback takes the GIL, and a callback that stalls the reader stalls
`read()` and overflows the kernel tty buffer. Controllers add one more thread,
which submits exactly one command per motor-status frame — writing in the
window the MCU is known to be idle, rather than on a free-running clock.

**The firmware owns real-time safety; the host does not keep a second copy.**
The MCU runs the motion-safety envelope and the stall test at 500 Hz, and it is
the only layer on the MIT command path that nothing can bypass. So this SDK has
no host-side contact detection and no second stall guard — saturating the
controller's torque budget *is* contact. Where the two could disagree, the
device wins. See [docs/CONTROL_LAYERING.md](docs/CONTROL_LAYERING.md).

**Drive the motor through a controller.** `ImpedanceController` follows a
position, `ForcePositionController` grasps; both expose the same two
non-blocking calls, `set_target(0..1)` and `snapshot()`. The raw `submit_*`
motor primitives are still there, but they are bare MIT frames with none of the
host-side protection on their path.

**Nothing is configured per unit.** Sides, roles and calibration come off the
device: discovery reads the firmware-burned serial (never the CH343 USB-chip
serial), and the fisheye intrinsics and travel span live in MCU flash, so a
gripper carries its own identity and calibration between benches.

**What is deliberately out of scope.** Visuotactile (OG) capture belongs to the
`xensesdk` wheel, not here — `xense.taccap` is gripper protocol plus wrist
camera. Teleoperation loops, grasp policy and episode recording live in the
consuming application; this SDK provides the real-time primitives, not the
policy. The wrist `Camera` is opt-in for the same reason: an external camera
service usually owns the V4L2 devices, so the gripper aggregates do not open it
unless you pass `open_cameras=True`.

A fork of `lerobot-xense` consumes this SDK through a `taccap_gripper` robot
class. It only *imports* `xense.taccap`, reimplements no device access, and is
not required to use this SDK.

## What's in

- **TC-GU-01 protocol** — async transport, ACK matching, per-command DATA
  subscribers, byte-stuffed framing.
- **`Motor`** — enable / disable / clear-fault, four control modes, blocking
  `set_*` and no-ACK `submit_*`.
- **`ImpedanceController`** — follow a position (teleoperation, leader-follower).
- **`ForcePositionController`** — grasp, with the force and the speed as the
  two knobs.
- **Normalized position** on both roles: `[0, 1]`, 0 = closed, 1 = open, on
  one-shot reads and on every streamed sample.
- **IMU, encoder, LEDs, button**, and power-on auto-calibration.
- **Wrist camera** with fisheye undistortion from the unit's own stored
  intrinsics.
- **`Calibration`** — the flash-persisted fisheye and encoder-max records.
  See [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **`Diagnostics`** — the firmware's own UART counters, which tell a frame the
  MCU never sent from one lost on the way.
- **OTA** and zero-config discovery by firmware-burned SN.

Visuotactile (OG) capture lives at the Python level via the `xensesdk` wheel —
`xense.taccap` is the gripper-protocol + wrist-camera surface only.

Full per-commit history in [CHANGELOG.md](CHANGELOG.md).

---

## Firmware

A gripper runs its own firmware, and this SDK requires a recent one.

**`FollowerGripper` refuses to open a follower below 1.2.5** — it throws rather
than warns, because older firmware lacks stall protection on the control path
and puts normalized 0.0 somewhere other than the closed stop.
`Config::allow_outdated_firmware=True` inspects such a device without driving
it. Leaders are not gated. The motor inside has a floor of its own; see
[Motor firmware](#motor-firmware-10504-or-newer).

**Flashable images ship in [`firmware/`](firmware/)** — the firmware source
does not. Pick the image by the gripper's role, which is the last character of
its firmware SN (`m` master, `s` slave), not by which hand it is on:

```bash
python python/examples/ota_update.py slave left    # role selector picks the image
python python/examples/ota_update.py --all         # every attached gripper
```

Flashing the wrong role's image bricks the MCU and needs an SWD probe to
recover, so check the SN before you flash.

**Power-cycle after any flash.** Unplug **both the USB cable and the power
cable, at the same time**, then reconnect — they feed different domains, so
pulling only one leaves the other half of the board energised and does not reset
it. The bank-swap reboot is a soft reset that leaves the device looking healthy
while quietly dropping status frames.

**The two roles carry independent version numbers.** At the time of writing the
leader is 1.2.4 and the follower 1.2.6; neither is behind the other, and
`gripper.firmware_version` returning different numbers for the two halves of a
pair is normal. Compare versions only within a role — the floors above are
follower numbers. A leader in the field may also report 1.2.5 or 1.2.6 from a
period when the roles were forced onto one number; that is the same code as
1.2.4, so flashing the current leader image lowers the number it reports, which
nothing refuses and nothing should read as a downgrade.

[`firmware/README.md`](firmware/README.md) has the image table, CRC32 values and
the rest of the flashing detail.

### Motor firmware: 1.0.5.0.4 or newer

This is the RobStride EL05 **inside** the follower, not the gripper MCU, and the
SDK does **not** enforce it: reading the motor's version needs follower firmware
1.2.6+ (`motor.motor_version()`, command `0x58`) and has so far only been
confirmed to answer under the private protocol, so a startup check has nothing
reliable to stand on. It is a documented requirement.

Measured before and after upgrading the same unit: on 1.0.5.0.2 the velocity
feedback read as a constant that did not track motion, with 117% motion ripple;
on 1.0.5.0.4 the same unit behaved normally. Because it is the same device
before and after, this is not a unit-to-unit difference — but causation is not
proven, and the corrupted velocity feedback is the part worth remembering:
`snapshot().observation.velocity` and anything derived from it goes wrong while
looking merely odd rather than broken.

Motor firmware can only be updated with the vendor's own upper-computer tool and
USB-CAN adapter, with the motor detached from the gripper; the protocol is not
public. It is not something you can script on a bench, so confirming it when a
gripper arrives is cheaper than debugging it later.

## Install

```bash
mamba env create -f environment.yml && mamba activate xense-taccap
uv pip install -e . --no-build-isolation
python -c "import xense.taccap as t; print(t.__version__)"
```

`uv` ships in the env and targets the activated conda env on its own. `pip`
works too — the flags are the same.

Then wire up the checks once, so they run at commit time rather than only in
CI:

```bash
pre-commit install
```

**Why `--no-build-isolation`.** `environment.yml` pins the build dependencies
(`pybind11`, `scikit-build-core`) next to the C++ ones (`libopencv`, `spdlog`),
and this flag says "build against what the env pins". Without it the installer
builds in a throwaway environment against a `pybind11` fetched from PyPI —
verified: `pybind11_DIR` then points into `~/.cache/uv/builds-v0/...` instead of
the env. pybind11 is header-only, so whichever copy is present at build time is
the one compiled into the extension, and this codebase is version-sensitive
there (see `python/tests/test_numpy_views.py`).

Two gotchas that cost people an afternoon each:

- Install from an **activated** env, not by absolute path. Otherwise cmake finds
  a `ninja` on `PATH` that is actually GNU Make and fails with a confusing
  version error.
- A C++ or bindings change is **not live in a consumer env until you reinstall
  there** — the editable install redirects Python sources to the checkout but
  keeps serving the compiled extension from `site-packages`.

Prerequisites, device permissions, C++-only builds, rebuild/clean and the
`PYTHONPATH` / `LD_LIBRARY_PATH` traps: **[docs/INSTALL.md](docs/INSTALL.md)**.

---

## Usage

The twelve scripts in `python/examples/` are the fastest way to learn this SDK:
each is the smallest program that exercises one capability, and they are also
the tools we bring hardware up with. Run them first, then write your own against
the same calls.

### One selector, everywhere

Every script takes the same **positional** argument:

```bash
python python/examples/follower_status.py left      # by side
python python/examples/follower_status.py right
python python/examples/follower_status.py TCGU01A28Z0015s   # by firmware SN
python python/examples/follower_status.py           # omit when exactly one is plugged in
```

There is no `--side` / `--sn` / `--device` flag — a rig has a left and a right
of everything, and the side is the only handle that means the same thing for the
gripper, its wrist camera and its tactile pair. Side comes from the
**firmware-burned SN** (`Cmd::GetSn`), never the CH343 USB-chip SN. With two
grippers plugged in and no argument, a script refuses rather than guessing which
half of the rig you meant; `calibrate.py` always requires it, because it writes.

### Bringing up a gripper, in order

```bash
# 1. Is it there, and what is it?  (read-only)
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn, g.mcu_device)"

# 2. Read its state. Every field explained in the script's header.  (read-only)
python python/examples/follower_status.py left

# 3. Configure the firmware motion-safety envelope. NOT enabled out of the box,
#    and it is the only protection layer on the MIT path nothing can bypass.
python python/examples/impedance_control.py left --show-envelope
python python/examples/impedance_control.py left --set-envelope --peak 2.0 --cont 1.6

# 4. First motion, interactively, with j/k/o/c keys.
python python/examples/gripper_console.py left

# 5. Grasp something.
python python/examples/force_position_control.py left --grasp-torque 1.1
```

Step 3 before step 4 is the load-bearing order — see
[The motion-safety envelope](#the-motion-safety-envelope) for what it protects
against and why the default is off.

### By task

**Control.** Both controllers take the same two non-blocking calls,
`set_target(0..1)` and `snapshot()`:

```bash
python python/examples/impedance_control.py left                  # follow a position
python python/examples/impedance_control.py left --targets 1.0,0.5,0.0
python python/examples/force_position_control.py left             # grasp: bounded torque
python python/examples/gripper_console.py left --mode force-position
python python/examples/control_and_read.py left                   # read state WHILE controlling
python python/examples/control_ripple.py left --controller both   # measure tracking quality
```

`control_and_read.py` answers the question everyone hits second: during control,
read `snapshot()` — never `motor.read_status()`, whose ACK collides with the
control frames on the same serial link.

**Calibration** — required before normalized `0..1` position means anything:

```bash
python python/examples/calibrate.py left          # leader: encoder zero + travel span
python python/examples/fisheye_cal.py show left   # inspect the flash-persisted records
python python/examples/read_intrinsics.py left --out cal.json    # read-only, JSON out
```

**Camera and leader:**

```bash
python python/examples/wrist_camera.py left --undistort
python python/examples/leader_normalized_position.py left
```

**Firmware:**

```bash
python python/examples/ota_update.py slave left   # then unplug USB + power together
```

### What each one does to the device

Check this column before running anything on a rig that matters.

| | scripts |
|---|---|
| **Read-only** | `follower_status`, `read_intrinsics`, `wrist_camera`, `leader_normalized_position`, `fisheye_cal show` |
| **Moves the motor** | `impedance_control`, `force_position_control`, `control_and_read`, `control_ripple`, `gripper_console` |
| **Writes flash** | `calibrate`, `fisheye_cal set-*`, `--set-envelope` on `impedance_control` / `gripper_console` |
| **Flashes firmware** | `ota_update` — destructive; afterwards unplug USB and power together, then reconnect |

### Two shared modules, not runnable

`_target.py` is the selector above plus version/colour helpers, imported by
every script but `wrist_camera.py`. `_calib_flow.py` is the guided calibration
walkthrough, used by the three scripts that can meet an uncalibrated unit. Both
stay out of `xense.taccap` deliberately: they prompt on stdin, and a library
that blocks on stdin breaks every headless consumer.

Per-script detail, including what each flag measures, is in
**[docs/EXAMPLES.md](docs/EXAMPLES.md)**. C++ examples build by default into
`build/cpp/examples/` (`-DTACCAP_BUILD_EXAMPLES=OFF` to skip them); the wheel
build turns them off on its own.

## Writing your own

The examples above are thin wrappers over the calls below. **Which half you have
decides the API**: a *leader* is the hand-held master you **read** — encoder,
IMU, opening angle. A *follower* is the actuated gripper you **drive** — a motor
behind a controller. The role is the last character of the firmware SN (`m`
master, `s` slave), not which hand it is on.

### Leader — read it

```python
import xense.taccap as t

g = t.LeaderGripper.open()          # the one attached gripper; MCU only, camera off
g.start_streaming(imu_hz=100, encoder_hz=100)

enc = g.encoder.on_data(lambda s: print("enc", s.position_rad, s.position))
imu = g.imu.on_data(lambda s: print(s))
# ... do work ...
g.stop_streaming()
```

`LeaderGripper.open()` throws `IoError` unless exactly one gripper is attached —
name the device explicitly for a bilateral rig (below). For `s.position` to mean
anything the unit needs its travel span calibrated; see
[Normalized leader position](#normalized-leader-position-0--closed-1--open).

### Follower — drive it

Never drive the motor with the raw `submit_*` primitives: they are bare MIT
frames with none of the host-side protection on their path. Use a controller.

```python
import xense.taccap as t

eps = t.find_follower()                       # or t.find_left() / t.find_right()
g = t.FollowerGripper(eps.mcu_device)

c = t.ForcePositionController(g)              # grasping: bounded torque
g.motor.clear_fault()
c.start()                                     # START BEFORE ENABLE — start()
g.motor.enable()                              # validates the motor's stored limit
try:
    c.set_target(0.0)                         # 0 = closed, 1 = open; non-blocking
    while True:
        s = c.snapshot()                      # one consistent view, no bus traffic
        print(s.state, s.observation.position, s.commanded_torque_nm)
        if s.holding or s.arrived:
            break
finally:
    c.stop()                                   # zero torque, then DISABLES the motor
```

Two calls are the whole interface: `set_target(0..1)` and `snapshot()`, both
non-blocking, safe to stream every frame. Swap in `t.ImpedanceController(g)` —
same two calls — when you want to *follow a position* rather than grasp.
`with t.ForcePositionController(g) as c:` does the `start()` / `stop()` pair for
you.

Before driving a follower for the first time, write the firmware motion-safety
envelope once; it is off out of the box. See
[The motion-safety envelope](#the-motion-safety-envelope) and
[Follower gripper control](#follower-gripper-control-mit-force-position).

### Both sides in one process

```python
from xense.taccap import FollowerGripper, scan_grippers, Side

eps = scan_grippers()                          # one USB sweep, no re-probe race
left  = next(e for e in eps if e.side == Side.Left)
right = next(e for e in eps if e.side == Side.Right)

g_left  = FollowerGripper(left.mcu_device)     # or LeaderGripper, per role
g_right = FollowerGripper(right.mcu_device)
```

Each gripper owns its own serial link and background threads, so the two are
independent — but give each its own controller; never point two controllers at
one gripper.

### Wrist camera

`open()` does not touch the camera: an external camera service usually owns the
V4L2 devices. Ask for it explicitly, or open it standalone with `t.Camera`.

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True)
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

Visuotactile (OG) sensors are read through the `xensesdk` wheel, not this SDK.

Runnable versions of all of the above are the scripts in
[Usage](#usage) above.

### Serial numbers (TacCap SN scheme)

The firmware-burned SN encodes both the side and the leader/follower role:

```
  TCGU01 A24 Z 0001 m        gripper      GSPS01 A24 Z 0001   visuotactile
  └─┬──┘ └┬┘ │ └┬─┘ │                                          (no patch suffix)
 product batch│  seq patch    product : TCGU01 gripper / GSPS01 sensor
              line            line    : Z = R&D/test, A = production
                              seq     : last digit odd → Left, even → Right
                              patch   : m = Master (leader), s = Slave (follower)
```

`scan_grippers()` parses this for every gripper; each `GripperEndpoints`
carries `.side` (`Side.Left/Right`) and `.role` (`Role.Leader/Follower/
Unknown`). Pick a unit by side **or** role:

```python
from xense.taccap import find_left, find_right, find_leader, find_follower, parse_serial

eps = find_leader()              # the gripper whose SN patch suffix is 'm'
p   = parse_serial("TCGU01A24Z0001m")
print(p.side, p.role, p.valid)   # Side.Left  Role.Leader  True
```

`parse_serial()` degrades gracefully: a legacy (`SN000002`) or empty SN
still yields a best-effort `side` (last digit) with `role = Role.Unknown`
and `valid = False`.

### Encoder zero calibration

```python
# Hold the gripper at the desired zero pose (usually fully closed) first.
g_right.encoder.set_zero()                      # throws ProtocolError on NACK
s = g_right.encoder.read_once()
print(s.position_rad, s.raw_position_rad)       # cooked (clamped >= 0) vs raw
```

See `python/examples/calibrate.py` for the full interactive walkthrough
(side selection by SN, pre/post drift display, full-open angle sanity
check, live readout).

### Normalized leader position (0 = closed, 1 = open)

`normalize_position=True` reads the firmware's encoder-max calibration
(`Cmd::EncoderMaxCal`, firmware ≥ V2.1) at open() time and installs the
converter on the encoder, so every sample carries `.position` in `[0,1]`:

```python
g = LeaderGripper(mcu_device=dev, normalize_position=True)

s = g.encoder.read_once()
s.position_rad      # 0.65  — always radians, meaning never changes
s.position          # 0.50  — normalized; float('nan') when the flag is off

g.position()        # 0.50  — one-shot read + convert
g.pos_to_rad(1.0)   # 1.30  — full open, in raw radians
g.rad_to_pos(0.325) # 0.25

# Streamed samples are normalized too, including subscribers registered
# before the map existed.
g.encoder.on_data(lambda s: print(s.position))
g.start_streaming(imu_hz=0, encoder_hz=100)
```

The travel span is the encoder shaft angle at full open, measured from the
encoder zero (fully closed) — so **zero the encoder first**, then store the
span. `measure-encoder-max` walks both steps:

```bash
python python/examples/fisheye_cal.py measure-encoder-max
```

Construction raises `ProtocolError` when the span has never been calibrated
(the firmware answers `CalNotSet` rather than returning a bogus zero) or when
the firmware predates V2.1. Pass `encoder_max_rad=<rad>` to supply the span
from the host and skip the firmware read entirely.

`position()` / `pos_to_rad()` / `rad_to_pos()` / `position_map` work without
the flag — it only controls whether `EncoderSample.position` gets filled in.
Call `reload_position_map()` after re-calibrating.

### Fisheye camera calibration

Fisheye intrinsics + distortion live in MCU flash and are readable from both
leader and follower:

```python
from xense.taccap import CameraFisheyeCal, FisheyeUndistorter

cal = g.calibration.read_fisheye()     # None when never calibrated
if cal is not None:
    # Prefer FisheyeUndistorter over calling cv2.fisheye.undistortImage with
    # cal.K / cal.D by hand: it builds the remap tables once, resamples with
    # INTER_CUBIC and applies the same focal-length balance as the PC
    # calibration tool, so a frame rectified here matches one rectified there.
    # `cal.K` / `cal.D` remain exposed for code that must do its own thing.
    undistorter = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
    undistorted = undistorter.apply(img)

g.calibration.write_fisheye(CameraFisheyeCal(
    fx=320.5, fy=321.0, cx=319.5, cy=240.2,
    k1=-0.031, k2=0.0072, k3=-0.0013, k4=0.0002))
```

The firmware stores the values verbatim — no unit conversion, no clamping,
only NaN/Inf rejection. `python/examples/fisheye_cal.py show` prints both
records; `set-fisheye --from-npz` loads `K`/`D` straight from an OpenCV
calibration file.

### Follower gripper control (MIT force-position)

The follower drives a FDCAN motor. Control is the **MIT impedance frame** — the
force-position hybrid primitive: the `kp`/`kd` terms track a target position,
the feed-forward torque adds a force component. The SDK exposes it three ways.

```python
import xense.taccap as t
g = t.FollowerGripper.open()
g.motor.clear_fault()
g.motor.enable()                 # required before anything moves

# Motion goes through a controller. The raw `submit_*` primitives are exposed
# too, but they write a control frame straight to the wire with no error clamp
# and no torque ceiling — the firmware envelope is then your only protection.
c = t.ImpedanceController(g)     # defaults are the tuned values
c.start()                        # seeds the target with the current position
c.set_target(0.35)               # normalized [0,1], 0 = closed
s = c.snapshot()                 # state + observation + command, non-blocking
c.stop()
```

**Normalized position** — work in `[0, 1]` (0 = closed, 1 = open) instead of raw
radians. Requires a calibrated gripper (`GripperConfig` Valid); throws otherwise.
```python
print(g.position())                       # -> 0.97   (nearly open)
g.pos_to_rad(0.5), g.rad_to_pos(-0.59)    # explicit conversions
```

The one-shot `FollowerGripper::set_position(0..1)` is **C++ only** — from Python
a normalized target goes through a controller's `set_target()`. The raw
`Motor.submit_*` primitives *are* exposed to Python and do take raw radians, but
nothing on the host side clamps them; see the warning above.

**`ImpedanceController`** — use this one to **follow a position**. A C++
background thread submits the latest normalized target **in phase with the
motor-status stream** while that stream keeps a thread-safe observation fresh.
Your policy only touches `set_target(0..1)` and `snapshot()`, both non-blocking
(no GIL fights, no status polling).

```python
c = t.ImpedanceController(g)              # t.ImpedanceConfig() to tune kp/kd/budget
c.start()                                 # seeds target = current pos (no jump)
try:
    while running:
        s = c.snapshot()                  # one lock, one consistent view
        if s.state == t.ImpedanceState.FAULT:
            break                         # s.fault_reason says why; c.reset() resumes
        obs = s.observation               # .position [0,1], .velocity, .torque, .age_ms
        c.set_target(policy(obs))         # your action, 0..1
finally:
    c.stop()                              # zero torque, then leaves the motor disabled
```

A blocked jaw is not a fault here: the error clamp saturates at
`max_position_torque_nm` (default 1.1 Nm, the EL05's continuous stall rating)
and holds there. Contact needs no detecting; saturation is what it looks like.

**`ForcePositionController`** — the one for **grasping**. It takes the grip
force with each command (`set_target(p, grasp_torque_nm)`) and reports
`holding`, so a blocked jaw settles at exactly the force you asked for.

```python
fp = t.ForcePositionController(g)         # defaults are the tuned values
fp.start()
try:
    fp.set_target(0.0)                    # close; 0 = closed, 1 = open
    while True:
        s = fp.snapshot()
        if s.holding: break               # gripping the object
        if s.arrived: break               # reached the target, nothing there
finally:
    fp.stop()                             # commands zero torque and disables
```

**Two parameters, and only these two are task-specific:**

| | default | what it is |
|---|---|---|
| `grasp_torque_nm` | **1.1 Nm** | the grip force. Free travel does not use it; a blocked jaw settles here |
| `close_speed_radps` | **0.5 rad/s** | travel speed |

```python
cfg = t.ForcePositionConfig()
cfg.grasp_torque_nm = 0.4        # gentler grip
fp = t.ForcePositionController(g, cfg)
```

Everything else has one right answer for this gripper and is not exposed: the
position gain, the travel damping, the arrival radius. They were measured on this
hardware, not guessed, and changing them is far likelier to make the gripper
judder than to improve it.

**Two observations, not states.** `snapshot().holding` means the setpoint has run
as far ahead of the jaw as the force budget allows and the jaw is not following —
that is what gripping *is*. `snapshot().arrived` means it reached the commanded
position. Nothing else needs interpreting.

> **How hard, and for how long.** 1.1 Nm is the EL05's continuous stall rating.
> Measured on a real workpiece at 24 V: a 600 s hold took the winding 36 → 70 °C
> with the rate decaying 8 → 1 °C/min, which fits a plateau near 75 °C, about
> 15 °C under the firmware's temperature wall — no fault, no link loss, no drift.
> Grips of seconds to minutes are comfortably inside that. For a hold measured in
> *tens of minutes*, or a warm cabinet, drop the force: 0.6 Nm settles flat at
> 49 °C. Above the device's continuous envelope the firmware derates rather than
> failing, and `start()` warns you.

**LEDs and power-on auto-calibration (V1.9):**

```python
g.led.set(t.Ws2812Mode.Override, 0, 255, 0, brightness=120)   # solid green
g.led.effect(t.Ws2812EffectType.ColorBreathe, 0, 0, 255)      # blue breathe
g.led.off()

cfg = g.get_auto_cal_config()             # if enabled, the firmware self-zeros
g.set_auto_cal_config(cfg)                # (close-to-stall) + captures max_open
                                          # (open-to-stall) on power-up
```

### The motion-safety envelope

**Write the motion-safety envelope first.** It is the firmware's torque and
thermal protection, and it is **off on a factory device**
(`GripperConfig.reserved` all zero). Until you write it, it does not exist:

```bash
python python/examples/impedance_control.py right --show-envelope        # read
python python/examples/impedance_control.py right --set-envelope \
       --peak 2.0 --cont 1.6                                             # write + enable
```

| Flag | Default | Meaning |
|---|---|---|
| `--peak` | 2.0 N·m | Transient ceiling during motion. **It also sets the approach speed**, roughly `peak/kd` |
| `--cont` | 1.6 N·m | Sustainable ceiling, the floor of the I²t derate |
| `--temp-derate-start` | 0 → firmware 90 °C | Where the thermal derate begins |
| `--temp-wall` | 0 → firmware 100 °C | Temperature wall; above it only 0.30 N·m remains |

The envelope lives in the `GripperConfig` record (commands `0x66`/`0x67`, **no
protocol change**), survives power loss, and is written once per device. It is
enforced in the firmware's MIT branch — the one point every motion command must
pass — so it holds **even against MIT frames sent without this SDK**. That is
precisely why it belongs in firmware rather than here.

Without it, a jaw driven at `kp=20` into a rigid object asks for around 12 N·m;
at 24 V that current demand collapses the rail, the motor's undervoltage
protection fires and the grip drops. With the envelope enabled the same test
holds for 25 s. See
[`docs/CONTROL_LAYERING.md`](docs/CONTROL_LAYERING.md).

**Two behaviours worth knowing:**

- **Grip force is not constant.** The I²t derate and the temperature wall apply
  to a pure-torque hold as well. `grasp_torque_nm` is what you ask for; the
  firmware lowers what actually comes out as the motor heats.
- **Do not poll `read_status()` while controlling.** The stream-locked phase
  protects telemetry frames, not ACKs: the request can land in the window the
  MCU is transmitting and destroy a telemetry frame, and its own reply can be
  corrupted by the control loop's submit. Position, velocity, torque, status
  bits and temperature are all in `snapshot()` — nothing needs the bus.

Runnable demos: `python/examples/impedance_control.py`,
`python/examples/force_position_control.py`, and
`python/examples/gripper_console.py` (interactive, both controllers).

## Diagnostics

`g.diagnostics` wraps the firmware's own UART counters (firmware 1.1.3+) and a
runtime log switch (1.1.4+). Both work on either gripper role.

```python
s = g.diagnostics.uart_stats()
# tx_calls_ok / tx_bytes_ok  — what the firmware's transmit call accepted,
#                              i.e. what actually reached the MCU's TX register
# tx_fail_timeout            — the firmware truncated frames itself
# rx_overflow                — the firmware's command task fell behind the host
# debug_tx_bytes             — bytes out the DEBUG UART; quantifies logging cost
```

The point of `tx_calls_ok` is attribution. Compare it against what the host
decoded over the same window: if the counts agree but bytes are missing, the
loss happened **after** the bytes left the MCU (cable or USB-serial bridge) and
no firmware change reaches it.

```python
from xense.taccap import LogLevel, LOG_OUTPUT_UART
g.diagnostics.set_log_config(LogLevel.DEBUG, LOG_OUTPUT_UART)   # on
g.diagnostics.disable_logging()                                  # off again
```

> **Logging is a diagnostic lever, not a setting.** Firmware 1.1.4 ships with it
> off because the log sink is a blocking polled UART write (~0.5 ms per line at
> 921600) that stalls whichever task emitted the line — logging on every received
> command is what livelocked the command channel. Output also goes to the MCU's
> DEBUG UART, which is **not routed over USB**: without a probe on that pin you
> pay the realtime cost and see nothing.

## Logging

The SDK uses **one singleton logger** named `"xense.taccap"` —
registered with spdlog, shared by every C++ TU and the Python
binding. Don't construct your own `std::make_shared<spdlog::logger>`
elsewhere, and don't reach for `std::cout` / `print` / `printf`
for diagnostic output — they bypass the file sink.

- **C++**: `#include <taccap/log.hpp>`, then `xense::taccap::logger()->info(...)`.
- **Python**: `from xense.taccap import log; log.info(...)` /
  `log.set_level("debug")` / `log.set_pattern(...)`. `set_level` and
  `set_pattern` affect the **console sink only** — the file sink keeps
  its archive format for grep stability.

Two sinks attached by default:

| Sink               | Level                             | Pattern                               |
| ------------------ | --------------------------------- | ------------------------------------- |
| stderr (colour)    | user-controllable, default `INFO` | `[%D %T.%e] [%n] [%^%l%$] %v`         |
| file (per-session) | always `DEBUG`                    | `[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v` |

File-sink behaviour:

- Directory: `$TACCAP_LOG_DIR` if set, else `~/.taccaplogs/`.
- Filename: `session_YYYYMMDD_HHMMSS.log` — one new file per process start.
- At most **10** session logs retained; oldest mtime pruned at startup.
- File-sink creation failures (disk full / permission denied) are not
  fatal — the console sink keeps working.

## Layout

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/        # Public C++ headers
│   ├── src/                   # SDK implementation (protocol, bus, components, ...)
│   ├── examples/              # C++ example programs (leader_demo)
│   └── tests/                 # gtest unit tests
├── python/
│   ├── bindings/              # pybind11 module sources
│   ├── examples/              # Python examples
│   └── xense/taccap/          # Python package (PEP 420 namespace under `xense`)
├── third_party/
│   └── firmware/              # Clone-on-demand reference repos (gitignored)
│       ├── tc-gu-01/          #   STM32 firmware that runs on the gripper
│       └── tc-gu-01-pc/       #   PyQt debug GUI (operator-side)
├── docs/                      # Architecture & API docs
├── environment.yml            # mamba env (Python 3.12, conda-forge only)
├── pyproject.toml             # scikit-build-core wheel config
└── CMakeLists.txt             # Top-level build orchestrator
```


## Documentation

- **[docs/USAGE.md](docs/USAGE.md)** — end-to-end usage: bringing up all three
  data paths — tactile (OG), wrist camera, and gripper readout/control. Also in
  Chinese as [docs/USAGE_CN.md](docs/USAGE_CN.md).
- **[docs/INSTALL.md](docs/INSTALL.md)** — prerequisites, C++-only builds,
  device permissions, rebuild/clean, environment traps.
- **[docs/CALIBRATION.md](docs/CALIBRATION.md)** — encoder zero + travel span,
  the flash-persisted records, drift handling.
- **[docs/FIRMWARE.md](docs/FIRMWARE.md)** — reference repos, building the
  firmware, and flashing over OTA. **Read the power-cycle note before you
  measure anything after a flash.**
- **[docs/EXAMPLES.md](docs/EXAMPLES.md)** — what each example script does.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — layered stack, module map,
  threading model, and the boundary between this SDK and downstream consumers.

## Citation

If this SDK supports published work, please cite the release you actually used —
the behaviour it documents is version-specific, and several defaults have moved
between releases.

```bibtex
@software{taccap_gripper,
  title        = {taccap-gripper: an SDK for the TacCap multimodal tactile gripper},
  author       = {{XenseRobotics Co., Ltd.}},
  year         = {2026},
  url          = {https://github.com/XenseRobotics-AI/TacCap-Gripper},
  version      = {VERSION},
  license      = {Apache-2.0}
}
```

Replace `VERSION` with the tag you built against — `python -c "import
xense.taccap as t; print(t.__version__)"` reports what is installed. Firmware
matters too: a measurement depends on the gripper firmware as much as on this
SDK, so state that version as well (`g.firmware_version`).

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.

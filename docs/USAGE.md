# Usage — bringing up the three data paths

> A Chinese version is available at [USAGE_CN.md](USAGE_CN.md). This English file is authoritative.

<!-- For anyone bringing the hardware up for the first time: one section each for
     tactile, vision, and gripper readout/control, and each section is the complete
     path from plugging the cable in to holding the data. API detail is in the README
     and the topic docs. -->

A TacCap-Gripper carries three independent data paths, and **each is owned by a
different software stack**. That is the single most important thing in this document:

| Data | Hardware | Owned by | Device node |
| --- | --- | --- | --- |
| **Gripper** readout and control | STM32 MCU (IMU / encoder / motor) | **this SDK**, `xense.taccap` | `/dev/serial/by-id/...-if02` |
| **Vision**, the wrist camera | XC-series UVC camera | **this SDK**'s `Camera` (opt-in) | `/dev/video*` |
| **Tactile**, the visuotactile (OG) sensor | GSPS01 visuotactile module | **the `xensesdk` wheel, not part of this SDK** | `/dev/video*` |

The three share no session: opening one does not require opening another, and one of
them breaking does not drag the other two down. The gripper speaks over a serial port
and the two cameras over USB video, so they never contend for the same handle — but
they do **share USB bandwidth**; see [Running all three at once](#4-running-all-three-at-once).

> **Every example script selects a device with the positional argument `left` /
> `right`** (or with a serial number directly). There is no `--sn` / `--side` /
> `--device` flag — see [EXAMPLES.md](EXAMPLES.md#选谁统一用-left--right).

Installation, device permissions and the `PYTHONPATH` / `LD_LIBRARY_PATH` traps are
covered in [docs/INSTALL.md](INSTALL.md). Everything below assumes the `xense-taccap`
env is activated and `import xense.taccap` works.

---

## 0. First, confirm the devices are there

Each of the three enumerates differently, so **confirm them separately before you
write any application code** — most "I can't read any data" reports end up being a
device that was never recognized.

```bash
# Gripper (MCU): discovered by the firmware-burned SN, no udev rule needed
python -c "from xense.taccap import scan_grippers, Side
for g in scan_grippers():
    s='L' if g.side==Side.Left else 'R'
    print(f'  [{s}] {g.role} ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"

# Vision (wrist camera) + tactile (OG): both under /dev/v4l/by-id, told apart by serial
python python/examples/wrist_camera.py --list

# Tactile (OG): scanned by xensesdk itself, returns {serial: cam_id}
python -c "from xensesdk import Sensor; print(Sensor.scanSerialNumber())"
```

Healthy output: the gripper line prints `[L]` / `[R]` with a non-empty `fw_sn`; the
`--list` line lists the wrist camera (`XC…`) and the visuotactile sensor (`GSPS01…`)
separately; the OG line prints a dict of the form `{'OG000352': 10}`. An empty `fw_sn`
means the firmware has not burned an SN yet (or the firmware predates V1.6), and the
side then comes back as `Side.Unknown`.

> **All three device classes are told apart by serial number, not by device number.**
> The number in `/dev/videoN` moves with plug order, and the wrist camera and the
> visuotactile sensor enumerate right next to each other — get it wrong and you open
> the tactile sensor as if it were the camera. The serial syntax is the same one the
> gripper firmware SN uses (**odd last digit = left, even = right**, `m` = leader /
> `s` = follower):
>
> | Device | Syntax | Example |
> | --- | --- | --- |
> | Gripper | `TCGU01<batch><line><seq><m\|s>` | `TCGU01A28Z0116m` |
> | Wrist camera | `XC<batch><line><seq><m\|s>` | `XCA28Z0116m` |
> | Visuotactile | `GSPS01<batch><line><seq>` | `GSPS01A31Z0049` |
>
> A wrist camera's sequence number matches that of the gripper it sits on, so the one
> `left` / `right` selector locates both the gripper and its wrist camera.

> **The wrist camera is absent from discovery by design.** The discovery layer only
> knows about MCUs: a UVC device is usually held by an external camera service, and
> the SDK does not fight it for the handle. So a gripper object does **not** open the
> wrist camera by default (`open_cameras=False`); pass the device path yourself when
> you want it.

---

## 1. Gripper: readout and control

### 1.1 Opening one

```python
import xense.taccap as t

g = t.LeaderGripper.open()        # when exactly one is plugged in; more than one throws IoError
```

With several attached, don't use `open()`. Take one scan, then pick by side or role —
that avoids re-probing:

```python
from xense.taccap import LeaderGripper, FollowerGripper, scan_grippers, Side, Role

eps   = scan_grippers()
left  = next(e for e in eps if e.side == Side.Left)
lead  = next(e for e in eps if e.role == Role.Leader)
g     = LeaderGripper(mcu_device=left.mcu_device)
```

Leader and follower are **two different pieces of hardware**, decided by the SN suffix
(`m` = leader, `s` = follower); the class only decides which command surface you get —
picking the wrong one is not corrected for you, it just NACKs when you send a command.

### 1.2 Reading: one-shot vs streaming

One-shot reads block waiting for an ACK, which suits low-rate work like calibration
and self-tests:

```python
s = g.encoder.read_once()
print(s.position_rad, s.raw_position_rad)   # cooked (clamped >= 0) vs raw
print(g.imu.read_once())
```

Streaming — the firmware pushes at a fixed rate and your callback runs on a background
thread — is the proper way to collect data:

```python
sub_e = g.encoder.on_data(lambda s: print("enc", s.position_rad))
sub_i = g.imu.on_data(lambda s: print("imu", s.accel_mps2, s.gyro_radps))

g.start_streaming(imu_hz=100, encoder_hz=100)   # leader: either stream can be turned off on its own (0)
...
g.stop_streaming()
```

A follower has only one stream: **motor status**. IMU and encoder are compiled out of
the firmware for that role, so the signature differs, and `motor_hz=0` throws
`IoError(EINVAL)` outright rather than opening an empty stream:

```python
f = t.FollowerGripper.open()
f.motor.on_status(lambda s: print(s.actual_pos, s.actual_torque))
f.start_streaming(motor_hz=100)     # the firmware divides 1 kHz; only divisors of 1000 land exactly
```

> The rate is not a free parameter: the firmware divides 1 kHz, and a rate that is not
> a divisor gets silently adjusted. The SDK warns when it detects this, but does not
> NACK.

### 1.3 Normalized opening (0 = closed, 1 = open)

The raw encoder reads in radians, while policies and datasets usually want `[0, 1]`.
With `normalize_position=True`, both one-shot reads and every streamed sample carry
`.position`:

```python
g = t.LeaderGripper(mcu_device=dev, normalize_position=True)
s = g.encoder.read_once()
s.position_rad     # 0.65  — always radians, meaning never changes
s.position         # 0.50  — normalized; nan when the flag is off
g.position()       # 0.50  — one-shot read + convert
```

This relies on the **travel span** stored in firmware (`Cmd::EncoderMaxCal`, firmware
≥ V2.1). On an uncalibrated unit, construction raises `ProtocolError` (the firmware
answers `CalNotSet` rather than handing back a bogus 0). The calibration order is
**zero first, then store the span**:

```bash
python python/examples/calibrate.py left            # interactive: zero + store span
python python/examples/fisheye_cal.py measure-encoder-max   # just the span step
```

Details in [docs/CALIBRATION.md](CALIBRATION.md).

### 1.4 Control (follower only)

A follower drives a FDCAN motor, and the control primitive is the **MIT impedance
frame** (the force-position hybrid): `kp`/`kd` track a target position and the
feed-forward torque adds the force component. The motor must be enabled before
anything moves, but **the order is `start()` first, then `enable()`**: both
controllers' `start()` reads the device's persisted `0x700B` startup limit and
validates the config against it, and that check is only worth anything before the
motor can move.

```python
f = t.FollowerGripper.open()
f.motor.clear_fault()
```

**The recommended path — `ImpedanceController`**: use it whenever you follow a
position target (teleoperation, leader-follower forwarding). A C++ background thread
submits in phase with the motor-status stream, and your policy touches only two
non-blocking calls:

```python
cfg = t.ImpedanceConfig()
cfg.kp = 20.0                      # stiffness Nm/rad: error-to-torque ratio, how stiff it feels
cfg.kd = 1.0                       # damping Nm·s/rad: suppresses oscillation, and sets approach speed (below)
cfg.feedforward_torque = 0.0       # constant feed-forward Nm on top of the PD. For gravity/preload; normally 0
cfg.max_position_torque_nm = 1.1   # error clamp Nm (this is the protection): the commanded target is held
                                   #   within ±(1.1/kp)=0.055 rad of the measured position, and the approach
                                   #   speed is thereby fixed at about 1.1/kd = 1.1 rad/s. A blocked jaw
                                   #   saturates here and holds
cfg.rated_torque_nm = 1.8          # torque ceiling Nm (the backstop), acting on the "measured" torque, not on
                                   #   the command. Once pressed against it, pure feed-forward frames with
                                   #   kp=kd=0 go out instead, pinned at the budget.
                                   #   Must be higher than max_position_torque_nm + |feed-forward|
cfg.status_timeout_ms = 350        # status stream gone this long → FAULT + zero torque + observation invalid
cfg.motor_stream_hz = 100          # status-stream rate Hz; submission locks to this phase, one per frame

c = t.ImpedanceController(f, cfg)
c.start()                         # validates the device limit; seeds the target with
                                  #   the current position, so no jump
f.motor.enable()                  # enable AFTER start() — see above
try:
    while running:
        s = c.snapshot()          # one lock, one consistent view
        if s.state == t.ImpedanceState.FAULT:
            print("fault:", s.fault_reason)
            break
        c.set_target(policy(s.observation))
finally:
    c.stop()                      # commands zero torque first
    f.motor.disable()
```

After `start()`, do **not** call `start_streaming()` yourself or register a
`motor.on_status()` callback — the controller owns the status stream and the control
path exclusively, and every observation comes from `snapshot()`.

#### What each of the seven fields controls

| Field | Default | What it controls |
|---|---|---|
| `kp` | 20.0 Nm/rad | Stiffness. The ratio that turns position error into torque — "how stiff it feels" |
| `kd` | 1.0 Nm·s/rad | Damping. Suppresses oscillation, and also sets the approach speed (below) |
| `feedforward_torque` | 0.0 Nm | A constant feed-forward torque added on top of the PD. For cancelling gravity or preload; normally left at 0 |
| `max_position_torque_nm` | 1.1 Nm | **Error clamp** — this is the protection. The commanded target is confined to within ±(that value / `kp`) of the measured position |
| `rated_torque_nm` | 1.8 Nm | **Torque ceiling**, the backstop. It acts on the **measured** torque, not on the commanded value |
| `status_timeout_ms` | 350 ms | Status stream gone this long → `FAULT` + zero torque + observation marked invalid |
| `motor_stream_hz` | 100 | Status-stream rate. Submission locks to this phase, one submit per frame |

The ones you actually tune per task are the first two plus `max_position_torque_nm`.
The last two are transport parameters, and `rated_torque_nm` normally just sits at the
motor's rated value.

**These two layers of protection are different things — don't conflate them.**

- `max_position_torque_nm` clamps the **command**: `kp × error` is not allowed to
  exceed it. The window is `max_position_torque_nm / kp`, by default 1.1/20 =
  **0.055 rad**. It also fixes the approach speed as a side effect — the jaw
  accelerates until damping balances the clamped torque, roughly
  `max_position_torque_nm / kd`, by default **1.1 rad/s**. So tuning `kd` changes the
  feel and the approach speed at the same time. When the jaw is blocked the command
  saturates at this value **and stays there**, and that is the grip force — contact
  needs no detecting, saturation itself is contact. The default of 1.1 is the EL05's
  continuous stall rating, because nothing puts a clock on that hold.
- `rated_torque_nm` looks at the torque the **motor actually delivers** (inferred from
  current). Once that has pressed against the value for long enough, the controller
  switches to pure feed-forward frames with `kp=kd=0`: position error can no longer
  enter the output at all, structurally, and the torque is **pinned** at the budget
  `max_position_torque_nm` (only when the budget is 0 does it pin at the ceiling)
  rather than merely "estimated not to exceed" it. It is a backstop; normal operation
  never reaches it, because config validation requires
  `max_position_torque_nm + |feedforward_torque| < rated_torque_nm`.
- Why both layers: the clamp governs the commanded value, and in a measured stall on
  1.1.5 the feedback was only about 0.59 of the command — a ratio measured on one
  unit, at one temperature, under one load. **The ceiling does not care about that
  ratio.**

`rated_torque_nm` is capped at **1.8 Nm (rated) rather than 6.0 Nm (peak)**, because
the hold it produces is **indefinite** — nothing puts a clock on it. For a long hold
above the rated value the main risk is not heat: the motor's undervoltage protection
is fast, and a sustained high current collapses the 24 V rail and takes USB down with
it — which shows up on the host as `SerialBus::write: Input/output error`, looking
nothing like a torque fault.

#### The state machine

`IDLE` → `TRACKING` → `TORQUE_CAPPED` → `FAULT`, with priority
**`FAULT` > `TORQUE_CAPPED` > `TRACKING`**.

| State | Meaning |
|---|---|
| `TRACKING` | Following normally. A blocked jaw whose error clamp has saturated at the budget is still in this state |
| `TORQUE_CAPPED` | Measured torque has hit the ceiling; pure feed-forward frames are going out |
| `FAULT` | Status stream lost / motor fault bit / submit failed. Zero torque; needs `reset()` |

**There is no `STALLED`.** The stall guard was removed: it clamped the effective
target to wherever the jaw had stopped, which zeroed the error and collapsed the
torque with it — measured, it let go 60 ms after contact at 0.35 Nm. To tell whether
the jaw is blocked, check whether `snapshot().commanded_torque_nm` has reached
`max_position_torque_nm` (that is exactly how `impedance_control.py` checks).

`snapshot()` additionally gives you a `torque_capped` boolean and a `torque_caps`
count; change gains at runtime with `set_gains(kp, kd, feedforward_torque)`.

The low-level `ControlLoop` that used to sit alongside it has been removed: it was a
second copy of the same control law and had already drifted from this one (different
budget, and it still carried the stall guard that collapses torque).

**Grasping rigid objects / force-limited holding — `ForcePositionController`**: plain
position impedance keeps accumulating `kp × position error` once an object blocks the
jaw, so it is a poor fit for parking the target at the fully-closed point for any
length of time. The force-position hybrid controller uses **one control law with a
bounded torque**, unchanged throughout:

```
command(target, kp, kd, torque budget = grasp_torque_nm)
```

- **Free travel**: the torque required is only as much as friction (measured on this
  mechanism at about 0.1 N·m), far below the budget, so the clamp never engages and
  the jaw follows the ramp at `close_speed_radps`;
- **Blocked**: the jaw stops, the ramp runs ahead, the error clamp saturates, and the
  command settles **exactly** at `grasp_torque_nm` and stays there.

**Contact needs no decision — saturation is contact**, and holding is the natural
consequence of saturation. There is no mode switch.

`grasp_torque_nm` defaults to **1.1 N·m**, the EL05's continuous stall rating.
Re-checked by measurement: the grip is indefinite by construction, so the default has
to be a torque this gripper can genuinely hold forever — which is a question about the
whole machine, not about the motor. Sustained holds on a real workpiece, measured
(24 V, envelope enabled):

| Grip force | Result |
| --- | --- |
| 1.1 N·m (default) | 36→70℃ / 600s, rate 8→4→3→2→1 ℃/min, **extrapolated plateau ≈75℃** |
| 0.6 N·m | 41→49℃ / 600s, 0.00℃/min over the last 60 s, **plateau already reached** |

At 1.1 the rate of temperature rise decays throughout — a first-order approach to a
plateau, not a linear climb — and the plateau sits about 15℃ below the firmware's
90℃ temperature wall. The margin is real but not generous: both runs started from
36–41℃, and a high ambient temperature eats it directly.

Also, `grasp_torque_nm` is **not a hard ceiling**: during a steady hold the measured
feedback runs 5~7% above the budget. The budget shapes the command; what actually
constrains the output is the firmware's motion envelope.

> Before 2026-09 what ran here was a host-side contact state machine (`kp=0`
> velocity-damped close → stall decision → pure `tau_ff` hold). It was removed for
> three reasons: it duplicated what the MCU already does at 500 Hz
> (`task_canmotor_is_stalled()`, and `docs/CONTROL_LAYERING.md` §3 had long since
> assigned contact detection to the firmware); to keep the stall signature clean, the
> travel phase was made `kp=0` **deliberately**, which measured out at 37% velocity
> ripple and a mean speed only 77% of the commanded value; and its most central
> judgement — telling "arrived" apart from "blocked" — is physically continuous in the
> empty-jaw case and therefore unreliable. See `docs/CONTROL_REFACTOR.md`.

**State here is an observation, not a control state.** `snapshot()` derives, every
frame:

| Observation | Meaning |
|---|---|
| `arrived` | `\|position error\| <= arrival_eps_rad` |
| `holding` | The setpoint has run as far ahead as the budget allows (lead ≥ half the limit) **and** the jaw has not kept up (velocity ≤ 25% of the commanded speed) |
| `state` | Derived from those two plus the direction of motion; for display only |

The velocity threshold in `holding` uses the firmware's own
`TASK_CANMOTOR_STALL_VEL_RATIO = 0.25` — deliberately the same number, so that host
and MCU describe the same physical event the same way.

**The criterion is not "does the command use up the budget".** That one is true the
instant the target changes: the setpoint jumps, the command saturates, and the jaw has
not had time to move. Measured, the consequence was that every "grasp" step completed
within 0.01 s at a measured torque of only 0.016 N·m — nothing was gripped, and the
acceptance run was all green. The lead carries its own timing: the ramp is planted on
the jaw, is zero at the start, and only grows once the jaw stops following.

`hold_position` is where the jaw **actually** stopped and `target_position` is the
position originally asked for; the two remain distinguishable.

**What separates the two is the velocity gate, not the torque number.** Measured over
a full unloaded close on a 1.1.5 follower (1578 frames): in free travel `|vel|` never
dropped below 0.183 rad/s, five times the 0.035 threshold, while `|torque|` stayed
≤ 0.142 Nm; at the mechanical stop `|vel|` ≈ 0.012 rad/s and `|torque|` ≈ 0.21 Nm.
**Not a single frame satisfied both conditions at once.** These two criteria now live
only in the firmware — the host-side copy went away with the contact state machine.

**Worth recording in passing, and still true**: with a `kd`-form MIT frame at stall,
the feedback torque does not reach the commanded value. From the same measurement:
commanded 0.359 N·m, feedback saturating at only 0.213 N·m (≈0.59). So any criterion
that "compares feedback torque against the commanded limit" can never be met. The
criterion above — setpoint lead plus velocity — works precisely because both of
its terms are **command-side** quantities, which are reachable by construction.

The controller splits the torque limit into two ceilings with clearly separated
jobs, and those two ceilings are **the motor's own two ratings**, not safety
margins picked by hand:

- `motion_torque_limit_nm`: the **instantaneous** torque ceiling for the velocity
  damping and the PD term during closing, opening and position holding, at most
  **6.0 Nm** — the motor's **peak torque**. A feedback torque above it drops the
  controller into a zero-torque fault state. 6.0 Nm is also the firmware's default and maximum for the
  `0x700B` startup limit (`storage.c`,
  `STORAGE_MOTOR_LIMIT_TORQUE_{DEFAULT,MAX}_NM`), so the default config agrees with a
  factory device.
- `hold_torque_limit_nm`: the ceiling on the **long-term** hold torque, at most
  **1.8 Nm** — the motor's **rated (nominal) torque**. Neither `grasp_torque_nm`
  nor a target torque passed in at runtime may exceed it. Note that it clamps
  **nothing** in the current control law: it only bounds `grasp_torque_nm` and
  warns when that exceeds the motor's actual rating.

`start()` reads the V2.2 persisted `0x700B limit_torque` and requires the device's
value to be no higher than the configured motion ceiling. What is persisted here is
the **startup configuration in the gripper MCU's flash**, not the motor's own flash.
After configuring the device limit for the first time you must power-cycle physically,
so that the MCU writes the value into the motor's runtime parameter 0x700B at boot:

```python
f = t.FollowerGripper.open()
f.motor.set_startup_limit_torque(6.0)   # writes MCU flash; configure once
print(f.motor.get_startup_limit_torque())
# exit here and replug the gripper; do not carry straight on into motion on this power cycle
```

After the restart, use the controller:

```python
cfg = t.ForcePositionConfig()
cfg.grasp_torque_nm = 0.35         # torque budget, Nm — the grip force setpoint; a
                                   #   blocked jaw settles here
cfg.close_speed_radps = 0.5        # close/open speed rad/s. Not independent of the above: the damping gain is
                                   #   grasp/close_speed (capped at 5), and anything below grasp/5 is rejected
cfg.hold_torque_limit_nm = 1.8     # indefinite-hold ceiling = motor rated torque, validated (0, 1.8]
cfg.motion_torque_limit_nm = 6.0   # motion transient ceiling = motor peak torque, validated (0, 6.0]
cfg.status_timeout_ms = 350        # status stream gone this long → FAULT + zero torque + observation invalid
cfg.motor_stream_hz = 100          # status-stream rate Hz; the confirm-frame count derives from it and the firmware's 30 ms

f = t.FollowerGripper.open()
f.motor.clear_fault()
grasp = t.ForcePositionController(f, cfg)
grasp.start()                 # validates the device limit first; does not close on its own yet
f.motor.enable()
try:
    grasp.set_target(0.0)     # close; blocked, the command saturates at 0.35 Nm and holds
    grasp.set_target(0.35, 0.45)  # change at runtime to 35% opening, 0.45 Nm
    while running:
        print(grasp.snapshot())
    grasp.release()           # open at bounded speed
finally:
    grasp.stop()              # commands zero torque first
    f.motor.disable()
```

Examples:

```bash
# how to use the two controllers
python python/examples/impedance_control.py right
python python/examples/force_position_control.py right --grasp-torque 0.35

# motion-safety envelope (the firmware's torque and thermal protection; off by default, written once per device)
python python/examples/impedance_control.py --show-envelope
python python/examples/impedance_control.py --set-envelope --peak 2.0 --cont 1.1
```

Note: 6 Nm is the motor's peak, and it only allows **instantaneous** torque of that
magnitude during the motion phase; it does not raise the gripper mechanism's safe
rating to 6 Nm. If the mechanism itself cannot take an instantaneous load above
1.8 Nm, lower `motion_torque_limit_nm` and the device's `0x700B` together. All the
software can guarantee is that the commanded torque during the hold does not exceed
`grasp_torque_nm`, whose own ceiling is 1.8 Nm.

This controller needs follower firmware ≥ 1.1.2 (which supports reading the V2.2
startup torque limit), and a follower whose open/close travel has been calibrated.
While it runs it owns motor control and the status stream exclusively — do not use
`ImpedanceController` concurrently, and do not send other motion commands directly.

**Why phase matters**: a host frame that lands while the MCU is transmitting costs it
the frame it was sending — the whole frame is voided. So a collision depends on **when
a frame lands**, not on how many you send. Both controllers submit once per received
status frame, inside the window the MCU is known to be idle: measured, 6000 submits :
6000 frames : 0 lost, while free-running at 100 Hz under the same conditions lost
156–308 frames per run. Do not treat 500 Hz as a budget to spend.

**There is no low-level path any more.** The raw motor primitives
(`motor.set_impedance` / `submit_impedance` / `set_position` / `set_velocity` /
`set_torque`, and the normalized wrapper `FollowerGripper.set_position`) are **no
longer exposed to Python**. All of them drop a control frame straight onto the bus: no
error clamp, no torque ceiling. Go through `ImpedanceController` or
`ForcePositionController`. The C++ side keeps these methods; the two controllers use
them internally.

**Feedback rate**: the motor's `actual_*` telemetry is only ~50–100 Hz, so read
observations through `observation()` / `snapshot()` and do not poll with
`read_status()` — above ~100 Hz it holds back the firmware's own refresh, and during
control the ACK round-trip corrupts telemetry frames.

> For **grip force**, use `ForcePositionController`: after contact is decided,
> `kp=kd=0` and `grasp_torque_nm` is the hold torque itself. The `max_torque` of
> position mode is not a tight force ceiling, and using it as a grip force will crush
> a soft object.

Runnable examples: `impedance_control.py`, `force_position_control.py` (above), and
`gripper_console.py` (interactive console, both modes).

---

## 2. Vision: the wrist camera

The wrist camera is an ordinary UVC device, and this SDK reads it with the `Camera`
class (`cv::VideoCapture` underneath). **Nothing opens it for you by default** — there
are two ways to open it:

### 2.1 Standalone (recommended, decoupled from the gripper)

```python
from xense.taccap import Camera, ColorMode

cam = Camera(device="/dev/video2", width=640, height=480, fps=30.0,
             use_mjpg=True, color_mode=ColorMode.BGR)

frame = cam.read(timeout_ms=500)      # synchronous one-shot read; returns None on failure
print(frame.frame_index, frame.image.shape)

cam.start(lambda f: handle(f.image))  # or asynchronously: a callback on a background thread
...
cam.stop()
```

Prefer a stable device path such as `/dev/v4l/by-id/...-video-index0`; the number in
`/dev/videoN` moves with plug order. List them with
`python python/examples/wrist_camera.py --list`.

### 2.2 Attached to the gripper object

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True,
                    undistort_wrist=True)     # frames come out already rectified
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

> **There is a deliberate inconsistency in channel order**: a bare `Camera` defaults to
> **BGR** (OpenCV's native order, so `imshow` / `imwrite` are correct as-is), while the
> gripper's `wrist_camera` defaults to **RGB** (it feeds vision and learning pipelines,
> and LeRobot datasets store RGB). Either can be changed back with `color_mode` /
> `wrist_color_mode`. Getting it backwards raises no error — it just records a dataset
> with the channels swapped.

### 2.3 Fisheye undistortion

The wrist lens is a fisheye, and its intrinsics live in MCU flash
(`Cmd::CameraFisheyeCal`, firmware ≥ V2.0). `FisheyeUndistorter` compiles the
intrinsics into remap tables once, after which each frame is only resampled:

```python
from xense.taccap import FisheyeUndistorter

cal, is_reference, reason = g.calibration.resolve_fisheye()
if is_reference:
    log.warning(f"using the SDK reference intrinsics, not this unit's: {reason}")

undist = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
cam.set_undistorter(undist)      # install into the capture path: read() and callbacks both get rectified frames
# or manually: rect = undist.apply(img)
```

Three constraints you must know:

- **Read intrinsics with `resolve_fisheye()`, not `read_fisheye()`.** An uncalibrated
  unit does not NACK; it returns an **all-zero** record, which passes an
  `if cal is None` check, but `fx = fy = 0` maps every pixel outside the frame — you
  get a pure black "rectified frame", with no error raised anywhere.
  `resolve_fisheye()` already puts that policy (read → judge usable → fall back to the
  reference values) in one place.
- **Only 640×480 is supported**, the calibration resolution. The firmware record
  carries no image size, so scaling the intrinsics would be guessing, and the
  constructor would rather throw. Another resolution means storing a matching
  calibration in firmware first.
- **The reference intrinsics are an approximation.** Lens mounting varies from unit to
  unit (the principal point especially), so falling back to `FISHEYE_FALLBACK_CAL` is
  only better than not rectifying at all — do not take pixel-accurate measurements
  from it. Once you have calibrated your own, write it to flash with
  `fisheye_cal.py set-fisheye --from-npz cam.npz`.

`balance` is a framing preference: 0 = keep the calibrated focal length (natural field
of view, matching the PC tool's default), 1 = squeeze the focal length to 0.70x for
the maximum field of view, at the cost of more black border around the edges.

> An image that looks off-centre or slightly tilted is **not necessarily a bad
> calibration** — the sensor does not necessarily sit exactly on the lens's optical
> axis, and rectification is done around the principal point, not around the centre of
> the image.

### 2.4 One command to see it

```bash
python python/examples/wrist_camera.py right                 # raw fisheye (default)
python python/examples/wrist_camera.py right --undistort     # rectified
python python/examples/wrist_camera.py right --compare       # side-by-side
python python/examples/wrist_camera.py XCA28Z0116m           # a serial number works too
python python/examples/wrist_camera.py right --no-mcu --no-display \
    --duration 10 --save-dir /tmp/shots                      # headless, saves images
```

The selector is `left` / `right`, as in every other example — the intrinsics are read
automatically from the gripper on the **same side**. **Undistortion is off by
default**, matching the SDK itself (a bare `Camera` carries no undistorter, and the
gripper's `undistort_wrist` defaults to `False`); in windowed mode the intrinsics are
read up front even when it starts in raw, because `u` may switch to rectified at any
moment. Headless + raw is the only combination that never touches the gripper.
In the window, `u` cycles through raw / rectified / comparison, `[` and `]` adjust the
balance, and `s` saves an image. With no gripper plugged in, add `--no-mcu` to use the
reference intrinsics; `--from-npz` uses an offline calibration file. Full arguments
with `--help`.

> **This script cannot open a visuotactile sensor, and that is deliberate.** Passing a
> GSPS serial number or a `/dev/videoN` path is refused with an explanation — OG
> capture and rectification live in `xensesdk`, not in this SDK (see the next section).

---

## 3. Tactile: the visuotactile (OG) sensor

**This part is not in this SDK.** `xense.taccap` covers the gripper protocol and the
wrist camera only; OG capture, rectification and force/depth inference all live in the
`xensesdk` wheel (version 2.1.1 on this machine). What follows is the shortest path to
getting it running; the wheel you have installed is the authority on the interface.

```python
from xensesdk import Sensor

print(Sensor.scanSerialNumber())     # {'OG000352': 10, 'OG000344': 8} — serial: cam_id

sensor = Sensor.create("OG000352")   # a cam_id works too; the first call builds a config cache and is slow
try:
    # pass several outputs to get several at once; the return values correspond in order
    rectify, depth, force = sensor.selectSensorInfo(
        Sensor.OutputType.Rectify,      # rectified image
        Sensor.OutputType.Depth,        # depth map, in mm
        Sensor.OutputType.Force,        # force distribution
    )
finally:
    sensor.release()
```

The `OutputType`s you will use most (full list with `dir(Sensor.OutputType)`):

| Output | Meaning |
| --- | --- |
| `Rectify` | Rectified image |
| `AugDifference` / `Difference` | Difference against the reference frame (contact visualization) |
| `Depth` | Depth map, mm |
| `Force` / `ForceNorm` / `ForceResultant` | Force distribution / normal force / resultant force (6-dimensional) |
| `Marker2D` / `Mesh3D` / `Mesh3DFlow` | Markers and the 3D mesh, plus its flow field |

A few practical points:

- **Pass `disable_infer=True` when you want images but no inference** — it saves
  loading the inference engine, and `Rectify` and `Difference` never needed inference
  in the first place.
- **The first `create()` is slow** because it is building the per-serial config cache
  (`~/.xensesdk/config`). With several sensors, warm the cache first and only then open
  them concurrently, otherwise they collide.
- **OG sensors and the wrist camera are both `/dev/video*`**, but the wrist camera is
  **not** in the results of `Sensor.scanSerialNumber()` — it is not an OG device, so
  do not expect the tactile SDK to enumerate it. The reverse holds too:
  `wrist_camera.py` refuses a GSPS serial number. Each side keeps the other out by the
  serial syntax.
- The force and mesh outputs have fixed shapes (force distribution `(35, 20, 3)`,
  resultant force `(6,)`); the image outputs change shape with `rectify_size`.

For full production-grade usage (async reads, config overrides, multi-sensor
orchestration), see `lerobot/cameras/xense/camera_xense.py` on the `lerobot` side.

---

## 4. Running all three at once

They are independent, but running them together has a few known traps:

- **USB bandwidth is shared.** Two OG sensors plus a wrist camera all running MJPG at
  full frame rate already eat a substantial share of the bus. The gripper is on a
  serial port and is unaffected, but the cameras squeeze each other. If you really
  intend to collect with the full set, measure the actual frame rate first; do not
  assume the nominal one.
- **Who holds the UVC device.** In production an external camera service holds
  `/dev/video*`, and in that case do **not** open the same node again with
  `open_cameras=True` or a bare `Camera` — it will either fail or grab the device
  halfway. That is exactly why the SDK does not open cameras by default.
- **Use `ImpedanceController` or `ForcePositionController` for the control loop.**
  Both lock submission to the phase of the status stream; the note on phase above
  matters most under full load, since the measurements were taken with every camera
  streaming and the motor cycling back and forth.
- **Logging.** The whole SDK shares one singleton logger (`xense.taccap.log`): the
  console defaults to INFO, the file sink is always DEBUG and lands in
  `~/.taccaplogs/session_*.log` (changeable with `$TACCAP_LOG_DIR`), with at most 10
  kept. When something goes wrong, read that file first — it has the lines the console
  filtered out.
- **After flashing firmware you must replug — unplug the USB cable and the power cable
  at the same time, then plug both back in together.** The bank swap is a soft reset,
  and the device looks entirely healthy afterwards (right version, stream running,
  clean counters) while silently dropping status frames. See
  [docs/FIRMWARE.md](FIRMWARE.md).

---

## When things go wrong

| Symptom | Most likely |
| --- | --- |
| `scan_grippers()` returns nothing | Serial port permissions (see [INSTALL.md](INSTALL.md)), or a cable that is not seated |
| `fw_sn` empty / `Side.Unknown` | The firmware has no SN burned, or predates V1.6 |
| Rectified image is all black | The all-zero record from `read_fisheye()` — use `resolve_fisheye()` instead |
| Rectified image looks off-centre | Not necessarily wrong; the principal point was never guaranteed to be at the image centre |
| Recorded colours are inverted | A bare `Camera` is BGR and `wrist_camera` is RGB; they got mixed up |
| Motor status frames lost in blocks | Submission phase (use one of the two controllers, don't send frames yourself), or a firmware flash without a power-cycle and replug |
| A C++/bindings change has no effect in Python | The consuming env was not reinstalled; see [INSTALL.md](INSTALL.md) |
| `import xense.taccap` complains about `libopencv_core.so` | Missing `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` |

More detail by topic: [INSTALL.md](INSTALL.md) · [CALIBRATION.md](CALIBRATION.md) ·
[FIRMWARE.md](FIRMWARE.md) · [EXAMPLES.md](EXAMPLES.md) ·
[ARCHITECTURE.md](ARCHITECTURE.md)

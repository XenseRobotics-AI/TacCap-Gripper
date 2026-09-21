# taccap-gripper

C++17 / Python SDK for the **TacCap-Gripper** —— XenseRobotics' multimodal
tactile data-collection gripper. Exposes a single namespace
(`xense::taccap::` / `xense.taccap`) for:

- IMU + encoder readout via the TC-GU-01 serial protocol
- Motor control (follower side, FDCAN→灵足 transparently routed via MCU)
- Leader / follower gripper objects that aggregate the MCU sensors and
  expose zero-config discovery
- Standalone `Camera` (wrist UVC, plain OpenCV V4L2) — **opt-in**: an
  external camera service owns the V4L2 devices now, so the gripper
  aggregates do **not** open it unless constructed with `open_cameras=True`.
  The **visuotactile (OG) sensors are not handled in this SDK**; capture and
  rectification live at the Python level via the `xensesdk` wheel.

Two adapter repos build on it and are released independently:

- `taccap_gripper_ros2` — ROS2 (Humble + Jazzy) hardware interface package
- a fork of `lerobot-xense` with a `taccap_gripper` robot class

Both only *import* the `xense.taccap` package; neither reimplements device
access, and neither is required to use this SDK.

## Status

**v0.2.0.** Command set **V2.3**, wire framing **V1.8**. Hardware-validated on
bilateral leader setups and on real follower grippers — including the V2.2
follower diagnostics, the MIT force-position control path, and `ControlLoop`
under a full production load (all cameras streaming, motor cycling).

**Firmware minimums:** leader >= 1.2.0, follower >= 1.1.0. V2.2 follower
diagnostics need follower >= 1.1.2; the V2.3 additions (`GetMotorSpec` 0x56,
`GetHomeDiag` 0x57) need follower >= 1.2.3. These are floors, not exact
matches — newer commands fail loudly with `ProtocolError(InvalidCmd)` rather
than misbehaving, and payload length is never a version probe. Check what a
device answers with `python python/examples/fisheye_cal.py show`.

**Firmware 1.2.3 aligns the two roles onto one version number.** They build
from one tree and share `protocol_handler.c`, so a change to the shared layer
obliges both — and with separate lines (leader 1.2.x, follower 1.1.x) it was
easy to bump one and forget the other, which is exactly what happened before
1.2.3: two leaders reported 1.2.1 while running a binary 17,809 bytes different
from the official 1.2.1. One number per release makes that impossible. The
cost is that a change touching only one role still bumps the other.

**V2.3 changed how a failed command answers.** A failure now comes back as a
pure ACK with `cmd == 0` and a one-byte error code; success keeps the original
command code. Before, a failure also carried the command code with a one-byte
error payload — indistinguishable on the wire from a success returning one byte
of data, so every no-data command's failure was invisible. If you talk to a
follower or leader older than 1.2.3 you get the old, ambiguous form.

> **[`firmware/`](firmware/) ships leader 1.2.2 and follower 1.1.6**, both local
> builds, both hardware-validated on two units each. They carry three fixes that
> live in code the two roles share: a command-channel livelock under sustained
> high-rate input, a blocking-log path that stalled realtime tasks, and an
> out-of-bounds write on every boot. Note that leader 1.2.2 replaces an
> *official* 1.2.1, so it trades that provenance for the fixes — see
> [`firmware/README.md`](firmware/README.md). **Power-cycle after any flash.**

### What's in

- **TC-GU-01 protocol** — async transport with ACK matching, per-command DATA
  subscribers, byte-stuffed framing.
- **Follower motor control (MIT force-position).** `Motor` enable / disable /
  clear-fault + four control modes. Blocking-ACK `set_*` and no-ACK `submit_*`.
- **`ControlLoop`** — the recommended way to drive a follower. Submits the
  latest normalized target **in phase with the motor-status stream** and keeps
  a thread-safe observation fresh, so your policy only touches
  `set_target(0..1)` and `observation()`, both non-blocking. See
  [Follower gripper control](#follower-gripper-control-mit-force-position) for
  why the phase matters.
- **`ForcePositionController`** — grasping for a follower. One bounded-torque
  law the whole way: the jaw tracks a ramp toward the target and the command is
  clamped at `grasp_torque_nm`, so free travel costs only friction and an
  obstruction settles at exactly the grasp force. No contact detection to tune —
  saturation *is* contact. Two knobs in practice: the force and the speed.
- **Normalized position** on both roles — `[0, 1]`, 0 = closed, 1 = open, on
  one-shot reads and on every streamed sample.
- **`Diagnostics`** (`g.diagnostics`) — the firmware's own UART counters and a
  runtime log switch. Answers a question the host cannot answer alone: when a
  frame arrives short, did the MCU fail to send it, or was it lost afterwards?
- **Wrist fisheye undistortion** — firmware-stored intrinsics turned into
  cached remap tables, standalone or wired in via `undistort_wrist=True`.
- **Calibration** (`g.calibration`) — the flash-persisted fisheye and
  encoder-max records. See [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **LEDs, power-on auto-calibration, OTA, zero-config discovery** by
  firmware-burned SN (never the CH343 chip SN).

Visuotactile (OG) capture lives at the Python level via the `xensesdk` wheel —
`xense.taccap` is the gripper-protocol + wrist-camera surface only.

Full per-commit history in [CHANGELOG.md](CHANGELOG.md).

---

## Install

```bash
mamba env create -f environment.yml && mamba activate xense-taccap
pip install -e . --no-build-isolation
python -c "import xense.taccap as t; print(t.__version__)"
```

Two gotchas that cost people an afternoon each:

- Call `pip` from an **activated** env, not by absolute path. Otherwise cmake
  finds a `ninja` on `PATH` that is actually GNU Make and fails with a
  confusing version error.
- A C++ or bindings change is **not live in a consumer env until you reinstall
  there** — the editable install redirects Python sources to the checkout but
  keeps serving the compiled extension from `site-packages`.

Prerequisites, device permissions, C++-only builds, rebuild/clean and the
`PYTHONPATH` / `LD_LIBRARY_PATH` traps: **[docs/INSTALL.md](docs/INSTALL.md)**.

---

## Quick start

### Single gripper

```python
import xense.taccap as t

# Auto-discover the one connected gripper (left or right) by its MCU serial.
# Throws IoError if 0 or >1 grippers are plugged in — use the explicit
# constructor (below) for bilateral setups.
gripper = t.LeaderGripper.open()      # MCU-only; cameras stay off
gripper.start_streaming(imu_hz=100, encoder_hz=100)

enc_sub = gripper.encoder.on_data(lambda s: print("enc", s.position_rad))
imu_sub = gripper.imu.on_data(lambda s: print(s))

# ... do work ...
gripper.stop_streaming()
```

The wrist camera is owned by an external camera service, so `open()` does
not touch it. To have a gripper drive the wrist UVC camera, construct it
explicitly with `open_cameras=True` and the device path:

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True)
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

Or open it on its own with the standalone `t.Camera` class — independent of
any gripper. The visuotactile (OG) sensors are read separately via the
`xensesdk` wheel, not through this SDK.

### Bilateral (left + right in one process)

```python
from xense.taccap import LeaderGripper, scan_grippers, Side

# scan_grippers() returns all endpoints in one USB sweep — no re-probe
# race when you ask for both sides.
endpoints = scan_grippers()
left  = next(e for e in endpoints if e.side == Side.Left)
right = next(e for e in endpoints if e.side == Side.Right)

# Alternatively: t.find_left() / t.find_right() are typed wrappers
# that throw if the requested side isn't visible.

def _open(eps):
    return LeaderGripper(eps.mcu_device)   # MCU-only; cameras off by default

g_left, g_right = _open(left), _open(right)
g_left.start_streaming(imu_hz=100, encoder_hz=100)
g_right.start_streaming(imu_hz=100, encoder_hz=100)
# ... attach callbacks, stop_streaming() on exit ...
```

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
# and no stall guard — the firmware envelope is then your only protection.
loop = t.ControlLoop(g, kp=20.0, kd=1.0)
loop.start()                     # seeds the target with the current position
loop.set_target(0.35)            # normalized [0,1], 0 = closed
obs = loop.observation()         # position/velocity/torque/temp, non-blocking
loop.stop()
```

**Normalized position** — work in `[0, 1]` (0 = closed, 1 = open) instead of raw
radians. Requires a calibrated gripper (`GripperConfig` Valid); throws otherwise.
Raw-radian motor control is not reachable from Python — see the controllers
above.

```python
print(g.position())                       # -> 0.97   (nearly open)
g.set_position(0.5, kp_nm_per_rad=8, kd_nm_s_per_rad=1)   # 50% open (no-ACK, realtime)
g.pos_to_rad(0.5), g.rad_to_pos(-0.59)    # explicit conversions
```

**`ControlLoop`** — the recommended way to drive a follower. A C++ background
thread submits the latest normalized target **in phase with the motor-status
stream** while that stream keeps a thread-safe observation fresh. Your policy
only touches `set_target(0..1)` and `observation()`, both non-blocking (no GIL
fights, no status polling).

```python
loop = t.ControlLoop(g, kp=20, kd=1)        # SubmitPhase.STREAM_LOCKED by default
loop.start()                              # seeds target = current pos (no jump)
try:
    while running:
        obs = loop.observation()          # .position [0,1], .velocity, .torque, .age_ms
        loop.set_target(policy(obs))      # your action, 0..1
finally:
    loop.stop()
g.motor.disable()
```

> **Why the phase matters, and why 500 Hz is not a budget.** The firmware
> applies the latest target at 500 Hz, but that says nothing about what
> submitting at that rate costs. Every host→MCU frame that lands while the MCU
> is transmitting makes it drop bytes out of the frame it is sending, and that
> frame is discarded whole. A 41-byte status frame at 3 Mbps fills only ~137 µs
> of each 10 ms period, so whether a submit collides depends on **when** it
> lands, not how many you send: 250 Hz lost 154 status frames on one 60 s run
> and none on the next.
>
> `SubmitPhase.STREAM_LOCKED` removes the collision instead of making it rarer —
> one submit per received status frame, landing in the ~9.86 ms the MCU is known
> to be idle. Measured with every camera on both grippers streaming and the
> motor cycling: **6000 submits : 6000 frames : 0 missing**, four runs, both
> units. Free-running at 100 Hz on the same bench lost 156–308 frames per run.
>
> It does not protect ACK responses — the loop knows when the MCU emits
> telemetry, not when it is answering somebody's command. Those survive because
> commands retry, at ~31 ms of latency each.
>
> **Feedback rate.** The motor's `actual_*` telemetry refreshes at ~50–100 Hz.
> Read observations from the **stream**, not by polling `read_status()` —
> polling `GetMotorStatus` above ~100 Hz can stall the firmware's refresh.

**`ForcePositionController`** — use this one to **grasp something**. `ControlLoop`
is position control: against a hard object it keeps adding `kp × error` and there
is no grip force you set. This one clamps the command at a force you choose, so a
blocked jaw settles at exactly that force and holds.

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

### 固件版本要求:从爪 ≥ 1.1.6

`FollowerGripper` 打开时会检查从爪固件版本,**低于 1.1.6 直接拒绝并打印升级提示**。
这不是建议而是硬性要求:1.1.6 之前的固件在 MIT 命令路径上**没有任何堵转保护**
(`can_motor_gripper_stop_on_limit_stall` 只挂在速度命令上、且只管行程末端),
夹爪被挡住时力矩只受电机 `0x700B` 限制。24 V 供电下实测:`kp=20` 顶硬物体时
命令索要约 12 Nm,夹爪以 5.5 rad/s 撞上去,电流把母线拉垮 —— 松手掉件,严重时
整机掉电重启。

```bash
python python/examples/ota_update.py slave      # role 选择器,自动挑镜像
python python/examples/ota_update.py --all      # 所有在插的夹爪各刷各的镜像
# 刷完必须断电重启
```

OTA 本身走 `LeaderGripper`(角色无关),**不受这个检查影响**,升级通道始终可用。
确实要在升级前读一台旧设备的配置,用 `allow_outdated_firmware=True` 显式绕过。

### 两个控制器的示例

```bash
# 阻抗控制 —— ControlLoop:set_target(0..1) + observation()
python python/examples/impedance_control.py --side right

# 夹持 —— ForcePositionController:同样的两个调用,但命令被钳在 grasp_torque_nm
python python/examples/force_position_control.py --side right
```

**先配好运动安全包络。** 它是固件侧的力矩与热保护,默认**不启用**(出厂设备
`GripperConfig.reserved` 全 0),不开就没有:

```bash
python python/examples/impedance_control.py --show-envelope              # 查看
python python/examples/impedance_control.py --set-envelope \
       --peak 2.0 --cont 1.6                                             # 写入并启用
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `--peak` | 2.0 Nm | 运动瞬态上限。**同时决定接近速度**,约 `peak/kd`(实测 peak=1.5、kd=1.0 → 1.70 rad/s) |
| `--cont` | 1.6 Nm | 可持续上限,I²t 降额的下限 |
| `--temp-derate-start` | 0 → 固件 90 °C | 温度降额起点 |
| `--temp-wall` | 0 → 固件 100 °C | 温度墙,之上只留 0.30 Nm |

包络存在 `GripperConfig` 记录里(命令 `0x66/0x67`,**协议未改动**),掉电保持,
每台设备配一次即可。它在固件的 MIT 分支执行 —— 那是所有运动命令的必经点,
**绕过本 SDK 直接发 MIT 帧也挡得住** —— 这正是它必须在固件里的原因。

不开包络的后果是实测过的:`kp=20` 顶硬物体时命令索要约 12 Nm(被 `0x700B`
削到 6),24 V 下这个电流需求会把母线拉垮 —— 电机欠压保护动作、松手,严重时
整机掉电重启。开启后同样条件 25 秒稳定夹持。详见
[`docs/CONTROL_LAYERING.md`](docs/CONTROL_LAYERING.md)。

**两个要知道的行为**:

- **握持力不是常数。** I²t 与温度墙对纯力矩保持同样生效。实测(墙 90/100,
  `cont=1.6`):91 °C 起降额,94 °C / 1.370 Nm 平衡。`grasp_torque_nm` 是设定值,
  固件在下面把实际输出降下来。
- **控制期间不要轮询 `read_status()`。** 相位锁只保护遥测帧不保护 ACK:它的请求
  可能落进 MCU 正在发送的窗口、把那一帧遥测撞废,它的应答也可能被控制循环的提交
  撞坏(重试可恢复,代价是 ~31 ms 延迟)。位置、速度、力矩、状态位、**温度**
  全部在 `observation()` / `snapshot()` 里,控制期间不需要碰总线。

Runnable demos: `python/examples/impedance_control.py`,
`python/examples/force_position_control.py` (see above), and
`python/examples/gripper_console.py` (interactive console, both modes).

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

## Examples

All scripts live under `python/examples/`; the table is in
**[docs/EXAMPLES.md](docs/EXAMPLES.md)**. Enable C++ examples with
`-DTACCAP_BUILD_EXAMPLES=ON` (off by default).

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

- **[docs/USAGE.md](docs/USAGE.md)** — 使用文档(中文):把触觉(OG)、视觉
  (腕相机)、夹爪读数与控制三路数据分别开起来的端到端步骤。
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

## License

Apache-2.0. Copyright (c) 2026 XenseRobotics Co., Ltd.

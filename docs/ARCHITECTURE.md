# TacCap-Gripper SDK — Architecture

This document describes the SDK as it is now. Where it once carried a banner
warning that the diagrams below were historical, the diagrams have been brought
up to date instead: the visuotactile path removed in 0.1.4 is gone from them,
and the follower motor stack and wrist rectification added since are in.

The SDK in this repository is the **C++ / Python device-access layer
only**. Higher-level products that build on top of it — dataset recording
tools and lerobot adapters — live in their own repositories and are out of
scope here. The follower gripper's motor stack is
*not* one of them: it lives here (see §8), because it is a device primitive
rather than a policy.

**Protocol tracked:** wire framing **V1.8** (the body between HEAD and TAIL
is byte-stuffed; CRC over the unstuffed HEAD..PAYLOAD) + command set **V2.1**
(V1.7 motor / CAN-id / gripper-config plus motor_status_t /
motor_impedance_ctrl_t growth; V1.9 WS2812 + private motor params; V2.0/V2.1
fisheye-camera and leader-encoder-max calibration). Discovery is MCU-only;
cameras are owned by an external camera service. Side/role come from the
firmware SN (`parse_serial`) with a GetDevType fallback.

**Worker threads outlive Python.** The transport reader, the transport
dispatcher and the camera capture thread are plain `std::thread`s — nothing
stops them when the Python interpreter finalizes. Two rules keep that from
aborting the process: `Transport::stop()` joins both workers *before* dropping
subscriptions (so the callback objects die on the caller's thread, never on a
worker mid-teardown, and no callback can be entered once the stop flag is up),
and every binding that calls into Python from a worker thread first checks
whether the interpreter is finalizing and drops the event if so. That second
rule lives in `python/bindings/gil_safe.hpp` — `make_gil_safe_callback()` to
own the callable and `call_into_python()` to invoke it — because a binding that
hand-rolled its own ownership got it wrong and segfaulted on every `stop()`.
The gripper `__exit__` handlers call `transport().stop()` so a `with` block
leaves nothing running.

**Firmware error wire path — a seam worth knowing.** Only handler *dispatch*
failures take the `cmd == 0` wire path that `AckResponse::is_nack` detects. A
handler that returns non-OK goes through `protocol_send_response(seq, cmd,
err, NULL, 0)`: the command byte is *echoed* and the error is the whole
payload, which is indistinguishable at the transport layer from a legitimate
1-byte success response. `bus::ack_error_code()` resolves the ambiguity in
favour of "error", so it is only valid for commands whose success payload is
never a single non-zero byte — true for `Cmd::Ota*` and the V2.0 calibration
commands, **not** for `MotorGetCanId` / `MotorGetProtocol`, whose success
payload is exactly one meaningful non-zero byte. Transport semantics are
deliberately unchanged; call sites opt in.

**Retry is not free — `bus::RetryMode`.** `send_cmd` retries on ACK timeout.
By default (`NewSeq`) each attempt gets a fresh seq, which is right for
idempotent commands but fatal for ones the firmware must not run twice: the
OTA write path demands strictly sequential offsets and fails the whole session
on a repeat. `RetryMode::SameSeq` reuses the seq so the firmware recognises
the repeat and replays its cached response (`protocol_resend_cached_response`)
rather than re-entering the handler. It relies on the firmware caching exactly
one request, so nothing else may share the transport between attempts — fine
for the single-threaded OTA flow, which is the only user.

---

## 1. Layered stack

```
┌────────────────────────────────────────────────────────────────────────┐
│                       USER CODE (Python / C++)                         │
│   - data-collection scripts, Jupyter notebooks, downstream products    │
└────────────────────────────┬───────────────────────────────────────────┘
                             │  xense.taccap (Python)  ↔  xense::taccap (C++)
┌────────────────────────────▼───────────────────────────────────────────┐
│  L4  AGGREGATE                                                         │
│  ─────────────                                                         │
│   LeaderGripper          aggregate object: owns Transport + IMU +      │
│                          Encoder + Key + Led; start/stop_streaming.    │
│                          Discovery open() auto-finds by MCU serial.    │
│                          Wrist camera is opt-in (open_cameras).        │
│                                                                        │
│   FollowerGripper        same + Motor + Led; normalized position       │
│                          (0..1); Impedance/ForcePositionController.    │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
┌────────────────────────────▼───────────────────────────────────────────┐
│  L3  COMPONENTS                                                        │
│  ─────────────                                                         │
│   IMU            Encoder           Camera            Motor            │
│   ──────────     ──────────────    ──────────────    ──────────────   │
│   read_once()    read_once()       read() (sync)     read_status()    │
│   on_data(cb)    on_data(cb)       start(callback)   read_status_ext()│
│                                    stop()            fault_report()   │
│   ImuSample      EncoderSample     set_undistorter() enable()/disable()│
│   - mcu_ts_us    - mcu_ts_us                                          │
│   - accel_mps2   - position_rad    CameraFrame       MotorStatus      │
│   - gyro_radps   - velocity_rad_s  - host_time       MotorStatusExt   │
│   - mag_uT       - status          - frame_index     MotorFaultReport │
│   - temp_c       - seq             - image (BGR8,                     │
│   - seq                              rectified when                   │
│                                      an undistorter                   │
│   Calibration    Led / Key           is installed)                    │
│   ──────────     ──────────────                                       │
│   read_fisheye() set()/read()      FisheyeUndistorter                 │
│   resolve_fisheye()                 - remap tables built once         │
│   ColorMode (BGR default / RGB)     - converted after undistortion    │
│   read/write_encoder_max()          - apply(cv::Mat)                  │
└──────────┬─────────────────┬──────────────┬──────────────┬─────────────┘
           │                 │              │              │
           ▼                 ▼              ▼              ▼
┌─────────────────────┐  ┌──────────────────────────────────────────────┐
│  L2a  ASYNC TRANSPORT│  │  L2b  V4L2 CAPTURE                           │
│  ───────────────────│  │  ────────────────────────────────────────────│
│   Transport         │  │   cv::VideoCapture (wrist camera only)       │
│   - background      │  │   cv::remap        (rectification, when an    │
│     reader thread   │  │                     undistorter is installed) │
│   - ACK matching    │  │                                              │
│     (seq → promise)│  │   The visuotactile (OG) path is NOT here: it  │
│   - subscribe(cmd)  │  │   left this SDK in 0.1.4 and lives at the     │
│   - send_cmd_no_ack │  │   Python level via the `xensesdk` wheel.      │
│   - host-side retry │  │                                              │
└──────────┬──────────┘  └──────────────────────────────────────────────┘
           │                                                  │
           ▼                                                  │
┌─────────────────────┐                                       │
│  L1   PROTOCOL +     │                                       │
│       BUS WIRE       │                                       │
│  ───────────────────│                                       │
│   pack_frame        │                                       │
│   try_parse_frame   │                                       │
│   FrameParser       │                                       │
│   crc16_modbus      │                                       │
│   stuff/unstuff     │                                       │
│   SerialBus(termios)│                                       │
└──────────┬──────────┘                                       │
           │                                                  │
           ▼                                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              KERNEL / DRIVERS                          │
│                                                                        │
│      /dev/ttyACM*  (CH343 USART3 @ 3 Mbps)        /dev/video*  (UVC)   │
└─────────────────────────────────────────────────────────────────────────┘
                                ▲
                                │ over USB
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                  TC-GU-01 GRIPPER HARDWARE                             │
│   STM32H562 + ThreadX  ──── USART3 (3 Mbps) ────  IMU / Encoder /      │
│                                                   Eskin (G2+) / Motor │
│                        ──── FDCAN1 (1 Mbps) ────  灵足 motor (follower) │
│                                                                        │
│   2× OG-series planar visuotactile sensors  ──── UVC                   │
│   1× XC-series wrist UVC camera              ──── UVC                  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Module map

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/
│   │   ├── version.hpp.in                        L0  generated by CMake
│   │   ├── error.hpp                             Error / ProtocolError /
│   │   │                                          CrcError / IoError /
│   │   │                                          TimeoutError
│   │   ├── log.hpp                               spdlog facade
│   │   ├── protocol/
│   │   │   ├── commands.hpp / .cpp               L1  Address, FrameType,
│   │   │   │                                          Cmd, ErrorCode enums
│   │   │   ├── payloads.hpp                      L1  packed POD structs
│   │   │   │                                          (ImuData, EncoderData,
│   │   │   │                                          MotorPosCtrl, ...)
│   │   │   └── codec.hpp / .cpp                  L1  typed encode/decode
│   │   ├── bus/
│   │   │   ├── frame.hpp / .cpp                  L1  CRC16, pack/parse,
│   │   │   │                                          FrameParser, byte
│   │   │   │                                          stuffing
│   │   │   ├── serial_bus.hpp / .cpp             L1  termios at 3 Mbps
│   │   │   └── transport.hpp / .cpp              L2a async transport:
│   │   │                                              reader + dispatcher
│   │   │                                              threads, ACK matching,
│   │   │                                              subscribe, loss stats
│   │   ├── components/
│   │   │   ├── imu.hpp / .cpp                    L3  ImuSample + IMU
│   │   │   ├── encoder.hpp / .cpp                L3  EncoderSample +
│   │   │   │                                          Encoder
│   │   │   ├── camera.hpp / .cpp                 L3  cv::VideoCapture
│   │   │   │                                          wrapper for wrist,
│   │   │   │                                          optional rectify
│   │   │   ├── fisheye_undistorter.hpp / .cpp    L3  remap tables from the
│   │   │   │                                          flash intrinsics
│   │   │   ├── calibration.hpp / .cpp            L3  flash-persisted records
│   │   │   │                                          (fisheye, encoder max)
│   │   │   ├── motor.hpp / .cpp                  L3  FDCAN motor: status,
│   │   │   │                                          diagnostics, params
│   │   │   ├── led.hpp / .cpp                    L3  indicator LED
│   │   │   ├── key.hpp / .cpp                    L3  on-gripper button
│   │   │   └── sensor_errors.hpp / .cpp          L3  decoded fault words
│   │   ├── gripper_position.hpp                  L4  normalized [0,1] map
│   │   ├── gripper_observation.hpp               L4  controller observation
│   │   ├── impedance_controller.hpp              L4  position tracking
│   │   ├── force_position_controller.hpp         L4  bounded-force grasp
│   │   ├── ota.hpp                               L4  firmware update
│   │   ├── discovery.hpp                         L4 (helper) zero-config
│   │   ├── leader_gripper.hpp                    L4  aggregate object
│   │   └── follower_gripper.hpp                  L4  aggregate object
│   ├── src/                                      mirrors include/, plus
│   │                                              stream_rate.hpp and
│   │                                              wrist_fisheye.hpp (internal)
│   ├── examples/
│   │   └── leader_demo.cpp                       5-second multistream on
│   │                                              real hardware
│   └── tests/                                    21 gtest files — codec per
│                                                  command set (V1.6/1.7/2.1/
│                                                  2.2), CRC, framing, byte
│                                                  stuffing, PTY-based fake
│                                                  firmware (transport,
│                                                  calibration, OTA retry),
│                                                  fisheye rectification,
│                                                  stream rate, discovery,
│                                                  C++ vs GUI Python parity
│
├── python/
│   ├── bindings/
│   │   ├── module.cpp                            pybind11 entry point
│   │   │                                          (enums, Frame, Transport,
│   │   │                                          SerialBus, codec helpers)
│   │   ├── components.cpp                        (ImuSample/EncoderSample
│   │   │                                          with numpy fields, IMU/
│   │   │                                          Encoder/Camera/Motor/
│   │   │                                          Calibration/Led/Key,
│   │   │                                          FisheyeUndistorter,
│   │   │                                          Leader/FollowerGripper,
│   │   │                                          discovery)
│   │   └── log.cpp                               logging controls
│   ├── examples/                                 calibrate.py, fisheye_cal.py,
│   │                                              OTA, MIT control, V4L2 probes
│   └── xense/taccap/                             PEP 420 namespace package
│       ├── __init__.py                           re-exports the C-extension
│       └── _version.py
│
├── firmware/                                     shipped leader + follower
│                                                  images + manifest.json
├── scripts/
│   └── check_protocol_drift.py                   fails when the SDK's
│                                                  hand-written protocol
│                                                  drifts from the firmware
│
├── docs/
│   └── ARCHITECTURE.md                           ← you are here
│
├── environment.yml                               mamba env (Python 3.12,
│                                                  conda-forge only,
│                                                  opencv-python==4.12.0.88
│                                                  pinned via pip)
├── pyproject.toml                                scikit-build-core wheel
├── CMakeLists.txt                                top-level orchestrator
└── README.md
```

---

## 3. Data flow

### 3.1 Synchronous command (e.g. `imu.read_once()`)

```
  Python / C++ user
        │ send_cmd(GetImu)
        ▼
  Transport::send_cmd()
        │ pack_frame(...)           [L1]
        │ SerialBus::write(...)     [L1]
        │ create promise<AckResponse>, register under seq
        │ future.wait_for(timeout)
        │
        │   ┌──────────────────────── concurrent ────────────────────────┐
        │   │  reader_thread:                                            │
        │   │    SerialBus::read() → bytes                               │
        │   │    FrameParser::feed() → emits Frame                       │
        │   │    dispatch_(Frame):                                       │
        │   │      type==ACK → handle_ack_:                              │
        │   │        match seq → promise.set_value(AckResponse)          │
        │   └────────────────────────────────────────────────────────────┘
        │
        ▼
  AckResponse{ seq, cmd, error_code, data, is_nack }
        │  (decode wire format: cmd==0 ⇔ NACK)
        ▼
  IMU::decode(ack.data) → ImuSample (unit-converted)
```

### 3.2 Streaming DATA (e.g. `imu.on_data(cb)`)

```
  Python / C++ user
        │ imu.on_data(callback)
        ▼
  IMU::on_data → Transport::subscribe(Cmd::GetImu, cb)

  Earlier: leader.start_streaming()
        │ pack StreamConfig{ source_mask, mode, rates, iface }
        │ Transport::send_cmd(StartStream, cfg) → ACK
        ▼
  Firmware now pushes DATA frames at the configured rate.

  reader_thread (background) — no user code, ever:
        SerialBus::read() → bytes
        FrameParser::feed() → emits Frame
        dispatch_(Frame):
          type==ACK  → handle_ack_(f)      (inline: just fulfils a promise,
                                            so send_cmd never queues behind
                                            a slow subscriber)
          type==DATA → enqueue_data_(f)    (bounded queue; when full, evicts
                                            the OLDEST and bumps queue_dropped)

  dispatcher_thread (background) — the only thread that runs user code:
        pop Frame → handle_data_(f):
          for each subscriber matching f.cmd:
            cb(f)  → IMU::decode → ImuSample → user callback
                    (Python: gil_scoped_acquire + try/discard_as_unraisable)
```

> **Why two threads.** A subscriber callback is user code, and a Python one
> must take the GIL. With callbacks inline on the reader, that GIL acquire sat
> in the read loop: CPython's 5 ms default switch interval is ~3x the per-frame
> budget of a 200 Hz three-source stream (~1.67 ms), so a busy main thread
> stalled `read()`, overflowed the 4 KB n_tty buffer and lost *bytes* — frames
> corrupted, not merely delayed, and the stream silently ran at a fraction of
> its configured rate. The split makes callback cost pay in latency and queue
> depth instead of in data. `Transport::stats()` tells the two apart:
> `crc_errors` / `resync_bytes` mean loss upstream of the parser,
> `queue_dropped` means the subscriber could not keep up, and
> `callback_max_us` names it.

> The Camera path below is **opt-in** — it runs only when the device was
> opened (a gripper constructed with `open_cameras=true`, or a standalone
> `Camera`). It is not part of the default gripper lifecycle, which is
> MCU-only, and it is why the wrist camera is usually owned by whatever
> external service already has the V4L2 device.

### 3.3 Camera (wrist, V4L2)

```
  user → camera.start(callback)
        ▼
  Camera::start → spawn capture_loop_ thread
        loop:
            cv::VideoCapture::read() → cv::Mat
            [if an undistorter is installed: cv::remap → rectified cv::Mat;
             on failure the raw frame passes through and the error is logged,
             so a bad calibration never tears down the capture loop]
            CameraFrame{ host_time, frame_index, image }
            cb(CameraFrame)
                Python: gil_scoped_acquire + numpy view via mat_to_numpy
```

### 3.4 Tactile (visuotactile) — not in this SDK

The gripper carries two OG-series visuotactile sensors, but nothing here drives
them: `xense.taccap` is the gripper-protocol + wrist-camera surface only. OG
capture and rectification live at the Python level in the prebuilt `xensesdk`
wheel, which is where a consumer should go for them.

They are ordinary UVC devices on the same hub, so a downstream tool wires them
as cameras beside the wrist — that is what lerobot's gripper camera discovery
does.

### 3.5 Follower motor control (MIT force-position)

```
  set_impedance(pos,kp,kd,ff)  --ACK-->  Cmd::MotorImpedanceCtrl (blocking)
  submit_impedance(...)        --no ACK-> Cmd::MotorImpedanceCtrl (realtime)
        (set_impedance is C++ only; submit_impedance IS exposed to Python but
         carries no host-side clamp -- prefer ImpedanceController /
         ForcePositionController, which call these internally)
        │                                   firmware runs a 500 Hz control task
        │                                   consuming the latest submitted target
  FollowerGripper.set_position(0..1) --> GripperPosition -> raw rad -> submit
                     (C++ only)
  ImpedanceController (bg thread) ---->  submit latest target on the status-
                                          frame doorbell; error clamped at
                                          max_position_torque_nm; torque
                                          ceiling on MEASURED torque;
                                          snapshot() under one lock
  ForcePositionController ------------>  same doorbell; ramped target, command
                                          clamped at grasp_torque_nm -- a
                                          blocked jaw settles at that force
                                          (saturation is contact)
                         (motion <= 6 Nm peak; force hold <= 1.8 Nm rated)
```

---

## 4. Discovery (zero-config, MCU-only)

Discovery is MCU-only. The wrist camera and OG visuotactile sensors are
owned by an external camera service now, so the scanner no longer
enumerates them — one CH343 MCU board = one gripper unit, and there is no
USB-hub grouping step any more.

The firmware SN follows the TacCap scheme and encodes both side and role:

```
  TCGU01 A24 Z 0001 m     line: Z=R&D / A=production   seq last digit: odd→Left
  product batch line seq patch  patch: m=Master(leader) / s=Slave(follower)
```

```
scan_grippers():
  1) walk /dev/serial/by-id/, keep usb-1a86_USB_Dual_Serial_<SN>-if02
  2) for each MCU, open a transient Transport and read the firmware SN
     (Cmd::GetSn), then parse_serial(): side := odd-last(seq) -> Left else
     Right; role := patch m|s -> Leader|Follower. Side falls back to the
     CH343 chip-SN parity if the SN read fails; role is Unknown for legacy/
     empty SNs.
  3) emit one GripperEndpoints{side, role, mcu_device, mcu_serial,
     firmware_sn} per MCU board.

API:
   scan_grippers()       -> list of GripperEndpoints
   find_one()            -> single gripper, throws if 0 or >1
   find_left()  / find_right()    -> by side  (odd/even seq digit)
   find_leader()/ find_follower() -> by role  (SN patch suffix m/s)
   parse_serial(sn)      -> ParsedSerial{product,batch,line,sequence,side,role,valid}
```

---

## 5. Public API surface

### C++

```cpp
#include <taccap/leader_gripper.hpp>

auto g = xense::taccap::LeaderGripper::open();   // unique_ptr; MCU-only, throws on no device

g->imu().on_data    ([](const xense::taccap::ImuSample& s)     { ... });
g->encoder().on_data([](const xense::taccap::EncoderSample& s) { ... });

g->start_streaming(/*imu_hz=*/100, /*encoder_hz=*/100);
std::this_thread::sleep_for(5s);
g->stop_streaming();

// The wrist camera is opt-in — an external camera service usually owns the
// V4L2 device. Construct explicitly with open_cameras=true to drive it:
//   LeaderGripper::Config cfg; cfg.mcu_device = ...; cfg.wrist_video = ...;
//   cfg.open_cameras = true;
//   auto g = std::make_unique<LeaderGripper>(cfg);
//   g->wrist_camera().start(...);

// Follower control (see follower_gripper.hpp / impedance_controller.hpp):
//   auto f = xense::taccap::FollowerGripper::open();
//   f->motor().enable();
//   f->set_position(/*0..1*/ 0.5f, /*kp=*/8, /*kd=*/1);   // normalized, no-ACK
//   xense::taccap::ImpedanceController c(*f);
//   c.start(); c.set_target(0.3f); auto s = c.snapshot();
```

### Python

```python
import time
from xense.taccap import LeaderGripper

with LeaderGripper.open() as g:          # MCU-only; cameras off by default
    g.imu.on_data           (lambda s: print(s.accel_mps2, s.temperature_c))
    g.encoder.on_data       (lambda s: print(s.position_rad))

    g.start_streaming(imu_hz=100, encoder_hz=100)
    time.sleep(5)
    g.stop_streaming()

# Opt-in cameras: LeaderGripper(mcu_device, wrist_video=..., open_cameras=True)
# then g.wrist_camera.start(...); or drive the standalone Camera class.
# With undistort_wrist=True the frames arrive rectified from the intrinsics in
# this gripper's flash; FisheyeUndistorter also works standalone, on frames
# this SDK never captured.
```

---

## 6. Threading model

| Thread                          | Owner                      | Lifetime                        |
|---------------------------------|----------------------------|---------------------------------|
| user thread (`send_cmd` blocks) | caller                     | per call                        |
| `Transport::reader_loop_`       | one per Transport          | open()→stop()                   |
| `Transport::dispatch_loop_`     | one per Transport          | open()→stop()                   |
| `Camera::capture_loop_`         | one per Camera::start()    | start()→stop()                  |
| controller run thread           | one per Impedance/ForcePositionController | start()→stop()   |

Transport subscriber callbacks fire on the **dispatcher**, never on the reader;
they are serialised with each other and delivered in frame order, and
`unsubscribe()` still applies to frames already queued. Camera callbacks still
fire on their own capture thread. Python callbacks reacquire the GIL via
`py::gil_scoped_acquire`; exceptions inside callbacks are reported via
`discard_as_unraisable` so a buggy callback never tears down the producer.

A slow callback no longer costs data, but it is not free: it costs queue depth
and staleness, and once the queue is full the oldest frames are dropped. Poll
`Transport::stats()` (`queue_dropped`, `queue_high_water`, `callback_max_us`)
rather than assuming.

Lifetime contract:
- `LeaderGripper` is *not* copyable / movable — its members own
  threads / mutexes. Construct once via `open()` (returns
  `std::unique_ptr`).
- `~LeaderGripper` calls `stop_streaming()` and component destructors,
  joining all threads. Idempotent.

---

## 7. Dependencies

```
build-time:
   conda-forge:  cmake, ninja, gcc-14, libopencv 4.12, eigen, openssl,
                 zlib, nlohmann_json, gtest, pybind11, scikit-build-core,
                 numpy, pyserial
   pip:          opencv-python==4.12.0.88   (single source for cv2 in
                                               python land — pinned across
                                               XenseRobotics SDKs)

runtime (linked into libtaccap_core.so):
   libopencv_core.so / imgproc / imgcodecs / video / videoio / calib3d
                    (calib3d + imgproc are what FisheyeUndistorter needs)
   spdlog (fetched via FetchContent, header-only)
   pthread

NOT linked in (deliberately): any ML inference runtime, ROS, lerobot.
This is a hardware-access SDK, not a model server.
```

---

## 8. What this SDK is **not**

The following live in their own repositories on top of this SDK and
will be implemented later:

| Concern                  | Where it lives                          |
|--------------------------|-----------------------------------------|
| Dataset recording (hdf5 / mcap, time alignment, episode markers) | a separate tool / script repo         |
| lerobot integration       | `lerobot-xense`, where TacCap is a **gripper backend** (`type: taccap_follower`) that any arm can mount — not a Robot class of its own |
| Master→slave follow / teleop loop, grasp state machine (contact/latch), episode orchestration | downstream apps — this SDK gives the realtime primitives (`ImpedanceController`, `ForcePositionController`, `submit_*`, normalized position), not the policy |
| Higher-level orchestration (episode controller, replay, visualisation) | downstream applications |

The follower motor stack **is** in this repo now (`Motor`, `FollowerGripper`,
`GripperPosition`, `ImpedanceController`, `ForcePositionController`, `Led`) and hardware-validated — what stays
out is the *policy* layer above the primitives.

Keeping this SDK narrow lets each downstream consumer pick exactly the
hardware it needs — a teleoperation loop may want only IMU + Encoder DATA
streams and no cameras — and assemble its own data-flow on top.

---

## 9. Verified end-to-end (real hardware)

The unit suite is 263 gtest cases across 21 files, PTY-based fake-firmware
tests included; it runs on any host, with no gripper attached. What follows is
a hardware run, which is a different claim.

5-second multistream capture on the lab leader (`OGXXXXXX` /
`OGXXXXXX` / `XCXXXXXX`, MCU `5CXXXXXXXX`). This historical run was taken
with the cameras opened (`open_cameras=true`); the default gripper
lifecycle is now MCU-only (IMU + encoder):

```
IMU         : 506 frames | 101.2 fps   (firmware caps at ~100 Hz)
Encoder     : 506 frames | 101.2 fps
Tactile L   : 156 frames |  31.2 fps   (OGXXXXXX)
Tactile R   : 153 frames |  30.6 fps   (OGXXXXXX)
Wrist cam   : 149 frames |  29.8 fps   (XCXXXXXX)

discovery  : side=Right (MCU SN '5CXXXXXXXX' last digit 8 → even)
```

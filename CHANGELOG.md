# 更新日志

SDK 与固件使用独立版本号。未归属发布的变更列在“未发布”，已发布历史保留原文。

## [Unreleased]

暂无新的变更。

## [0.2.0b1] - 2026-09-15

首个统一的 beta 源码版本，收敛先前 0.1.10 / 0.1.11 开发快照及后续修订。
版本号标识本次候选构建，不表示已发布到 PyPI，也不表示完成全部硬件验收。

### 新增

- Python 客户入口 `Gripper`：位置跟随、抓取、阻抗、保持、释放、清错与统一生命周期。
- 不可变状态、操作结果、设备信息及频率统计；原生运行时统一控制权、反馈与停止流程。
- 固件 1.1.7 执行诊断兼容路径，以及 1.1.8 扩展遥测与闭合补偿。
- 正弦轨迹扫描、控制 / 读取频率测试、CSV 记录与离线图表。
- 中文 MkDocs 用户文档、API 参考、迁移指南与开发记录分层。

### 变更

- 持续位置目标使用 `set_position()`；受阻时保留有界位置施力，等待超时不自动取消目标。
- RS05 编码按说明书使用 ±5.5 N·m；客户扭矩预算上限仍为 1.2 N·m。
- `_version.py` 成为 Python、wheel 元数据与原生完整版本字符串的唯一来源。
- 示例统一使用 `gripper_control.py`、`frequency_benchmark.py` 和 `requirements-plot.txt`；旧命令路径保留兼容入口。

### 修复

- 补偿到位判断使用原始电机目标误差，避免错误到位或一直等待。
- 保留旧顶层及 wildcard 导入，包括 `OtaSession`，原生接口集中到 `advanced`。
- 清理高级接口重复导出赋值及已过时的模块说明。

### 迁移与兼容

- 客户 `Gripper` 至少要求已标定的从爪固件 1.1.7；推荐配套 1.1.8。
- 启用闭合补偿要求固件 1.1.8。仓库历史从爪 1.1.6 镜像不能用于客户 API。
- 相对 0.1.9，Python 原始无保护运动方法已撤下；原生控制器和维护接口继续保留。
- 腕相机默认 RGB；OpenCV BGR 消费者需显式指定 `ColorMode.Bgr`。
- 原生控制示例使用位置参数 `left/right/SN`，替代旧 `--side`。

完整迁移与命名对照见 [beta 迁移指南](docs/sdk/guides/migration.md)。
旧开发快照的逐项记录见 [beta 前开发记录](docs/history/PRE_BETA_CHANGELOG.md)。

## [0.1.9] - 2026-08-21

追一个客户报的"夹爪状态流偶发降频"，最后拆出四个性质不同的问题，这一版是其中
落在 SDK 侧的部分。

核心结论：**状态帧丢失不是固件缺陷**——用新加的固件计数器证明了字节是在离开 MCU
的发送寄存器之后丢的。SDK 能做的是不去触发它：`ControlLoop` 现在默认把提交锁相
到状态流之后，让写入永远落在 MCU 不发送的窗口里。实测在两只夹爪、所有相机取流、
电机往复开合的条件下，6000 次提交对 6000 帧、零丢失。

同时撤回了三处会误导使用方的说法：500Hz 不是可以花掉的提交预算、`ControlLoop`
不是固定速率、以及一条根本不可能发生在从爪上的警告。撤回的依据都在条目里。

`firmware/` 里的两个镜像都更新并经过硬件验证（leader 1.2.2 / follower 1.1.5），
它们带的三条固件修复对主从都成立。**刷完固件必须断电重插**——软复位会让设备停在
一个所有计数器都干净、却在悄悄丢帧的状态。

### Added

- **`Diagnostics` component** — `gripper.diagnostics()` on both leader and
  follower, wrapping the two firmware commands added in tc-gu-01 1.1.3/1.1.4.

  `uart_stats()` (Cmd 0x54) returns the firmware's free-running UART counters.
  It exists to answer a question nothing on the host can: when a status frame
  arrives a couple of bytes short, did the MCU fail to send them, or were they
  lost after leaving it? `tx_bytes_ok` / `tx_calls_ok` count only what the
  firmware's transmit call accepted, so comparing them against what the host
  decoded over the same window separates the two. That comparison is what
  established the byte loss is downstream of the MCU (tc-gu-01#1): firmware
  reported 6013 frames and 246417 bytes out with zero failures while the host
  still lost 40 frames.

  `set_log_config()` / `disable_logging()` (Cmd 0x55) turn firmware logging on
  and off at runtime. Firmware 1.1.4 ships with logging off because its log
  sink is a blocking polled UART write (~0.5 ms per line at 921600) that stalls
  whichever task emitted the line — logging on every received command is what
  livelocked the firmware's command channel. Treat this as a diagnostic lever,
  not a setting: the docstrings say so, and note the output goes to the MCU's
  DEBUG UART, which is not routed over USB, so enabling it without a probe on
  that pin costs the realtime penalty and shows nothing.

  `ControlLoop`'s stream-locked phase is now documented as holding under a full
  production load, which is the condition that actually matters to callers: with
  every camera on both grippers streaming (4 tactile at 640x480 MJPG 120 fps + 2
  wrist at 30 fps, same USB tree), four 60 s runs came back at exactly 6000
  submits : 6000 frames : 0 missing. Free-running at 100 Hz on the same bench
  lost 156-308 frames per run with or without the cameras — the camera load
  barely moves it, because what matters is *when* a write lands, not how busy
  the bus is.

  The header also now warns about a trap we fell into: an early 600 s
  free-running run at 100 Hz came back clean, but only one gripper assembly was
  plugged in (4 USB devices instead of 8). With both attached, that same
  configuration loses 4% of its frames. Bench population changes the answer.

  One caveat was **withdrawn as unreachable**. `ControlLoop` warned that enabling
  IMU and encoder streaming would shrink the idle window stream-locking aims at.
  It cannot: the follower firmware emits motor status and nothing else — every
  other source sits inside `#ifdef ENABLE_MASTER_GRIPPER` in the firmware's
  stream task. Requesting 1000 Hz of IMU and encoder was measured to yield zero
  frames and leave byte volume unchanged, with stream-locked runs still at
  2000 submits : 2000 frames : 0 missing. The transmit duty cycle on a follower
  is fixed at ~1.4%, so the margin cannot erode. The warning was real for the
  leader, which does stream all four sources — but `ControlLoop` only takes a
  `FollowerGripper`, so it never applied to that class.

### Changed

- **BREAKING: `FollowerGripper::start_streaming()` takes only `motor_hz`.**
  It was `(imu_hz = 100, encoder_hz = 100, motor_hz = 0)`, mirroring the
  leader — and on a follower those first two rates set their mask bits and then
  produced nothing, because the firmware compiles IMU, encoder and eskin
  streaming out on this role. The defaults made it worse than misleading: the
  bare call `g.start_streaming()` requested two sources the follower ignores and
  *no* motor status, so it started a stream that carried nothing and returned
  success.

  Now `start_streaming(motor_hz = 100)`, and `motor_hz = 0` raises
  `IoError(EINVAL)` instead of starting an empty stream. Nothing in this repo
  passed IMU or encoder rates to a follower, so nothing here needed updating;
  callers that did were getting silence for them anyway. Verified on hardware:
  the bare call now yields 100 Hz, `motor_hz=0` throws, and the old keyword
  arguments are gone from Python.

  Two documentation corrections landed alongside it:

  - **`Motor::submit_*` no longer advertises a 500 Hz submission budget.** The
    firmware really does apply the latest target at 500 Hz, but that was being
    read as "submitting at 500 Hz is free", and it is not: every host->MCU frame
    that lands while the MCU is transmitting costs a whole status frame
    (tc-gu-01#1). Rate does not predict the loss — 250 Hz lost 154 frames on one
    60 s run and none on the next, 300 Hz was clean where 250 Hz was not, and
    1000 Hz has produced both 0 and 146 on the same firmware. It only sets how
    many chances to collide you take per second. The header now says that, and
    points at `SubmitPhase::StreamLocked`.
  - **`ControlLoop`'s ACK-response caveat now carries firmware 1.1.4.0 numbers.**
    Concurrent traffic still corrupts command responses that the quiet control
    runs never lose (1-2 per 6000 commands on 1.1.4.0, 5-6 on 1.1.2.0; control
    arms zero on both). The count fell, the exposure did not. 1.1.4.0 separately
    halved command latency (877 us -> 489 us mean) by no longer blocking tasks
    on debug logging — a latency gain, not collision immunity.

  `decode_uart_stats()` accepts both the 32-byte packet firmware 1.1.3 answers
  with and the 36-byte one from 1.1.4, zero-filling the missing tail, so one
  SDK build talks to either. `log_dropped` therefore reads 0 against 1.1.3 and
  is not distinguishable from a genuine zero — gate on `firmware_version()` if
  that matters. `cpp/tests/test_diagnostics_codec.cpp` pins both lengths.

- **`ControlLoop` submits in phase with the status stream**, via
  `Config::phase` (`SubmitPhase::StreamLocked`, the new default; the old
  behaviour is `SubmitPhase::FreeRunning`). StreamLocked fires one MIT frame
  per received motor-status frame, so every write lands in the ~9.8 ms the MCU
  is not transmitting instead of landing wherever an independent timer happens
  to put it.

  This exists because the MCU drops bytes out of the middle of a status frame
  it is transmitting when host->MCU traffic overlaps that transmission. A
  41-byte status frame at 3 Mbps occupies only ~137us of each 10 ms period, so
  a free-running submitter collides rarely and at random -- which is why the
  resulting rate drop reads as sporadic and refuses to reproduce on demand.

  Measured on hw_v1.1.0, 100 Hz motor-status stream, 60 s per run, submits at
  250/s:

  | phase | submits | frames | missing | resync_bytes |
  |---|---|---|---|---|
  | free-running | 15002 | 5995 | 5 | 195 |
  | free-running | 15002 | 5846 | 154 | 6005 |
  | free-running | 15003 | 5983 | 17 | 662 |
  | free-running | 15002 | 5987 | 13 | 505 |
  | stream-locked | 18000 | 6000 | 0 | 0 |
  | stream-locked | 18000 | 6000 | 0 | 0 |
  | stream-locked | 18000 | 6000 | 0 | 0 |
  | stream-locked | 18000 | 6000 | 0 | 0 |

  Four out of four free-running runs lost frames; four out of four locked runs
  lost none. The locked runs put *more* traffic on the link (18000 frames vs
  15002) and lost nothing, so this is not about bandwidth or command volume -- only about
  whether a write overlaps the MCU's send. `crc_errors` stays 0 throughout: the
  damaged frame is short by a couple of bytes and fails the LEN check before
  CRC is ever reached.

  `Config::hz` now defaults to 100 to match the rate StreamLocked actually
  produces, and is consulted only by FreeRunning. A StreamLocked loop that
  receives no status frames for 2 s logs an error rather than silently
  submitting nothing (the usual cause is riding a caller's stream that has no
  `StreamSrc::MotorStatus`).

  **This is avoidance, not a fix, and its scope is narrower than the headline.**
  The loop knows when the MCU emits telemetry; it has no idea when the MCU is
  answering a command, so **ACK responses are still exposed**. Measured with no
  stream running and a 100 Hz `GetMotorStatusExt` poll, adding 250 Hz of
  concurrent no-ACK traffic corrupted 5-6 responses per 6000 commands against
  zero in the paired control runs:

  | arm | commands | retries | ack timeouts | resync bytes |
  |---|---|---|---|---|
  | control | 6000 | 0 | 0 | 0 |
  | control | 5997 | 1 | 1 | 0 |
  | + 250 Hz no-ACK | 5984 | 5 | 5 | 364 |
  | + 250 Hz no-ACK | 5981 | 6 | 6 | 441 |

  Commands survive this because they retry — `ack_fail` was 0 in every run, so
  a corrupted response costs ~31 ms of latency and nothing else. Stream frames
  have no retry, which is why one defect reads as a rate drop on telemetry and
  as nothing at all on commands. Also unmeasured: streams with several sources
  enabled raise the MCU's transmit duty cycle well above the 1.4 % that makes
  the idle window so forgiving today.

  The defect itself is in the firmware's UART path — tracked as tc-gu-01 issue
  #1, and still worth fixing there.

- **`Transport` serialises its writers.** `send_cmd()` and `send_cmd_no_ack()`
  now take an internal mutex around the write, so calling them from several
  threads cannot splice two frames together. This is hardening, not a fix for
  an observed failure: on a blocking tty `write()` is whole-call atomic (n_tty
  holds `atomic_write_lock` and loops internally), measured at 40 x 100 kB
  writes against a starved drain with zero short writes, so
  `SerialBus::write()`'s partial-write loop never actually ran. The mutex makes
  the invariant the code's own rather than a property of the kernel's tty
  layer that a non-blocking fd would silently remove.
  `cpp/tests/test_transport_concurrent_write.cpp` pins it.

### Fixed

- **Documentation that blamed the wrong layer.** `ControlLoop` claimed two
  writers on one serial link corrupt frames; they do not (see above). The real
  reason to keep one owner on the link is the firmware: host->MCU traffic that
  overlaps the MCU's own transmission makes it drop bytes out of the middle of
  the frame it is sending. Measured against hw_v1.1.0 with a 100Hz
  motor-status stream, the damaged frame arrives a couple of bytes short, fails
  the LEN check before CRC is reached, and is discarded whole -- so
  `crc_errors` stays at 0 while `resync_bytes` climbs in one-frame steps and
  `bytes_read` stays at full volume. `Transport::Stats` documented that
  signature as host-side byte loss only; it now names both causes and how
  `bytes_read` tells them apart.

## [0.1.8] - 2026-08-20

Wrist fisheye rectification, command set **V2.2** follower diagnostics, and
the first tagged release since `v0.1.0` — 0.1.1 through 0.1.7 were written
into this file but never tagged, so `git describe` had been understating the
tree by seven releases. Tagging resumes here.

### Added

- **Loss accounting on `Transport::stats()`**, so a rate drop can be attributed
  instead of guessed at. Read together: `crc_errors` / `resync_bytes` rising
  means bytes were lost before the parser (the host stopped draining the tty);
  `queue_dropped` rising means the bytes were fine but the subscriber could not
  keep up; both flat with a low rate means the firmware really is sending less.
  `callback_max_us` names the culprit in the second case, and
  `queue_high_water` / `parser_overflow_bytes` fill in the margins. All six are
  exposed on `TransportStats` in Python, and `repr()` prints the useful subset.
  `FrameParser::stats()` carries the parser-side counters directly.
- **Command set V2.2 — follower motor diagnostics** (firmware `hw_v1.1.0` @
  `bf0a06e`, follower 1.1.2; the leader is unchanged at 1.2.1 because every
  V2.2 command is follower-only). The upgrade is **purely additive**:
  `Cmd::GetMotorStatus` (0x50) and the `MotorStatus` DATA stream still carry
  the same 31-byte payload, so existing code — `read_status()`, `on_status()`,
  `ControlLoop` — is untouched and keeps working against both firmware
  generations. Payload length is *not* a firmware probe; use `Cmd::GetVersion`.
  - `Motor::read_status_ext()` (`Cmd::GetMotorStatusExt`, 0x53) — the 72-byte
    `MotorStatusExt`. Bytes 0..30 are byte-identical to `MotorStatus`; the tail
    adds the motor's fault word, its power-on latched OR, a stop-time snapshot,
    firmware collection-health flags and the raw cmd-5 CAN reply.
  - `Motor::fault_report(force=False)` (`Cmd::GetMotorFault`, 0x52) — the
    64-byte `MotorFaultReport`, merging the motor fault word, the MCU's own
    firmware-level fault state and the CAN evidence into one snapshot.
    `force=True` costs a CAN round trip and can disturb a running control loop.
  - `Motor::get/set_startup_limit_torque()` (0x3A / 0x3B) — the power-on
    limit-torque value the firmware writes to the motor's `0x700B` on every
    boot, replacing the old hard-coded 6 Nm. Works under MIT as well as
    Private. Note the coupling: a successful
    `set_private_param(0x700B, ...)` now *also* rewrites this stored value.
  - `FollowerGripper::set_auto_cal_stall_param()` — short-form
    `Cmd::SetGripperAutoCalConfig` (0x68) writes that patch only the
    stall-detection fields, no read-modify-write needed.
    `GripperAutoCalStallParam` (10 B) / `GripperAutoCalStallParamEx` (16 B).
  - Four decoded motor-status bits the firmware now reports in the existing
    16-bit `status` field: `DriverFault`, `PositionInitError`,
    `HardwareIdError`, `EncoderUncalibrated`. These reach `read_status()` and
    the DATA stream too, not just 0x53.
  - Supporting constants: `MotorMonitorFlag`, `MotorMonitorDiag`,
    `MotorStopReason`, `MotorFaultBit`, `MotorFaultSource`,
    `MotorFaultReportFlag`, `FirmwareFaultCode`. Following the existing
    `MotorStatusBit` convention, the bit *masks* stay C++-only; Python gets the
    raw integer fields plus the `MotorStopReason` enum.

- **Wrist fisheye undistortion.** The SDK could already *read* the intrinsics
  the firmware persists (`Calibration::read_fisheye()`, `cal.K` / `cal.D` in
  Python) but had nowhere to apply them. New `FisheyeUndistorter` builds the
  remap tables once from a `CameraFisheyeCal` and rectifies frames with
  `cv::remap`, mirroring the PC tool's `create_undistort_maps()`. It is
  transport-free, so it also works on frames this SDK never captured — the
  common case, since the wrist UVC device is usually owned by an external
  service.
  - `Camera::set_undistorter()` makes `read()` and the streaming callback hand
    out rectified frames. Safe to call while streaming; if rectification throws,
    the raw frame passes through and the error is logged rather than killing
    the capture loop.
  - `Config::undistort_wrist` / `Config::fisheye_balance` on both grippers wire
    it automatically at `open()`. A unit that cannot supply a calibration —
    never calibrated (`CalNotSet`), an all-zero record, or firmware older than
    command set V2.0 (`InvalidCmd`) — falls back to `FISHEYE_FALLBACK_CAL`, the
    SDK's reference intrinsics, with a warning naming which of the three it was.
    Undistortion is therefore always installed; only a camera that is not at the
    calibrated resolution throws.
  - `balance` interpolates the output focal length from the calibrated value
    (0, the default and the PC tool's default) to 0.70x (1, widest field of
    view). Only fx/fy move — the principal point stays put so the view does not
    drift as the knob turns.
  - **Only 640x480 is accepted.** The firmware record holds just the 8
    intrinsic/distortion floats and no image size, so serving another
    resolution would mean guessing a scale factor and silently rectifying
    wrongly. Adding the image size to the firmware payload is the right fix if
    a second resolution is ever needed.
  - Links `opencv_calib3d` and `opencv_imgproc` on top of the existing core and
    videoio.
- **`scripts/check_protocol_drift.py`** — fails when the SDK's hand-written
  protocol mirror falls behind the firmware headers. The existing
  `static_assert(sizeof(...) == N)` lines catch mirroring something *wrong*;
  nothing caught *not mirroring it at all*, which is how both V2.2 and the
  `EncoderConfig` shrink went unnoticed. Three checks: command table and error
  codes are compared by wire value straight out of `protocol_cmd.h` (zero
  maintenance — a new `#define CMD_*` fails until the enum gains it), and
  payload sizes come from actually compiling both headers, so the numbers are
  the compiler's rather than a regex's. Skips cleanly when no firmware clone is
  present; `--require` makes its absence an error.
- **GitHub Actions CI** (`.github/workflows/ci.yml`) — build + unit tests, plus
  the drift check. `tc-gu-01` is private while this repo is public, so the
  drift job needs a `FIRMWARE_REPO_TOKEN` secret (read-only deploy key or
  repo-scoped fine-grained PAT). The secret is optional by design: GitHub
  withholds secrets from fork pull requests, where the check degrades to a
  SKIP rather than failing work it cannot verify.

### Changed

- **The docs describe the code again.** `docs/ARCHITECTURE.md` had opened with
  a banner saying its own diagrams were historical — that `libxensesdk`,
  `vision.hpp`, `TactileSensor` and `TactileFrame` had gone in 0.1.4 and the
  sections below still described them. The diagrams are updated and the banner
  is gone. The module map is rebuilt from the tree: it had listed four files
  that no longer exist and 7 test files where there are 21, and was missing the
  whole follower stack, `calibration`, `ota`, `log` and `fisheye_undistorter`.
  §5 recommended the `TactileSensor` class §3.4 said was deleted; §7 linked a
  library from a submodule that is gone; the intro and §8 disagreed about
  whether the motor stack lives here; §8 pointed lerobot integration at a fork
  and a Robot class that are not how it works (TacCap is a gripper *backend*);
  §9 quoted a gtest count from a suite a fifth its current size.
- **The README's fisheye section pointed at the wrong API.** It told callers to
  run `cv2.fisheye.undistortImage(img, cal.K, cal.D)` themselves, which was the
  only option when it was written and now yields a *different* image from both
  this SDK and the PC calibration tool. It points at `FisheyeUndistorter`, with
  `cal.K` / `cal.D` still exposed for code that must do its own.

- **Shipped follower image bumped to 1.1.2** (`hw_v1.1.0` @ `bf0a06e`), the
  build that carries command set V2.2. The leader image is untouched at 1.2.1
  (`6b4605a`) — every V2.2 command is follower-only, so it had no reason to be
  rebuilt. `firmware/manifest.json` consequently moves `protocol` and `source`
  from the top level down into each image entry; the fields `ota_update.py`
  actually reads (`crc32`, `sn_suffix`) are unchanged.
  - **This follower image is a local build**, produced with
    `arm-none-eabi-gcc 13.2.1` rather than the firmware team's release
    toolchain, and is **not yet hardware-validated**. Replace it with the
    official artifact when that lands.
  - Its size and CRC32 are **not comparable with the 1.1.1 row's**: rebuilding
    the previous image from its own commit with this toolchain also fails to
    reproduce it (150,044 B against the shipped 149,256 B), so roughly 800 of
    the ~6,800 added bytes are toolchain, not new firmware code.
- **Follower power-on auto-calibration retuned in firmware 1.1.2** — wire
  layout unchanged, behaviour is not. Stall is now confirmed from a single
  sample held for `stall_hold_ms` instead of averaged over several, the open
  stall records the frame *before* the trigger rather than the fully-jammed
  pose, and 0.013 rad is subtracted from the saved `max_open` as a safety
  margin. Expect a slightly smaller `max_open` than the same hardware reported
  on 1.1.1, and re-run calibration after upgrading if you depend on the exact
  span. `close_confirm_count` / `open_confirm_count` are now compat-only fields.
- **Firmware versions are presented as `MAJOR.MINOR.PATCH`** — the fourth
  "build" byte is no longer shown anywhere user-facing. Firmware pins it to 0
  and it carries no meaning, so printing `1.2.1.0` only invited people to type
  the trailing zero into version comparisons. The byte is untouched on the
  wire and still readable as `FirmwareVersion::build` / `OtaTargetVersion.build`.
  Formatting is now funnelled through one place per language —
  `protocol::version_string()` in C++ (used by the gripper `open()` logs and
  the OTA log) and `_calib_flow.format_version()` in the example CLIs — so the
  format cannot drift between surfaces again. `--target-version` accepts three
  parts and still accepts the legacy four so existing scripts keep working.
- **Shipped firmware images bumped to leader 1.2.1 / follower 1.1.1**
  (firmware `hw_v1.1.0` @ `6b4605a`, tags `master_v1.2.1.0` /
  `slave_v1.1.1.0`). **No protocol change** — `protocol_cmd.h`,
  `protocol_data.h`, `protocol_frame.h` and `PROTOCOL.md` are byte-identical
  to 1.2.0, so the command set stays V2.1 and no SDK API moves. The release
  only retunes the status LED: normal state is solid **white** at brightness
  20 (was green at 10), the fault blink halves to 500 ms, and the key-press
  LED reactions are commented out. Upgrading is therefore optional as far as
  the SDK is concerned.
  `firmware/manifest.json` is regenerated alongside the images — the OTA
  role guard identifies them by CRC32, so a stale manifest silently stops
  protecting against flashing the wrong role's image.
  Both bench leaders were flashed to 1.2.1 over OTA and re-checked
  afterwards: version reads back 1.2.1, and the flash-persisted encoder zero,
  travel span (1.1582 / 1.1486 rad) and IMU all survived the bank swap
  untouched.
- `protocol::Ws2812EffectType` doc comments corrected: `NormalSolid` is solid
  **white** as of leader 1.2.1 (the comment still said green), and
  `FaultBlink` is 500 ms. These presets live in firmware and have now changed
  once, so the comments say which firmware they describe rather than implying
  the enum fixes them.
- **`calibrate.py` selects the gripper by side**: `calibrate.py left` /
  `calibrate.py right` instead of hunting for a firmware SN first. The SN is
  resolved from `scan_grippers()` (side comes from the burned SN read over the
  wire, `Cmd::GetSn`) and printed in the header along with every gripper the
  scan saw, so the pick is verifiable before anything reaches flash. Two
  grippers reporting the same side is an error naming both SNs, never a guess.
  Passing an explicit firmware SN still works unchanged.

- **BREAKING: a stream rate of 0 now turns that source off.** `start_streaming()`
  built the `source_mask` from a hard-coded `Imu | Encoder`, so `imu_hz=0` did
  not disable the IMU — the firmware gates emission on the mask bit alone and
  treats rate 0 as its 100 Hz default, so callers who asked for "no IMU" were
  served a 100 Hz IMU stream (a third of the frame load on a two-source setup,
  plus 100 callbacks/s). The mask is now derived from the rates. Callers who
  relied on the old behaviour to get 100 Hz from a 0 must pass 100 explicitly.
  All rates zero now raises `IoError(EINVAL)` instead of starting a stream that
  carries nothing.
- **`start_streaming()` warns when the firmware will not honour a rate.**
  Verified against `third_party/firmware/tc-gu-01` @ `bf0a06e`
  (`App/tasks/task_data_stream.c`), none of which is NACKed on the wire:
  - Motor status is capped at `STREAM_MOTOR_MAX_RATE_HZ` = **100 Hz** ("leave
    bandwidth for the control channel"), so `motor_hz=200` is served at 100 Hz.
    There is no setting that streams motor status faster.
  - The scheduler divides a 1 kHz tick by an integer, so only divisors of 1000
    are exact: 300 Hz arrives as 333 Hz, 150 Hz as 167 Hz.
  - Above 1000 Hz the divider underflows to 0 and the firmware rewrites it to
    10 — asking for 2000 Hz yields 100 Hz, not 1000 Hz.

  The model lives in `cpp/src/stream_rate.hpp` and is pinned by
  `test_stream_rate.cpp` so it fails loudly if the firmware scheduler changes.

- **Subscriber callbacks moved off the transport reader thread.** `Transport`
  now runs a second thread: the reader does `read()` -> parse -> enqueue, and a
  dispatcher drains a bounded queue and fans out to `on_data()` / `on_status()`
  subscribers. Callbacks are still serialised and still delivered in order, and
  `unsubscribe()` still takes effect for already-queued frames — but they no
  longer sit inside the read loop.

  This fixes stream rates collapsing under a slow subscriber. A Python callback
  must take the GIL, and CPython's default 5ms switch interval is roughly 3x the
  per-frame budget of a 200Hz three-source stream (~1.67ms), so a busy main
  thread would stall `read()`, overflow the kernel tty buffer (4KB on n_tty) and
  lose *bytes* — corrupting frames rather than merely delaying them. Callback
  cost now shows up as latency and queue depth instead of as data loss.

  Two consequences worth knowing:
  - ACK matching stays on the reader thread, so `send_cmd()` no longer times out
    and retries behind a slow subscriber.
  - The queue is a burst absorber, not a backlog: when full it evicts the
    **oldest** frame (state telemetry wants currency, not history) and counts it.
    Depth is `Transport::Config::dispatch_queue_frames` /
    `Transport(dispatch_queue_frames=...)`, default 256 (~400ms at 600 frames/s).

### Fixed

- **`EncoderConfig` was 14 bytes on the wire; the firmware wants 5.** Firmware
  `0086da6` (2026-05-27) dropped `baudrate` / `resolution` / `ratio` along with
  the retired RS485 Modbus encoder driver — the encoder is SPI MT6816 now — but
  the SDK kept mirroring the old layout. That broke both directions of the
  command: `Cmd::SetEncoderConfig` sent 14 bytes where the firmware's command
  table accepts exactly `sizeof(encoder_config_t)`, so it NACKed
  `LengthMismatch`, and `Cmd::GetEncoderConfig` threw `ProtocolError` on the
  5-byte response. `EncoderConfig` is now `{uint8_t direction; float
  offset_rad;}`. No published API surface changes — the struct was reachable
  only through the codec layer, never exposed on `Encoder` or in Python.
  Found by the new drift check below, ~2.5 months after the fact.

- **Segfault in `Transport.stop()` whenever a raw `Transport.subscribe()`
  callback was live.** The binding held the Python callable through a bare
  `make_shared`, so the final decref ran on a thread with no GIL: the binding
  releases the GIL around `stop()`, which then drops the subscriptions. `Motor.on_status` and friends were
  fixed for this previously; the raw `subscribe()` path was missed. The GIL-safe
  holder and invoker now live in one shared header (`python/bindings/gil_safe.hpp`)
  rather than in `components.cpp`'s anonymous namespace, so there is no longer a
  copy for a binding to get wrong.

## [0.1.7] - 2026-08-03

Sync to firmware protocol **V2.1** (`hw_v1.1.0` @ f5dd086; leader firmware
1.2.0, follower 1.1.0).

Validated on two leader grippers flashed from source to 1.2.0 over OTA.

> **Downstream on pybind11 2.x should update.** The Python IMU vector fix
> below changes the data affected consumers read — `accel_mps2` /
> `gyro_radps` / `mag_uT` were returning the x component three times. It only
> bites builds made against **pybind11 2.9**, which here means the system
> py3.10 wheel used by ROS 2 Humble; conda py3.12 builds (pybind11 3.0.x)
> resolved the overload correctly and were never affected. Check yours with
> `python -c "from xense.taccap import ...; print(sample.accel_mps2.strides)"`
> — `(0,)` means affected, `(4,)` means fine.

### Added
- **Fisheye camera calibration** (`Cmd 0x2B`). New `protocol::CameraFisheyeCal`
  (32 B body: `fx, fy, cx, cy, k1..k4`) and a `Calibration` component reachable
  as `g.calibration` on **both** leader and follower:
  `read_fisheye()` / `write_fisheye()`. Bound in Python, where
  `CameraFisheyeCal` also exposes OpenCV-shaped `.K` (3×3) and `.D` (4,)
  numpy views for `cv2.fisheye.undistortImage`.
- **Leader encoder max travel angle** (`Cmd 0x2C`, leader only). Same
  component: `read_encoder_max_rad()` / `write_encoder_max_rad()`. The
  follower NACKs `InvalidCmd` — it has no MT6816.
- **`ErrorCode::CalNotSet` (0x60)** — the firmware returns this instead of
  zeros when a calibration record has never been written, so "never
  calibrated" is distinguishable from "calibrated to exactly 0". Both read
  methods surface it as an empty `std::optional` / Python `None`; every other
  error still throws `ProtocolError`.
- **Normalized leader gripper position.** `EncoderSample` gains
  `position` — the opening in `[0,1]` (0 = closed, 1 = fully open), derived
  from the encoder-max calibration. `LeaderGripper::Config` gains
  `normalize_position` (installs the converter on the `Encoder`, so one-shot
  reads *and* streamed samples carry it) and `encoder_max_rad` (host-side
  override that skips the firmware read — needed on pre-V2.1 firmware).
  `LeaderGripper` also gains `position()` / `pos_to_rad()` / `rad_to_pos()` /
  `position_map()` / `reload_position_map()`, mirroring `FollowerGripper`.
- `Encoder::set_position_map()` / `clear_position_map()` /
  `has_position_map()` / `position_map()`; `GripperPosition::from_travel()`
  for calibration sources that aren't a `GripperConfig` record.
- `python/examples/fisheye_cal.py` (show / set-fisheye / set-encoder-max /
  guided `measure-encoder-max`) and
  `python/examples/leader_normalized_position.py`.

### Fixed
- **Python IMU vectors returned the x component three times.**
  `ImuSample.accel_mps2` / `.gyro_radps` / `.mag_uT` came back as `[x, x, x]`:
  `make_vec3` built its array with `py::array_t<float> arr(3)`, which on
  pybind11 2.9 picks a different overload and yields a shape-(3,) array with
  **stride 0** — a broadcast view of element [0], so the writes to `p[1]` and
  `p[2]` landed on the same address. No error, no crash, just silently wrong
  data on every Python IMU read.
  **Scope: builds made against pybind11 2.9 only.** Here that is the system
  py3.10 wheel (ROS 2 Humble); the conda py3.12 builds use pybind11 3.0.x,
  which resolves the overload correctly — verified against a pre-fix 3.0.4
  build that reported correct per-axis strides. The C++ side was never
  affected either way. Spelling the shape as a container makes it correct on
  every pybind11 version rather than relying on which one happens to be
  installed.
  Pre-existing, and independent of the V2.1 work — reproduced on a gripper
  still running the old firmware. Verified on two units: before
  `accel=[9.561, 9.561, 9.561]` (|a| = 16.56), after
  `accel=[9.660, 0.020, -2.412]` (|a| = 9.96 ≈ g). Same overload trap as
  `CameraFisheyeCal.D`; `make_vec3` was the last remaining instance.
  **Downstream consumers should update** — this changes the data they see.
- **A failed OTA reported success.** `OtaSession` only checked
  `ack.is_nack`, but firmware handler errors take the echoed-cmd path
  (`protocol_send_response(seq, cmd, err, NULL, 0)` — command byte intact,
  error as the whole payload), which the transport surfaces as a
  "successful" 1-byte response. A rejected `OtaWriteBlock` therefore did not
  throw: the loop wrote every remaining block, `verify()` and `apply()`
  swallowed their errors too, and `update_from_file()` returned normally on a
  firmware that had aborted the session. Nothing was bricked — the firmware
  refuses to swap banks — but the host lied about it. All OTA calls now go
  through the new `bus::ack_error_code()`.
- **Host retry could kill an otherwise fine OTA.** `Transport::send_cmd`
  allocated a fresh seq per retry attempt, so a merely-slow `OtaWriteBlock`
  ACK caused the same offset to be re-sent as a brand-new request. The
  firmware demands strictly sequential offsets and marks the whole session
  failed on a repeat (`ota_driver.c` — `offset != bytes_written` ⇒
  `OTA_STATE_ERROR`). New `bus::RetryMode::SameSeq` reuses the seq so the
  firmware recognises the repeat and replays its cached response instead of
  re-running the handler — which is exactly what firmware V2.1's
  `protocol_resend_cached_response` was added for, and which the SDK could
  never reach while it bumped the seq. `OtaSession` uses it for every
  command; `RetryMode::NewSeq` stays the default everywhere else, so no other
  call site changes behaviour.
- **`OtaStart` could time out on the follower.** Firmware >= V2.1 stops the
  motor and switches it to MIT inside the handler (300 ms feedback confirm +
  CAN-id probes + parameter waits + two flash writes), which can exceed the
  old 1000 ms default; the retry then found the session already open and got
  `OtaBusy`. Default raised to 5000 ms, and `write_block` 500 → 1000 ms
  (every 8th block triggers an 8 KB erase + program).
- **Process abort at interpreter shutdown** (`FATAL: exception not rethrown`)
  when a Python `on_data` / `on_status` subscriber was still registered at
  exit. The transport reader thread is a plain `std::thread` that nothing
  stops at teardown, so a DATA frame arriving after `Py_Finalize` — easy to
  hit, because the firmware keeps flushing queued frames for a while after
  `StopStream` — called into a finalized interpreter and aborted inside
  `PyGILState_Ensure`. Pre-existing; reproduced at 0.1.6 with none of the
  V2.1 work applied. Three parts:
  - `bus::Transport::stop()` now drops all subscriptions **before** joining
    the reader, so no callback can be entered once shutdown has begun and
    every callback object is destroyed on the caller's thread. The callbacks
    are destroyed outside `sub_mu_` — their destructors take the GIL, and
    holding the subscription mutex across that invites a lock-order inversion
    against the reader.
  - The Python binding's callback wrappers and the GIL-acquiring `shared_ptr`
    deleter now check for a finalizing/finalized interpreter and drop the
    late event (the deleter leaks the object rather than aborting — the
    interpreter is tearing down anyway).
  - `Motor.on_status` used a bare `make_shared` instead of the GIL-safe
    wrapper every other component uses — the exact hazard. Fixed.

### Changed
- `EncoderSample::position_rad` is **unchanged** — it still always reports
  radians. Normalization adds the `position` field rather than repurposing an
  existing one; `position` is NaN while no map is installed.
- **`calibrate.py` now stores the travel span instead of checking it.** It
  used to compare the measured full-open angle against a 1.7 rad "design
  baseline" and warn on deviation, while never storing the value anywhere
  (it predates `Cmd::EncoderMaxCal`). Both halves were wrong: the span *is*
  the calibration, and the baseline was stale — three measurements across two
  units gave 1.1582 / 1.1589 / 1.1486 rad (~66°), all outside its ±0.5 band.
  `--expected-max-open-rad` and `--open-tolerance-rad` are **removed**; step 2
  now persists the measurement, and the firmware-capability probe runs
  *before* the zero is latched so a pre-V2.1 gripper is never left
  half-calibrated. New shared `python/examples/_calib_flow.py` backs
  `calibrate.py`, `fisheye_cal.py` and `leader_normalized_position.py`; the
  latter now offers the guided flow when it finds no calibration at startup.
  It lives in `examples/`, not the package: it prompts on stdin, and the SDK
  must stay usable headless.
- **`LeaderGripper.__exit__` / `FollowerGripper.__exit__` now stop the
  transport**, not just the stream, so the reader thread and its callbacks are
  gone at the end of the `with` block. A gripper used after its `with` block
  now raises `IoError("send_cmd on stopped transport")` instead of appearing
  to work; re-entering a `with` block on the same object is no longer
  supported. This matches `ControlLoop.__exit__`, which already stopped.

### Notes
- The firmware reports handler errors via
  `protocol_send_response(seq, cmd, err, NULL, 0)`, which echoes the command
  byte, so `bus::Transport` surfaces them as a "successful" 1-byte response
  rather than a NACK. `Calibration` resolves that locally (its success
  responses are 33 B / 5 B / `[0x00]`, so a non-zero 1-byte payload is
  unambiguously an error). Transport semantics are unchanged; commands whose
  legitimate success response *is* a single byte (`MotorGetCanId`,
  `MotorGetProtocol`) would need per-command knowledge to disambiguate.
- Firmware V2.1 also raised the auto-cal stall-torque defaults (0.22/0.20 →
  0.35/0.35 Nm) and shortened `stall_hold_ms` (120 → 30) /
  `post_zero_delay_ms` (100 → 30). `gripper_auto_cal_config_t` is still 32 B
  and the SDK reads the config from the device, so no SDK change — but old
  devices get migrated to the new defaults on flash.
- Firmware V2.0 made the cal-result commands (`0x27`/`0x28`/`0x29`) available
  on the follower as well as the leader.

## [0.1.6] - 2026-07-06

Sync to firmware `hw_v1.1.0` @ ab3f98c.

### Added
- **Private-protocol single-parameter access** (`Cmd 0x38/0x39`). New
  `Motor::get_private_param(index)` / `set_private_param(index, raw_value)` and
  `MotorPrivateParam` (8 B: index / type / access / raw_value), bound in Python.
  Only valid when the motor runs the Private CAN protocol — under MIT (the SDK's
  assumed mode) these NACK `InvalidParam`. The firmware whitelists index + R/W.

### Notes
- Firmware changed the *default* auto-cal speeds / confirm count (close 0.15→0.25,
  open 0.20→0.35, confirm 3→1). These are firmware-side `#define`s applied on the
  device; the SDK reads `GripperAutoCalConfig` from the device, so no SDK change.

## [0.1.5] - 2026-07-03

Sync to firmware protocol **V1.9** (`hw_v1.1.0` @ 94273b4).

### Changed
- **BREAKING (wire): `motor_status_t` shrank 40 → 31 bytes.** Firmware V1.9
  dropped `actual_current` / `target_current` / `current_source`; the SDK's
  `protocol::MotorStatus` and `MotorStatusSample` drop them too, and
  `control_mode` now follows `target_torque` directly. The first 18 bytes
  (`actual_pos`..`status`) are unchanged, so position / torque / status stay
  correct across firmware versions — only `target_*` / `control_mode` require
  V1.9 firmware (a V1.9 SDK reading an older 40-byte status gets the wrong
  `target_*`). Decode targets the 31-byte layout.

### Added
- **Gripper power-on auto-calibration config** (`Cmd 0x68/0x69`). New
  `GripperAutoCalConfig` (32 B) + `FollowerGripper::get_auto_cal_config()` /
  `set_auto_cal_config()`, bound in Python. When enabled, the firmware
  self-calibrates on power-up (close-to-stall ⇒ zero, open-to-stall ⇒
  max_open), automating the manual zero + max_open capture.
- **WS2812 LED control** (`Cmd 0x0A/0x0B`). New `Led` component
  (`set()` / `off()` / `effect()` / `effect_off()`) on both `LeaderGripper` and
  `FollowerGripper` (`g.led`), with `Ws2812Set` / `Ws2812Effect` payloads and
  `Ws2812Mode` / `Ws2812EffectType` enums (blink / breathe / HSV / LERP +
  presets). Bound in Python.
- Documented the V1.9 external-control gate: while the firmware owns the motor
  (e.g. power-on auto-cal), `enable`/`set_*` NACK `SysBusy` and `submit_*` are
  silently dropped; there is no query for the state — watch
  `control_stats().applied_seq`.

## [0.1.4] - 2026-06-26

### Fixed
- **Logger no longer crashes when another spdlog copy is loaded in-process.**
  `xense::taccap::logger()` used to fetch the logger from spdlog's global
  registry (`spdlog::get`), whose singleton is a process-wide `STB_GNU_UNIQUE`
  symbol. When a consumer also loads a different spdlog version (e.g. the one
  bundled in `xensesdk`'s `libxense_c.so`), the two share one mismatched-layout
  registry and `get()` returns `nullptr` → the first `logger()->...()` call
  segfaults. The logger is now built once and cached in `cpp/src/log.cpp`
  without touching the registry, making it independent of any other spdlog in
  the process.

### Removed
- **Dropped the `libxensesdk` dependency and the C++ visuotactile path.** The
  `third_party/libxensesdk` git submodule, the `vision.hpp` alias header, and the
  `TactileSensor` / `TactileFrame` classes are gone, along with the
  `LeaderGripper` / `FollowerGripper` `tactile_left()` / `tactile_right()`
  accessors and their `tactile_*_serial` / `rectify_tactile` config. Visuotactile
  (OG) capture and rectification now live at the Python level via the `xensesdk`
  wheel; `xense.taccap` is the gripper-protocol + wrist-camera surface only.
- Removed the now-unused `libxense_version` attribute.

### Changed
- Build no longer requires internal-network submodule access. CMake no longer
  pulls in `libxensesdk`; `taccap_core` links `opencv_core` + `opencv_videoio`
  directly (previously transitive through libxense). Dropped the
  `eigen` / `openssl` / `zlib` / `nlohmann_json` build deps (all libxense-only).

## [0.1.3] - 2026-06-25

### Added
- **MIT force-position control submission path** on `Motor`: no-ACK
  `submit()` overloads (impedance / position / velocity / torque) plus float
  `submit_impedance()` / `submit_position()` / `submit_velocity()` /
  `submit_torque()` wrappers, bound in Python. These send `CMD_NO_ACK` frames
  fire-and-forget for a host-driven realtime loop up to the firmware's 500 Hz
  slave-control rate — no ACK, retry, or throw (only `IoError` on a stopped
  transport). Health is out-of-band via `control_stats()` / `on_status()` /
  `SensorErrors`. The MIT impedance frame is the force-position hybrid primitive
  (kp/kd track `target_pos`; `feedforward_torque` adds the force term). The
  follow/teleop loop and grasp FSM stay in the upper layer.
- `python/examples/motor_mit_control.py`: primitive demo of the submission API
  with the out-of-band health channel.
- **Normalized gripper position** (0 = closed, 1 = open). New `GripperPosition`
  pure converter (raw shaft rad ↔ normalized [0,1], built from `GripperConfig`)
  and `FollowerGripper::position()` / `set_position(pos, kp, kd, ff)` /
  `pos_to_rad()` / `rad_to_pos()` / `position_map()` / `reload_config()`, bound
  in Python. `set_position()` is the normalized counterpart of
  `Motor::set_impedance` (fire-and-forget, no ACK). NOTE: `FollowerGripper::
  set_position()` is normalized [0,1] and distinct from `Motor::set_position()`
  (raw rad). Throws if the gripper isn't calibrated (`GripperConfig` not Valid).
  Validated on real follower hardware (max_open = 1.1802 rad, Reverse).
- **`ControlLoop`** — a fixed-rate send/receive loop for embodied control. A C++
  background thread submits the latest normalized position target as a MIT
  impedance frame at `hz` (fire-and-forget), while the firmware motor-status
  STREAM keeps a thread-safe `GripperObservation` fresh. The policy thread only
  touches `set_target(0..1)` and `observation()` (both non-blocking). Reads
  observations from the push stream instead of polling `GetMotorStatus` (polling
  > ~100 Hz can stall the firmware's status refresh). Bound in Python with
  context-manager support. Validated on hardware (200 Hz submit, ~100 Hz obs,
  obs age a few ms).
- `python/examples/gripper_control_test.py`: interactive open/close control test
  exercising both `set_position()` (one-shot) and `ControlLoop` (realtime).

### Changed
- Promoted the V1.7 follower / motor command surface from "reserved, pending
  hardware" to first-class, validated against firmware `hw_v1.1.0`. The leader
  mismatch behavior is unchanged: these NACK `SensorOffline` → `ProtocolError`
  on leader hardware.

### Fixed
- **Discovery never guesses a side from the CH343 chip SN.** Side now comes from
  firmware sources only — the burned SN (`Cmd::GetSn`, sequence-digit parity) with
  `GetDevType` as a secondary firmware fallback; when neither answers the side is
  reported as the new `Side::Unknown` (bound in Python) instead of the WCH chip
  SN's meaningless parity, which could confidently report the wrong side. The
  `GetSn` probe in `scan_all()` now retries on cold start (the first command(s)
  after a fresh plug-in could be dropped while the USB-CDC link settled, which
  previously left the side falling back to the chip SN on the very first scan).
  `McuEndpoint` drops its chip-parity `side` field. **Minor API addition**
  (`Side::Unknown`); existing `Left`/`Right` are unchanged.

## [0.1.1] - 2026-06-14

### Fixed
- **V1.8 global byte stuffing** in the framing layer (`pack_frame` escapes the
  body ADDR..CRC; `try_parse_frame` is TAIL-delimited and unstuffs before the
  CRC check, which stays over the unescaped HEAD..PAYLOAD). The firmware and PC
  tool escape the wire as of V1.8 — without this, `GetSn` and any frame whose
  body contains 0xAA/0x55/0x7D silently timed out.
- libxense (submodule): VID:PID `3938:1300` added to the device whitelist and
  the `GSPS` serial prefix mapped to the Omni sensor type.

### Changed
- **Discovery is MCU-only.** `scan_grippers()` no longer enumerates the wrist
  camera or visuotactile sensors (an external camera service owns them);
  `GripperEndpoints` drops `wrist_video` / `tactile_*_serial`. `LeaderGripper` /
  `FollowerGripper` no longer open cameras at construction — gated behind a new
  `open_cameras` flag (default off). **Breaking.**
- Examples reworked: `leader_demo` / `calibrate` / `ota_update` are MCU-only;
  `rerun_visualize` opens wrist/tactile only via `--wrist` / `--tactile-*`;
  `rerun_dual_with_tracker` drops the camera panels.

### Added
- **TacCap SN scheme** (`TCGU01A24Z0001m` / `GSPS01A24Z0001`): `parse_serial()`,
  a `Role` enum, `GripperEndpoints.role`, and `find_leader()` / `find_follower()`.
  Side comes from the SN sequence digit, with a `GetDevType` (firmware
  LEFT/RIGHT) fallback, then CH343 chip-SN parity.
- **V1.7 command set** (follower / motor — interfaces reserved, pending
  follower hardware): motor set-zero, CAN-id read/write, protocol switch/query,
  control-stats, and follower gripper-config get/set. New `GripperConfig`,
  `MotorControlStats`, `MotorProtocol`; `MotorStatus` grew 18→40 B and
  `MotorImpedanceCtrl` 16→20 B (lenient decode keeps legacy 18 B working).

## [0.1.0] - 2026-05-27

First usable release. Everything below landed on `main` since the
v0.0.1 bootstrap (c9e8267) — protocol, transport, components, both
gripper aggregates, Python bindings, six example scripts, logging,
encoder calibration ergonomics, and a dual-gripper + Pico-tracker
visualiser.

### Added

**Protocol layer**
- TC-GU-01 wire protocol mirror in C++17 — Cmd enum, FrameType, ErrorCode,
  POD payload structs (IMU / Encoder / KeyStatus / SensorError / OTA /
  IMU MagCal / EncoderConfig). 1:1 with firmware `protocol_cmd.h` /
  `protocol_data.h`, pinned by `static_assert(sizeof(...) == N)`.
- V1.6 mirror — OTA session commands, KeyStatus DATA, IMU MagCal,
  per-sensor calibration result flags, SensorError reports.

**Bus / transport**
- Async `bus::Transport` over termios serial with ACK matching, retry,
  and per-command DATA subscriber dispatch (single reader thread).
- Frame parser with HEAD/TAIL detection + CRC16 verification, including
  recovery from false-positive HEAD bytes mid-stream.

**Components**
- `IMU`, `Encoder`, `Camera`, `TactileSensor` (V4L2 + libxense XU
  rectify), `Motor` (follower-only, FDCAN-via-MCU).
- `LeaderGripper` / `FollowerGripper` aggregates with `read_once` +
  `on_data` callback patterns on every component.
- `IMU::set_mag_calibration(hard, soft)` — write hard-iron + soft-iron
  matrix to firmware (Cmd::SetImuMagCal, 48-byte payload).
- `Encoder::set_zero(timeout=500ms)` — latch current encoder reading
  as the new zero position (Cmd::SetEncoderZero). Throws on NACK /
  timeout.
- `Encoder::normalize()` post-process — clamp `position_rad` to ≥ 0
  to absorb post-calibration drift; rate-limited warning (1 / s per
  instance) when raw drift exceeds -0.1 rad. Raw firmware value
  preserved in `raw.position_rad`.
- `OtaSession` — full V1.3 OTA state machine, including the high-level
  `update_from_bytes()` orchestrator (start / write_block / verify /
  apply / status polling).
- `Key` + `SensorErrors` DATA subscribers (V1.4 / V1.6 streams).

**Discovery**
- `scan_grippers()` enumerates all plugged grippers; `find_left()` /
  `find_right()` / `find_one()` typed lookups.
- Bilateral discovery via USB hub-path grouping so two grippers sharing
  a hub are reliably split into left/right endpoints.
- Side detection reads firmware-burned SN via `Cmd::GetSn` (not the
  CH343 USB chip SN) — survives MCU swaps without relabeling sides.

**Python bindings (pybind11)**
- `xense.taccap` package exposing all components, `LeaderGripper` /
  `FollowerGripper`, `GripperEndpoints`, `Side`, `Cmd` enum,
  `EncoderSample.raw_position_rad` / `raw_velocity_rad_s` for
  pre-clamp diagnostics.
- `xense.taccap.log` submodule — `set_level` / `set_pattern` /
  `info` / `debug` / `warn` / `error` etc., shares the underlying
  C++ spdlog instance (one logger program-wide).
- Python 3.10 + 3.12 build paths (system py3.10 for ROS 2 Humble,
  conda py3.12 for primary dev).
- GIL-safe shared-ptr deleter for callbacks held across worker threads.

**Logging**
- Single-instance `xense::taccap::logger()` (header-only, spdlog
  registry-backed) shared between C++ and Python.
- Two sinks attached by default: stderr (colour) with user-controllable
  level, file (per-session) always DEBUG.
- File sink writes `session_YYYYMMDD_HHMMSS.log` under `$TACCAP_LOG_DIR`
  (default `~/.taccaplogs/`). At most `kMaxSessionLogs` (= 10) sessions
  retained; oldest mtime pruned at process start. File-sink failures
  degrade gracefully — console keeps working.
- Constants `SPDLOG_PATTERN` / `FILE_LOG_PATTERN` define the canonical
  console + archive formats.

**Examples**
- `python/examples/rerun_visualize.py` — single-gripper rerun-sdk
  multimodal viewer (wrist + 2× tactile raw/rect + IMU/encoder time
  series + observed FPS panel).
- `python/examples/v4l2_probe.py`, `v4l2_sweep.py` — manual V4L2
  bringup probes; useful when firmware SN isn't burned yet.
- `python/examples/ota_update.py` — firmware OTA CLI with progress +
  status verification.
- `python/examples/calibrate.py` — per-SN encoder zero calibration
  with raw/cooked side-by-side display, full-open angle sanity check,
  live readout.
- `python/examples/rerun_dual_with_tracker.py` — dual-leader viewer
  augmented with Pico4 motion-tracker 6-DoF poses. JPEG-compressed
  image streams + tight rerun flush knobs for low-latency teleop.

### Changed

- ACK wire format corrected — `cmd=0` is NACK; ACK frame's payload
  carries the cmd's return data, not a status struct.
- `cmd=0` ACK with `err=Ok` now treated as success (firmware quirk on
  some no-payload commands).
- Firmware reference repos (`tc-gu-01`, `tc-gu-01-pc`) relocated to
  `third_party/firmware/` and made clone-on-demand (gitignored, never
  submodules — they have separate release cadences).
- `LeaderGripper` constructor pre-stops any in-flight firmware stream
  before opening + bumps the ACK timeout to absorb the post-reset
  warm-up window.
- `LeaderGripper` ctor logs firmware version + SN — visible in every
  session log without needing extra scaffolding.

### Fixed

- `bus::Transport` parser rewinds past a false-positive HEAD byte
  when downstream framing reports NeedMoreData, preventing stuck
  frames on noisy serial.
- `Encoder` / `IMU` / `Camera` callbacks: GIL-safe `shared_ptr<py::function>`
  deleter so worker threads can release callbacks without UB.
- Logger: replaced the per-TU static cache (which produced split
  storage across `_taccap_native.so` and `libtaccap_core.so` under
  `-fvisibility-inlines-hidden`) with stateless `spdlog::get()`
  lookups — fixes a SIGSEGV in `LeaderGripper` construction.
- `rerun_visualize.py` summary FPS anchored to streaming-start instead
  of process-start so ~4 s of libxense/V4L2 init doesn't drag the
  reported rate.

### Tests

- gtest suite covers protocol codec round-trip, frame parser edge
  cases, transport ACK/NACK paths over PTY, all component decoders,
  V1.3-V1.6 end-to-end (Key / SensorErrors / IMU MagCal / OTA), CRC32
  boundary cases against reference zlib vectors, encoder set_zero
  wire format + NACK, encoder normalize clamp + warn behaviour.
- 126 tests pass; PTY-driven fake-firmware harness in
  `cpp/tests/pty_helper.hpp` for transport-level coverage.

### Docs

- `README.md` — Python + C++ install flows, hardware smoke test,
  example index, calibration walkthrough, logging behaviour.
- `docs/ARCHITECTURE.md` — layered stack, module map, data-flow
  diagrams, threading model, USB-topology discovery, boundary
  between this SDK and downstream consumers (dataset recording /
  ROS 2 / lerobot adapters).
- `CLAUDE.md` — house-rules file for AI-assisted maintenance.

## [0.0.1] - 2026-04-29

### Added
- Initial repository skeleton: CMake + scikit-build-core + pybind11
  with the `xense::taccap::` / `xense.taccap` namespace alias.
- libxensesdk vendored as a git submodule pinned at commit `7d4687e`,
  configured in lite mode (no ML backends).

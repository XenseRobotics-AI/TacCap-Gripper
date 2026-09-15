# Beta 整理前的开发变更记录

归档于 2026-09-15。以下版本分组保留原貌，仅用于追溯，不作为当前发布清单。

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Documentation and frequency benchmark

- Add the MkDocs + Material SDK website with searchable user guides, API reference and customer release notes.
- Add `frequency_test.py` for independently configured application target/read rates and SDK update rates, with native frame-counter deltas, API duration/jitter/freshness statistics, raw CSV, and offline PNG/SVG plots. Cache sequence gaps are reported as unobserved updates, not UART packet loss.

### Customer documentation and arrival correction

- Organize the customer manual, complete Python API reference, install/OTA/calibration guidance and current validation status around the position-control workflow. Separate historical control notes and mark the packaged slave 1.1.6 image as incompatible with the new customer API.
- Fix compensated arrival checks: when a raw compensated target error is available, use it instead of also requiring the uncompensated normalized target. Add regressions for compensated midpoint arrival and incomplete closing compensation. This source revision is not included in the existing 0.1.11 wheel.

### Python customer API

- Unified position, grasp and impedance control in one `Gripper` session. `move/release` now use position trajectories with speed and torque budgets and both-direction obstruction hold; `grasp` retains contact-to-torque hold. Added sustained `impedance()` with bounded PD/feedforward parameters and blocking `hold()` support. Native controller methods and legacy `set_target()` remain available.
- Added passive `Gripper.discover()`, immutable device information and SDK limits, and explicit firmware `diagnostics()`. State now separates control mode from execution phase and includes command/readiness/feedback sequence information. SDK limits are distinct from firmware dynamic torque allowance.
- Waiting requires feedback after command submission, a stability window and a firmware execution-status check. Added configurable position/near-closed tolerances and typed operation, fault and busy errors. Firmware and SDK sequence counters are distinct; this is not an exact per-command motor execution acknowledgement.
- Customer sessions monitor firmware execution about every 200 ms on the existing native worker, including sustained impedance; ownership loss, firmware faults and query failures fault control and request disable. Native monitoring remains opt-in for legacy 1.1.6 controllers. Passive/busy-rejected context cleanup closes only its connection, while active/uncertain sessions request disable. Explicit disable remains a global override; failures propagate and require recovery instead of reporting successful shutdown.
- Added operation-specific `GripperResult`, optional blocking `wait=True`, `wait_result()`, partial release opening and explicit `disable()`. Retained legacy non-blocking defaults, `wait()` state returns and `stop()` disable behavior. Waiters reject replacement operations; contact is not reported as a verified grasp.
- Grouped native controllers, device maintenance and protocol types under `xense.taccap.advanced`; previous named and wildcard imports remain compatible.
- Updated customer guidance and daily/mode-switching examples, with workflow and Python-to-native UART integration coverage. No Python control thread or additional firmware protocol changes for these customer modes; firmware 1.1.7 remains required.


### SDK 0.1.10 / follower firmware 1.1.7

- Shared controller runtime with exclusive motor ownership, callback shutdown synchronization, and disable on stop/stale feedback.
- RS05 manual limits: 5.5 Nm protocol peak, 1.2 Nm stationary holding ceiling; reject invalid raw commands before transmission.
- Added C++/Python `motor.execution_status()` for firmware owner, requested/applied targets, effective envelope and latched error.
- Firmware now expires external targets after 500 ms, disables and latches until explicit clear. **This may release a held object.** Single commands must be refreshed even for stationary holds.
- Firmware default envelope is now 1.0/1.5 Nm when unset; invalid enabled envelopes reject motion. Private-protocol impedance is rejected.
- Protocol drift checks cover envelope and execution-status offsets as well as sizes. See [current control contract](../../docs/CONTROL_LAYERING.md).

### Earlier unreleased changes

### Changed

- **BREAKING: the raw motor primitives are no longer exposed to Python.**
  Removed from the bindings: `Motor.set_impedance` / `set_position` /
  `set_velocity` / `set_torque`, their no-ACK `submit_*` counterparts, and
  `FollowerGripper.set_position` (a normalized wrapper that was itself just
  `motor_.submit_impedance()`).

  Every one of them writes a control frame straight to the wire with no error
  clamp, no torque ceiling and no stall guard — those live in `ControlLoop` and
  `ForcePositionController`, and a caller reaching past them gets none of it.
  This is not hypothetical: a customer console driving `submit_impedance()` at
  `kp=20` into a rigid object let `kp * error` grow to the motor's own `0x700B`
  ceiling; at 24 V the current draw browned out the whole board, the gripper
  dropped what it was holding and the USB link disappeared. See
  [`docs/CONTROL_LAYERING.md`](../../docs/CONTROL_LAYERING.md).

  **What breaks:** any Python code calling those methods now raises
  `AttributeError`. Port it to `ControlLoop` (impedance) or
  `ForcePositionController` (force/position hybrid), both of which take a
  normalized `set_target(0..1)` and expose observations without polling the bus.

  **The C++ methods are unchanged** — both controllers and
  `FollowerGripper::set_position` call them internally. This is a bindings-only
  change.

- **Example cleanup.** Removed `motor_mit_control.py` (its whole purpose was
  demonstrating the raw MIT primitive), `gripper_force_grasp_test.py`
  (host-side contact detection, superseded by `ForcePositionController`),
  `v4l2_probe.py` / `v4l2_sweep.py`, and `rerun_dual_with_tracker.py`.

- **BREAKING: the example device selector is a positional argument everywhere.**
  `impedance_control.py`, `force_position_control.py` and the new
  `gripper_console.py` used to take `--side left|right|follower`; they now take
  the same positional `left` / `right` / firmware-SN that every other example
  already used, and omit it when exactly one gripper is plugged in.

  **What breaks:** scripts passing `--side right` to those three. Drop the flag
  and pass `right`. The convention was already documented in
  `docs/EXAMPLES.md` — these three were the violators, not the rule.


- **BREAKING: the wrist camera now hands out RGB by default.** Its consumers are
  vision and learning pipelines and every one of them wants RGB — LeRobot
  datasets store RGB — so a BGR default meant each consumer converting at its own
  call site, or forgetting to and recording swapped channels with nothing raised
  to notice. `LeaderGripper::Config::wrist_color_mode` /
  `FollowerGripper::Config::wrist_color_mode` default to `ColorMode::Rgb`; pass
  `ColorMode::Bgr` to get the old behaviour back.

  **What breaks:** code that takes a wrist frame straight to `cv::imshow`,
  `cv::imwrite` or any other OpenCV sink and does not say `ColorMode::Bgr` will
  render with red and blue swapped. It will not raise — check any such call site.

  **A bare `Camera` is unaffected** and keeps OpenCV's native BGR: it can be any
  V4L2 device and its frames go anywhere, so the convention every OpenCV caller
  already assumes still holds there.

### Added


- **`python/examples/read_intrinsics.py`** — read-only wrist-camera intrinsics
  export as JSON. Goes through `Calibration.resolve_fisheye()` rather than
  `read_fisheye()`: an uncalibrated unit answers a read with an all-zero record
  rather than a NACK, and handing that to `FisheyeUndistorter` remaps every
  frame to black. The `source` field says whether the numbers came from the
  device (`device`) or the SDK reference values (`reference`); `--require-device`
  turns the latter into a non-zero exit. JSON goes to stdout and diagnostics to
  the log, so it pipes cleanly.

- **`python/examples/gripper_console.py`** — single-gripper keyboard console,
  with `--mode impedance` (`ControlLoop`) and `--mode force-position`
  (`ForcePositionController`) sharing one UI. It is also the motion-safety
  envelope's configuration entry point (`--show-envelope` / `--set-envelope
  --peak --cont --temp-wall`), and the header carries a permanent
  `ENFORCED` / `*** INACTIVE ***` marker — the envelope is the only layer on
  the MIT path that nothing can bypass, and it ships disabled.



- **`Camera::Config::color_mode`** (`ColorMode::Bgr` / `Rgb`) — the channel order
  a camera hands frames out in, and the mechanism behind the wrist default above.

  Conversion runs **after** undistortion, so `FisheyeUndistorter` still only ever
  sees BGR and its contract is unchanged — `remap` is per-channel, so the order
  of the two steps cannot change the result anyway.

  Exposed in Python as `ColorMode`, a `color_mode` argument on `Camera`
  (defaulting to BGR), and a `wrist_color_mode` argument on `LeaderGripper` /
  `FollowerGripper` (defaulting to RGB).

### Fixed

- **The 0.1.8 entry described the wrong fisheye behaviour.** It said a unit that
  cannot supply a calibration "degrades to raw frames with a warning" — the
  behaviour before `39e4a24`, which landed in that same release. What 0.1.8 and
  0.1.9 actually do is fall back to `FISHEYE_FALLBACK_CAL`. The distinction
  matters downstream: reference-rectified frames and raw fisheye frames are not
  interchangeable, and a reader trusting the old line would treat them as such.
  Corrected in place, since it describes what that release shipped.
- **`docs/CALIBRATION.md` claimed an unwritten record always reads back as
  `None`.** An uncalibrated unit may answer the fisheye read with a *present*,
  all-zero record, which passes an `is None` check and then rectifies to a
  uniformly black frame. Documented, with `is_usable_fisheye_cal()` as the test
  to use. First seen on firmware 1.1.1 and still the behaviour on **1.2.2**
  (measured on two leader units), so it is not an old version's quirk.
- **`docs/CALIBRATION.md` gains a caveat on the optical axis.** A rectified frame
  can look off-centre or tilted while the calibration is correct: the sensor is
  not always mounted at the lens's optical centre, and rectification is built
  around the principal point rather than the middle of the frame. Measured on one
  unit, the axis sits 39 px right of centre — corroborated by the gripper jaws,
  which are mechanically symmetric about it — and `FISHEYE_FALLBACK_CAL` is
  37.7 px away from it. Raw fisheye hides the offset by compressing the
  periphery; rectification reveals it. Written down because the natural reaction
  is to "correct" `cx`, which makes it worse.
- **The fisheye fallback was undocumented outside the headers.**
  `docs/CALIBRATION.md` gains a section on `resolve_fisheye()`: the three ways a
  unit fails to supply its own calibration, the reference values that stand in,
  the per-assembly principal-point drift that makes them approximate, and why a
  non-640x480 camera throws instead of falling back.

## [0.1.11] — position streaming

- Add `set_position`, connection-level limits/rate, `start`, cached timing and MCU limit feedback.
- Keep bounded position torque under obstruction; remove automatic position blockage latch.
- Follow reference velocity within the shared torque budget; fade it near travel endpoints.
- Negotiate firmware 1.1.8 execution/CAN-age/raw-position telemetry without periodic ACK polling.
- Add opt-in closing compensation (0..0.05 rad), raw target error and position/CSV example.
- Preserve legacy imports and grasp/impedance APIs; 1.1.7 retains the legacy polling path.

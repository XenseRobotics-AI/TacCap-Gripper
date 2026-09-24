# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- **`Motor::switch_protocol()` and `get_protocol()` send with their own 3000 ms
  ACK timeout.** Measured: while the motor speaks the private protocol both
  timed out on every attempt at the transport's 200 ms default and both
  answered at 3000 ms. The switch triggers a discovery scan of up to ~1.25 s in
  firmware. The old symptom was a `TimeoutError` on a command that works fine in
  the other protocol, which reads as a dead link rather than a slow command.

### Changed

- **`docs/USAGE.md` is now English, with the Chinese original kept as
  `docs/USAGE_CN.md`** — the same split the READMEs use. Translating it surfaced
  four content bugs in the original, all verified against the source and fixed in
  both files: the `holding` criterion was described as
  `commanded_torque_nm >= 95% × grasp_torque_nm`, which appears nowhere in the
  code (it is setpoint lead ≥ half the limit *and* velocity ≤ 25% of commanded,
  as the table two paragraphs earlier already said); `hold_torque_limit_nm` was
  described as clamping the hold, which it does not — it only bounds
  `grasp_torque_nm`; two passages still described contact detection and a
  `kp=kd=0` mode switch that were removed in 2026-09; and the
  `ImpedanceController` example enabled the motor before `start()`, while both
  controllers validate the persisted `0x700B` limit inside `start()`.
- **Documentation corrected where it described APIs that do not exist.** The
  READMEs showed `g.set_position(...)` in a Python snippet — that method exists
  in C++ but is not bound — and claimed raw-radian motor control is unreachable
  from Python, while `Motor.submit_*` is bound and takes raw radians.
  `docs/ARCHITECTURE.md` carried the same claim. Two `--side` flags that no
  script accepts were left in `docs/USAGE.md`.
- **`pyserial` and `rerun-sdk` dropped from `environment.yml`.** Nothing imports
  either: the serial transport is C++ (termios), and the examples that used
  rerun were deleted in an earlier cleanup.
- **`firmware/` ships only the current release.** The 1.2.5 images were removed;
  `manifest.json` has pointed at 1.2.6 since it shipped, and `ota_update.py`
  resolves images through the manifest by CRC32, so the older pair was
  unreachable through any supported path. Older images come from git history.

## [0.2.4] - 2026-09-24

Documentation and tooling only — no library code changed, so a consumer pinned
to 0.2.3 loses nothing by staying there.

### Added

- **CI covers Python and lint, not only C++.** The 96 `python/tests` cases now
  run on every push — they guard what a C++ test structurally cannot: the bound
  config surface, single-sourced versions, non-zero-stride numpy views,
  docstrings. Added alongside them: `pre-commit` (gitleaks, typos, zizmor,
  whitespace/format) and two ruff gates. Ruff is pinned to one exact version in
  three places — the workflow, `environment.yml` and `.pre-commit-config.yaml` —
  because its defaults move between minor releases.
- **`README_CN.md`**, a full Chinese translation of the README. The English
  file remains authoritative; the Chinese one links back to it.
- **A citation entry** at the end of both READMEs. Its `version` field is a
  placeholder on purpose: cite the release you actually used, and state the
  gripper firmware version alongside it — several documented behaviours are
  specific to a firmware version, not just to an SDK version.
- **`uv` in `environment.yml`.** Installs now read
  `uv pip install -e . --no-build-isolation`; `pip` still works with the same
  flags. uv targets the activated conda env on its own. Measured on a warm
  build dir, 0.5 s against pip's 1.0 s — the CMake compile dominates either way.

### Changed

- **The README is English-only and reorganised.** It opens with how the SDK is
  built rather than a feature list, folds the four places that separately taught
  usage into `## Usage` (example-led) plus `## Writing your own` (the API), and
  states firmware requirements without the protocol internals behind them.
- **`--no-build-isolation` is now explained** where it is used. Without it the
  build resolves `pybind11` from PyPI into a throwaway environment instead of
  the version `environment.yml` pins next to `libopencv` and `spdlog`; pybind11
  is header-only, so whichever copy is present at build time is the one compiled
  into the extension.

### Fixed

- **The README never said what flashing the wrong role's image costs** — a
  bricked MCU needing an SWD probe. The rule to pick by SN suffix was there
  without the reason behind it.

## [0.2.3] - 2026-09-23

Paired with firmware **1.2.6**.

### Added

- **Docstrings across the Python bindings.** pybind11 fills `__doc__` with a
  signature whether or not anyone wrote a docstring, so ~426 of 457 `.def` sites
  looked documented and were not. Classes and methods are now documented
  throughout; fields only where the name does not already carry the unit.
  `python/tests/test_binding_docstrings.py` guards it.

### Changed

- **The post-OTA power-cycle is now stated as "unplug USB and power at the same
  time".** Every place that documented it said "cut 24 V, not just USB", which
  is necessary but not sufficient: the two cables feed different domains — USB
  the MCU and its serial bridge, 24 V the motor — so pulling either alone leaves
  the other half energised, and pulling them one after another (the first
  already back in) never gives the board a moment with no power at all.
  Corrected in the READMEs, `docs/FIRMWARE.md`, `docs/USAGE.md`,
  `docs/EXAMPLES.md`, `manifest.json`, the `OtaSession` docstrings,
  `ota_update.py`'s banner and the firmware-gate hint printed on a refusal.

- **The examples' shared helper `_calib_flow.py` is split into `_target.py` and
  `_calib_flow.py`.** Eleven of the twelve scripts imported it, but 26 of its 37
  call sites were device selection, not calibration — so `impedance_control.py`
  imported a module named "calib flow" purely to get a `left|right|SN` argument.
  `_target.py` is now the selector plus version/colour helpers; `_calib_flow.py`
  is the guided encoder-max walkthrough used by three scripts. Anyone who
  vendored these scripts needs both files.

- **`ImpedanceController` has no stall guard, and the budget defaults to
  1.1 N·m.** The guard clamped the effective target to wherever the jaw had
  stopped, which drove the position error — and with it the torque — to zero:
  measured, a grip let go at 0.35 N·m 60 ms after contact. A blocked jaw now
  saturates the error clamp at `max_position_torque_nm` and holds there, which
  is the grip. `ImpedanceState.STALLED` and the snapshot's `stalled` /
  `stall_trips` are removed. With nothing timing the hold out, the budget has to
  be a torque the motor can sustain indefinitely, so it drops from 1.5 to the
  EL05's 1.1 N·m continuous stall rating.

  `TORQUE_CAPPED` now holds the budget rather than `rated_torque_nm`, and
  `ImpedanceConfig` is rejected unless
  `max_position_torque_nm + |feedforward_torque| < rated_torque_nm`, so the
  ceiling stays a backstop that normal operation never reaches.

### Removed

- **`ControlLoop`, `SubmitPhase` and `StallAction` are gone** (C++ and Python).
  Use `ImpedanceController` to follow a position and `ForcePositionController`
  to grasp. `GripperObservation` stays; it moved to
  `taccap/gripper_observation.hpp`.

  `ControlLoop` carried a second copy of the impedance law, and that copy had
  already drifted from `ImpedanceController`'s: a 1.5 N·m budget against 1.1,
  and the stall guard below that collapses the grip. Nothing in the SDK
  instantiated it and no shipped recipe selected it. What it offered that
  `ImpedanceController` does not was `SubmitPhase::FreeRunning` — measured to
  cost status frames — and runtime gain changes, which `ImpedanceController`
  has as `set_gains()`.

  Migration: `ControlLoop(g, kp=…, kd=…)` becomes
  `ImpedanceController(g, cfg)` with the gains on an `ImpedanceConfig`;
  `loop.observation()` becomes `c.snapshot().observation`; `stalled` /
  `stall_trips` have no replacement (see next entry).

- **`Camera::read()`'s timeout parameter**, in C++ and Python. It was never
  used: the C++ signature commented the parameter out and the call blocks for
  however long `cv::VideoCapture::read` blocks, which is its own V4L2 timeout.
  A caller passing `timeout_ms=50` waited the V4L2 timeout anyway with no way to
  tell. `cam.read(timeout_ms=...)` is now a `TypeError`; drop the argument.
  Add one back only together with poll/select wrapping that honours it.

### Fixed

- **Five pieces of documentation that contradicted the code**: a `GripperEnvelope`
  docstring describing a `max_velocity_rad_s` field that does not exist,
  `follower_gripper.hpp` naming a 1.1.6 firmware floor against its own 1.2.5
  constants, `imu.cpp` / `encoder.hpp` naming the reader thread as the callback
  context where `Transport` guarantees the dispatcher thread, scaffold-era text
  in `module.cpp` and the package docstring, and `motor.hpp` asserting a
  private-parameter MIT gate that firmware 1.2.6 deliberately removed. Also
  `find_leader` / `find_follower` were each exported twice, and nine exported
  names were missing from `__all__`.

## [0.2.2] - 2026-09-23

Released as a tag and a version bump only; this section was written after the
fact, from the tag message, so that the file does not skip the version. The
range is the nine commits between v0.2.1 and v0.2.2.

- `Motor::motor_version()` — read the motor's own firmware version over `0x58`
  (needs follower firmware >= 1.2.6, and so far confirmed only under the
  private protocol).
- **Follower firmware floor raised to 1.2.5** (breaking: `FollowerGripper`
  refuses to open an older unit).
- `ControlLoop` gained config validation, banding its torque fields by how long
  the motor can hold them. Removed again in 0.2.3 along with the class.
- `gripper_console --rated-torque` now derives its default from the library
  instead of a stale literal (it had drifted to 2.0 against a 1.8 ceiling).
- OTA resolves the manifest entry by CRC32 instead of trusting the filename.
- C++ examples build by default, so CI compiles them.

## [0.2.1] - 2026-09-22

Paired with firmware **1.2.5** (one version for both roles — 1.2.5 collapses the
two `#ifdef`-guarded version definitions into a single one, so a role can no
longer be left behind).

### Added

- **`ForcePositionConfig::close_preload_nm`, default 0.25 N·m.** Feed-forward
  torque applied only while holding the *closed endpoint*, to seat the jaw
  against its mechanical stop and take up the gear train's backlash.

  The jaw used to arrive at the closed end and stop pressing, because
  `position_hold_` commands `kp*(target − actual)` and that goes to zero exactly
  at the target: measured holding torque at the closed position was **0.001 N·m**
  — parked, not clamped, with the backlash unseated and visible as a gap.

  A position bias cannot fix it. The firmware clamps every target to the
  calibrated range, and that range's closed end is hard-coded `0.0f` in
  `can_motor_get_gripper_target_range()`, so a target biased past the stop is
  simply truncated — measured: sweeping the commanded target from 0 to +40 mrad
  past the stop moved the jaw not at all and left the torque flat at 0.061 N·m.
  A feed-forward term has no such clamp.

  **Why 0.25.** Pure feed-forward (`kp=kd=0`) swept against the closed stop on
  a follower unit:

  | feed-forward | resulting raw |
  |---|---|
  | 0.05 N·m | −0.00249 |
  | 0.10 N·m | −0.00096 |
  | 0.15 N·m | **+0.00000** |
  | 0.20 / 0.30 / 0.40 / 0.50 N·m | +0.00000 (unmoved) |

  raw 0 is a *hard stop*, not a compliant region: 0.15 N·m seats the jaw fully
  and 3.3× more torque moves it not one microradian. So the useful range ends at
  ~0.15 N·m, and 0.25 is that with 1.7× headroom for friction growth and
  mechanical drift. **Raising it further buys heat and gear load, never a tighter
  close** — worth stating because "more torque = tighter" is the obvious guess
  and it is wrong here.

  Bounded and thermally governed: the preload is *reserved out of*
  `grasp_torque_nm` rather than added on top, so the invariant that the total
  request never exceeds the budget still holds; and it passes through the
  firmware's `can_motor_envelope_clamp_torque()`, which is applied
  unconditionally alongside the target clamp. At 0.25 N·m it is 23% of the
  envelope's measured `cont_torque_nm` (1.1 N·m), so the I²t accumulator stays
  pinned at 0. Measured over a five-minute continuous hold: position did not
  drift, torque decayed 1.2%, temperature rose **+1.0 °C** and was flat within
  the sensor's 1 °C resolution after 90 s.

  Self-compensating, which a fixed position inset is not: however far the stop
  drifts with wear, the force presses until the jaw is seated again.

  Its sign is derived from the position map, not a constant — `to_rad()`
  multiplies by the direction, so a rising normalized position is a *falling*
  raw angle on a reversed unit. Set it to 0 to restore the pure spring hold.

### Changed

- **BREAKING — `FollowerGripper` now refuses firmware below 1.2.5** (was 1.1.6).
  It throws on open; `Config::allow_outdated_firmware` still opens the device
  so it can be inspected before upgrading, and OTA is unaffected because
  `ota_update.py` goes through `LeaderGripper`, which is not gated.

  Raising it is about the host and the firmware disagreeing, not about the
  firmware alone. Auto-calibration used to write the closed zero with an inset
  — 20 mrad through 1.2.3, 5 mrad in 1.2.4 — so normalized 0.0 sat short of the
  mechanical stop. This release's closed-endpoint preload is built on 1.2.5's
  semantics, that normalized 0.0 *is* the stop, and on older firmware it
  presses the jaw past what the position map calls fully closed. The force
  stays bounded (the firmware clamps the target to the calibrated range and the
  torque to the envelope's `cont`), so this is not the brown-out class of
  problem the 1.1.6 floor guards against. What breaks is the meaning of the
  scale at the closed end: a position that reads 0.0 while the jaw is 20 mrad
  tighter than 0.0 is supposed to be. Refusing beats warning there — a scale
  that quietly means something else is worse to debug than a device that will
  not open.

  **The leader is deliberately still not gated.** `ota_update.py` opens every
  gripper, followers included, through `LeaderGripper`; a floor there would
  block the upgrade path for exactly the devices that need it.

- **BREAKING — the shipped firmware images are now named with their version.**

  ```
  firmware/tc-gu-01-master.bin  ->  firmware/tc-gu-01-master-1.2.5.bin
  firmware/tc-gu-01-slave.bin   ->  firmware/tc-gu-01-slave-1.2.5.bin
  ```

  A `.bin` copied out of `firmware/` used to be unidentifiable: the version
  lived only inside the binary and in `manifest.json`, so a file sitting in a
  download folder told you nothing about what it was.

  **If you pass the image by name, update it** — a script that says
  `ota_update.py tc-gu-01-master.bin` will now fail. It fails loudly, listing
  what is actually in `firmware/`, not silently:

  ```
  [ERROR] firmware file not found: tc-gu-01-master.bin
          shipped images live in .../firmware
          shipped images: tc-gu-01-master-1.2.5.bin, tc-gu-01-slave-1.2.5.bin
          release filenames carry the version, so a name from an older README
          will not resolve — prefer the role selectors (master / slave / --all),
          which read the current name from manifest.json.
  ```

  **Prefer the role selectors**, which are unaffected by this and by every
  future bump, because they resolve the current filename from the manifest:

  ```bash
  python python/examples/ota_update.py --all      # both roles, matched by SN suffix
  python python/examples/ota_update.py slave      # one role
  ```

  Consequence worth knowing if you maintain this repo: `manifest.json` is now
  load-bearing rather than descriptive. Its `images.<role>.file` entry is how
  role selectors and `--all` find an image, so a version bump must rename the
  `.bin` *and* update the manifest in the same change. Getting that wrong
  breaks exactly the path the docs tell you to use while explicitly-named
  flashing still works — so it would survive casual testing.
  `test_ota_update_all.py` now fails if the manifest names a file that is not
  on disk, or if a filename disagrees with its declared version.

  Build artifacts keep the Makefile's unversioned names
  (`build/master/tc-gu-01-master.bin`); only released images are versioned. A
  build artifact is whatever you just compiled, a release is a specific version
  someone may still be holding months later.

- **Paired firmware is 1.2.5.** Follower closed zero (`min_open_rad`, written by
  auto-calibration) goes 20 mrad → 5 mrad → **0**: normalized 0.0 is now the
  closed hard stop itself. The 20 mrad inset had been measured under the
  pre-`5b8b3bf` kd-form control law and was never revisited when the law
  changed; the 5 mrad step assumed a residual the control could not close, which
  the feed-forward sweep above disproved. Leader code is untouched by this (its
  binary differs from 1.2.3 by 2 bytes, the version byte) and is bumped only to
  keep the roles on one number.

  Closed position on one follower, six cycles each, `arrived` true and
  `HOLDING_POSITION` throughout:

  | | closed raw | holding torque |
  |---|---|---|
  | 1.2.3 | −0.02014 | 0.001 N·m |
  | 1.2.4 | −0.00403 | 0.001 N·m |
  | **1.2.5 + preload** | **−0.00049** | **0.298 N·m** |

  19.65 mrad tighter than 1.2.3, repeatability 0.383 mrad.

## [0.2.0] - 2026-09-22

Paired with firmware **1.2.3** (both roles — 1.2.3 is where the leader and
follower version lines merge) and command set **V2.3**. Both were
developed and hardware-validated together; the SDK's protocol mirror is checked
against the firmware headers by `scripts/check_protocol_drift.py`.

### Changed

- **A failed command now answers with `cmd == 0`.** Success keeps the original
  command code; failure comes back as a pure ACK carrying a one-byte error
  code. Previously a failure also carried the command code with a one-byte
  error payload, which is byte-for-byte identical to a success returning one
  byte of data — so the failure of every no-data command was invisible and the
  SDK had to treat it as success. Requires firmware >= 1.2.3 on whichever role
  you are talking to; older firmware still answers the old, ambiguous way.

- **The motor-status DATA stream is 59 bytes, not 31.** Firmware 1.1.6 started
  streaming `MOTOR_STATUS_V2_SIZE` so that diagnostics arrive with the stream
  instead of forcing a `GetMotorStatusExt` poll — polling during control
  collides with the control frames. The `GetMotorStatus` (0x50) ACK is still
  31 bytes, so payload length remains useless as a version probe; use
  `GetVersion`.


- **BREAKING: `ForcePositionController` lost its contact state machine.** It is
  now ONE control law: the jaw is commanded toward the target with the PD
  request error-clamped against `grasp_torque_nm`, for the whole move. Free
  travel costs only friction (~0.1 Nm measured), an obstruction saturates the
  clamp at exactly the grasp torque and holds there. Contact is not detected --
  saturation is what contact IS.

  Deleted with it: the velocity/torque stall test, the N-frame confirmation
  window, the startup guard, the arrival band, the effort test that separated
  "arrived" from "blocked", the brake phase, and the `kp=0` velocity-damped
  travel command. `ForcePositionConfig` keeps its six fields; the tuning struct
  loses `contact_torque_nm`, `contact_vel_radps`, `contact_vel_ratio`,
  `contact_moved_rad`, `stall_hold_ms`, `startup_guard_ms`, `arrival_band_rad`
  and `brake_distance_rad`, and gains `travel_damping_fraction` and
  `arrival_eps_rad`. `contact_samples_for()` is gone.

  **Why.** The MCU already runs that stall test at 500 Hz
  (`task_canmotor_is_stalled()`), and `docs/CONTROL_LAYERING.md` section 3
  already assigns contact detection to the firmware -- the host was keeping a
  second copy of one physical event. Worse, the state machine forced the travel
  command to carry `kp=0` so the stall signature stayed clean, which left a
  0.7 Nm/(rad/s) proportional velocity loop. Measured over 10 runs at full
  stream rate: 37% relative velocity ripple closing, a 12 Hz limit cycle, and a
  mean speed 77% of what was asked for. And its central judgement -- arrived
  versus blocked -- is a threshold on distance-to-target, which on empty jaws is
  physically continuous. Full derivation in `docs/CONTROL_REFACTOR.md`.

  **Measured after** (same rig, same script): ripple 19.5% closing / 18%
  opening, mean speed 98% / 104% of commanded, arrival error 0.0013-0.0060 rad
  against 0.0087 before, no false contact in 6 runs.

- **BREAKING: `ForcePositionSnapshot.contact_count` is replaced by `holding`
  and `arrived`.** They are observations derived per frame, not states the
  controller switches on: `arrived` is `|position error| <= arrival_eps_rad`,
  `holding` is the setpoint having run as far ahead of the jaw as the force
  budget allows while the jaw moves at no more than 25% of the commanded speed.
  That velocity share is the firmware's own `TASK_CANMOTOR_STALL_VEL_RATIO`,
  deliberately the same number so host and MCU describe one event the same way.

  It deliberately is NOT "the command reached its budget", which is true the
  instant a target changes -- the setpoint jumps, the command saturates, and the
  jaw has not moved because it has not had time to. `force_position_control.py`
  caught that: every blocked step "settled" into a grasp in 0.01 s carrying
  0.016 Nm, gripping nothing, and the verification passed without ever moving
  the jaw. The ramp's lead is self-timing instead -- it starts at zero because
  the ramp is seeded on the jaw, and only fills if the jaw refuses to follow. `ForcePositionState` still
  exists and is still reported, but it is derived from these two plus the
  direction of travel; only `Fault` latches.

- **BREAKING: `ForcePositionConfig::grasp_torque_nm` default is 1.1 Nm, up from
  0.35.** 1.1 Nm is the EL05's continuous stall rating and the grip force this
  gripper is specified to deliver. The old 0.35 had been copied from the
  firmware's `GRIPPER_AUTO_CAL_DEFAULT_CLOSE_TORQUE` -- the push
  auto-calibration uses to find the mechanical stop during homing -- and was
  never justified as a grip force. **Callers who relied on the default now grip
  about three times harder.** Pass `grasp_torque_nm` explicitly if that matters.

  Sustained holds on a real workpiece, 24 V, envelope enforced, winding
  temperature from the status stream:

  | grasp | 600 s | rate |
  |---|---|---|
  | 1.1 Nm | 36 -> 70 C | 8 -> 4 -> 3 -> 2 -> 1 C/min |
  | 0.6 Nm | 41 -> 49 C | flat by the end |

  At 1.1 Nm the rate decays the whole way, which is a first-order approach
  rather than a linear climb; fitting it puts the plateau near 75 C, about 15 C
  under the firmware's 90 C temperature wall. No fault, no undervoltage latch,
  no link loss, no drift in the grasp. The margin is real but not generous --
  both runs started from 36-41 C rather than cold, and a warm ambient eats into
  it directly, so a hold measured in tens of minutes is worth confirming on the
  unit and ambient it will run in.

- **`grasp_torque_nm` is not a hard ceiling.** Measured feedback runs 5-7% above
  the budget at a settled hold (0.643 Nm against 0.600, 1.154 against 1.100).
  The budget shapes the command; the firmware's motion envelope is what bounds
  the output.

- **`ForcePositionConfig` no longer couples `close_speed_radps` to
  `grasp_torque_nm`.** The rule that rejected `close_speed < grasp/5` existed
  because the travel damping gain WAS `grasp/close_speed` and would saturate; a
  time-based ramp regulates speed now, so a slow close is just slow, not soft.

- **`ForcePositionController::start()` now cross-checks the grasp against the
  device's continuous envelope.** `HoldingForce` is indefinite by construction,
  so `grasp_torque_nm` is a *continuous* rating request and `cont_torque_nm` is
  the device's own answer for what it can hold indefinitely. Exceeding it is not
  mainly a thermal problem — the motor's undervoltage protection is prompt, and
  a sustained draw can brown out the 24 V rail and take the USB link with it,
  which reaches the host as `SerialBus::write: Input/output error` rather than
  as anything resembling a torque fault. It now warns when the grasp is over the
  continuous envelope or within 10% of it, and separately when the envelope is
  not enforced at all (the firmware's I²t derate and temperature wall are then
  inactive and an indefinite hold has no protection below the host). Advisory
  only, and skipped silently on firmware that cannot answer.

- **`ControlLoop::Config::rated_torque_nm` default lowered 2.0 → 1.8 Nm**, the
  EduLite05's rated torque and the same ceiling `ForcePositionController` uses
  for its indefinite hold. Nothing bounds how *long* the ceiling holds, so the
  old default applied a peak-torque allowance continuously.

- **`ControlLoop`'s Python constructor now takes `status_timeout_ms`**, and all
  of its kwarg defaults are read from `ControlLoop::Config{}` instead of being
  written out a second time in the binding. The hand-copied list had already
  drifted: the C++ `rated_torque_nm` default moved to 1.8 Nm and Python went on
  receiving 2.0.

- **`gripper_console.py --mode impedance` now drives `ImpedanceController`**
  and shows `state` and any fault reason on its detail line, the way the
  force-position mode already did. `--kp` / `--kd` / `--max-position-torque` /
  `--rated-torque` are unchanged; `--hz` is now only the UI refresh rate, since
  both controllers lock their submits to the status stream.

- **`impedance_control.py` drives `ImpedanceController`** and is now a
  verification run too: it settles on velocity rather than a fixed 2 s sleep,
  and checks the same invariant as the force-position script — a guard
  (`STALLED` / `TORQUE_CAPPED`) may only be reported when the jaw is measurably
  short of the commanded position.

- **`python/examples/force_position_control.py` is now a verification run**, not
  a print. Its target sequence carries intermediate positions
  (`1.0,0.5,0.0,0.5,1.0`, overridable with `--targets`) because an endpoints-only
  sweep cannot see the arrival/contact confusion above at all, and it checks the
  invariant that separates them — **a force hold requires the jaw to be
  measurably short of the commanded position** — exiting non-zero on any step
  that violates it.

  Both verification scripts wait for the controller to actually pick the target
  up (`snapshot().target_position`) before judging the outcome. Without that
  they raced the queued command, read the *previous* target's terminal state,
  and passed every step in 0.00 s with the jaw never moving.

- **BREAKING: the raw motor primitives are no longer exposed to Python.**
  Removed from the bindings: `Motor.set_impedance` / `set_position` /
  `set_velocity` / `set_torque` and `FollowerGripper.set_position` (a
  normalized wrapper that was itself just `motor_.submit_impedance()`).

  **The no-ACK `submit_*` forms are exposed again** -- see the separate entry
  below. The incident that motivated removing everything happened on firmware
  1.1.5, before the motion envelope existed.

  Every one of them writes a control frame straight to the wire with no error
  clamp, no torque ceiling and no stall guard — those live in `ControlLoop` and
  `ForcePositionController`, and a caller reaching past them gets none of it.
  This is not hypothetical: a console driving `submit_impedance()` at
  `kp=20` into a rigid object let `kp * error` grow to the motor's own `0x700B`
  ceiling; at 24 V the current draw browned out the whole board, the gripper
  dropped what it was holding and the USB link disappeared. See
  [`docs/CONTROL_LAYERING.md`](docs/CONTROL_LAYERING.md).

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

- **`Motor.get_spec()` — the device now tells you its own motor ratings**
  (`Cmd::GetMotorSpec`, firmware `f204ff3`). Returns the model name, the
  position/velocity/torque ranges, the kp/kd ceilings, rated and continuous-stall
  torque, and the winding/board temperature limits.

  `MOTOR_RATED_TORQUE_NM` and `MOTOR_PEAK_TORQUE_NM` are **demoted to fallback
  defaults** — they are EL05 numbers compiled into this SDK, and this repository
  has no way to know which actuator is plugged in. An RS00 is rated 5.0 Nm and
  peaks at 14.0 against EL05's 1.8 / 6.0, so with the ratings hardcoded
  `validate_config()` would cheerfully enforce the wrong motor's limits. The
  firmware's `motor_spec.c` is now the single source and the constants only
  initialise the config structs.

  `ForcePositionController::start()` cross-checks the configured limits against
  what the device reports and warns on anything above the installed motor's
  ratings. Advisory, not fatal: a config that is too *conservative* is safe, just
  weaker than it could be, and refusing to start would strand a caller whose
  numbers are merely stale. Skipped silently on firmware without 0x56.

  A zero field means the firmware's table does not have that number yet — most
  of RS00..RS06's thermal fields are still zero. Treat 0 as unknown, not as zero.

- **`MotorStatusSample` carries the follower's diagnostics now** —
  `stop_reason`, `fault_code`, `latched_fault_code`, `monitor_reserved` and
  `monitor_version`, exposed to Python as well. They arrive on the DATA stream
  because the follower started emitting the 59-byte V2 status instead of the
  31-byte legacy prefix (firmware `f5264ef`).

  They were previously reachable only by polling `Cmd::GetMotorStatusExt`, and
  polling collides with the phase-locked control frames — `docs/CONTROL_LAYERING.md`
  section 1 says so and this refactor hit it anyway: a 0.25 s poll loop during a
  432 s hold eventually lost the race, produced a `TimeoutError`, and the failure
  was briefly misread as the documented 24 V brownout. Diagnosing a fault no
  longer requires stopping control, which matters because faults happen while
  controlling.

  On firmware that still streams the 31-byte prefix the new fields decode as
  zero — the decoder zero-fills the tail — so check `monitor_version` rather than
  assuming a field is meaningful.

- **`Motor.submit_impedance` / `submit_position` / `submit_velocity` /
  `submit_torque` are exposed to Python again.** They are no-ACK direct
  commands: the frame goes on the wire and returns, with none of the host-side
  error clamp, torque ceiling or stall handling that lives in the controllers.

  They were withheld after a console drove `submit_impedance()` at
  kp=20 into a rigid object, walked the torque up to the motor's own 0x700B
  ceiling, and browned out the whole board at 24 V -- the jaw dropped its
  workpiece and the USB link disappeared. **That was firmware 1.1.5, before the
  motion envelope.** The envelope sits on the firmware's MIT branch, the single
  path every impedance/position/torque command must pass, so a raw command
  cannot get around it; on the same rig and object, flipping only the envelope
  flag turned "board reboots" into "25 s stable hold at a constant 0.549 Nm"
  (`docs/CONTROL_LAYERING.md` section 1). Exposing the command layer is also
  what section 3's split already prescribed: the SDK states intent, the firmware
  enforces the envelope at 500 Hz.

  Two limits survive and are documented on the bindings: there is **no
  low-level thermal protection** (the motor's own over-temperature protection
  did not act at 100 C case temperature; the firmware temperature wall is part
  of the envelope), and the host watchdog still runs -- 300 ms to the T1
  zero-speed hold, 30 s to T2 disable.

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

- **`Cmd::GetHomeDiag` (0x57) and `HomeDiagReport`** — the follower's
  auto-calibration state machine, exposed. Homing has five failure paths and
  before this command none of them were visible to a host: they only reach
  `LOG_E` on UART7, which is not wired to USB on this board. All a host could
  see was `StopReason::Emergency` after the fact, with no indication of which
  step failed.

  The report carries the current phase, how long it has sat there, the
  commanded velocity and current limit, the stall threshold, and the torque,
  velocity, peak velocity and travel actually observed. That turns three
  distinct faults into three distinct readings: no travel and no torque means
  the speed frames are not being executed; no travel with torque at the
  threshold means the stall test is not satisfied; travel that stops short
  means resistance exceeds the current limit.

  It is a read-only snapshot and deliberately bypasses the motor-admin gate —
  a running control loop is exactly when you most want to read it.

- **`Cmd::GetMotorSpec` (0x56) reached `to_string`.** The enumerator was added
  earlier but its `to_string` branch was missed, so it logged as an unknown
  command.

- **`python/tests/conftest.py`** — `pytest python/tests` now tests *this*
  checkout. The `taccap` and `lerobot-xense` envs each carry an editable
  install whose `ScikitBuildRedirectingFinder` sits on `sys.meta_path`, which
  runs before `sys.path` and therefore beats both `PYTHONPATH` and a
  `sys.path.insert`. The symptom was not an import error but 38 assertion
  failures claiming `ForcePositionConfig` still had `brake_distance_rad` —
  fields this release deletes — which reads exactly like a regression here and
  was a different repository answering. The conftest also asserts that
  `_taccap_native` came from this checkout, since `xense.taccap` is a namespace
  package and `__file__` pointing here does not prove the extension did.


- **`MotorStopReason::HostTimeout` (0x06)** — mirrors the firmware's new
  `MOTOR_STOP_REASON_HOST_TIMEOUT`, set when the slave control task finds its
  cached target stale because the host stopped sending.

- **`scripts/check_protocol_drift.py` now covers the stop-reason table**, and
  its macro parser accepts integer suffixes. Both gaps were found the hard way:
  a firmware change added `MOTOR_STOP_REASON_HOST_TIMEOUT` and the script still
  printed "OK", because that table was never wired up and because
  `MOTOR_STOP_REASON_*` is written `0x00U` — the `\b` after the digits never
  matches when a suffix letter follows, so every one of those macros parsed as
  nothing. That is precisely the "the firmware added something and we never
  noticed" case the script exists to catch. Verified by deleting the SDK
  enumerator and confirming the check fails.

- **`ImpedanceController`** — the supervised sibling of
  `ForcePositionController`, same shape: a pure `detail::ImpedancePolicy` state
  machine that tests drive without hardware, wrapped in a transport owner that
  submits on the status-frame doorbell. Use it when the task is "follow a
  position" (teleoperation, a leader-follower relay).

  `ControlLoop` implements the same control law but keeps it inline in its
  status callback, reports `stalled` and `torque_capped` as independent flags
  read through independent locks — so a caller polling both can observe a
  combination that never existed — and has no notion of being faulted at all: a
  failed submit breaks its loop and `running` goes false with nothing saying
  why. Here the guards are ordered states (`FAULT` > `TORQUE_CAPPED` >
  `STALLED` > `TRACKING`), every field comes from one `snapshot()` under one
  lock, and a fault carries a reason and clears through `reset()`. The stall
  flag tracks the clamp rather than the instantaneous test, so it cannot blink
  once and go quiet while the gripper is still clamped.

  **`ControlLoop` is unchanged and is not deprecated.** It keeps
  `SubmitPhase::FreeRunning`, runtime gain changes and the smallest possible
  wrapper around `submit_impedance()`; `ImpedanceController` gives up the
  free-running phase (measured to cost status frames) in exchange for
  supervision. `ImpedanceConfig` is seven fields — the gains and the error
  clamp are what a task tunes, `rated_torque_nm` is the motor's own rating, two
  describe the transport — with the measured stall/ceiling constants in
  `detail::ImpedanceTuning`, unreachable from Python. `rated_torque_nm` is
  capped at the RATED torque rather than the peak because the hold it produces
  is indefinite. Covered by `test_impedance_controller.cpp` (15 hardware-free
  policy cases) and `test_impedance_pty.cpp` (7).

- **`MOTOR_RATED_TORQUE_NM` / `MOTOR_PEAK_TORQUE_NM`** in
  `components/motor.hpp`, exported to Python. The two nameplate ratings now
  have one home; `FORCE_POSITION_MAX_{HOLD,MOTION}_TORQUE_NM` remain as aliases.

- **`python/tests/test_control_config_surface.py`** — the Python-facing config
  surface of both controllers had no tests, and the surface *is* the contract:
  downstream consumers address the controllers through these attribute names
  and constructor kwargs. pybind11 restates every default
  by hand on the far side of the boundary, so a C++ default can change and
  Python keep the old one with nothing failing — which is exactly what happened
  to `rated_torque_nm` (see below). Hardware-free.

- **`cpp/tests/test_control_loop_pty.cpp`** — `ControlLoop` previously had no
  behavioural tests at all, despite being the layer that exists to stop a
  measured 6 Nm runaway. Six pty-driven cases cover the error clamp, the stall
  clamp and its release, the torque ceiling entering and releasing, and the new
  stream-liveness safe state. The fake follower firmware moved to
  `cpp/tests/fake_follower.hpp` so both controller suites drive the same device.

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

- **Fixed in firmware: a dropped link mid-grasp left the motor pushing
  forever.** Previously documented here as a hazard; now fixed in the gripper
  firmware and verified on hardware.

  Two layers, and the distinction is the whole point. The actuator's own
  `0x7028` CAN timeout (manual §3.3.6: no CAN command within the window → the
  motor enters Reset mode) covers **MCU/RTOS death** — it is now written and
  independently read back as an enable prerequisite, having been a bare
  `#define` never written since 1.1.5. It provably **cannot** cover host loss:
  the slave control task re-sends its cached target every 2 ms, so the motor
  goes on receiving CAN commands and the timeout never fires. Measured: 0x7028
  written, ACKed, read back as 4000, saved — and killing the host mid-grasp
  still left the motor at 0.854 Nm, twice.

  Host loss is covered by a two-stage watchdog in the slave control task:
  300 ms → zero-speed hold at 0.35 Nm (keeps the workpiece, kills the runaway),
  30 s → disable. Verified on hardware with a real grasp: torque fell
  **0.851 → 0.157 Nm within 1 s** with the object still held, then the motor
  disabled on its own. Before the fix the same test held 0.854 Nm indefinitely
  and heated 33 → 45 °C.

- **Documented: the hazard, as it stood before the fix.**
  Not a code change — a hazard found while bench-testing and written down where
  the submit path is defined (`components/motor.hpp`). The firmware has no
  host-command watchdog (no last-command timestamp or staleness check anywhere
  anywhere in the firmware's task layer, and its hardware watchdog is
  disabled), so it keeps applying the last submitted frame indefinitely.

  Measured on 1.1.6: a 1.2 Nm grasp, USB link dropped 4 s into the close, and
  the motor was still pushing **1.256 Nm two minutes later** with its
  temperature risen 33 → 45 °C. Both host-side safe states failed with
  `Input/output error` — the controllers' zero-torque frame and
  `Motor::disable()` alike — because the device they needed to write to was
  gone. This is the case the safe states exist for and the one case they cannot
  cover; only a manual reconnect and disable recovered it. The remaining
  backstops are the envelope's I²t derate and temperature wall, which are slow
  and only apply when the envelope is ENFORCED. Fixing it needs a
  control-frame timeout in the firmware.

- **Entering `Fault` left no trace in the session log.** `fail()` set the reason
  and returned, so a run that died because its device disappeared was
  indistinguishable in `~/.taccaplogs/` from one that exited cleanly — which is
  precisely the case where the log is the only evidence left. Both controllers
  now log the transition into `Fault` at error level with the reason, and the
  transition out of it on `reset()`. Edge-triggered, because `step()` re-calls
  `fail()` on every frame while the condition persists and an unconditional log
  would write 100 lines a second.

- **`stop()` verified not to hang when the device disappears mid-run**
  (`StopReturnsAfterTheDeviceDisappears`, both controllers). Reported from the
  field as the controller hanging; measured on the same bench, the USB tree
  dropped and re-enumerated 15 times in 45 minutes, most of them while nothing
  was running, so the controller kept losing its device underneath it. `stop()`
  returns in ~25 ms with the link dead. Note that ACK-bearing calls around it
  (`motor.disable()`, `clear_fault()`) still cost up to
  `ack_timeout_ms * (max_retries + 1)` = 3 s each against a dead device, which
  reads as an unresponsive console.

- **`ImpedanceController`'s stall guard could not fire under its own error
  clamp.** Inherited from `ControlLoop`, it tripped on an absolute 1.2 Nm of
  *feedback* torque — but the command is bounded by `max_position_torque_nm`
  and the feedback runs under the command, so that trip is unreachable.
  Measured on 1.1.6 hardware with a genuine 5-second stall at
  `max_position_torque_nm = 0.35`: the command saturated at 0.362 Nm, the
  feedback peaked at **0.271 Nm** — 4.4x below the trip — and the controller
  reported `TRACKING` for the entire time with the jaw hard against the
  mechanism. At the 1.5 Nm default clamp the measured command-to-feedback ratio
  (0.18–0.21 free, 0.75 stalled) still leaves it on the wrong side of 1.2 Nm.

  This is the same structural error `ForcePositionController` documents at
  length — a threshold placed near the commanded cap can never be reached — and
  it mattered rather than being cosmetic: with the guard silent, a blocked jaw
  sits at the full clamp torque indefinitely, and sustained torque is what
  browns out the 24 V rail and drops the USB link.

  The trip is now **the error clamp binding while the jaw is not moving**, which
  is reachable by construction: it means the controller is asking for every bit
  of torque it is allowed to ask for. `stall_torque_nm` becomes
  `stall_torque_floor_nm` (0.080 Nm), a floor whose only job is to reject
  "commanded but the motor is not actually pushing". Verified on the same
  bench: the identical run went from 0 trips / 100% `TRACKING` to 3 trips with
  `STALLED` reported, while normal motion at the default clamp still completes
  with 0 trips.

- **A dead status stream left `snapshot().observation.valid` true whenever the
  fault arrived by any route other than staleness.** Reported from the field: a
  console showing a plausible position and 1.540 Nm of torque beside
  `age=10553.7ms` and `fault: submit failed: SerialBus::write: Input/output
  error` — ten seconds of frozen numbers presented as a live reading.

  The invalidation was gated together with the `fail()` on
  `stale && state != Fault`, so it only ran when staleness was what *caused* the
  fault — and that is never the case when the link actually drops. A dead USB
  link makes the next submit throw, which faults the policy within one status
  period, well inside `status_timeout_ms`; by the time the stream was judged
  stale the policy was already in `Fault` and the branch was skipped. The one
  case the flag exists for was the one case it missed. Invalidation is now
  independent of why the policy is faulted, and the *first* fault reason is kept
  rather than being overwritten by "stream stale" — the submit error is the
  diagnosis, staleness is only what happened next.

- **`ControlLoop` left a live-looking observation behind when the link died.**
  Same shape: a failed submit breaks straight out of the phase loop, so
  `guard_stale_()` never runs again and nothing else clears `obs_`. A caller
  polling `observation()` kept reading the last frame as live, with only
  `age_ms` rising to give it away. The loop now invalidates on exit.

- **A streamed target made contact detection unreachable.** A caller relaying a
  leader gripper at 100 Hz restarted the startup guard and the travel history on
  every target update, so the confirmation counter was zeroed on every tick and
  contact could never confirm. Fixed at the time by tying the guard to the
  motion rather than to the target value; moot now that the contact state
  machine is gone, but recorded because the failure mode -- a control decision
  reset by a stream of commands that were not meant to disturb it -- is easy to
  reintroduce.

- **`ControlLoop::stalled()` read false while the stall clamp was engaged.**
  Clamping the effective target drops the position error to ~0, so the torque
  falls back under `stall_torque_nm` on the next frame — that is the clamp
  working — and the flag was being recomputed from that instantaneous test. A
  caller polling it saw one blink and then false for the whole time the gripper
  was still clamped. It now tracks the clamp, which is what the header always
  documented.

- **`ControlLoop` had no safe state when the motor-status stream died.** Every
  guard in the class reasons from that stream, so losing it froze them rather
  than degrading them: the error clamp kept bounding against an `obs_.raw_pos`
  that no longer moved, and — the sharp edge — a torque ceiling that was
  *engaged* when the stream died could never release, because both of its
  release tests are evaluated against frames that had stopped arriving. The
  motor would sit at `rated_torque_nm` indefinitely. There is now a
  `status_timeout_ms` (350 ms): the loop drops any latched ceiling, invalidates
  the observation, puts one zero-torque frame on the wire, and stays off the wire
  until frames resume.

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

## [0.1.9] - 2026-08-21

追查"夹爪状态流偶发降频"，最后拆出四个性质不同的问题，这一版是其中
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
  follower, wrapping the two firmware commands added in 1.1.3/1.1.4.

  `uart_stats()` (Cmd 0x54) returns the firmware's free-running UART counters.
  It exists to answer a question nothing on the host can: when a status frame
  arrives a couple of bytes short, did the MCU fail to send them, or were they
  lost after leaving it? `tx_bytes_ok` / `tx_calls_ok` count only what the
  firmware's transmit call accepted, so comparing them against what the host
  decoded over the same window separates the two. That comparison is what
  established the byte loss is downstream of the MCU: firmware
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
    that lands while the MCU is transmitting costs a whole status frame.
    Rate does not predict the loss — 250 Hz lost 154 frames on one
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

  Measured on follower firmware 1.1.x, 100 Hz motor-status stream, 60 s per run, submits at
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
  the frame it is sending. Measured against follower firmware 1.1.x with a 100Hz
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
- **Command set V2.2 — follower motor diagnostics** (follower firmware
  1.1.2; the leader is unchanged at 1.2.1 because every
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

- **Shipped follower image bumped to 1.1.2**, the
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
  (leader firmware 1.2.1, follower 1.1.1). **No protocol change** —
  `protocol_cmd.h`,
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
  Verified against the firmware protocol headers
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

Sync to firmware protocol **V2.1** (leader firmware 1.2.0, follower 1.1.0).

Validated on two leader grippers flashed from source to 1.2.0 over OTA.

> **Downstream on pybind11 2.x should update.** The Python IMU vector fix
> below changes the data affected consumers read — `accel_mps2` /
> `gyro_radps` / `mag_uT` were returning the x component three times. It only
> bites builds made against **pybind11 2.9**, which here means the system
> system py3.10 wheel; conda py3.12 builds (pybind11 3.0.x)
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
  py3.10 wheel; the conda py3.12 builds use pybind11 3.0.x,
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

Sync to the firmware protocol headers.

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

Sync to firmware protocol **V1.9**.

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
  hardware" to first-class, validated against real firmware. The leader
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
- Python 3.10 + 3.12 build paths (system py3.10,
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
  between this SDK and downstream consumers (dataset recording,
  lerobot adapters).

## [0.0.1] - 2026-04-29

### Added
- Initial repository skeleton: CMake + scikit-build-core + pybind11
  with the `xense::taccap::` / `xense.taccap` namespace alias.
- libxensesdk vendored as a git submodule pinned at commit `7d4687e`,
  configured in lite mode (no ML backends).

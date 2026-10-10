# 固件参考与刷写

> 本文讲的是**怎么构建和刷写**，不记录版本号——那会变成又一处会过时的硬编码。
> 仓库里当前发布的镜像版本、大小、CRC32 和源提交，以
> [`firmware/manifest.json`](../firmware/manifest.json) 为准，人读的说明在
> [`firmware/README.md`](../firmware/README.md)。设备上实际跑的版本用
> `GetVersion` 查（`gripper.firmware_version`）——它返回的是编译进镜像的常量，
> 是唯一能证明刷进去了什么的东西。

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

## Firmware / PC GUI reference repos

The wire protocol this SDK speaks is defined by the firmware that runs
on the gripper's STM32H562 MCU. The protocol PDF + Python prototype
(in PyQt) live in two **internal** repos that we **read but don't
ship** — they have separate release cadences and build toolchains and
shouldn't be linked into this SDK's git history. Ask the firmware team
for access if you are working on the wire format.

If you have them, clone into `third_party/firmware/tc-gu-01` and
`third_party/firmware/tc-gu-01-pc`: both paths are in `.gitignore`, so
they sit next to the SDK for easy `grep` / IDE discovery but never
appear in `git status`.

You do **not** need either to flash a gripper — the released images ship
in [`firmware/`](../firmware/).

What's in them:

- `tc-gu-01/App/protocol/protocol_cmd.h` + `protocol_data.h` — canonical
  command enum + POD payload layouts. The SDK's
  `cpp/include/taccap/protocol/{commands.hpp,payloads.hpp}` mirror these
  1:1 with `static_assert(sizeof(...) == ...)` size checks. Currently
  mirrored through command `0x5B` (protocol doc V2.6: V2.2 plus the
  0x54 / 0x55 diagnostic pair, 0x56–0x58 motor spec / homing diagnostics /
  motor version, 0x59 / 0x5A motor model, 0x5B CAN extended-frame relay);
  `scripts/check_protocol_drift.py` is green against firmware `07ab9bb`.
- `tc-gu-01/App/protocol/PROTOCOL_SPEC.md` + `tc-gu-01/docs/PROTOCOL.md` —
  the human-readable spec, including the §10 offset table for the 72-byte
  extended motor status that `test_codec_v22.cpp` is transcribed from.
- `tc-gu-01/App/tasks/task_data_stream.c` + `task_imu.c` +
  `task_encoder.c` — explains why IMU/encoder unique-data rate caps at
  ~60 Hz even when you request 100 (see the SDK's stream-dup note in
  the Claude memory).
- `tc-gu-01-pc/core/protocol.py` + `core/serial_worker.py` — Python
  reference implementation of the same wire protocol; useful as a
  cross-check when debugging the C++ codec.

### Building the firmware (Ubuntu) and flashing it over OTA

The firmware builds with a plain Makefile — no CubeIDE needed. `GRIPPER` is
mandatory; it selects `-DENABLE_MASTER_GRIPPER` / `-DENABLE_SLAVE_GRIPPER`,
which is what splits the command table and the version constant.

```bash
sudo apt install gcc-arm-none-eabi

cd third_party/firmware/tc-gu-01
env -u CFLAGS -u CXXFLAGS -u CPPFLAGS -u LDFLAGS make GRIPPER=master -j"$(nproc)"
env -u CFLAGS -u CXXFLAGS -u CPPFLAGS -u LDFLAGS make GRIPPER=slave  -j"$(nproc)"
# -> build/master/tc-gu-01-master.bin   (leader — version is a compiled-in
#                                        constant, check it with GetVersion)
# -> build/slave/tc-gu-01-slave.bin     (follower — version is a compiled-in
#                                        constant, check it with GetVersion)
```

> **`env -u CFLAGS ...` is load-bearing.** The `taccap` conda env exports host
> x86 build flags (`-march=nocona -mtune=haswell -isystem <env>/include`), and
> the firmware Makefile uses `CFLAGS +=`, so they get appended to the ARM
> cross-compile and it fails with `unrecognized -march target: nocona`.
> `conda deactivate` works too.

Then flash over the wire — no SWD probe. The plain `.bin` is the OTA artifact:
the image always links at `0x08000000`, and the firmware writes it to the
inactive bank and uses the STM32H5 bank swap, so one build serves both banks.

```bash
python python/examples/ota_update.py \
    third_party/firmware/tc-gu-01/build/master/tc-gu-01-master.bin \
    left --target-version 1.2.6
```

**The two roles have independent version numbers** — at the time of writing the
leader is 1.2.6 and the follower 1.2.16, and neither is "behind" the other. Both
carry the same control-UART receive fix, which lives in shared code.
The roles were briefly forced onto one number; that was dropped on 2026-09-24.

**"Leader 1.2.6" means two different images.** A leader flashed during that
aligned-number period can report 1.2.6 while running the same code as 1.2.4 --
*without* the control-UART receive fix. The leader line then reached 1.2.6 again
on its own (2026-09-28), this time *with* the fix. The version number cannot tell
them apart. If a leader reports 1.2.6, check `g.diagnostics.uart_stats()`: the
fixed image answers with 44 bytes, so `rx_errors` / `rx_rearms` exist and are
meaningful; the old one answers 32 bytes and the SDK reads both as 0 -- which is
indistinguishable from a healthy zero. When in doubt, reflash
`tc-gu-01-master-1.2.6.bin` from `firmware/gripper/`; OTA compares no versions
(`--target-version` is informational), so reflashing the same number is fine.
Compare versions only within one role.

Note the build output keeps the Makefile's unversioned name
(`build/master/tc-gu-01-master.bin`), while the images released under
`firmware/gripper/` carry the version (`tc-gu-01-master-1.2.6.bin`). That is deliberate:
a build artifact is whatever you just compiled, a release is a specific version
someone may still be holding a copy of months later. **If you promote a local
build into `firmware/gripper/`, rename it and update `firmware/manifest.json` in the
same change** — the manifest's `file` entry is what the role selectors and
`--all` resolve, so the two drifting apart breaks exactly the path customers
are told to use. `test_ota_update_all.py` fails if they disagree.

> **刷完必须断电重启,这是升级流程的一部分,不是排障手段。**
>
> 做法:**从爪拔掉 24V 电源线,等约 2 秒再插回**,USB 线可以不拔。从爪的 MCU 和电机
> 都靠 24V 运行,断 24V 两者一起重启;反过来只拔 USB,MCU 靠 24V 照常运行,什么都没
> 复位。在一台 EL05 从爪上实测:拔 24V 的瞬间(<0.3 秒)板载 USB 集线器连同串口芯片
> 一起掉线、MCU 重启,从最快的一碰到断开 10 秒,15/15 次都如此;OTA 后只断一次 24V,
> 60 秒状态流 6000 帧一帧不丢。**主爪**没有 24V:拔插它的 USB 即可。
> 确认真的断过电:看 `gripper.device.heartbeat().uptime_ms` 是否从接近 0 重新计起。
> `/dev/serial/by-id/` 的时间戳通常也会跟着变,但 uptime 才是 MCU 真的重启过的直接证据。
>
> bank-swap 重启是软复位：MCU 重新初始化，但片外的 USB 转串口桥从没断过电。设备
> 回来之后停在一个降级状态，而这个状态和健康状态**从任何可观测的角度都分不出来**
> —— 版本号正确、数据流在跑、`uart_stats()` 的收发计数全是干净的。唯一的症状是它
> 在悄悄丢状态帧。
>
> 实测，同一只夹爪、同一个固件、同一条线、60 秒一轮：仅 OTA 之后每轮丢 35~39 帧，
> 断电重插之后连续三轮为 0。
>
> 在这上面栽过两次：先把它误判成"某只夹爪链路劣化、疑似硬件损坏"，又用两组都没
> 断电的数据去比较固件版本，得出了并不存在的版本差异。**任何在断电之前取的数字
> 都不可信。**

> Only builds you made yourself need that path. To flash the **released**
> images, name them and let the script find them in [`firmware/`](../firmware/) —
> that resolves from any working directory, including a parent repo that
> vendors this one as a submodule:
>
> ```bash
> python python/examples/ota_update.py master left
> ```

Notes:

- **`make` succeeding does not mean it will flash.** The linker script declares
  the full 2048K, but OTA caps a single bank at 456 KB — check
  `ls -l build/*/tc-gu-01-*.bin` (builds at `07ab9bb` with
  `arm-none-eabi-gcc 13.2.1`: master 118,220 B, slave 165,056 B, i.e. 25% / 35%
  of the cap). Sizes vary by several hundred bytes across toolchains and by a few
  hundred between firmware revisions, so treat these as approximate — the check
  that matters is that the `.bin` fits under 456 KB.
- Flash the artifact matching the *role*, not the side. A gripper's role is the
  `m` / `s` suffix on its firmware SN; both leaders take the `master` build.
- `make download` is Windows-only (`STM32_Programmer_CLI.exe`, and it flashes
  the `.elf`). On Ubuntu use the OTA path above.
- `Cmd::GetVersion` returns the **compiled-in** constant, not the OTA bank
  metadata, so `--target-version` is bookkeeping only — the version you read
  back afterwards is proof of what actually got flashed.

### Motor firmware (RobStride) over USB-C

For a guided **motor-only** update to an explicit version, use:

```bash
python python/examples/motor_firmware_update.py firmware/motor/EL05_1.0.5.0.4.bin --target-version 1.0.5.0.4
```

Pass a firmware SN after the image when multiple devices are connected. The
script requires a recorded matching model and follower >= 1.2.14. It guides
PRIVATE switching and manual 24 V cycles, flashes only the motor, requires a
live version match (not a flash cache), records the verified version, then
restores MIT with another cycle. It does not change the MCU image, motor model,
direction or torque parameters. Restoring MIT may trigger power-on homing;
keep the travel clear and inspect the physical endpoints before use.
`--dry-run` validates the image and device without switching protocols or
querying a live motor version. Downgrades require `--allow-downgrade` plus an
interactive confirmation; EL05 below 1.0.5.0.4 is for comparison tests only
because of known velocity-feedback issues. Same-version reflashes also ask.
On failure it stops without automatic transfer retries or motion restoration;
the device may remain in PRIVATE. A filename/embedded string check is not a
cryptographic authenticity check: use only trusted vendor images.

The motor module runs its own firmware, separate from the gripper's. It can be
flashed through the follower's USB-C port — no RobStride USB-CAN adapter, no
removing the motor — with `python/examples/motor_ota_update.py`, built on
`MotorOtaSession` (`cpp/include/taccap/motor_ota.hpp`). Every CAN frame is
relayed by the follower MCU through `Motor.can_ext_xfer` (`0x5B`).

For a follower that needs **both** images, use the combined state-machine
wrapper. It performs the MCU-first ordering, model/startup-limit checks,
MIT/PRIVATE round trips, motor-version readback, and the final homing check in
one process:

```bash
python python/examples/full_ota_update.py \
    firmware/motor/rs00-0.0.3.32.bin TCGU01A28Z0086s
```

The command cannot operate the physical 24 V supply. At each required hard
restart it tells the operator to cut 24 V for at least two seconds and waits
for that exact firmware SN **and** CH343 serial to report an actual MCU uptime
rollback of more than one second, followed by an advancing reading still below
the pre-reset value. It does not compare MCU ticks to the host clock: slow or
paused ticks are not evidence of reboot. USB reconnects with continuous uptime,
scan errors and normal 32-bit uptime wrap do not count. If no rollback is
observed (including an ambiguous very-early-boot cycle), the check times out
rather than assuming a reset happened.
A heartbeat confirms an MCU restart, not physical removal of 24 V: a soft reset
also restarts uptime. The operator must cut the 24 V supply as instructed.
After reconnect,
the script waits up to 20 seconds for two consecutive target-protocol readings
before proceeding; it never bypasses the motor OTA preflight.

Before motor configuration writes it waits for active homing to end. Only an
explicit `NACK: SysBusy` retries the rejected command, within a 45-second budget;
other command errors stop the workflow. Errors name the failed step. No OTA
transfer is automatically retried by this busy handler.

For an unrecorded or mismatched motor model on follower firmware >= 1.2.12,
model setup runs before waiting for homing (and before optional MCU reflash).
A compile-time EL05 default is not physical motor identification. The operator
must type the actual model, matching the selected image, even with `--yes`.
The script saves auto-calibration as disabled, then requires a 24 V cycle:
the config write alone does **not** stop ongoing motion. After confirming
homing is idle, it records the model and requires another cycle before using
the new motor ranges. It restores auto-calibration using fresh model-specific
defaults, not the previous model's torque settings. If interrupted, leave the
device out of normal service and rerun to verify the model and restore the
disabled flag; do not manually force old calibration settings back. Older
MCU firmware must first be upgraded; this bootstrap is not supported there.
This sequence still needs hardware validation; mocked tests cannot establish
physical direction, torque safety, or actual power removal.

Matching firmware versions prompt separately for the follower and motor:
enter `y` to reflash; Enter, `n`, other answers, or EOF skip that image.
The motor prompt appears after switching to PRIVATE and reading the motor's
version, not at initial discovery. Skipping a flash does not skip the protocol
round trip or final checks. `--dry-run` never asks to reflash. `--yes` skips only
the SN confirmation, not these reflash questions. The final
summary reports whether each image was actually flashed or skipped. Use
`--reflash-motor` or `--reflash-follower` only when intentionally rewriting the
same version; each flag bypasses its corresponding reflash question.

`--direction` defaults to `model`, the follower 1.2.12 per-model default:
`negative` (Reverse) for EL05, `positive` for RS00. Power-on homing must close
first and record the closed stop as zero, then open. Software cannot tell which
physical stop it reached first, so before the final 24 V cycle the script asks
you to watch the jaw, and after homing it asks `y/n` whether it closed first.
`--yes` does not skip this. On `n` it flips Reverse, asks for one more 24 V
cycle and asks again; a second `n` stops with an error, because then neither
direction zeroes on the closed stop. EOF also stops. Use explicit `positive`,
`negative` or `keep` only for an intentionally different setup; the same
confirmation still applies. The final check also rejects a suspiciously short
calibration (default `< 0.8 rad`, configurable with `--min-travel-rad`).

If firmware is already upgraded but homing opens first (zero on the open stop),
use direction-only mode (the motor image is read only to validate its model;
neither image is flashed and the MCU manifest is not needed):

```bash
python python/examples/full_ota_update.py \
    firmware/motor/rs00-0.0.3.32.bin TCGU01A24Z0000s \
    --direction-only --direction positive
```

Pass the direction you want (`model`, `positive` or `negative`; not `keep`).
This mode requires a recorded matching model, MIT and enabled auto-calibration.
It reads back the direction, requires a 24 V cycle, checks the new homing
direction and travel, and asks the same close-first question, flipping once on
`n`. Keep the mechanism clear. Full OTA also reapplies the direction immediately
before its final MIT/homing cycle.

The two lower-level tools remain available when only one image needs flashing
or when debugging a failed stage:

```bash
python python/examples/motor_ota_update.py firmware/motor/rs00-0.0.3.32.bin TCGU01A28Z0086s
```

- **Follower firmware >= 1.2.8** (the `0x5B` relay).
- **The motor must be on the PRIVATE protocol.** Under MIT the motor does not
  answer extended frames (measured). Switch with
  `motor.switch_protocol(MotorProtocol.Private)`, then power-cycle — the 24 V
  supply included; an MCU restart does not switch the motor.
- **RobStride's OTA protocol does not check the model** — an RS00 image is
  accepted by an EL05 motor. `preflight()` checks the image against the motor
  model recorded on the gripper (`motor.get_model()`), so record it first.
- **Gripper OTA first, motor OTA second.** A gripper OTA switches the motor back
  to MIT.
- After the update the motor restarts on **MIT**, so its version cannot be read
  right away (the version frame needs the private protocol). Power-cycle; to
  confirm the version, switch to private, power-cycle, then `motor.motor_version()`.

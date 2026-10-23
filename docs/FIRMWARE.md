# 固件参考与刷写

> 本文讲的是**怎么构建和刷写**，不记录版本号——那会变成又一处会过时的硬编码。
> 仓库里当前发布的镜像版本、大小、CRC32 和源提交，以
> [`firmware/manifest.json`](../firmware/manifest.json) 为准，人读的说明在
> [`firmware/README.md`](../firmware/README.md)。设备上实际跑的版本用
> `GetVersion` 查（`gripper.firmware_version`）——它返回的是编译进镜像的常量，
> 是唯一能证明刷进去了什么的东西。

## 本地候选：IMU 单调时间与协议兼容

固件与 SDK 当前本地分支均为 `codex/gripper-imu-support`，由原从爪分支延续。
主爪也启用 TIM2 单调时间和 V2，并只发送新缓存样本；请求 1000 Hz 不代表
1000 Hz 独立采样。主爪原校准和从爪电机控制算法不在本轮修改范围。

尚未正式发布。新固件支持 `TimeSession (0x16)` 和 `GetImuV2 (0x17)`；
SDK 的 `read_once()` 和主/从爪显式 IMU streaming 自动协商：
明确 InvalidCmd 才使用旧 28 字节包；超时/忙/畸形响应报错，不静默降级。
从爪默认仍只开电机状态流，主爪现有 `start_streaming` 参数不变。
V2 使用 source_mask=0x0020，保持电机与编码器 source 位和速率不变。

新采样字段（C++ 与 Python）：

| 字段 | 含义 |
| --- | --- |
| `mcu_timestamp_us` | C++ uint64_t / Python int；V2 完整捕获时间，旧包为首次观测起的展开时间 |
| `boot_session_id` | V2 本次启动的非零会话号；旧包为 0，应与设备 SN 联合使用 |
| `timestamp_is_64bit` | 是否来自原生 V2，而不是 SDK 展开 |
| `timestamp_valid` | 本帧时间是否可用于当前观测段；异常旧包跳变或 V2 乱序为 false |
| `timestamp_discontinuity` | 会话改变或不确定跳变，消费者应重置积分/滤波/时间拟合 |

会话号由 SDK 提议随机 64 位 nonce，固件每次启动只接受第一个，保存在 RAM；
重复协商不改变它，不写 Flash。设备重启后必须重新协商、重开流。
会话号是概率唯一的诊断标识，不是认证凭据。V2 可跨任意多次 32 位回绕和断流恢复，
但重启会重置时间，应按会话分段，不能把新启动拼成旧启动的连续轨迹。

旧包只在短间隔、序列正常的连续观测下展开自然回绕。旧 DWT 固件的约 17 秒回退、
重启样跳变和超过约 35.8 分钟的观测空档标记 discontinuity/invalid，不制造假时间。
检测到上述旧包异常后，该时间线保持 invalid/discontinuity，不因下一帧重复或正常前进
自动恢复。流可 `off(id)` 后重新 `on_data(...)` 建立新观测段；轮询需新建 IMU 实例。
有效原生 V2 可恢复可信时间。重新建立观测段不保证恢复绝对开机时间，不能将两段直接积分；
旧 DWT 固件仍会再次回退，根治需要升级固件。旧包无法排除所有重启歧义。
`IMU::decode` 是无状态解码；`read_once` 与每个 `on_data` 订阅分别维护时间线。

`on_data` 现在过滤连续字节完全相同的 IMU DATA 载荷，减少缓存重复引起的重复积分；
原始传输订阅与 `read_once` 不过滤。多个订阅者各自保有去重及时间线状态。
回调频率因此可能由约 1000 Hz 降至约 100 Hz；不要把回调次数当成串口接收帧数。
使用者必须检查 `timestamp_valid`，无效样本不能用于积分；这不是坐标系或精度修正。

**集成限制：**这是 C++ `ImuSample` 布局变更，所有链接消费者必须重编译，不能仅替换共享库。
Python 扩展也须重建并核实实际加载路径。ROS 的时间字段须为 uint64，并增加会话/有效性字段，
否则旧 uint32 消息仍会截断；本任务不修改独立 ROS 仓库或其依赖副本。
ROS header 时间仍是另外的时钟域，以上修改不代表相机/IMU 同步通过验收。
固件/SDK 本地构建成功也不等于已刷设备或安装到业务环境。

软件检查：`ctest --test-dir <build> --output-on-failure`、`python -m pytest python/tests -q`。
真机还需超过 75 分钟连续采集与冷重启验收；默认 motor-only、旧 SDK、新旧协议互通、
IMU 同时上报时的控制时延和 OTA 都需验证。IMU 数值精度另行验收。

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
  `task_encoder.c` — acquisition, cached samples and stream scheduling are
  separate; requested rate is not evidence of independent sample rate.
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
`tc-gu-01-master-1.2.6.bin` from `firmware/`; OTA compares no versions
(`--target-version` is informational), so reflashing the same number is fine.
Compare versions only within one role.

Note the build output keeps the Makefile's unversioned name
(`build/master/tc-gu-01-master.bin`), while the images released under
`firmware/` carry the version (`tc-gu-01-master-1.2.6.bin`). That is deliberate:
a build artifact is whatever you just compiled, a release is a specific version
someone may still be holding a copy of months later. **If you promote a local
build into `firmware/`, rename it and update `firmware/manifest.json` in the
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

The motor module runs its own firmware, separate from the gripper's. It can be
flashed through the follower's USB-C port — no RobStride USB-CAN adapter, no
removing the motor — with `python/examples/motor_ota_update.py`, built on
`MotorOtaSession` (`cpp/include/taccap/motor_ota.hpp`). Every CAN frame is
relayed by the follower MCU through `Motor.can_ext_xfer` (`0x5B`).

```bash
python python/examples/motor_ota_update.py ~/Downloads/rs00-0.0.3.32.bin TCGU01A28Z0086s
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

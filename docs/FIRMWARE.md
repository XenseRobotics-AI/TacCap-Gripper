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
  mirrored from branch `hw_v1.1.0` @ `81835a6` (command set V2.2 plus the
  0x54 / 0x55 diagnostic pair).
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
    left --target-version 1.2.5
```

Note the build output keeps the Makefile's unversioned name
(`build/master/tc-gu-01-master.bin`), while the images released under
`firmware/` carry the version (`tc-gu-01-master-1.2.6.bin`). That is deliberate:
a build artifact is whatever you just compiled, a release is a specific version
someone may still be holding a copy of months later. **If you promote a local
build into `firmware/`, rename it and update `firmware/manifest.json` in the
same change** — the manifest's `file` entry is what the role selectors and
`--all` resolve, so the two drifting apart breaks exactly the path customers
are told to use. `test_ota_update_all.py` fails if they disagree.

> **刷完必须重新插拔,这是升级流程的一部分,不是排障手段。**
>
> 做法:**USB 数据线和电源线同时拔下**,两条都断开之后再一起插回。这两条线供的
> 是不同的部分 —— USB 给 MCU 和 USB 转串口桥,24 V 给电机 —— 所以只拔其中一条,
> 另一半始终带电。先后拔、拔第二条时第一条已经插回去了,板子就从来没有过完全
> 断电的一刻,也就没有复位。
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
  `ls -l build/*/tc-gu-01-*.bin` (builds at `bf0a06e` with
  `arm-none-eabi-gcc 13.2.1`: master ~118 KB, slave ~156 KB, i.e. 25% / 33% of
  the cap). Sizes vary by several hundred bytes across toolchains and by a few
  hundred between firmware revisions, so treat these as approximate — the check
  that matters is that the `.bin` fits under 456 KB.
- Flash the artifact matching the *role*, not the side. A gripper's role is the
  `m` / `s` suffix on its firmware SN; both leaders take the `master` build.
- `make download` is Windows-only (`STM32_Programmer_CLI.exe`, and it flashes
  the `.elf`). On Ubuntu use the OTA path above.
- `Cmd::GetVersion` returns the **compiled-in** constant, not the OTA bank
  metadata, so `--target-version` is bookkeeping only — the version you read
  back afterwards is proof of what actually got flashed.

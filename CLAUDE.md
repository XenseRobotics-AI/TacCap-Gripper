# TacCap-Gripper — Claude working notes

Loaded into every Claude session in this repo. Keep terse — auto-memory
carries deep background; this file is *house rules*.

## Repo identity
- C++17 SDK + Python (pybind11) bindings. Apache-2.0. Long-term maintained.
- Downstream consumers only *import* the `xense.taccap` package and never
  reimplement lower-layer comms. `lerobot-xense` is the one in use, via the
  `third_party/taccap-gripper` submodule; it releases independently.

## Build & test (C++)
- Build dir is `build/` (existing CMake/Ninja generator).
- **GTest is not in `xense-taccap`** — it lives in the `taccap` env, while
  OpenCV+spdlog live in `xense-taccap`. Neither env alone can configure the
  test build; pass both prefixes (compiler/OpenCV/spdlog still resolve from
  `xense-taccap`, which must come first):
  ```bash
  X=~/miniforge3/envs/xense-taccap; T=~/miniforge3/envs/taccap
  env -u LD_LIBRARY_PATH -u PYTHONPATH $X/bin/cmake -S . -B build -G Ninja \
    -DCMAKE_PREFIX_PATH="$X;$T" -DTACCAP_BUILD_TESTS=ON
  ```
  The system GTest at `/usr/lib/x86_64-linux-gnu/cmake/GTest` looks usable but
  is not: conda's g++ compiles against its own sysroot and never searches
  `/usr/include`, so `gtest/gtest.h` is not found. The "conflicting OpenCV in
  implicit directories" warning at generate time is expected and harmless
  (both envs ship OpenCV).
- Build a target: `cmake --build build --target <tgt>` (e.g. `taccap_unit_tests`).
- Run full test suite: `./build/cpp/tests/taccap_unit_tests`
- Run one suite: `./build/cpp/tests/taccap_unit_tests --gtest_filter='<Suite>.*'`
- After touching protocol / payload structs, verify the
  `static_assert(sizeof(...) == N)` lines in
  `cpp/include/taccap/protocol/payloads.hpp` — they fail the build the
  instant our own layout drifts.
- Those asserts only check the SDK against *itself*. To check it against the
  firmware, run `scripts/check_protocol_drift.py` (needs the `tc-gu-01` clone;
  skips cleanly without it). It compares the command/error tables by wire value
  and every mapped payload by real compiled `sizeof`. Run it whenever the
  firmware clone is updated — this is what catches "the firmware added a
  command and we never noticed", which the asserts structurally cannot.
- Linking the test binary can fail with `undefined reference to
  curl_*@CURL_OPENSSL_4` / `__cxa_call_terminate` — that is the
  `lerobot-xense` conda env leaking through `LD_LIBRARY_PATH`. Build with
  `env -u LD_LIBRARY_PATH cmake --build build`.

## Build & test (Python)
- `pytest python/tests` — hardware-free cases always run; the IMU cases skip
  when no gripper is connected. Guards against zero-stride numpy views (see
  `test_numpy_views.py`); `py::array_t<T> a(n)` is a trap on pybind11 2.9.
  `test_dispatch_decoupling.py` is pty-backed and needs no hardware.
- `pytest` is available in all three envs (`xense-taccap`, `taccap`,
  `lerobot-xense`) — an older note here claimed it was missing from
  `xense-taccap`; that is wrong.
- **The `sys.meta_path` hijack is now handled by `python/tests/conftest.py`**,
  so a plain `pytest python/tests` tests *this* checkout. Keep that file: both
  `taccap` and `lerobot-xense` carry an editable install of `taccap_gripper`
  whose `ScikitBuildRedirectingFinder` beats `PYTHONPATH` *and* a
  `sys.path.insert`, silently redirecting `xense.taccap` to a different
  checkout. The symptom is not an import error — it is ~38 assertion failures
  claiming `ForcePositionConfig` still has `brake_distance_rad`, which reads
  exactly like a regression in this repo and is not.
  Scripts outside `python/tests/` still have to strip it themselves:
  ```python
  import sys
  sys.meta_path[:] = [f for f in sys.meta_path
                      if 'taccap' not in type(f).__module__.lower()]
  sys.path.insert(0, '<repo>/python')
  ```
  The conftest also asserts that `_taccap_native` came from this checkout —
  `xense.taccap` is a namespace package, so `__file__` pointing here does not
  prove the compiled extension did. Importing still needs
  `LD_LIBRARY_PATH=<xense-taccap>/lib` for OpenCV.

## Build & install (Python wheel)
- conda env `xense-taccap` (py3.12, primary dev env):
  `pip install -e . --no-build-isolation`
- System py3.10:
  `/usr/bin/python3 -m pip install --user --no-build-isolation .`
- C++ examples build by default (`-DTACCAP_BUILD_EXAMPLES=OFF` to skip). They
  are on so CI compiles them: while they were off, an SDK signature change
  could break every example and the build stayed green. `pyproject.toml` pins
  them OFF for wheel builds, so `pip install` is unaffected.
- Active conda env's python:
  `/home/vertax/miniforge3/envs/xense-taccap/bin/python`.
- **Always run Python with `env -u PYTHONPATH`.** The shell stacks
  `xense-taccap` on top of `lerobot-xense`, whose activation exports
  `PYTHONPATH=<lerobot-xense>/lib/python3.12/site-packages`. That injects its
  editable install of `xense.taccap` — pinned to whatever commit the
  `lerobot-xense` submodule sits at — into *every* interpreter, so you silently
  test old code. With `PYTHONPATH` cleared, `taccap` resolves to this repo.
- **Importing needs `LD_LIBRARY_PATH=$CONDA_PREFIX/lib`.** OpenCV comes from
  conda, the locally built `libtaccap_core.so` is linked with
  `INSTALL_RPATH "$ORIGIN"`, and conda does not put its own `lib` on the loader
  path — so `import xense.taccap` fails with `libopencv_core.so.412` even
  inside an activated env. Distinct from the `env -u LD_LIBRARY_PATH` rule
  above, which is about a leaked *build*-time path; a session can need both.
- There is no system OpenCV, so a bare `cmake -S . -B build` fails at
  `find_package(OpenCV REQUIRED)`. `conda activate xense-taccap` sets
  `CMAKE_PREFIX_PATH` and puts the env's cmake first, which is enough; only if
  you call `/usr/bin/cmake` directly do you need
  `-DCMAKE_PREFIX_PATH=$CONDA_PREFIX`.
- **Put `$CONDA_PREFIX/bin` on `PATH` before `pip install -e .`**, i.e. call
  `pip` through an activated env rather than by absolute path. scikit-build-core
  probes the build tool by running `ninja --version`, and on a bare `PATH` it
  finds something that answers with GNU Make's banner instead. The failure names
  the wrong culprit entirely:
  ```
  CMake Error: The detected version of Ninja (GNU Make 4.3 ...) is less than the
  ```
  ```bash
  export PATH=$CONDA_PREFIX/bin:$PATH
  pip install -e . --no-build-isolation
  ```
- **A C++ or bindings change is not live in a consumer env until you reinstall
  there.** The editable install goes through
  `_taccap_gripper_editable.ScikitBuildRedirectingFinder`, which redirects the
  *Python* sources to this checkout but keeps serving `_taccap_native` from the
  env's `site-packages`. So after touching `python/bindings/` you get this
  checkout's `__init__.py` paired with whatever extension was built last time —
  and a newly exported symbol fails as
  `module 'xense.taccap._taccap_native' has no attribute '<Name>'`, which reads
  like a typo in `__init__.py` and is not. Reinstall into *each* env that
  consumes the package (`lerobot-xense` is a separate install from
  `xense-taccap`), and note that `xense.taccap` is a namespace package whose
  search path covers both the checkout and site-packages, so `__file__` pointing
  at this repo does **not** mean the extension came from here. Confirm with:
  ```python
  import sys, xense.taccap
  print(sys.modules['xense.taccap._taccap_native'].__file__)
  ```

## Hardware smoke test (when a gripper is plugged in)
```bash
python -c "from xense.taccap import scan_grippers, Side
for g in scan_grippers():
    s='L' if g.side==Side.Left else 'R'
    print(f'  [{s}] ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"
```
Healthy output: `[L]` + `[R]`, both with non-empty `firmware_sn`. Empty SN
means firmware hasn't burned the SN yet, or firmware < V1.6. The raw V4L2
bringup probes were removed in the example cleanup; use `v4l2-ctl --list-devices`
directly if you need to go below the MCU.

## 从爪现场排查（实测得来，别再重走）

**设备路径永远用 `/dev/serial/by-id/`，不要写死 `/dev/ttyACM1`。** 每次重新枚举
内核都可能换 minor：实测见过 `ttyACM1 → ttyACM2`，脚本里写死的路径会一直等不
到设备，或者对着旧 fd 写出 `IoError: Input/output error`。`find_follower()`
返回的 `mcu_device` 就是稳定的 by-id 路径。

**判断"是否真的断电重插"看枚举时间戳**，不要凭记忆：
```bash
stat -c '%y' /dev/serial/by-id/usb-1a86_USB_Dual_Serial_*-if02
```
OTA 的 bank-swap 软复位会在**同一秒**重新枚举；物理重插则晚十几秒以上。这条
拦下过好几次"以为拔过了其实没拔"的过早测量。

**"power-cycle" 指断 24V，不只是拔 USB。** 24V 是独立电源；拔 USB 只重启
MCU 和 CH343，电机一直带电。两次"OTA 后只拔 USB"的开机出现了自动标定卡死或
读到中途值，三次断过 24V 的都正常 —— 相关性，未做对照实验证实，但代价是断一次
电，没必要赌。**切换电机协议也必须断 24V**，MCU 重启不生效。

**读状态前要等自动标定跑完（约 9–10 秒）。** 标定期间读到的是中途值：位置在
−1.23 和 0 之间任意一点，`stop_reason` 还停在 3(CLEAR_FAULT)。判据是
`stop_reason == 1` 且位置连续两次不变。更糟的是在标定期间发命令（尤其
`switch_protocol`，内含 350ms `HAL_Delay` 和一次 `rb_flush`）会干扰它。

**`/dev/ttyACM0`(if00) 不是固件 DEBUG 口。** 把日志开到 DEBUG +
`output_mask=0x01` 之后 if00 十四秒零字节。日志走物理 UART7，没引到 USB，要看
只能上硬件串口探头。另外 `LOG_BAUDRATE` 是 **921600**，DESIGN.md 写的 115200
是过时的（DESIGN.md 整体停在 v1.1，从爪相关内容一概不可信）。

**MIT Command 12/13/14/15（保存/主动上报/读参数/写参数）一律不应答**，后果是
**MIT 模式下读不到任何电机参数**，`vBus`、`boardTemp` 都要切私有协议才能读。
`0x7028`(canTimeout) 存得下、读得回、跨断电保持，但**电机不执行它** —— 实测
使能后静默 6 秒（指令5 间隔 1000ms，远超 200ms 阈值）电机始终停在 Motor 模式。
详见 `can_motor.c` 文件头的实测记录。

**这个现象的机理在 1.2.6 上变了，结论没变。** 以前"MIT 下读不到参数"有两道闸叠
在一起：固件的 `cmd_handler_motor_get/set_private_param` 按协议模式一刀切，在发出
任何 CAN 流量之前就回 `InvalidParam`（实测于 1.1.5 的 `0x7017`）。**1.2.6 把那道
一刀切的闸刻意拿掉了** —— `protocol_handler.c` 记着理由：正是它让 `vBus` / `iqf` /
`0x7028` / 型号识别在 MIT 下全读不到，而 MIT 自己的指令 17/18 参数通道固件里本来
就有。现在只剩 `protocol_check_motor_param_access_allowed()`：OTA 期间拒绝，控制
任务在跑时回 `SysBusy`，跟协议模式无关。

结果是症状照旧、原因换了层：2026-09-23 实测 `0015s`（固件 1.2.6、
`get_protocol() == Mit`、控制环停止），`0x701C` / `0x7017` / `0x7028` / `0x700B`
四个 index **全部返回 `Timeout`，不是 `InvalidParam`**。`Timeout` 本身就是"固件确实
转发了"的证据，所以**不应答的是电机**。判据因此也变了：见到 `InvalidParam` 说明
撞的是固件侧的闸（或这个 index 不在白名单），见到 `Timeout` 说明请求出去了、电机
没回。探测脚本：`~/taccap-gripper-work/probe_private_param_under_mit.py`（只读）。

**仍未做的对照**：同一台机器切到私有协议再读这几个 index。答了就能把原因钉在 MIT
上，不答则指向电机固件本身。代价是来回各断一次 24V（切协议本身也要断电才生效）。

**这里曾经记着"本机电机固件低于 1.0.5.0.4"，那是推断，已被实测证伪。** 手册对
且仅对这四条标了"需固件更新至 1.0.5.0.4"，不应答的恰好也是这四条，所以当时倒推
出版本偏低。2026-09-23 用固件 1.2.6 的 `0x58` 真读出来：`0018s` 正好就是
**1.0.5.0.4**，那四条命令照样不应答。版本门槛解释不了这个现象，原因仍未知。

**读电机固件版本**：`motor.motor_version()`，需从爪固件 >= 1.2.6。**目前只在私有
协议下证实可读**（`0074s` / `0018s`），MIT 下能否读到没验证过——请求是扩展帧，
理论上 MIT 忽略，但没在跑 1.2.6 的 MIT 机器上试过，别基于任一假设写开机检查。
切协议每个方向都要断一次 24V。

**要求：电机固件 >= 1.0.5.0.4。** 实测对照：

| 机器 | 电机固件 | 状态 |
|---|---|---|
| `0018s` | 1.0.5.0.4 | 正常 |
| `0074s` | 1.0.5.0.2 → 升级后 1.0.5.0.4 | 升级前速度反馈恒为常值，运动纹波 117%、逆向帧 80 |
| `0073s` | 未读 | 同类故障 |

`0074s` 是**同一台设备升级前后的对照**，所以不是机器间差异。但相关性只到这里，
因果未证实，`0073s` 的版本也还没读。电机固件只能走官方上位机升级（手册 §3.3.3，
需灵足时代官方 USB-CAN、电机要从夹爪上拆下来单独接），协议未公开。

**电机规格的单一真值源是 `App/drivers/motor_spec.c`**（EL05 数据取自 260713 版
官方说明书）。母线 24V，实测 `0x701C VBUS = 24.21V`。改量程、力矩↔电流换算、
温度限、堵转额定都改那里，不要再往 `can_motor.c` 里加常数。

## Commit convention
- Conventional commits with subsystem scope:
  `feat(protocol): ...`, `fix(parser): ...`, `test: ...`, `chore: ...`,
  `feat(examples): ...`
- Do **not** use `--no-verify`. If a hook fails, fix the underlying issue.
- Do **not** amend already-pushed commits — dual-remote sync becomes
  very painful afterwards.

## Pushing
On **this machine there is exactly one remote**, and it is GitHub:
```
origin  git@github.com:XenseRobotics-AI/TacCap-Gripper.git
```
So the only push is `git push origin main`. There is no `github` remote here —
`git fetch github` fails with "does not appear to be a git repository". (Other
clones name the remotes the other way round, `origin` = internal GitLab and
`github` = GitHub. Check `git remote -v` before trusting either convention.)

The internal GitLab **is reachable from this machine** (verified 2026-09-22;
an older note here said it was not). It mirrors the **firmware** repo only:

```
# in third_party/firmware/tc-gu-01
gitlab  git@192.168.110.140:xense/tc-gu-01.git
```

Use SSH, not HTTP — HTTP prompts for a username that a non-interactive shell
cannot supply. No GitLab mirror of this SDK repo is known; if one exists, ask
for its URL rather than guessing.

**GitHub is canonical for the firmware; GitLab is a mirror.** Land every change
by merging the PR on GitHub, then push the resulting `hw_v1.1.0` to GitLab.
**Never open an MR on GitLab** — that is what caused the divergence this rule
exists to prevent: the same branch got merged separately on each side, so
`hw_v1.1.0` carried a different merge commit per remote while the trees stayed
identical. Two rounds of that were force-pushed away on 2026-09-22 (GitLab
`b66d97c` / `89d8f2c` dropped in favour of GitHub `7b52fa5`), with the branch
owner's agreement. It was harmless only because each feature branch happened to
sit on a common ancestor, so both sides could still merge cleanly; a third round
would not have been.

```bash
# after the GitHub PR is merged
git fetch origin && git push gitlab origin/hw_v1.1.0:hw_v1.1.0
```

That push is a fast-forward as long as nobody merges on GitLab. If it is ever
rejected, someone did — do not reach for `--force` on your own, ask first: the
trunk is shared and a rewrite makes every other clone of it need a hard reset.

`origin/main` takes external contributions, so **expect a rejected push**.
Fetch, look at what landed, then rebase — never force-push to `main` as a
shortcut, ask first. After a rebase across someone else's commits, rebuild and
re-run the suites before pushing: the merged tree is code neither side tested.
If a test fails there, check it against plain `origin/main` first — it may not
be yours.

## Logging
- The entire SDK uses **one singleton logger**: `xense::taccap::logger()`,
  registered with spdlog under the name `"xense.taccap"`. C++ and Python
  share the same instance and the same sinks. Never construct an ad-hoc
  `std::make_shared<spdlog::logger>(...)` elsewhere.
- Do not use `std::cout` / `printf` / `print()` / `std::cerr` for diagnostic
  output — go through the logger. Examples and one-shot CLI tools (e.g. a
  `scan_grippers` table printed for a human) may use `print` / `std::cout`
  because that output *is* the program's feature, not logging.
- C++: `#include <taccap/log.hpp>`, then `xense::taccap::logger()->info(...)`.
- Python: `from xense.taccap import log; log.info(...)` /
  `log.set_level("debug")`.

### Sinks (both attached by default)

| Sink | Level | Pattern (constant in `log.hpp`) |
|---|---|---|
| stderr (color) | user-controllable (default INFO) | `SPDLOG_PATTERN` = `[%D %T.%e] [%n] [%^%l%$] %v` |
| file (per-session) | always DEBUG | `FILE_LOG_PATTERN` = `[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v` |

The logger itself sits at DEBUG so the file sink sees everything; the
console sink filters via its own sink-level. `log.set_level(...)` and
`log.set_pattern(...)` affect the **console sink only**; the file sink's
archive format never changes (keeps historical greps parseable).

### File sink behavior
- Directory: `$TACCAP_LOG_DIR` if set, else `~/.taccaplogs/`.
- Filename: `session_YYYYMMDD_HHMMSS.log`. One new file per process start.
- On startup, files are sorted by mtime and the oldest are deleted so at
  most `kMaxSessionLogs` (= 10) session logs remain.
- File-sink creation failures (disk full, permission denied, missing path
  that can't be created) must **not** be fatal — the console sink keeps
  working regardless.

## Load-bearing constraints (don't touch unless asked explicitly)
- The visuotactile (OG) capture/rectify path is **not** part of this SDK as of
  0.1.4 — `libxensesdk`, `vision.hpp` and `TactileSensor` were removed. Tactile
  imaging is handled at the Python level via the `xensesdk` wheel. Don't
  reintroduce a libxensesdk submodule; `xense.taccap` is gripper-protocol +
  wrist-camera only.
- `third_party/firmware/` is a **clone-on-demand** firmware reference dir,
  **not** a submodule. `.gitignore` already excludes it. Never `git add -f`
  or convert it into a submodule. It is a separate repository in the same
  `XenseRobotics-AI` org as this repo:
  ```bash
  gh repo clone XenseRobotics-AI/tc-gu-01 third_party/firmware/tc-gu-01 -- --depth=1
  ```
  Treat it as read-only. The stream scheduler in
  `App/tasks/task_data_stream.c` is the authority for what `start_streaming()`
  rates actually do — `cpp/src/stream_rate.hpp` mirrors it.
- Side L/R detection reads the firmware-burned SN via `Cmd::GetSn`, **not**
  the CH343 USB chip SN.

## Risky actions — confirm before
- Running `python/examples/ota_update.py` (flashes firmware; wrong artifact
  bricks the MCU). **Always power-cycle the gripper after an OTA.** The
  bank-swap reboot is a soft reset and leaves the device in a degraded state
  that looks entirely healthy — right version, stream running, counters clean —
  while quietly dropping status frames. Measured: 35-39 lost per 60s run after
  OTA alone, zero after a replug, same unit and firmware both ways. Any number
  you take before replugging is suspect. **从爪上"power-cycle"指断 24V，不是
  只拔 USB** —— 24V 独立供电，拔 USB 时电机一直带电；判断是否真的重插看枚举
  时间戳，见「从爪现场排查」。
- Any change under `third_party/firmware/` or to the firmware-protocol
  mirror headers in `cpp/include/taccap/protocol/`.
- `git push --force*` to `main` (the only remote here is GitHub `origin`).

## When in doubt
Auto-memory for this repo lives in
`~/.claude/projects/-home-xense-sn0-TacCap-Gripper/memory/` and is loaded
automatically — reference it when relevant. Note it is currently **empty on
this machine**: the deeper background it used to hold (subsystem map, runtime
gotchas, wire-protocol notes, firmware stream-dup behavior) was written under
the old `-home-ubuntu-` user and did not come across. Treat anything you need
from it as unknown until re-derived, and prefer writing durable findings into
this file or into code comments where they can be reviewed.

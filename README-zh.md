# taccap-gripper

[English](README.md)

**TacCap-Gripper** 的 C++17 / Python SDK —— XenseRobotics 的多模态触觉数据采集夹爪。
统一命名空间:C++ 为 `xense::taccap::`,Python 为 `xense.taccap`。

SDK 只用 C++ 实现一次,Python 包是同一份代码的 pybind11 绑定,两种语言行为完全一致。
上位机通过 TC-GU-01 串口协议与夹爪 MCU 通信,由 MCU 驱动电机 —— 实时安全(运动安全包络、
堵转与过热保护)在固件里执行,上位机无法绕过。

## 功能

- **主爪与从爪** —— `LeaderGripper`(手持主爪:编码器、IMU、按键)与
  `FollowerGripper`(带 RobStride 电机的执行夹爪)。
- **控制器** —— `ImpedanceController` 跟随位置,`ForcePositionController` 以有界力抓取。
  两者接口相同,只有两个非阻塞调用:`set_target(0..1)` 和 `snapshot()`。
- **归一化开口度** —— `0` = 闭合,`1` = 张开,单次读取和流式数据都提供。
- **传感器** —— IMU、编码器、LED、按键、上电自动标定。
- **腕部相机** —— V4L2 采集,使用设备自身存储的内参做鱼眼去畸变。
- **零配置发现** —— 左右与主从角色由固件烧录的序列号决定,无需逐台配置。
- **OTA 升级** —— 夹爪 MCU 固件与从爪电机固件均可通过 USB 升级。

不在本 SDK 范围内:视触觉(OG)采集由 `xensesdk` wheel 提供;遥操作、策略与数据录制属于
上层应用(例如 LeRobot 集成)。

---

## 环境要求

| | |
| --- | --- |
| 操作系统 | Linux(Ubuntu 22.04+)。相机走 V4L2,不支持 macOS / Windows |
| Python | 3.12 |
| 构建 | 通过 Miniforge 安装的 [Mamba](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install) —— `environment.yml` 锁定了完整工具链 |
| 固件 | 从爪 **≥ 1.2.5**(SDK 强制检查),见 [固件](#固件) |

## 安装

**第 0 步 —— Mamba。** 安装 Miniforge,它自带 `mamba`(比 conda 快得多,默认使用
conda-forge)。如果 `mamba --version` 已经能用,可跳过。

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
# 按提示完成安装,然后打开一个新终端
```

**第 1 步 —— 系统软件包。** `v4l-utils` 提供 `v4l2-ctl`,设置和检查相机格式要用它
([主机配置 → 3](#3-腕部相机必须使用-mjpeg))。先 `apt update`,确保装到最新版本:

```bash
sudo apt update
sudo apt install -y git curl v4l-utils
sudo apt install -y --only-upgrade libusb-1.0-0   # 内核更新后保持 USB 运行库为最新
```

> [!NOTE]
> 如果 `apt upgrade` 带来了新内核,请按上面的命令升级 USB 软件包并重启,让运行中的内核与
> 用户态 USB 栈来自同一次更新。内核升级后 USB 栈没跟上,是"昨天还好好的相机今天打不开"
> 的常见原因。

**第 2 步 —— 克隆仓库。**

```bash
git clone https://github.com/XenseRobotics-AI/TacCap-Gripper.git
cd TacCap-Gripper
```

**第 3 步 —— 创建并激活环境。** 环境名为 `taccap`,会从 conda-forge 安装编译器、CMake、
OpenCV、pybind11 和 Python 3.12:

```bash
mamba env create -f environment.yml
mamba activate taccap
```

拉取新版本后更新已有环境:`mamba env update -f environment.yml --prune`。

**第 4 步 —— 安装 SDK**(在已激活的环境中):

```bash
uv pip install -e . --no-build-isolation
```

请保留 `--no-build-isolation`:它保证用环境里锁定的 `pybind11` 编译。拉取新版本后要重新
执行这一步 —— 可编辑安装在重装之前仍会使用之前编译好的扩展。

**第 5 步 —— 验证。**

```bash
python -c "import xense.taccap as t; print(t.__version__)"
v4l2-ctl --version
```

纯 C++ 构建、测试与环境陷阱见 [docs/INSTALL.md](docs/INSTALL.md)。

## 主机配置(一次性)

第一次插上夹爪之前,先完成以下三步。

### 1. 串口与相机权限

夹爪 MCU 枚举为 `/dev/ttyACM*`(属 `dialout` 组),腕部相机为 `/dev/video*`(属 `video` 组)。
用户不在 `dialout` 组时,SDK 能列出夹爪但打不开串口读固件 SN,于是 `scan_grippers()` 返回
`role=Unknown` / 空 `firmware_sn`,打开时报 `Permission denied`。

```bash
sudo usermod -aG dialout,video "$USER"
# 注销后重新登录(或重启),然后重新插拔夹爪
```

确认 MCU 能完整读取 —— `role` 必须是 `Leader` / `Follower`(不是 `Unknown`),
`firmware_sn` 不为空:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side.name, g.role.name, repr(g.firmware_sn))"
```

### 2. 让 ModemManager 不要占用夹爪串口

MCU 是 CH343 USB 转串口芯片(`1a86:55d2`)。每次热插拔,Ubuntu 默认安装的
**ModemManager** 会用 AT 指令探测新串口并占用几秒,在这段时间内打开夹爪会失败:

```
IoError: SerialBus: open(/dev/serial/by-id/usb-1a86_USB_Dual_Serial_...-if02): Device or resource busy
```

典型现象:第一次启动正常,拔插后立即重启就报 *busy*。添加 udev 规则让 ModemManager
忽略这类设备(不影响真正的调制解调器):

```bash
sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
# TacCap-Gripper MCUs are CH343 USB-serial (1a86:55d2) — keep ModemManager off them
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

插上夹爪后验证:

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE   # -> ID_MM_DEVICE_IGNORE=1
```

如果是没有蜂窝模块的专用机器人主机,也可以直接 `sudo systemctl disable --now ModemManager`。
若装有 `brltty`(盲文驱动),它同样会抢占 `1a86` 设备,用 `sudo apt remove brltty` 卸载。

### 3. 腕部相机:必须使用 MJPEG

> [!IMPORTANT]
> 腕部相机**必须**以 **MJPEG**(`MJPG`)格式出流。未压缩的 YUYV 在 640×480 @ 30 fps 下
> 需要预留约 147 Mbit/s 的等时带宽;夹爪上的相机共用一条 **USB 2.0** 链路,两路 YUYV
> 就会把它占满。结果是 `VIDIOC_STREAMON: No space left on device`、相机打不开,或者
> 静默丢帧。

SDK 的 `Camera` 默认请求 MJPEG(`use_mjpg=True`)—— 整套设备采集时不要关掉它
(`use_mjpg=False`、`wrist_camera.py --no-mjpg`)。其他打开相机的程序(OpenCV、ffmpeg、
GStreamer、相机服务)必须自己请求 MJPEG;OpenCV 中要在设置宽高**之前**把
`CAP_PROP_FOURCC` 设为 `MJPG`。

用 `v4l2-ctl`(`v4l-utils` 包,已在[安装 → 第 1 步](#安装)中安装)检查和设置格式:

```bash
CAM=/dev/v4l/by-id/<腕部相机>-video-index0       # 用 ls /dev/v4l/by-id/ 查看

v4l2-ctl -d $CAM --list-formats-ext              # 列表中必须有 'MJPG'
v4l2-ctl -d $CAM --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d $CAM --set-parm=30
v4l2-ctl -d $CAM --get-fmt-video                 # -> Pixel Format : 'MJPG'
```

V4L2 格式是驱动状态:重新插拔或重启后会复位,应用打开设备时也可能重新设置。
`v4l2-ctl` 用来设置和确认格式,同时要保证每个采集程序都请求 MJPEG。

---

## 快速上手

所有示例脚本都只接受一个位置参数:`left`、`right` 或固件 SN。只接了一台夹爪时可省略。

```bash
# 1. 接了什么?(只读)
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn, g.mcu_device)"

# 2. 读状态(只读)
python python/examples/follower_status.py left

# 3. 写入固件运动安全包络(每台从爪一次,写 flash)
python python/examples/impedance_control.py left --set-envelope

# 4. 首次运动,键盘交互(j/k/o/c)
python python/examples/gripper_console.py left

# 5. 抓取
python python/examples/force_position_control.py left
```

### 主爪 —— 读数

```python
import xense.taccap as t

g = t.LeaderGripper.open(normalize_position=True)   # 唯一接入的夹爪
g.encoder.on_data(lambda s: print(s.position_rad, s.position))   # position: 0..1
g.imu.on_data(lambda s: print(s))
g.start_streaming(imu_hz=100, encoder_hz=100)
# ...
g.stop_streaming()
```

### 从爪 —— 驱动

始终通过控制器驱动电机,并用 `for_spec()` 构造配置,使限值与实际电机(EL05 或 RS00)匹配。

```python
import xense.taccap as t

g = t.FollowerGripper(t.find_follower().mcu_device)
cfg = t.ForcePositionConfig.for_spec(g.motor.get_spec())
c = t.ForcePositionController(g, cfg)

g.motor.clear_fault()
c.start()                       # 先启动控制器,再使能电机
g.motor.enable()
try:
    c.set_target(0.0)           # 0 = 闭合,1 = 张开;非阻塞
    while True:
        s = c.snapshot()        # 一致的状态快照,不占总线
        if s.holding or s.arrived:
            break
finally:
    c.stop()                    # 力矩归零后失能电机
```

要跟随位置而不是抓取,换成
`t.ImpedanceController(g, t.ImpedanceConfig.for_spec(g.motor.get_spec()))`,接口相同。
控制器运行期间从 `snapshot()` 读状态,不要调用 `motor.read_status()`。

| `ForcePositionConfig` | 默认值 | 含义 |
| --- | --- | --- |
| `grasp_torque_nm` | 电机持续堵转额定(EL05 1.1 N·m,RS00 3.6 N·m) | 夹爪被挡住时稳定的夹持力 |
| `close_speed_radps` | 1.1 rad/s | 行进速度 |

### 腕部相机

```python
from xense.taccap import Camera

cam = Camera(device="/dev/v4l/by-id/<腕部相机>-video-index0",
             width=640, height=480, fps=30.0)       # 默认 MJPEG
cam.start(lambda f: print(f.frame_index, f.image.shape))
```

也可以在夹爪对象上通过 `open_cameras=True, wrist_video=...` 打开(加 `undistort_wrist=True`
直接输出去畸变图像)。夹爪对象默认不打开相机,因为相机通常由相机服务占用。

### 一个进程内同时用左右两台

```python
from xense.taccap import FollowerGripper, scan_grippers, Side

eps = {e.side: e for e in scan_grippers()}
left, right = FollowerGripper(eps[Side.Left].mcu_device), FollowerGripper(eps[Side.Right].mcu_device)
```

每台夹爪各自持有串口和线程;每台配一个独立的控制器。

### 序列号规则

```
TCGU01 A24 Z 0001 m
└─┬──┘ └┬┘ │ └┬─┘ └ 角色:m = 主爪(master),s = 从爪(slave)
 产品  批次 产线 流水号  末位:奇数 = 左,偶数 = 右
```

腕部相机(`XC…`)与所在夹爪的流水号相同,因此一个 `left` / `right` 能同时选中两者。

## 示例脚本

| 脚本 | 作用 | 对设备的影响 |
| --- | --- | --- |
| `follower_status.py` | 读取从爪状态 | 只读 |
| `leader_normalized_position.py` | 流式输出主爪开口度 0..1 | 只读 |
| `wrist_camera.py` | 查看 / 列出腕部相机(`--undistort`、`--list`) | 只读 |
| `read_intrinsics.py` | 以 JSON 导出标定数据 | 只读 |
| `impedance_control.py` | 位置跟随;`--show-envelope` / `--set-envelope` | 驱动电机 / 写 flash |
| `force_position_control.py` | 有界力抓取 | 驱动电机 |
| `gripper_console.py` | 键盘交互控制 | 驱动电机 |
| `control_and_read.py` | 控制同时读状态 | 驱动电机 |
| `control_ripple.py` | 测量跟踪质量 | 驱动电机 |
| `calibrate.py` | 编码器零点 + 行程 | 写 flash |
| `fisheye_cal.py` | 查看 / 设置鱼眼与编码器最大值记录 | 只读 / 写 flash |
| `ota_update.py` | 升级夹爪固件 | **刷固件** |
| `motor_ota_update.py` | 升级从爪电机固件 | **刷固件** |

详见 [docs/EXAMPLES.md](docs/EXAMPLES.md)。C++ 示例构建在 `build/cpp/examples/`。

## 固件

预编译镜像在 [`firmware/`](firmware/)。按**角色**选镜像(固件 SN 最后一个字符:`m` 主爪,
`s` 从爪)—— 刷错角色的镜像会让 MCU 变砖。

```bash
python python/examples/ota_update.py slave left     # 或:--all
```

**每次刷写后都要断电重启。** 从爪:拔掉 **24 V** 电源线约 2 秒(USB 可以不拔)。
主爪:重新插拔 USB。不断电的话,设备看起来正常但会丢状态帧。

`FollowerGripper` 拒绝打开固件低于 1.2.5 的从爪。主爪与从爪固件版本号相互独立。
电机固件要求:EL05 ≥ 1.0.5.0.4,RS00 ≥ 0.0.3.32。镜像列表、电机型号、RS00 配置与电机
升级见 [firmware/README.md](firmware/README.md)。

### 查看版本

`follower_status.py` 会打印夹爪固件、电机型号和电机自身固件版本,并对照最低要求。
该脚本只读:

```bash
python python/examples/follower_status.py left
# [fw] FirmwareVersion(1.2.16.0)
# [motor] MotorModel(RS00, id=1, from flash, t_max=14.0Nm, v_max=33.0rad/s)
# [motor fw] 0.0.3.32  (...)  OK
```

## 常见问题

反馈问题时,请附上每台从爪的 `follower_status.py` 输出,以及 `~/.taccaplogs/` 下最新的日志。

| 现象 | 处理 |
| --- | --- |
| `role=Unknown`、`firmware_sn` 为空、`Permission denied` | 把用户加入 `dialout`([第 1 步](#1-串口与相机权限)) |
| 插上后立即打开报 `Device or resource busy` | ModemManager 规则([第 2 步](#2-让-modemmanager-不要占用夹爪串口)) |
| `No space left on device`、相机打不开、丢帧 | 相机没用 MJPEG([第 3 步](#3-腕部相机必须使用-mjpeg)) |
| 刷固件后丢状态帧 | 断电重启(从爪:断 24 V) |
| 改了 C++ / 绑定但没生效 | 在使用方环境里重新安装 |
| 归一化开口度为 NaN 或报错 | 先标定:`calibrate.py`,见 [docs/CALIBRATION.md](docs/CALIBRATION.md) |

日志:每个进程都会把 DEBUG 日志写到 `~/.taccaplogs/session_*.log`(可用 `$TACCAP_LOG_DIR`
修改)。出问题时先看这个文件。

## 文档

- [docs/USAGE-zh.md](docs/USAGE-zh.md)([English](docs/USAGE.md))—— 夹爪、腕部相机、视触觉传感器的完整上手流程
- [docs/INSTALL.md](docs/INSTALL.md) —— 构建选项、纯 C++ 构建、环境陷阱
- [docs/CALIBRATION.md](docs/CALIBRATION.md) —— 编码器零点、行程、鱼眼内参
- [docs/EXAMPLES.md](docs/EXAMPLES.md) —— 每个示例脚本与控制器参数
- [firmware/README.md](firmware/README.md) —— 固件镜像、刷写、电机配置
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) —— 分层、线程模型、模块划分
- [CHANGELOG.md](CHANGELOG.md)

## 引用

```bibtex
@software{taccap_gripper,
  title   = {taccap-gripper: an SDK for the TacCap multimodal tactile gripper},
  author  = {{XenseRobotics Co., Ltd.}},
  year    = {2026},
  url     = {https://github.com/XenseRobotics-AI/TacCap-Gripper},
  version = {VERSION},
  license = {Apache-2.0}
}
```

把 `VERSION` 换成实际安装的版本(`xense.taccap.__version__`),并同时注明夹爪固件版本
(`g.firmware_version`)。

## 许可证

Apache-2.0。Copyright (c) 2026 XenseRobotics Co., Ltd.

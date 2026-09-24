# taccap-gripper

> 本文是 [README.md](README.md) 的中文翻译,英文版为准。

面向 **TacCap-Gripper** 的 C++17 / Python SDK —— 这是 XenseRobotics 的多模态
触觉数据采集夹爪。只有一个命名空间:C++ 里是 `xense::taccap::`,Python 里是
`xense.taccap`。

## 它是怎么搭起来的

**一套实现,两种语言。** 整个 SDK 都是 C++17;Python 包是在同一份代码之上的
pybind11 绑定,不是第二份移植。修一次两边都修好,你从 Python 上量到的行为就是
C++ 的行为。

上位机从不直接碰电机:它用 TC-GU-01 串口协议和夹爪的 MCU 通信,再由 MCU 通过
FDCAN 转发给 RobStride 电机。
所以本 SDK 里每一次 `Motor` 调用都是发给 MCU 的一条命令,而不是一次 CAN 写入
—— 这正是 MCU 能施加上位机绕不过去的限制的原因。

**四层,每一层都能单独用。** L1 是 TC-GU-01 线上协议 —— 字节填充组帧、CRC、
带类型的载荷编解码。L2 是异步传输层,它持有串口,按序列号把 ACK 匹配回对应
命令,并把 DATA 帧分发给按命令订阅的订阅者。L3 是带类型的组件(`Motor`、
`Encoder`、`IMU`、`Camera`、`Led`、`Calibration`、`Diagnostics`)。L4 把它们
聚合成 `LeaderGripper` / `FollowerGripper`,并加上后台控制器。你可以打开一个
`Transport` 直接收发帧,也可以打开一台夹爪、自始至终不见一帧。参见
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

**两个线程,而用户代码一个关键线程都不占。** 传输层的读线程只做读取、解析、
交付这三件事;订阅回调跑在另一个独立的分发线程上。这个拆分是承重的:Python
回调要拿 GIL,而一个卡住读线程的回调会卡住 `read()`,进而把内核 tty 缓冲冲满。
控制器再加一个线程,它每收到一帧电机状态就恰好提交一条命令 —— 写在已知 MCU
空闲的那个窗口里,而不是按一个自由运行的时钟去写。

**实时安全归固件管,上位机不再留第二份副本。** MCU 以 500 Hz 跑运动安全包络和
堵转判定,它是 MIT 命令通路上唯一一层谁都绕不过去的。所以本 SDK 没有上位机侧的
接触检测,也没有第二道堵转保护 —— 把控制器的力矩预算跑饱和,*那就是*接触。
两者一旦可能分歧,以设备为准。参见
[docs/CONTROL_LAYERING.md](docs/CONTROL_LAYERING.md)。

**驱动电机要通过控制器。** `ImpedanceController` 用来跟随位置,
`ForcePositionController` 用来抓取;两者都只暴露同样两个非阻塞调用 ——
`set_target(0..1)` 和 `snapshot()`。原始的 `submit_*` 电机原语仍然在,但它们是
裸的 MIT 帧,通路上没有任何上位机侧的保护。

**没有任何东西需要按单台配置。** 左右、主从角色和标定都是从设备上读出来的:
发现流程读的是固件烧录的序列号(绝不是 CH343 USB 芯片的序列号),鱼眼内参和
行程量程存在 MCU flash 里,所以一台夹爪换到哪个工位都自带自己的身份和标定。

**哪些是刻意不做的。** 视触觉(OG)采集属于 `xensesdk` wheel,不在这里 ——
`xense.taccap` 只覆盖夹爪协议加腕部相机。遥操作回路、抓取策略和 episode 录制都
属于上层应用;本 SDK 提供实时原语,不提供策略。腕部 `Camera` 默认不开也是同一个
道理:V4L2 设备通常被外部的相机服务占着,所以除非你传 `open_cameras=True`,
夹爪聚合类不会去打开它。

`lerobot-xense` 的一个 fork 通过 `taccap_gripper` 机器人类来消费本 SDK。它只是
*import* `xense.taccap`,没有重新实现任何设备访问;用本 SDK 并不需要它。

## 包含什么

- **TC-GU-01 协议** —— 异步传输层、ACK 匹配、按命令订阅 DATA、字节填充组帧。
- **`Motor`** —— 使能 / 失能 / 清故障,四种控制模式,阻塞的 `set_*` 和不等 ACK
  的 `submit_*`。
- **`ImpedanceController`** —— 跟随位置(遥操作、主从随动)。
- **`ForcePositionController`** —— 抓取,力和速度是仅有的两个旋钮。
- **归一化位置**,主从两种角色都有:`[0, 1]`,0 = 闭合,1 = 张开,单次读取和
  每一个流式样本上都带。
- **IMU、编码器、LED、按键**,以及上电自动标定。
- **腕部相机**,用本机自己存的内参做鱼眼去畸变。
- **`Calibration`** —— 持久化在 flash 里的鱼眼记录和编码器最大值记录。
  参见 [docs/CALIBRATION.md](docs/CALIBRATION.md)。
- **`Diagnostics`** —— 固件自己的 UART 计数器,能把「MCU 根本没发出去的帧」和
  「发出去却在路上丢了的帧」区分开。
- **OTA**,以及按固件烧录 SN 的零配置设备发现。

视触觉(OG)采集在 Python 层通过 `xensesdk` wheel 完成 —— `xense.taccap` 只暴露
夹爪协议 + 腕部相机这一层接口。

逐条提交的完整历史见 [CHANGELOG.md](CHANGELOG.md)。

---

## 固件

夹爪跑自己的固件,而本 SDK 要求固件足够新。

**`FollowerGripper` 拒绝打开固件低于 1.2.5 的从爪** —— 它是直接抛异常,而不是
给个警告,因为旧固件在控制通路上没有堵转保护,而且会把归一化的 0.0 放在闭合
限位以外的地方。`Config::allow_outdated_firmware=True` 可以在不驱动的前提下
查看这类设备。主爪不设这个门槛。里面的电机自己还有一条下限,见
[电机固件](#电机固件-10504-或更新版本)。

**可刷写的固件镜像放在 [`firmware/`](firmware/)** —— 固件源码不在。按夹爪的
角色选镜像,角色是固件 SN 的最后一个字符(`m` 主 / `s` 从),而不是看它装在
哪只手上:

```bash
python python/examples/ota_update.py slave left    # 角色选择器决定用哪个镜像
python python/examples/ota_update.py --all         # 所有已连接的夹爪
```

刷错角色的镜像会把 MCU 刷成砖,只能靠 SWD 探针恢复,所以刷之前先核对 SN。

**任何一次刷写之后都要断电重启。** 把 **USB 线和电源线同时拔掉**,然后再插
回去 —— 它们供的是不同的电域,只拔一根会让板子的另一半继续带电,复位不了。
bank-swap 重启只是一次软复位,它会让设备看上去一切正常,却在悄悄丢状态帧。

**主从两个角色的版本号是各自独立的。** 写这段时主爪是 1.2.4、从爪是 1.2.6,
谁也不比谁旧;一对夹爪的两半 `gripper.firmware_version` 读出不同的数是正常的。
**只在同一个角色内部比较版本号** —— 上面那些门槛都是从爪的数。另外,现场的主爪
还可能报 1.2.5 或 1.2.6,那是两个角色曾被强行同号的那段时期留下的,和 1.2.4 是
同一份代码;所以刷当前的主爪镜像会让它报的数变小,这既不会被拒绝,也不应该被
当成降级。

[`firmware/README.md`](firmware/README.md) 里有镜像对照表、CRC32 值,以及其余的
刷写细节。

### 电机固件 1.0.5.0.4 或更新版本

这说的是从爪**内部**那台 RobStride EL05,不是夹爪的 MCU,而且 SDK **不会**
强制检查它:读电机版本需要从爪固件 1.2.6+(`motor.motor_version()`,命令
`0x58`),而且到目前为止只在私有协议下证实能应答,所以开机检查没有可靠的
立足点。这是一条写在文档里的要求。

同一台设备升级前后的实测:在 1.0.5.0.2 上,速度反馈读出来是一个不跟随运动的
常值,运动纹波 117%;换到 1.0.5.0.4,同一台设备表现正常。因为这是同一台设备的
前后对照,所以不是机器间差异 —— 但因果未获证实,真正值得记住的是速度反馈被
破坏这件事:`snapshot().observation.velocity` 以及一切由它派生的量都会出错,
而表面上只是看着有点怪,不像坏掉。

电机固件只能用厂商自己的上位机工具加 USB-CAN 适配器升级,而且要把电机从夹爪上
拆下来;协议未公开。这件事没法在工位上脚本化,所以夹爪到手时就确认一遍,比事后
去排查便宜得多。

## 安装

```bash
mamba env create -f environment.yml && mamba activate xense-taccap
uv pip install -e . --no-build-isolation
python -c "import xense.taccap as t; print(t.__version__)"
```

`uv` 随环境一起装好,并且会自己认准当前激活的 conda 环境。用 `pip` 也行 ——
参数完全一样。

然后把检查挂上去,一次即可,这样它们在提交时就跑,而不是只在 CI 里跑:

```bash
pre-commit install
```

**为什么必须 `--no-build-isolation`。** `environment.yml` 把构建依赖
(`pybind11`、`scikit-build-core`)和 C++ 依赖(`libopencv`、`spdlog`)钉在
一起,这个参数的意思是「就按环境里钉住的版本来编」。不加它,安装器会在一个
用完即弃的环境里、对着从 PyPI 拉下来的 `pybind11` 编译 —— 实测确认过:此时
`pybind11_DIR` 指向 `~/.cache/uv/builds-v0/...` 而不是这个环境。pybind11 是
纯头文件库,构建时在场的是哪一份,编进扩展里的就是哪一份,而本代码库在这里对
版本敏感(见 `python/tests/test_numpy_views.py`)。

两个坑,每个都让人搭进去过一个下午:

- 要在**已激活**的环境里安装,不要用绝对路径调用。否则 cmake 会在 `PATH` 上
  找到一个其实是 GNU Make 的 `ninja`,然后报一个让人摸不着头脑的版本错误。
- C++ 或绑定层的改动**在你没有到那个消费方环境里重装之前是不生效的** ——
  editable 安装会把 Python 源码重定向到这个 checkout,但编译出来的扩展仍然是从
  `site-packages` 提供的。

前置依赖、设备权限、纯 C++ 构建、重新构建 / 清理,以及 `PYTHONPATH` /
`LD_LIBRARY_PATH` 的坑:**[docs/INSTALL.md](docs/INSTALL.md)**。

---

## 用法

`python/examples/` 下的十二个脚本是上手本 SDK 最快的路径:每一个都是把某一项
能力跑通的最小程序,同时也是我们自己做硬件 bring-up 用的工具。先把它们跑一遍,
再照着同样的调用写你自己的程序。

### 到处都是同一个选择器

每个脚本都接受同一个**位置参数**:

```bash
python python/examples/follower_status.py left      # 按左右
python python/examples/follower_status.py right
python python/examples/follower_status.py TCGU01A28Z0015s   # 按固件 SN
python python/examples/follower_status.py           # 只插了一台时可以省略
```

没有 `--side` / `--sn` / `--device` 这类开关 —— 一套整机上什么都有左右两份,
而「左右」是唯一一个对夹爪、它的腕部相机和它那对触觉传感器都表示同一件事的
抓手。左右来自**固件烧录的 SN**(`Cmd::GetSn`),绝不是 CH343 USB 芯片的 SN。
插了两台夹爪又不给参数时,脚本会直接拒绝,而不去猜你指的是整机的哪半边;
`calibrate.py` 则永远要求给出参数,因为它要写入。

### 按顺序把一台夹爪带起来

```bash
# 1. 它在不在,它是什么?  (只读)
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn, g.mcu_device)"

# 2. 读它的状态。每个字段在脚本头部都有说明。  (只读)
python python/examples/follower_status.py left

# 3. 配置固件的运动安全包络。出厂默认没有启用,
#    而它是 MIT 通路上唯一一层谁都绕不过去的保护。
python python/examples/impedance_control.py left --show-envelope
python python/examples/impedance_control.py left --set-envelope

# 4. 第一次运动,用 j/k/o/c 按键交互进行。
python python/examples/gripper_console.py left

# 5. 抓个东西。
python python/examples/force_position_control.py left --grasp-torque 1.1
```

第 3 步要排在第 4 步之前,这个顺序是承重的 —— 它防的是什么、以及默认为什么是关的,
见[运动安全包络](#运动安全包络)。

### 按任务分

**控制。** 两个控制器都只用同样两个非阻塞调用 —— `set_target(0..1)` 和
`snapshot()`:

```bash
python python/examples/impedance_control.py left                  # 跟随位置
python python/examples/impedance_control.py left --targets 1.0,0.5,0.0
python python/examples/force_position_control.py left             # 抓取:力矩有上限
python python/examples/gripper_console.py left --mode force-position
python python/examples/control_and_read.py left                   # 控制的同时读状态
python python/examples/control_ripple.py left --controller both   # 测量跟随质量
```

`control_and_read.py` 回答的是每个人第二个会撞上的问题:控制期间要读
`snapshot()` —— 绝不要用 `motor.read_status()`,它的 ACK 会在同一条串口链路上
和控制帧撞车。

**标定** —— 归一化的 `0..1` 位置要有意义,必须先做这一步:

```bash
python python/examples/calibrate.py left          # 主爪:编码器零位 + 行程量程
python python/examples/fisheye_cal.py show left   # 查看持久化在 flash 里的记录
python python/examples/read_intrinsics.py left --out cal.json    # 只读,输出 JSON
```

**相机与主爪:**

```bash
python python/examples/wrist_camera.py left --undistort
python python/examples/leader_normalized_position.py left
```

**固件:**

```bash
python python/examples/ota_update.py slave left   # 之后要同时拔掉 USB 和电源
```

### 每个脚本对设备做了什么

在一台出不得事的整机上跑任何东西之前,先看这一栏。

| | 脚本 |
|---|---|
| **只读** | `follower_status`、`read_intrinsics`、`wrist_camera`、`leader_normalized_position`、`fisheye_cal show` |
| **会让电机动** | `impedance_control`、`force_position_control`、`control_and_read`、`control_ripple`、`gripper_console` |
| **会写 flash** | `calibrate`、`fisheye_cal set-*`、`impedance_control` / `gripper_console` 上的 `--set-envelope` |
| **会刷固件** | `ota_update` —— 破坏性操作;之后要同时拔掉 USB 和电源,再一起插回去 |

### 两个共享模块,不能直接运行

`_target.py` 就是上面那个选择器,外加版本 / 颜色辅助函数,除 `wrist_camera.py`
之外每个脚本都会 import 它。`_calib_flow.py` 是带引导的标定流程,被那三个可能
碰到未标定设备的脚本使用。两者都刻意留在 `xense.taccap` 之外:它们会在 stdin
上提示输入,而一个会阻塞在 stdin 上的库会把每一个无头运行的使用方都搞坏。

每个脚本的细节,包括每个参数各自测的是什么,都在
**[docs/EXAMPLES.md](docs/EXAMPLES.md)**。C++ 示例默认会构建到
`build/cpp/examples/`(加 `-DTACCAP_BUILD_EXAMPLES=OFF` 可以跳过);wheel 构建
会自己把它们关掉。

## 写你自己的程序

上面那些示例只是下面这些调用的薄封装。**你手上是哪一半,决定了用哪套 API**:
*主爪*是手持的主端,你**读**它 —— 编码器、IMU、张开角度。*从爪*是带驱动的
夹爪,你**驱动**它 —— 一台藏在控制器后面的电机。角色由固件 SN 的最后一个字符
决定(`m` 主 / `s` 从),而不是看它装在哪只手上。

### 主爪 —— 读它

```python
import xense.taccap as t

g = t.LeaderGripper.open()          # 唯一连接的那台夹爪;只开 MCU,相机不开
g.start_streaming(imu_hz=100, encoder_hz=100)

enc = g.encoder.on_data(lambda s: print("enc", s.position_rad, s.position))
imu = g.imu.on_data(lambda s: print(s))
# ... 干活 ...
g.stop_streaming()
```

`LeaderGripper.open()` 在连接的夹爪不是恰好一台时会抛 `IoError` —— 双臂整机上
要显式指定设备(见下)。`s.position` 要有意义,这台设备必须标定过行程量程;见
[主爪的归一化位置](#主爪的归一化位置0--闭合1--张开)。

### 从爪 —— 驱动它

绝不要用原始的 `submit_*` 原语去驱动电机:它们是裸的 MIT 帧,通路上没有任何
上位机侧的保护。请用控制器。

```python
import xense.taccap as t

eps = t.find_follower()                       # 或 t.find_left() / t.find_right()
g = t.FollowerGripper(eps.mcu_device)

c = t.ForcePositionController(g)              # 抓取:力矩有上限
g.motor.clear_fault()
c.start()                                     # 必须先 START 再 ENABLE —— start()
g.motor.enable()                              # 会校验电机里存着的限值
try:
    c.set_target(0.0)                         # 0 = 闭合,1 = 张开;非阻塞
    while True:
        s = c.snapshot()                      # 一份一致的视图,不占总线
        print(s.state, s.observation.position, s.commanded_torque_nm)
        if s.holding or s.arrived:
            break
finally:
    c.stop()                                   # 力矩置零,然后会让电机失能
```

整个接口就两个调用:`set_target(0..1)` 和 `snapshot()`,都是非阻塞的,每帧都调
也没问题。想*跟随位置*而不是抓取时,换成 `t.ImpedanceController(g)` 即可 ——
还是这两个调用。`with t.ForcePositionController(g) as c:` 会替你配好 `start()` /
`stop()` 这一对。

第一次驱动一台从爪之前,先把固件的运动安全包络写一次;它出厂是关的。见
[运动安全包络](#运动安全包络)和[从爪控制](#从爪控制mit-力位混合)。

### 在一个进程里同时用左右两侧

```python
from xense.taccap import FollowerGripper, scan_grippers, Side

eps = scan_grippers()                          # 一次 USB 扫描,不会有重复探测的竞态
left  = next(e for e in eps if e.side == Side.Left)
right = next(e for e in eps if e.side == Side.Right)

g_left  = FollowerGripper(left.mcu_device)     # 按角色也可能是 LeaderGripper
g_right = FollowerGripper(right.mcu_device)
```

每台夹爪各自持有自己的串口链路和后台线程,所以两者互不影响 —— 但要给每台配
各自的控制器;绝不要让两个控制器指向同一台夹爪。

### 腕部相机

`open()` 不会碰相机:V4L2 设备通常被外部的相机服务占着。要用就显式要求,或者
用 `t.Camera` 单独打开它。

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True)
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

视触觉(OG)传感器通过 `xensesdk` wheel 读取,不走本 SDK。

上面这些的可运行版本,就是前面[用法](#用法)一节里的那些脚本。

### 序列号(TacCap SN 规则)

固件烧录的 SN 同时编码了左右和主 / 从角色:

```
  TCGU01 A24 Z 0001 m        夹爪         GSPS01 A24 Z 0001   视触觉传感器
  └─┬──┘ └┬┘ │ └┬─┘ │                                          (没有 patch 后缀)
 product batch│  seq patch    product : TCGU01 夹爪 / GSPS01 传感器
              line            line    : Z = 研发 / 测试,A = 量产
                              seq     : 末位数字为奇数 → 左,偶数 → 右
                              patch   : m = Master(主爪),s = Slave(从爪)
```

`scan_grippers()` 会为每台夹爪解析这个 SN;每个 `GripperEndpoints` 都带
`.side`(`Side.Left/Right`)和 `.role`(`Role.Leader/Follower/Unknown`)。
按左右**或**按角色挑一台:

```python
from xense.taccap import find_left, find_right, find_leader, find_follower, parse_serial

eps = find_leader()              # SN 的 patch 后缀是 'm' 的那台
p   = parse_serial("TCGU01A24Z0001m")
print(p.side, p.role, p.valid)   # Side.Left  Role.Leader  True
```

`parse_serial()` 会优雅降级:遇到老格式(`SN000002`)或空 SN,仍然会尽力给出
`side`(按末位数字),同时 `role = Role.Unknown`、`valid = False`。

### 编码器零位标定

```python
# 先把夹爪保持在想设为零位的姿态(通常是完全闭合)。
g_right.encoder.set_zero()                      # 收到 NACK 会抛 ProtocolError
s = g_right.encoder.read_once()
print(s.position_rad, s.raw_position_rad)       # 处理后的值(钳到 >= 0)与原始值
```

完整的交互式流程见 `python/examples/calibrate.py`(按 SN 选左右、标定前后的漂移
显示、全开角度合理性检查、实时读数)。

### 主爪的归一化位置(0 = 闭合,1 = 张开)

`normalize_position=True` 会在 open() 时读取固件的编码器最大值标定
(`Cmd::EncoderMaxCal`,需固件 ≥ V2.1),并把换算器装到编码器上,于是每个样本都
带一个落在 `[0,1]` 里的 `.position`:

```python
g = LeaderGripper(mcu_device=dev, normalize_position=True)

s = g.encoder.read_once()
s.position_rad      # 0.65  —— 永远是弧度,含义从不改变
s.position          # 0.50  —— 归一化值;开关关闭时为 float('nan')

g.position()        # 0.50  —— 单次读取 + 换算
g.pos_to_rad(1.0)   # 1.30  —— 完全张开,原始弧度
g.rad_to_pos(0.325) # 0.25

# 流式样本同样会被归一化,包括在映射存在之前就注册好的订阅者。
g.encoder.on_data(lambda s: print(s.position))
g.start_streaming(imu_hz=0, encoder_hz=100)
```

行程量程指的是完全张开时的编码器轴角,从编码器零位(完全闭合)算起 —— 所以要
**先给编码器置零**,再存量程。`measure-encoder-max` 会带你把这两步都走一遍:

```bash
python python/examples/fisheye_cal.py measure-encoder-max
```

如果量程从未标定过(固件会回 `CalNotSet`,而不是返回一个假的零值),或者固件
早于 V2.1,构造时会抛 `ProtocolError`。传 `encoder_max_rad=<rad>` 可以从上位机
直接给出量程,完全跳过读固件这一步。

`position()` / `pos_to_rad()` / `rad_to_pos()` / `position_map` 不开这个开关也
能用 —— 开关只决定 `EncoderSample.position` 会不会被填上。重新标定之后要调
`reload_position_map()`。

### 鱼眼相机标定

鱼眼内参 + 畸变参数存在 MCU flash 里,主爪和从爪都能读:

```python
from xense.taccap import CameraFisheyeCal, FisheyeUndistorter

cal = g.calibration.read_fisheye()     # 从未标定过时为 None
if cal is not None:
    # 优先用 FisheyeUndistorter,而不是自己拿 cal.K / cal.D 去手工调
    # cv2.fisheye.undistortImage:它只建一次 remap 表,用 INTER_CUBIC 重采样,
    # 并且采用和 PC 标定工具相同的焦距 balance,所以在这里矫正出来的一帧和
    # 在那边矫正出来的是一致的。
    # `cal.K` / `cal.D` 仍然暴露着,留给必须自己处理的代码。
    undistorter = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
    undistorted = undistorter.apply(img)

g.calibration.write_fisheye(CameraFisheyeCal(
    fx=320.5, fy=321.0, cx=319.5, cy=240.2,
    k1=-0.031, k2=0.0072, k3=-0.0013, k4=0.0002))
```

固件原样存储这些值 —— 不做单位换算,不做钳位,只拒绝 NaN/Inf。
`python/examples/fisheye_cal.py show` 会把两条记录都打印出来;
`set-fisheye --from-npz` 可以直接从 OpenCV 的标定文件里读 `K`/`D`。

### 从爪控制(MIT 力位混合)

从爪驱动的是一台 FDCAN 电机。控制用的是 **MIT 阻抗帧** —— 也就是力位混合
原语:`kp`/`kd` 项负责跟踪目标位置,前馈力矩叠加出力的分量。SDK 提供了三种
用法。

```python
import xense.taccap as t
g = t.FollowerGripper.open()
g.motor.clear_fault()
g.motor.enable()                 # 任何动作之前都必须先做

# 运动要走控制器。原始的 `submit_*` 原语也是暴露的,但它们会把控制帧直接写到
# 线上,没有误差钳位,也没有力矩上限 —— 这时固件包络就是你唯一的保护。
c = t.ImpedanceController(g)     # 默认值就是调好的那一套
c.start()                        # 用当前位置给目标打底
c.set_target(0.35)               # 归一化 [0,1],0 = 闭合
s = c.snapshot()                 # 状态 + 观测 + 指令,非阻塞
c.stop()
```

**归一化位置** —— 用 `[0, 1]`(0 = 闭合,1 = 张开)代替原始弧度。要求夹爪已经
标定过(`GripperConfig` 为 Valid),否则抛异常。原始弧度的电机控制在 Python 侧
不可达 —— 请用上面的控制器。

```python
print(g.position())                       # -> 0.97   (接近张开)
g.pos_to_rad(0.5), g.rad_to_pos(-0.59)    # 显式换算
```

一次性的 `FollowerGripper::set_position(0..1)` **只有 C++ 有** —— 在 Python 里,
归一化目标一律通过控制器的 `set_target()` 下发。原始的 `Motor.submit_*` 倒是暴露给
Python 了,参数也确实是弧度,但主机侧不对它们做任何钳位,见上面的警告。

**`ImpedanceController`** —— 要**跟随位置**就用这个。一个 C++ 后台线程
**与电机状态流同相位地**提交最新的归一化目标,同时这条流把一份线程安全的观测
保持在最新。你的策略只需要碰 `set_target(0..1)` 和 `snapshot()`,两者都是
非阻塞的(不抢 GIL,也不轮询状态)。

```python
c = t.ImpedanceController(g)              # 用 t.ImpedanceConfig() 调 kp/kd/预算
c.start()                                 # 目标先置为当前位置(不会突跳)
try:
    while running:
        s = c.snapshot()                  # 一次加锁,一份一致的视图
        if s.state == t.ImpedanceState.FAULT:
            break                         # s.fault_reason 说明原因;c.reset() 可恢复
        obs = s.observation               # .position [0,1], .velocity, .torque, .age_ms
        c.set_target(policy(obs))         # 你的动作,0..1
finally:
    c.stop()                              # 力矩置零,然后让电机保持失能
```

夹爪被挡住在这里不算故障:误差钳位会在 `max_position_torque_nm`(默认 1.1 Nm,
即 EL05 的连续堵转额定)上饱和并保持住。接触不需要检测;饱和就是接触的样子。

**`ForcePositionController`** —— 用来**抓取**的那一个。它在每条命令里带上夹持力
(`set_target(p, grasp_torque_nm)`),并报告 `holding`,所以被挡住的夹爪会稳定在
你要求的那个力上。

```python
fp = t.ForcePositionController(g)         # 默认值就是调好的那一套
fp.start()
try:
    fp.set_target(0.0)                    # 闭合;0 = 闭合,1 = 张开
    while True:
        s = fp.snapshot()
        if s.holding: break               # 已经夹住物体
        if s.arrived: break               # 到达目标,那里什么都没有
finally:
    fp.stop()                             # 下发零力矩并失能
```

**两个参数,而且只有这两个是跟任务相关的:**

| | 默认值 | 它是什么 |
|---|---|---|
| `grasp_torque_nm` | **设备自报的连续堵转额定** | 夹持力。空行程不用它;被挡住的夹爪会稳定在这个值上。EL05 上 1.1 Nm,RS00 上 3.6 —— 用 `ForcePositionConfig.for_spec(g.motor.get_spec())` 取,别写死 |
| `close_speed_radps` | **0.5 rad/s** | 行进速度 |

```python
cfg = t.ForcePositionConfig()
cfg.grasp_torque_nm = 0.4        # 更轻柔的夹持
fp = t.ForcePositionController(g, cfg)
```

其余的量对这台夹爪只有一个正确答案,所以没有暴露出来:位置增益、行进阻尼、
到位半径。它们是在这套硬件上实测出来的,不是猜的,改它们让夹爪发抖的可能性
远大于把它改好。

**是两个观测量,不是状态。** `snapshot().holding` 的意思是设定点已经在力预算
允许的范围内尽可能跑到夹爪前面去了,而夹爪没跟上 —— 夹住*本来就是*这么回事。
`snapshot().arrived` 的意思是它到达了指令位置。除此之外没有别的需要解读。

> **能夹多紧,能夹多久。** 1.1 Nm 是 EL05 的连续堵转额定。在 24 V 下对着真实
> 工件实测:一次 600 s 的持续夹持把绕组温度从 36 °C 带到 70 °C,升温速率从
> 8 °C/min 衰减到 1 °C/min,拟合下来平台在 75 °C 附近,比固件的温度墙低约
> 15 °C —— 没有故障,没有掉链接,没有漂移。几秒到几分钟的夹持完全在这个范围
> 之内。如果要按*几十分钟*计的长时间夹持,或者机柜偏热,就把力降下来:0.6 Nm
> 会平稳停在 49 °C。超出设备的连续包络之后,固件是降额而不是报错,并且
> `start()` 会给你警告。

**LED 与上电自动标定(V1.9):**

```python
g.led.set(t.Ws2812Mode.Override, 0, 255, 0, brightness=120)   # 常亮绿色
g.led.effect(t.Ws2812EffectType.ColorBreathe, 0, 0, 255)      # 蓝色呼吸
g.led.off()

cfg = g.get_auto_cal_config()             # 若启用,固件会在上电时自己找零位
g.set_auto_cal_config(cfg)                # (闭合到堵转),并记录 max_open
                                          # (张开到堵转)
```

### 运动安全包络

**先把运动安全包络写进去。** 它是固件的力矩与热保护,而在出厂设备上它是
**关着的**(`GripperConfig.reserved` 全为零)。在你写入它之前,它根本不存在:

```bash
python python/examples/impedance_control.py right --show-envelope   # 只读,不写
python python/examples/impedance_control.py right --set-envelope    # 修复,写 flash
```

**没有数要你来挑。** 设备自己知道它装的电机额定多少,而固件无论写进去什么都按那个
额定钳,所以 SDK 直接读(`0x56`)并推出这条记录:

| 字段 | 取值 | 含义 |
|---|---|---|
| `peak_torque_nm` | 电机的**额定**力矩(EL05 为 1.8 N·m) | 运动瞬态上限,以位置误差钳位的形式生效。**它同时决定接近速度**,大致是 `peak/kd` |
| `cont_torque_nm` | 电机的连续**堵转**额定(1.1 N·m) | 被挡住的爪子可以无限期维持的力矩,也是 I²t 降额的下限。**不是**额定力矩 —— 那是*旋转*额定,而夹爪的主工况就是堵住不放 |
| `temp_derate_start_c` | 0 → 固件取 90 °C | 热降额从哪里开始 |
| `temp_wall_c` | 0 → 固件取 100 °C | 温度墙;超过之后只剩 0.30 N·m |

**设备存的不等于它执行的。** 固件把 `cont` 钳到堵转额定,而这件事只记在一条没接到
USB 的 UART 上,读回来的则是 flash。我们台架上有一台存着 `cont=1.800`、按 `1.100`
跑了几周。`audit_envelope()` 把 `stored` / `effective` / `recommended` 分开报,
`effective` 为 `None` 表示固件什么都不执行;`ensure_envelope()` 负责修,只在需要时
写,并且**绝不放宽**特意收紧过的记录。

包络存在 `GripperConfig` 记录里(命令 `0x66`/`0x67`,**不改协议**),断电不丢,
每台设备写一次即可。它在固件的 MIT 分支里强制执行 —— 那是每条运动命令都必须
经过的唯一一个点 —— 所以**哪怕 MIT 帧不是由本 SDK 发出的,它同样成立**。这正是
它该放在固件里而不是放在这里的原因。

不写它的话,用 `kp=20` 把夹爪顶到刚性物体上,会要到大约 12 N·m;在 24 V 下这个
电流需求会把电源轨拉垮,电机的欠压保护动作,夹持随之丢失。启用包络之后,同样的
测试能稳稳保持 25 s。参见
[`docs/CONTROL_LAYERING.md`](docs/CONTROL_LAYERING.md)。

**两个值得知道的行为:**

- **夹持力不是恒定的。** I²t 降额和温度墙对纯力矩保持同样适用。
  `grasp_torque_nm` 是你要求的值;随着电机发热,固件会把实际输出降下来。
- **控制期间不要轮询 `read_status()`。** 锁相到数据流上保护的是遥测帧,不是
  ACK:请求可能正好落在 MCU 正在发送的窗口里而毁掉一帧遥测,而它自己的回复也
  可能被控制回路的提交破坏。位置、速度、力矩、状态位和温度全都在 `snapshot()`
  里 —— 没有哪一项需要动总线。

可运行的演示:`python/examples/impedance_control.py`、
`python/examples/force_position_control.py`,以及
`python/examples/gripper_console.py`(交互式,两个控制器都有)。

## 诊断

`g.diagnostics` 封装了固件自己的 UART 计数器(固件 1.1.3+)和一个运行时日志
开关(1.1.4+)。两者在两种夹爪角色上都能用。

```python
s = g.diagnostics.uart_stats()
# tx_calls_ok / tx_bytes_ok  —— 固件的发送调用接受了多少,
#                               也就是真正进到 MCU TX 寄存器里的量
# tx_fail_timeout            —— 固件自己把帧截断了
# rx_overflow                —— 固件的命令任务跟不上上位机
# debug_tx_bytes             —— 从 DEBUG UART 发出的字节数;用来量化日志开销
```

`tx_calls_ok` 的意义在于定责。把它和上位机在同一时间窗里解出来的量作对比:
如果计数对得上但字节少了,那丢失发生在字节离开 MCU **之后**(线缆或 USB 转
串口桥),改固件够不着它。

```python
from xense.taccap import LogLevel, LOG_OUTPUT_UART
g.diagnostics.set_log_config(LogLevel.DEBUG, LOG_OUTPUT_UART)   # 打开
g.diagnostics.disable_logging()                                  # 再关掉
```

> **日志是一个诊断手段,不是一项设置。** 固件 1.1.4 出厂就是关着的,因为日志
> 输出端是阻塞式的轮询 UART 写(921600 下每行约 0.5 ms),会把发出这行日志的
> 那个任务卡住 —— 当初就是「每收到一条命令都打日志」把命令通道搞成了活锁。
> 输出还是走 MCU 的 DEBUG UART,而它**没有引到 USB 上**:不在那个引脚上接
> 探头,你就是付了实时性的代价却什么也看不到。

## 日志

SDK 使用**一个单例 logger**,名字是 `"xense.taccap"` —— 注册在 spdlog 上,
所有 C++ 编译单元和 Python 绑定共用同一个。不要在别处自己构造
`std::make_shared<spdlog::logger>`,也不要用 `std::cout` / `print` / `printf`
来输出诊断信息 —— 它们绕过了文件 sink。

- **C++**:`#include <taccap/log.hpp>`,然后 `xense::taccap::logger()->info(...)`。
- **Python**:`from xense.taccap import log; log.info(...)` /
  `log.set_level("debug")` / `log.set_pattern(...)`。`set_level` 和
  `set_pattern` **只影响控制台 sink** —— 文件 sink 保持它的归档格式,以便
  grep 的结果长期稳定。

默认挂上两个 sink:

| Sink               | 级别                              | 格式                                  |
| ------------------ | --------------------------------- | ------------------------------------- |
| stderr(彩色)       | 用户可控,默认 `INFO`              | `[%D %T.%e] [%n] [%^%l%$] %v`         |
| 文件(每会话一个)   | 始终 `DEBUG`                      | `[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v` |

文件 sink 的行为:

- 目录:设置了 `$TACCAP_LOG_DIR` 就用它,否则用 `~/.taccaplogs/`。
- 文件名:`session_YYYYMMDD_HHMMSS.log` —— 每次进程启动新建一个。
- 最多保留 **10** 个会话日志;启动时按 mtime 把最旧的删掉。
- 文件 sink 创建失败(磁盘满 / 权限不足)不是致命错误 —— 控制台 sink 照常
  工作。

## 目录结构

```
taccap-gripper/
├── cpp/
│   ├── include/taccap/        # 公开的 C++ 头文件
│   ├── src/                   # SDK 实现(协议、总线、组件等)
│   ├── examples/              # C++ 示例程序(leader_demo)
│   └── tests/                 # gtest 单元测试
├── python/
│   ├── bindings/              # pybind11 模块源码
│   ├── examples/              # Python 示例
│   └── xense/taccap/          # Python 包(`xense` 下的 PEP 420 命名空间)
├── third_party/
│   └── firmware/              # 按需 clone 的参考仓库(已 gitignore)
│       ├── tc-gu-01/          #   跑在夹爪上的 STM32 固件
│       └── tc-gu-01-pc/       #   PyQt 调试 GUI(操作端)
├── docs/                      # 架构与 API 文档
├── environment.yml            # mamba 环境(Python 3.12,仅 conda-forge)
├── pyproject.toml             # scikit-build-core wheel 配置
└── CMakeLists.txt             # 顶层构建编排
```


## 文档

- **[docs/USAGE_CN.md](docs/USAGE_CN.md)** —— 端到端使用说明:把三条数据通路
  都带起来 —— 触觉(OG)、腕部相机,以及夹爪的读取 / 控制。英文版见
  [docs/USAGE.md](docs/USAGE.md)。
- **[docs/INSTALL.md](docs/INSTALL.md)** —— 前置依赖、纯 C++ 构建、设备权限、
  重新构建 / 清理、环境陷阱。
- **[docs/CALIBRATION.md](docs/CALIBRATION.md)** —— 编码器零位 + 行程量程、
  持久化在 flash 里的记录、漂移处理。
- **[docs/FIRMWARE.md](docs/FIRMWARE.md)** —— 参考仓库、固件构建,以及通过 OTA
  刷写。**刷写之后、测量任何东西之前,先读那条关于断电重启的说明。**
- **[docs/EXAMPLES.md](docs/EXAMPLES.md)** —— 每个示例脚本各自做什么。
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** —— 分层结构、模块地图、
  线程模型,以及本 SDK 与下游使用方之间的边界。

## 引用

如果本 SDK 支撑了公开发表的工作,请引用你**实际使用的那个版本** —— 文档描述的
行为是跟版本走的,好几个默认值在不同版本之间变过。

```bibtex
@software{taccap_gripper,
  title        = {taccap-gripper: an SDK for the TacCap multimodal tactile gripper},
  author       = {{XenseRobotics Co., Ltd.}},
  year         = {2026},
  url          = {https://github.com/XenseRobotics-AI/TacCap-Gripper},
  version      = {VERSION},
  license      = {Apache-2.0}
}
```

把 `VERSION` 换成你构建时用的 tag —— `python -c "import xense.taccap as t;
print(t.__version__)"` 会打印当前装的是哪个版本。固件同样要写:一次测量取决于
夹爪固件的程度不亚于取决于本 SDK,所以把固件版本也一并注明
(`g.firmware_version`)。

## 许可证

Apache-2.0。Copyright (c) 2026 XenseRobotics Co., Ltd.

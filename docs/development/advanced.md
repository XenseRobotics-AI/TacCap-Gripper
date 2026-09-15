# 高级接口 —— 多传感器与原生控制器

> 日常从爪操作请从 [Gripper 客户指南](../USAGE.md) 开始。以下保留已有集成路径；
> 固件 1.1.8 / 1.1.7 的超时和包络行为以 [当前控制契约](../CONTROL_LAYERING.md) 为准。

<!-- 面向"第一次要把设备跑起来"的人:触觉、视觉、夹爪读数与控制,各一节,
     每节都是从插上线到拿到数据的完整路径。API 细节见 README 与各专题文档。 -->

一台 TacCap-Gripper 上有三路互相独立的数据,**分别由不同的软件栈负责**,
这是本文档最重要的一件事:

| 数据 | 硬件 | 由谁负责 | 设备节点 |
| --- | --- | --- | --- |
| **夹爪** 读数与控制 | STM32 MCU(IMU / 编码器 / 电机) | **本 SDK** `xense.taccap` | `/dev/serial/by-id/...-if02` |
| **视觉** 腕部相机 | XC 系列 UVC 相机 | **本 SDK** 的 `Camera`(可选) | `/dev/video*` |
| **触觉** 视触觉(OG)传感器 | GSPS01 视触觉模组 | **`xensesdk` wheel,不在本 SDK 内** | `/dev/video*` |

三者没有共享的会话:开一路不需要开另一路,某一路坏了也不会拖垮其它两路。
夹爪走串口,两个相机走 USB 视频,互不抢占同一个句柄 —— 但它们**共享 USB 带宽**,
见[三路一起跑](#4-三路一起跑)。

> **示例脚本一律用位置参数 `left` / `right` 选设备**(或直接给序列号),没有
> `--sn` / `--side` / `--device` 之类的开关 —— 见
> [示例选择器说明](../EXAMPLES.md)。

前置的安装、设备权限、`PYTHONPATH` / `LD_LIBRARY_PATH` 陷阱见
[docs/INSTALL.md](../INSTALL.md)。下文假定 `taccap` 环境已激活、
`import xense.taccap` 能成功。

---

## 0. 先确认设备在

三条线各有各的枚举方式,**先分别确认,再写业务代码** —— 大多数"读不到数据"
最后都是设备没被认出来。

```bash
# 夹爪(MCU):按固件烧录的 SN 发现,不依赖 udev 规则
python -c "from xense.taccap import scan_grippers, Side
for g in scan_grippers():
    s='L' if g.side==Side.Left else 'R'
    print(f'  [{s}] {g.role} ch343={g.mcu_serial} fw_sn={g.firmware_sn!r}')"

# 视觉(腕相机)+ 触觉(OG):都在 /dev/v4l/by-id 下,按序列号区分
python python/examples/wrist_camera.py --list

# 触觉(OG):由 xensesdk 自己扫,返回 {序列号: cam_id}
python -c "from xensesdk import Sensor; print(Sensor.scanSerialNumber())"
```

健康的输出:夹爪那条打出 `[L]` / `[R]` 且 `fw_sn` 非空;`--list` 那条把腕相机
(`XC…`)和视触觉(`GSPS01…`)分开列出;OG 那条打出形如 `{'OG000352': 10}` 的
字典。`fw_sn` 为空说明固件还没烧 SN(或固件早于 V1.6),此时侧别会是
`Side.Unknown`。

> **三类设备靠序列号区分,不靠设备号。** `/dev/videoN` 的编号随插拔顺序变,而且
> 腕相机和视触觉挨着枚举 —— 认错了就会把触觉传感器当相机打开。序列号语法和
> 夹爪固件 SN 是同一套(**序号末位单左双右**,`m` = leader / `s` = follower):
>
> | 设备 | 语法 | 例 |
> | --- | --- | --- |
> | 夹爪 | `TCGU01<批次><产线><序号><m\|s>` | `TCGU01A28Z0116m` |
> | 腕相机 | `XC<批次><产线><序号><m\|s>` | `XCA28Z0116m` |
> | 视触觉 | `GSPS01<批次><产线><序号>` | `GSPS01A31Z0049` |
>
> 腕相机的序号和它所在夹爪的序号一致,所以 `left` / `right` 一个选择器就能同时
> 定位夹爪和它的腕相机。

> **腕相机不在发现结果里,这是设计使然。** 发现层只认 MCU:UVC 设备通常由外部
> 相机服务持有,SDK 不去抢。所以夹爪对象默认 **不打开** 腕相机
> (`open_cameras=False`),要用就自己给设备路径。

---

## 1. 夹爪:读数与控制

### 1.1 打开

```python
import xense.taccap as t

g = t.LeaderGripper.open()        # 恰好插了一台时;插了多台会抛 IoError
```

多台的场合别用 `open()`,一次扫描拿全再按侧别/角色挑,避免重复探测:

```python
from xense.taccap import LeaderGripper, FollowerGripper, scan_grippers, Side, Role

eps   = scan_grippers()
left  = next(e for e in eps if e.side == Side.Left)
lead  = next(e for e in eps if e.role == Role.Leader)
g     = LeaderGripper(mcu_device=left.mcu_device)
```

leader 与 follower 是**两种硬件**,由 SN 尾缀决定(`m` = leader,`s` = follower),
类只是决定你拿到哪套命令面 —— 用错类不会自动纠正,只会在发命令时 NACK。

### 1.2 读数:一次性 vs 流式

一次性读(阻塞,等 ACK),适合标定、自检这类低频场合:

```python
s = g.encoder.read_once()
print(s.position_rad, s.raw_position_rad)   # 熟化值(钳到 >= 0) vs 原始值
print(g.imu.read_once())
```

流式(固件按固定频率推,回调在后台线程里跑),这是采数据的正路:

```python
sub_e = g.encoder.on_data(lambda s: print("enc", s.position_rad))
sub_i = g.imu.on_data(lambda s: print("imu", s.accel_mps2, s.gyro_radps))

g.start_streaming(imu_hz=100, encoder_hz=100)   # leader:两路都可单独关(0)
...
g.stop_streaming()
```

follower 只有一路可流:**电机状态**。IMU/编码器在这个角色上被固件编译掉了,
所以签名不同,`motor_hz=0` 会直接抛 `IoError(EINVAL)`,而不是开一条空流:

```python
f = t.FollowerGripper.open()
subscription = f.motor.on_status(lambda s: print(s.actual_pos, s.actual_torque))
f.start_streaming(motor_hz=100)     # 固件把 1 kHz 整除,只有 1000 的因数能精确命中
```

> 频率不是随便填的:固件用 1 kHz 整除,非因数的频率会被它悄悄调整;
> SDK 检测到这种情况会打 warning,但不会 NACK。

### 1.3 归一化开口度(0 = 闭合,1 = 张开)

原始编码器是弧度,策略/数据集通常要 `[0, 1]`。打开 `normalize_position=True`
后,一次性读和每一个流式样本都会带上 `.position`:

```python
g = t.LeaderGripper(mcu_device=dev, normalize_position=True)
s = g.encoder.read_once()
s.position_rad     # 0.65  —— 永远是弧度,含义不变
s.position         # 0.50  —— 归一化;没开这个标志时是 nan
g.position()       # 0.50  —— 一次性读+换算
```

这依赖固件里存的 **行程上限**(`Cmd::EncoderMaxCal`,固件 ≥ V2.1)。没标定过时
构造会抛 `ProtocolError`(固件答 `CalNotSet`,而不是给个假的 0)。标定顺序是
**先置零,再存行程**:

```bash
python python/examples/calibrate.py left            # 交互式:置零 + 存行程
python python/examples/fisheye_cal.py measure-encoder-max   # 引导零点与行程流程
```

细节见 [docs/CALIBRATION.md](../CALIBRATION.md)。

### 1.4 从爪原生控制器（兼容与调试）

日常位置控制使用 [Gripper](../USAGE.md)。下列原生接口面向已有集成、控制策略调试与 C++：

| 接口 | 语义 |
|---|---|
| `ControlLoop.set_target()` | 旧阻抗控制器，存在自身的受阻 / stall-clamp 行为；不等同于新持续位置跟随 |
| `ForcePositionController.set_target()` | 旧接触策略：闭合、推断接触、纯前馈保持 |
| `ForcePositionController.set_position_target(position, torque_limit_nm, speed_rad_s)` | 新位置分支；受阻不撤销目标；客户 `Gripper.move/set_position` 调用此方法 |
| `set_grasp_target()` / `set_impedance_target()` / `hold_position()` | 显式策略切换，客户 facade 在同一运行时上封装这些方法 |
| `snapshot()` / `observation()` | 控制期间读缓存，避免高频同步轮询串口 |

两个控制器共用 `detail::ControlRuntime` 的生命周期与控制权机制。
同一个原生 Motor 对象不可被两个控制器同时持有，但这不是跨进程互斥。
Python 不重新开放原始 `motor.set_position/set_velocity/set_torque/set_impedance/submit_*`；
C++ 内部原语仍保留，单次提交不等于持续控制。

以下完整示例只演示旧原生接触策略，会真实闭合；新项目不必照此管理控制器：

```python
from xense.taccap import advanced as t

followers = [e for e in t.scan_grippers() if e.role == t.Role.Follower]
if len(followers) != 1:
    raise RuntimeError("需要唯一从爪，或按固件 SN 显式选择")
with t.FollowerGripper(mcu_device=followers[0].mcu_device) as device:
    cfg = t.ForcePositionConfig()
    cfg.grasp_torque_nm = 0.35
    cfg.monitor_execution = True  # 固件 >= 1.1.7；原生默认 False 用于旧版本兼容
    controller = t.ForcePositionController(device, cfg)
    input("清空运动区域，按 Enter 使能并闭合：")
    try:
        controller.start()
        controller.set_target(0.0)
        input("控制保持活跃；支撑物体后按 Enter 失能退出：")
    finally:
        controller.stop()
```

`start()` 已负责使能 / 初始化；`stop()` 请求失能。
不要在示例之外同时启用另一个控制器或手动启停相同数据流。
`monitor_execution=True` 在 1.1.8 使用扩展流，在 1.1.7 使用同步兼容查询。

`ForcePositionConfig` 完整可配置字段（与当前绑定对应）：

| 字段 | 默认值 | 含义 |
|---|---|---|
| `close_position` | 0.0 | 旧接触策略闭合端点，归一化 |
| `close_speed_radps` | 0.5 | 参考闭合速率 rad/s |
| `grasp_torque_nm` | 0.35 | 接触后请求的纯前馈扭矩 Nm |
| `hold_torque_limit_nm` | 1.2 | 原生保持请求上限 Nm，不是无限期热安全承诺 |
| `motion_torque_limit_nm` | 5.5 | 原生运动请求上限 Nm；客户 `Gripper` 当前仍限制到 1.2 |
| `contact_torque_nm` | 0.080 | 接触判断的扭矩下限 Nm，不是本次预算比例 |
| `contact_vel_radps` | 0.035 | 停止判断绝对速度门限 rad/s |
| `contact_vel_ratio` | 0.25 | 已发生运动后使用的速度比例门限 |
| `contact_moved_rad` | 0.010 | 启用比例判断所需运动距离 |
| `position_kp` / `position_kd` | 20.0 / 1.0 | 位置策略基础增益 |
| `close_compensation_rad` | 0.0 | 只对显式位置策略应用，最大 0.05，固件 >= 1.1.8 |
| `brake_distance_rad` | 0.10 | 旧接触策略近端制动距离 |
| `contact_samples` | 3 | 连续确认反馈数 |
| `startup_guard_ms` | 250 | 闭合启动接触判断保护窗口 |
| `status_timeout_ms` | 350 | 反馈超时 |
| `motor_stream_hz` | 100 | 电机状态流配置频率 |
| `monitor_execution` | False | 是否监控固件执行状态；客户 Gripper 自动开启 |

### 1.5 固件配置与维护

先停止并关闭运动程序，再建立维护连接。只读示例（路径换为发现的设备）：

```python
from xense.taccap import advanced as t

with t.FollowerGripper(mcu_device="/dev/serial/by-id/实际设备路径") as device:
    print(device.firmware_version)
    print(device.get_gripper_config())
    print(device.get_auto_cal_config())
    print(device.get_envelope())
    print(device.motor.get_startup_limit_torque())
    print(device.motor.execution_status())  # 固件 >= 1.1.7
```

| 维护能力 | 接口 / 注意点 |
|---|---|
| 行程与安装方向 | `get_gripper_config()` / `set_gripper_config()`；写入必须保持完整记录，见[标定](../CALIBRATION.md) |
| 上电自动标定 | `get_auto_cal_config()` / `set_auto_cal_config()`；可能改变下一次上电的运动 |
| 部分标定参数 | `set_auto_cal_stall_param()`；保留其他字段的定向写入 |
| 包络 | `get_envelope()` / `set_envelope()`；维护时持久化，不在每个控制周期写 |
| 电机启动限扭 | `motor.get_startup_limit_torque()` / `set_startup_limit_torque()`；存 MCU Flash，下次完整上电写入电机 |
| 执行诊断 | `motor.execution_status()`、`control_stats()`、`read_status_ext()`、`fault_report()`；先用 `help()` 核对对应版本可用方法 |
| 固件升级 | `OtaSession` 或 [OTA 脚本](../FIRMWARE.md) |

固件 >= 1.1.7 未写包络时也启用 RAM 默认 cont=1.0 / peak=1.5 Nm；
不能再以 `get_envelope().flags==0` 推断“无保护”。有效持久化配置会覆盖默认值。
启用记录需 `0 < cont <= 1.2`、`cont <= peak <= 5.5`；这不是推荐工作点。
旧 1.8 Nm cont 记录不符合当前契约，需维护人员按机构能力迁移。

原生启动会读取设备启动限扭：高于所配运动上限会拒绝，低于上限会提示设备上限优先。
不要为消除 3.0 < 5.5 的提示把设备额度抬高。历史 `6.0` 协议数值已修正为 `5.5`，
但也不要将 5.5 当作默认夹持额度。所有外部模式仍受固件包络影响。

---

## 2. 视觉:腕部相机

腕相机是普通 UVC 设备,本 SDK 用 `Camera` 类(底层 `cv::VideoCapture`)读它。
**默认没人替你打开它** —— 两种开法:

### 2.1 独立开(推荐,和夹爪解耦)

```python
from xense.taccap import Camera, ColorMode

cam = Camera(device="/dev/video2", width=640, height=480, fps=30.0,
             use_mjpg=True, color_mode=ColorMode.BGR)

frame = cam.read(timeout_ms=500)      # 同步一次性读;失败返回 None
print(frame.frame_index, frame.image.shape)

cam.start(lambda f: handle(f.image))  # 或者异步:后台线程回调
...
cam.stop()
```

设备路径优先用 `/dev/v4l/by-id/...-video-index0` 这种稳定路径,`/dev/videoN`
的编号会随插拔顺序变。用 `python python/examples/wrist_camera.py --list` 列。

### 2.2 挂在夹爪对象上

```python
g = t.LeaderGripper(mcu_device, wrist_video="/dev/video2", open_cameras=True,
                    undistort_wrist=True)     # 帧出来就是矫正过的
g.wrist_camera.start(lambda f: print("wrist", f.frame_index))
```

> **通道顺序有个刻意的不一致**:裸 `Camera` 默认 **BGR**(OpenCV 原生,`imshow`/
> `imwrite` 直接对);夹爪的 `wrist_camera` 默认 **RGB**(喂视觉/学习管线,
> LeRobot 数据集存 RGB)。两边都能用 `color_mode` / `wrist_color_mode` 改回去。
> 搞反了不会报错,只会录进去一份通道颠倒的数据。

### 2.3 鱼眼去畸变

腕部是鱼眼镜头,内参存在 MCU flash 里(`Cmd::CameraFisheyeCal`,固件 ≥ V2.0)。
`FisheyeUndistorter` 把内参编译成一次性的 remap 表,之后每帧只做重采样:

```python
from xense.taccap import FisheyeUndistorter

cal, is_reference, reason = g.calibration.resolve_fisheye()
if is_reference:
    log.warning(f"用的是 SDK 参考内参,不是这一台的:{reason}")

undist = FisheyeUndistorter(cal, width=640, height=480, balance=0.0)
cam.set_undistorter(undist)      # 装进采集路径:read() 和回调拿到的都是矫正帧
# 或者手动:rect = undist.apply(img)
```

三个必须知道的约束:

- **读内参用 `resolve_fisheye()`,不要用 `read_fisheye()`。** 没标定过的机器
  不是 NACK,而是返回一条**全零**记录,它能过 `if cal is None` 检查,但
  `fx = fy = 0` 会把每个像素映射到画面外 —— 得到一张纯黑的"矫正帧",全程无报错。
  `resolve_fisheye()` 已经把这个策略(读 → 判可用 → 回退参考值)做在一处了。
- **只支持 640×480**,即标定分辨率。固件记录里不带图像尺寸,所以缩放内参等于
  猜,构造函数宁可抛错。要别的分辨率就得先在固件里存一份对应的标定。
- **参考内参是近似的**。每台的镜头装配位置有差异(尤其主点),回退到
  `FISHEYE_FALLBACK_CAL` 只是比完全不矫正强,不能拿它去按像素做测量。
  自己标定后用 `fisheye_cal.py set-fisheye --from-npz cam.npz` 写进 flash。

`balance` 是取景口味:0 = 保持标定焦距(自然视角,和 PC 工具默认一致),
1 = 焦距压到 0.70x 换最大视场,代价是四周更多黑边。

> 画面看起来偏心或轻微倾斜,**不一定是标定错了** —— 传感器未必正好落在镜头
> 光轴上,矫正是围绕主点做的,不是围绕画面中心。

### 2.4 一条命令看效果

```bash
python python/examples/wrist_camera.py right                 # 原始鱼眼(默认)
python python/examples/wrist_camera.py right --undistort     # 矫正
python python/examples/wrist_camera.py right --compare       # 左右对照
python python/examples/wrist_camera.py XCA28Z0116m           # 也可以直接给序列号
python python/examples/wrist_camera.py right --no-mcu --no-display \
    --duration 10 --save-dir /tmp/shots                      # 无头,存图
```

选择器就是 `left` / `right`,和其它示例一致 —— 内参会自动去**同侧**的夹爪上读。
**去畸变默认关**,和 SDK 本身一致(裸 `Camera` 不带 undistorter,夹爪的
`undistort_wrist` 默认 `False`);窗口模式下即使起在 raw 也会先把内参读好,
因为 `u` 随时可能切到矫正。无头 + raw 是唯一不会去碰夹爪的组合。
窗口里 `u` 键在 原始 / 矫正 / 对照 之间循环切,`[` `]` 调 balance,`s` 存图。
没插夹爪时加 `--no-mcu` 走参考内参;`--from-npz` 用离线标定文件。完整参数 `--help`。

> **这个脚本打不开视触觉传感器,是故意的。** 传 GSPS 序列号或 `/dev/videoN` 路径
> 都会被拒绝并说明原因 —— OG 的采集和矫正在 `xensesdk` 里,不走本 SDK(见下一节)。

---

## 3. 触觉:视触觉(OG)传感器

**这部分不在本 SDK 里。** `xense.taccap` 只覆盖夹爪协议和腕相机;OG 的采集、
矫正、力/深度推理都在 `xensesdk` wheel 里(本机版本 2.1.1)。下面给的是把它
跑起来的最短路径,接口以你装的那版 wheel 为准。

```python
from xensesdk import Sensor

print(Sensor.scanSerialNumber())     # {'OG000352': 10, 'OG000344': 8} —— 序列号: cam_id

sensor = Sensor.create("OG000352")   # 也接受 cam_id;第一次会建配置缓存,较慢
try:
    # 一次要多个输出就传多个,返回值按顺序一一对应
    rectify, depth, force = sensor.selectSensorInfo(
        Sensor.OutputType.Rectify,      # 矫正后的图像
        Sensor.OutputType.Depth,        # 深度图,单位 mm
        Sensor.OutputType.Force,        # 力分布
    )
finally:
    sensor.release()
```

常用的 `OutputType`(完整列表 `dir(Sensor.OutputType)`):

| 输出 | 含义 |
| --- | --- |
| `Rectify` | 矫正后的图像 |
| `AugDifference` / `Difference` | 与参考帧的差分图(接触可视化) |
| `Depth` | 深度图,mm |
| `Force` / `ForceNorm` / `ForceResultant` | 力分布 / 法向力 / 合力(6 维) |
| `Marker2D` / `Mesh3D` / `Mesh3DFlow` | 标记点与三维网格及其流场 |

实践上要注意的几点:

- **只要图像、不要推理时加 `disable_infer=True`**,可以省掉推理引擎的加载;
  `Rectify` 和 `Difference` 本来就不需要推理。
- **首次 `create()` 慢**是在建每序列号的配置缓存(`~/.xensesdk/config`);
  多传感器场景建议先预热缓存再并发开,否则会撞在一起。
- **OG 和腕相机都是 `/dev/video*`**,但腕相机**不在** `Sensor.scanSerialNumber()`
  的结果里 —— 它不是 OG 设备,别指望用触觉 SDK 去枚举它。反过来也一样:
  `wrist_camera.py` 会拒绝 GSPS 序列号。两边都按序列号语法把对方挡在门外。
- 力/网格类输出形状是固定的(力分布 `(35, 20, 3)`、合力 `(6,)`),
  图像类的形状随 `rectify_size` 变。

要看完整的产品级用法(异步读、配置覆盖、多传感器编排),参考
`lerobot` 侧的 `lerobot/cameras/xense/camera_xense.py`。

---

## 4. 三路一起跑

各自独立,但同时开有几个已知的坑:

- **USB 带宽是共享的。** 两个 OG + 一个腕相机同时 MJPG 满帧,已经吃掉相当一部分
  总线;夹爪走的是串口,不受影响,但相机之间会互相挤。真要满配采集,
  先测一遍实际帧率,别假设标称值。
- **谁持有 UVC 设备。** 生产环境里外部相机服务持有 `/dev/video*`,这时
  **不要**再用 `open_cameras=True` 或裸 `Camera` 去开同一个节点 —— 会失败或抢到
  半路。SDK 默认不开相机,就是为了这个。
- **控制回路要用 `ControlLoop`(`STREAM_LOCKED`)。** 上面的相位说明在满负载下
  尤其重要:实测就是在"所有相机都在流 + 电机在往复"的条件下做的。
- **日志。** 全 SDK 一个单例 logger(`xense.taccap.log`),控制台默认 INFO,
  文件 sink 恒为 DEBUG,落在 `~/.taccaplogs/session_*.log`(可用 `$TACCAP_LOG_DIR`
  改),最多留 10 份。出问题先翻这个文件,里面有控制台被过滤掉的那些行。
- **固件刷完必须断电重插。** bank-swap 之后是软复位,设备看起来完全正常
  (版本对、流在跑、计数干净),但会静默丢状态帧。见 [docs/FIRMWARE.md](../FIRMWARE.md)。

---

## 出问题时

| 现象 | 多半是 |
| --- | --- |
| `scan_grippers()` 返回空 | 串口权限(见 [INSTALL.md](../INSTALL.md))或线没插好 |
| `fw_sn` 为空 / `Side.Unknown` | 固件没烧 SN,或固件早于 V1.6 |
| 矫正后画面全黑 | 用了 `read_fisheye()` 的全零记录 —— 改用 `resolve_fisheye()` |
| 矫正后画面偏心 | 未必是错的,主点本来就不一定在画面中心 |
| 录下来的颜色是反的 | 裸 `Camera` 是 BGR、`wrist_camera` 是 RGB,搞混了 |
| 电机状态帧成片丢失 | 提交相位(用 `ControlLoop`),或刷完固件没断电重插 |
| 改了 C++/bindings 但 Python 里没生效 | 消费端的环境没重装,见 [INSTALL.md](../INSTALL.md) |
| `import xense.taccap` 报 `libopencv_core.so` | 缺 `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` |

更细的专题:[INSTALL.md](../INSTALL.md) · [CALIBRATION.md](../CALIBRATION.md) ·
[FIRMWARE.md](../FIRMWARE.md) · [EXAMPLES.md](../EXAMPLES.md) ·
[ARCHITECTURE.md](../ARCHITECTURE.md)

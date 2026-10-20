# 示例脚本

`python/examples/` 下 13 个可运行脚本的参考手册:每个脚本做什么、对设备做什么,
以及力位混合控制器的调参说明。按顺序的上手路径见 README 的
[Quick start](../README.md#quick-start)。

C++ 示例默认构建进 `build/cpp/examples/`,`-DTACCAP_BUILD_EXAMPLES=OFF` 可跳过;
wheel 构建不包含它们。

> [!WARNING]
> **不要直接用电机原语控制。** `submit_position` / `submit_velocity` /
> `submit_torque` / `submit_impedance` 是裸 MIT 帧,没有主机侧的误差钳位和力矩上限。
> 拿它们顶住硬物体时,`kp × 位置误差` 会一直涨到电机自身上限(EL05 6 N·m /
> RS00 14 N·m),24 V 母线被拉垮、报 `EN|FAULT|UNDER_VOLT`,之后必须重新上电。
> 请使用下面的两个控制器。

## 选择设备:统一用 `left` / `right`

所有需要指定设备的示例都用**同一个位置参数**:`left` / `right`,或直接给序列号。

```bash
python python/examples/calibrate.py right
python python/examples/fisheye_cal.py show right
python python/examples/leader_normalized_position.py right
python python/examples/wrist_camera.py right
python python/examples/ota_update.py master right
python python/examples/ota_update.py --get-status right
```

只接了一台时可以省略;`calibrate.py` 例外,它会写设备,所以必须显式指定。
接了多台又没指定时,脚本会报错而不是猜。左右一律来自**固件烧录的 SN**,腕部相机
的流水号与所在夹爪一致,所以 `left` 在任何脚本里都指同一侧设备。C++ 示例使用同样的
选择规则。

`python/examples/` 下有两个下划线开头的共享模块,不能单独运行:

- **`_target.py`** —— 上述设备选择器,以及版本显示和终端颜色。
- **`_calib_flow.py`** —— 引导式行程标定流程(闭合取零 → 张开取跨度 → 写 flash),
  由 `calibrate.py` / `fisheye_cal.py` / `leader_normalized_position.py` 使用。
  它会在 stdin 上等待输入,所以不放进 `xense.taccap` 库。

## 脚本一览

上真机前先看「对设备做什么」一列。

### 控制

| 脚本 | 对设备做什么 | 说明 |
|---|---|---|
| `impedance_control.py` | 动电机 | `ImpedanceController` 用法:`set_target(0..1)` + `snapshot()` 走一遍开合序列。也是**运动安全包络**的配置入口(`--show-envelope` / `--set-envelope`)。|
| `force_position_control.py` | 动电机 + 夹持力 | `ForcePositionController` 用法。一条有界力矩控制律,饱和即接触;闭合端点有小的前馈预压。输出中 `cmd` 与实测力矩之差就是固件的热降额。|
| `gripper_console.py` | 动电机 + 夹持力 | 键盘控制台,`--mode impedance` / `--mode force-position` 两种控制器共用一个界面,内置包络配置。表头显示包络是否生效(`ENFORCED` / `*** INACTIVE ***`)。|
| `control_and_read.py` | 动电机 | **边控制边读**:控制器运行时从 `snapshot()` 读状态,不发任何命令。控制期间调用 `read_status()` 会让 ACK 和控制帧在串口上冲突。|
| `control_ripple.py` | 动电机 | 测量控制质量:行进段速度纹波、峰峰值、均速/命令速度比、逆向帧。两个控制器都能测。|

### 状态与诊断

| 脚本 | 对设备做什么 | 说明 |
|---|---|---|
| `follower_status.py` | 只读 | 夹爪固件、电机型号与电机固件版本(对照最低要求);从爪状态:一次性读取(位置/速度/力矩/温度/状态位/故障)、归一化开口度、流式速率。用固件的 `status_timestamp_ms` 判断数据是否新鲜。|

### 标定

| 脚本 | 对设备做什么 | 说明 |
|---|---|---|
| `calibrate.py` | 写 flash | 编码器零点 + 行程跨度标定;完成后归一化开口度才可用。写入前检查固件支持,写入后实时显示 `raw \| cooked \| 0..1`。见 [CALIBRATION.md](CALIBRATION.md)。|
| `fisheye_cal.py` | `show` 只读,`set-*` 写 flash | 读写 flash 中的标定记录:`show`、`set-fisheye`(命令行参数或含 `K`/`D` 的 OpenCV `.npz`)、`set-encoder-max`、`measure-encoder-max`(引导式行程标定)。|
| `read_intrinsics.py` | 只读 | 以 JSON 输出腕部相机内参。`source` 字段标明来自设备标定(`device`)还是 SDK 参考值(`reference`),`--require-device` 让后者直接失败。JSON 输出到 stdout,可以 `> cal.json` 或管道给 `jq`。|

### 相机与主爪

| 脚本 | 对设备做什么 | 说明 |
|---|---|---|
| `wrist_camera.py` | 只读 | 腕部相机查看器,按 `left` / `right` 或 XC 序列号选择;`--list` 列出设备。只接受 XC 腕部相机(视触觉传感器归 `xensesdk`)。鱼眼去畸变默认关闭:`--undistort`、`--compare`(原图与矫正图并排)、`--balance`,运行时按 `u` / `[` `]` 切换。无界面模式 `--no-display`(配合 `--save-dir` 每秒存一张 PNG)。默认 MJPEG,整套采集时不要用 `--no-mjpg`。|
| `leader_normalized_position.py` | 只读 | 以 `0..1` 流式显示主爪开口度。需要已标定行程(或用 `--encoder-max-rad` 直接指定)。|

### 固件

| 脚本 | 对设备做什么 | 说明 |
|---|---|---|
| `full_ota_update.py` | **依次刷从爪 MCU + 电机固件** | 锁定设备身份，编排协议切换、实时版本核验和最终标定检查。按提示手动断 24 V；默认保留方向。 |
| `ota_update.py` | **刷固件** | 夹爪固件 OTA,带进度条。**按角色选镜像,刷错会让 MCU 无法启动。** 刷完断电重启(从爪断 24 V,主爪拔插 USB)。|
| `motor_ota_update.py` | **刷电机固件** | 通过从爪 USB-C 升级 RobStride 电机固件。需要从爪固件 ≥ 1.2.8、电机处于私有协议;不指定镜像时按设备记录的电机型号取 `firmware/` 里的镜像并核对 CRC32;脚本会核对镜像与设备记录的电机型号。先刷夹爪再刷电机。详见 [firmware/README.md](../firmware/README.md#motor-firmware)。|

### C++ 示例

| 示例 | 说明 |
|---|---|
| `leader_demo` | 单台主爪 5 秒流式采集,报告各路速率。|
| `follower_status` | `follower_status.py` 的 C++ 版本。|
| `follower_impedance` | 阻抗控制 + 边控制边读。|
| `follower_force_position` | 力位混合控制,检查 `holding` 时爪子确实没到达命令位置。|

## 力位混合控制器

`ForcePositionController` 全程只有一条控制律:PD 请求被钳位在每次
`set_target(p, grasp_torque_nm)` 给定的力矩上。爪子被挡住时命令**恰好**停在这个值,
所以它就是夹持力;饱和本身就是接触,不需要另外的接触判定(堵转判定由固件以 500 Hz
执行)。阻抗模式被挡住时也会饱和,但饱和值是固定的配置常数 `max_position_torque_nm`。
需要逐次设定夹持力时用力位混合。

### 0. 先写运动安全包络(每台从爪一次,掉电保持)

包络在**固件**里,是 MIT 路径上谁都绕不过的一层;主机侧控制器无法替代它。从爪固件
1.2.12+ 未写入时使用电机型号默认值,更早的固件则完全不保护。

```bash
# 写入并显示(值由设备自报的电机额定推出,已正确则不写)
python python/examples/gripper_console.py right --set-envelope --show-envelope

# 写入后直接进控制台
python python/examples/gripper_console.py right --set-envelope --mode force-position

# 只看不写
python python/examples/gripper_console.py right --show-envelope

# 等价入口
python python/examples/impedance_control.py right --set-envelope
```

读回 `flags=0x2003` 即生效(layout 2 | `VALID` | `ENFORCE`)。

包络没有命令行数值参数:设备知道自己的电机额定,SDK 读出后推导记录:

| 字段 | 取值 | 含义 |
|---|---|---|
| `peak_torque_nm` | 电机**额定**力矩(EL05 1.8,RS00 5.0 N·m) | 运动瞬态上限,同时决定接近速度(约 `peak/kd`) |
| `cont_torque_nm` | 电机持续**堵转**额定(EL05 1.1,RS00 3.6 N·m) | 长期保持的上限,也是 I²t 降额的下限 |
| `temp_derate_start_c` | 0 → 固件默认 90 °C | 温度降额起点 |
| `temp_wall_c` | 0 → 固件默认 100 °C | 温度墙,超过后只保留 0.30 N·m |

**存储值不等于生效值。** 固件会把 `cont` 钳到堵转额定,而读回的是 flash 中的原值。
`--show-envelope` 因此分开打印:

```
[envelope] stored    GripperEnvelope(cont=1.800 Nm, peak=3.000 Nm, ...)
[envelope] effective GripperEnvelope(cont=1.100 Nm, peak=3.000 Nm, ...)
```

`effective` 显示固件不执行任何保护时,位置误差钳位、I²t、温度墙都不存在。

包络不是 `start()` 的前提:`start()` 校验的是设备的 0x700B 启动上限(必须
≤ `motion_torque_limit_nm`)和电机额定。没写包络也能启动,只是长时间夹持没有 I²t 和
温度墙保护。

> 控制器的力矩参数同样按电机推导:`ImpedanceConfig.for_spec()` /
> `ForcePositionConfig.for_spec()` 从 `motor.get_spec()` 取默认值,示例都使用它。
> 接近速度约为 `预算 / kd`,所以只调高夹持力而不调 `kd`,爪子会夹得更紧,也撞得更快。

### 1. 启动

```bash
python python/examples/gripper_console.py right --mode force-position
python python/examples/gripper_console.py right --mode force-position --grasp-torque 0.8
python python/examples/gripper_console.py right --mode force-position \
       --grasp-torque 0.8 --close-speed 0.5

# 非交互版本:同一个控制器,跑一遍固定目标序列
python python/examples/force_position_control.py right --grasp-torque 0.8
```

### 2. 命令行参数

| 开关 | 映射到 | 默认 | 说明 |
|---|---|---|---|
| `--grasp-torque` | `grasp_torque_nm` | 电机持续堵转额定(EL05 1.1 / RS00 3.6) | 夹持力设定值。自由行程不使用,被挡住时停在这个值 |
| `--close-speed` | `close_speed_radps` | 1.1 | 闭合速度 rad/s |
| `--step` | — | 0.05 | `j`/`k` 步进量,归一化 0..1 |

`--kp` / `--kd` 只用于 `--mode impedance`。

`--grasp-torque` 上限是电机持续堵转额定,超过时 `start()` 抛 `ValueError`(EL05 上
1.2 这类值会被拒)。它和包络的 `cont_torque_nm` 是同一个数:更大的设定值只会被 I²t
降额拉回,不会夹得更紧。

### 3. `ForcePositionConfig` 字段

真正需要按任务调整的只有前两个。请用 `ForcePositionConfig.for_spec(motor.get_spec())`
构造;直接 `ForcePositionConfig()` 得到的是 EL05 默认值,且 `close_speed_radps` 为 0.5。

| 字段 | for_spec(EL05 / RS00) | 说明 |
|---|---|---|
| `grasp_torque_nm` | 1.1 / 3.6 | 夹持力,= 持续堵转额定 |
| `close_speed_radps` | 1.1 / 1.1 | 闭合速度 rad/s |
| `hold_torque_limit_nm` | 1.8 / 5.0 | 无限期保持上限 = 电机额定力矩 |
| `motion_torque_limit_nm` | 6.0 / 14.0 | 运动瞬态上限 = 电机力矩量程 `t_max` |
| `close_preload_nm` | 0.25 / 0.25 | 仅在闭合端点叠加的前馈,使爪子贴紧机械止点;不得超过 `grasp_torque_nm` |
| `status_timeout_ms` | 350 | 状态流超时 → 零力矩 + `FAULT` |
| `motor_stream_hz` | 100 | 状态流速率 |

构造时做合理性检查(力矩在 (0, 20] N·m、`grasp ≤ hold ≤ motion`);`start()` 再对照
设备检查:`hold_torque_limit_nm` 超过额定、`motion_torque_limit_nm` 超过 `t_max`、
`grasp_torque_nm` 超过持续堵转额定,都会抛异常。

位置增益、阻尼和到位半径是针对这款硬件实测确定的,不对外开放配置。

### 4. 观测量

状态是观测量而不是控制状态:控制律始终不变,状态每帧从结果导出,只用于判断和显示。
`FAULT` 是唯一会锁存的状态。

| 观测 | 含义 |
|---|---|
| `arrived` | 到达命令位置 |
| `holding` | 设定点已领先到力矩预算允许的程度,且爪子没有跟上 —— 即正在夹持 |

控制台底部状态行:

```
state=HOLDING_FORCE   hold=Y arr=N  cmd=1.100Nm  grasp=1.100Nm  limit hold=1.80/motion=6.00/dev=6.00Nm
```

`cmd` / `grasp` 是**设定值**,实测力矩看 `Torq(Nm)` 列,两者之差就是固件的 I²t 与温度
降额。`dev` 是设备的 0x700B 启动上限,`start()` 会校验它。

### 5. 按键

| 键 | 力位混合模式下的行为 |
|---|---|
| `j` / `k` / `c` | `set_target()`;被挡住时停在夹持力上 |
| `o` | `release()` —— 以有界速度张开 |
| `h` | `hold_position()` —— 取消运动或夹持,停在当前位置 |
| `f` | `motor.clear_fault()` 然后 `reset()`,退出 `FAULT`(顺序不能反) |
| `e` / `d` | 恢复控制 / 失能;`d` 会先停控制器并下发零力矩 |
| `q` / ESC | 退出:先 `stop()` 下发零力矩,再 `disable()` |

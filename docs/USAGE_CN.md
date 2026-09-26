# 使用文档 —— 三路数据怎么开起来

> 本文是 [USAGE.md](USAGE.md) 的中文版,英文版为准。

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
> [EXAMPLES.md](EXAMPLES.md#选谁统一用-left--right)。

前置的安装、设备权限、`PYTHONPATH` / `LD_LIBRARY_PATH` 陷阱见
[docs/INSTALL.md](INSTALL.md)。下文假定 `xense-taccap` 环境已激活、
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
f.motor.on_status(lambda s: print(s.actual_pos, s.actual_torque))
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
python python/examples/fisheye_cal.py measure-encoder-max   # 只做行程那一步
```

细节见 [docs/CALIBRATION.md](CALIBRATION.md)。

### 1.4 控制(仅 follower)

follower 驱动一个 FDCAN 电机,控制原语是 **MIT 阻抗帧**(力位混合):
`kp`/`kd` 跟踪目标位置,前馈力矩叠加力的分量。动之前必须 enable,但**顺序是
先 `start()` 再 `enable()`**:两个控制器的 `start()` 都会读设备持久化的
`0x700B` 启动上限并校验配置,这个检查要发生在电机能动之前才有意义。

```python
f = t.FollowerGripper.open()
f.motor.clear_fault()
```

**推荐路径 —— `ImpedanceController`**:跟随一个位置目标(遥操作、leader-follower
转发)就用它。C++ 后台线程按电机状态流的相位提交,你的策略只碰两个非阻塞调用:

```python
cfg = t.ImpedanceConfig.for_spec(f.motor.get_spec())   # 按本机电机取默认值
# 下面是 for_spec() 在 EL05 上给出的值(括号里是 RS00):
cfg.kp = 20.0                      # 刚度 Nm/rad:位置误差换成力矩的比例,手感硬不硬
cfg.kd = 1.0                       # 阻尼 Nm·s/rad:抑振,并决定接近速度(见下)。
                                   #   for_spec:预算 / 1.1 rad/s(RS00 ≈ 3.27)
cfg.feedforward_torque = 0.0       # 恒定前馈 Nm,叠在 PD 上。抵消重力/预载用,平时 0
cfg.max_position_torque_nm = 1.1   # 误差钳位 Nm(就是保护):命令目标限制在实测位置
                                   #   ±(1.1/kp)=0.055 rad 内,并把接近速度定在
                                   #   约 1.1/kd = 1.1 rad/s。被挡住时饱和在这里并保持。
                                   #   for_spec:堵转额定(RS00 3.6)
cfg.rated_torque_nm = 1.8          # 力矩天花板 Nm(兜底),作用在“实测”力矩上,不是命令值。
                                   #   顶住后改发 kp=kd=0 的纯前馈帧,钉在预算上。
                                   #   须高于 max_position_torque_nm + |前馈|(RS00 5.0)
cfg.peak_torque_nm = 6.0           # 电机力矩量程 t_max;实测反馈超过它 → FAULT(RS00 14.0)
cfg.status_timeout_ms = 350        # 状态流断这么久 → FAULT + 零力矩 + 观测置 invalid
cfg.motor_stream_hz = 100          # 状态流速率 Hz;提交锁在这个相位,一帧一提交

c = t.ImpedanceController(f, cfg)
c.start()                         # 校验设备上限;以当前位置为初始目标,不会跳
f.motor.enable()                  # start() 之后再使能,见上
try:
    while running:
        s = c.snapshot()          # 一把锁、一致视图
        if s.state == t.ImpedanceState.FAULT:
            print("fault:", s.fault_reason)
            break
        c.set_target(policy(s.observation))
finally:
    c.stop()                      # 先下发零力矩
    f.motor.disable()
```

`start()` 之后**不要**再自己 `start_streaming()` 或注册 `motor.on_status()` ——
控制器独占状态流与控制路径,观测一律从 `snapshot()` 取。

#### 八个字段分别控制什么

| 字段 | 默认 | 控制什么 |
|---|---|---|
| `kp` | 20.0 Nm/rad | 刚度。位置误差换算成力矩的比例,"手感硬不硬" |
| `kd` | 1.0 Nm·s/rad | 阻尼。抑制振荡,同时决定接近速度(见下) |
| `feedforward_torque` | 0.0 Nm | 恒定前馈力矩,叠加在 PD 之上。抵消重力/预载用,平时留 0 |
| `max_position_torque_nm` | 1.1 Nm | **误差钳位**,就是保护。命令目标被限制在实测位置 ±(该值/`kp`) 内 |
| `rated_torque_nm` | 1.8 Nm | **力矩天花板**,兜底。作用在**实测**力矩上,不是命令值;同时也是 `\|feedforward_torque\|` 的上界 |
| `peak_torque_nm` | 6.0 Nm | 电机力矩量程(`t_max`)。**实测**力矩超过它按不可信读数处理,控制器进 FAULT |
| `status_timeout_ms` | 350 ms | 状态流断这么久 → `FAULT` + 零力矩 + 观测置 invalid |
| `motor_stream_hz` | 100 | 状态流速率。提交锁在这个相位上,一帧一提交 |

表里的默认值是编译进来的 **EL05** 兜底值。应当用
`ImpedanceConfig.for_spec(f.motor.get_spec())` 构造,它从设备自报的额定取
`max_position_torque_nm` / `rated_torque_nm` / `peak_torque_nm`(EL05 1.1 / 1.8 / 6.0,
RS00 3.6 / 5.0 / 14.0),并把 `kd` 设为 预算 / 1.1 rad/s。

真正要按任务调的是前两个加 `max_position_torque_nm`;后两个是传输参数,
`rated_torque_nm` / `peak_torque_nm` 一般就放在电机额定值上。

**这两层保护是不同的东西,别混。**

- `max_position_torque_nm` 钳的是**命令**:`kp × 误差` 不允许超过它。窗口是
  `max_position_torque_nm / kp`,默认 1.1/20 = **0.055 rad**。它也顺带定死了
  接近速度 —— 爪子一直加速到阻尼平衡住钳位后的力矩,约
  `max_position_torque_nm / kd`,默认 **1.1 rad/s**。所以调 `kd` 会同时改手感
  和接近速度。被挡住时命令饱和在这个值上**并一直保持**,那就是夹持力 —— 接触
  不需要检测,饱和本身就是接触。默认 1.1 是 EL05 的持续堵转额定,因为这个保持
  没有任何东西给它计时。
- `rated_torque_nm` 看的是**电机实际发出来的**力矩(由电流推算)。压住这个值
  够久之后,控制器改发 `kp=kd=0` 的纯前馈帧:位置误差在结构上再也加不进输出,
  力矩被**钉死**在预算 `max_position_torque_nm` 上(预算为 0 时才钉在天花板上),
  而不是"估计不超过"。它是兜底,正常工作到不了:配置校验要求
  `max_position_torque_nm + |feedforward_torque| < rated_torque_nm`。
- 为什么两层都要:钳位管的是命令值,而 1.1.5 实测堵转时反馈只有命令的约 0.59
  —— 那个比例是一台机、一个温度、一个负载测出来的。**天花板不关心这个比例。**

`rated_torque_nm` 上限卡在 **设备额定力矩(EL05 1.8 Nm、RS00 5.0 Nm)而不是峰值**,
由 `start()` 对照设备检查,因为它产生的
保持是**无限期**的,没有任何东西给它计时。超过额定值长期保持,主要风险不是热:
电机欠压保护很快,持续大电流会把 24 V 拉垮并把 USB 一起带走 —— 到主机这边表现为
`SerialBus::write: Input/output error`,完全不像力矩故障。

#### 状态机

`IDLE` → `TRACKING` → `TORQUE_CAPPED` → `FAULT`,优先级
**`FAULT` > `TORQUE_CAPPED` > `TRACKING`**。

| 状态 | 含义 |
|---|---|
| `TRACKING` | 正常跟随。被挡住、误差钳位饱和在预算上,也还是这个状态 |
| `TORQUE_CAPPED` | 实测力矩顶到天花板,正在发纯前馈帧 |
| `FAULT` | 状态流断 / 电机故障位 / 提交失败 / 实测力矩超过 `peak_torque_nm`。零力矩,需 `reset()` |

**没有 `STALLED`。** 失速守卫已删除:它把有效目标钳到爪子停住的位置,误差归零,
力矩随之塌掉 —— 实测接触后 60 ms 在 0.35 Nm 松手。要判断是不是被挡住,看
`snapshot().commanded_torque_nm` 有没有顶到 `max_position_torque_nm`
(`impedance_control.py` 就是这么查的)。

`snapshot()` 另外给了 `torque_capped` 布尔和计数 `torque_caps`;运行期改增益用
`set_gains(kp, kd, feedforward_torque)`。

原先与它并列的底层 `ControlLoop` 已删除:它是同一套控制律的第二份拷贝,已经和
这里漂移开(预算不同,还留着那个会塌力矩的失速守卫)。

**硬物夹持/限力保持 —— `ForcePositionController`**:普通位置阻抗在物体挡住夹爪后
会继续积累 `kp × 位置误差`,不适合把目标长期放在完全闭合点。力位混合控制器改用
**一条有界力矩的控制律**,全程不变:

```
command(target, kp, kd, 力矩预算 = grasp_torque_nm)
```

- **自由行程**:所需力矩只有摩擦那么多(本机构实测约 0.1 N·m),远低于预算,
  钳位不生效,爪子沿斜坡以 `close_speed_radps` 行进;
- **被挡住**:爪子停下、斜坡跑到前面、误差钳位饱和,命令**恰好**停在
  `grasp_torque_nm` 并保持在那里。

**接触不需要判定 —— 饱和就是接触**,保持就是饱和的自然结果。没有模式切换。

`grasp_torque_nm` 默认 **取设备自报的连续堵转额定**(`ForcePositionConfig.for_spec()`)
—— EL05 上 1.1 N·m,RS00 上 3.6。下面这组实测是在 EL05 上做的。实测复核:夹持按构造是
无限期的,所以默认值必须是这台夹爪真能一直保持的力矩 —— 那是关于整机的问题,
不是关于电机的。真实工件持续保持实测(24V、包络启用):

| 夹持力 | 结果 |
|---|---|
| 1.1 N·m(默认) | 36→70℃ / 600s,速率 8→4→3→2→1 ℃/min,**外推平台 ≈75℃** |
| 0.6 N·m | 41→49℃ / 600s,末 60 秒 0.00℃/min,**已进平台** |

1.1 的温升速率全程在衰减,是一阶趋近平台而非线性爬升,平台低于固件 90℃ 温度墙
约 15℃。余量真实但不宽裕:两轮都从 36–41℃ 起测,高环境温度会直接吃掉它。

另外 `grasp_torque_nm` **不是硬上限**:稳定保持时实测反馈比预算高 5~7%。预算塑造
命令,真正约束输出的是固件的运动包络。

> 2026-09 之前这里跑的是一套主机侧接触状态机(`kp=0` 速度阻尼闭合 → 堵转判定 →
> 纯 `tau_ff` 保持)。删掉的原因有三:它重复了 MCU 已经在 500 Hz 做的事
> (`task_canmotor_is_stalled()`,而 `docs/CONTROL_LAYERING.md` §3 早已把接触判定
> 划给固件);为了让堵转特征干净,行进段被**刻意**做成 `kp=0`,实测代价是 37% 的
> 速度纹波和只有命令值 77% 的均速;而它最核心的判断——区分「到位」与「被挡住」
> ——在空爪场景下物理上本就连续,不可靠。详见 `docs/CONTROL_REFACTOR.md`。

**状态是观测量,不是控制状态。** `snapshot()` 每帧导出:

| 观测 | 含义 |
|---|---|
| `arrived` | `\|位置误差\| <= arrival_eps_rad` |
| `holding` | 设定点已跑到预算允许的最前面(前导量 ≥ 一半上限)**而且**爪子没跟上(速度 ≤ 命令速度的 25%) |
| `state` | 由上面两者与运动方向导出,仅用于显示 |

`holding` 的速度门限用的是固件自己的 `TASK_CANMOTOR_STALL_VEL_RATIO = 0.25`,
刻意取同一个数,让主机和 MCU 用同样的方式描述同一个物理事件。

**判据不是「命令是否用满预算」。** 那个在目标一变的瞬间就成立:设定点跳了、命令
饱和了,而爪子还没来得及动。实测后果是每一步"夹持"都在 0.01 秒内完成、实测力矩
只有 0.016 N·m —— 什么都没夹住,验收却全绿。前导量自带时序:斜坡是种在爪子上的,
起步时为零,只有爪子不跟才会涨上去。

`hold_position` 是爪子**实际**停住的位置,`target_position` 是当初要的位置,
两者仍然分得清。

**做分离的是速度门,不是力矩数字。** 在 1.1.5 从爪上实测整段空载闭合(1578 帧):
自由行程 `|vel|` 始终 ≥ 0.183 rad/s,是 0.035 门限的五倍,同时 `|力矩|` ≤ 0.142 Nm;
机械止点处 `|vel|` ≈ 0.012 rad/s、`|力矩|` ≈ 0.21 Nm。**没有任何一帧同时满足两个
条件**。这两条判据现在只留在固件里 —— 主机侧的那份已随接触状态机删除。

**顺带记下一条仍然有效的事实**:`kd` 形式的 MIT 帧在堵转时,反馈力矩达不到命令值。
同一次实测:命令 0.359 N·m,反馈只饱和到 0.213 N·m(≈0.59)。所以任何"拿反馈力矩
去比命令上限"的判据都是够不到的 —— 上面那个判据(前导量 + 速度)之所以成立,正是
因为它比的是**命令侧**的量,按构造一定够得到。

控制器把力矩限制拆成两个职责明确的上限,它们就是**电机自身的两个额定值**,不是
随手取的安全裕度:

- `motion_torque_limit_nm`:闭合/张开/位置保持过程中的速度阻尼和 PD **瞬时**力矩
  上限,最大为电机型号的**力矩量程 `t_max`**(EL05 6.0 Nm、RS00 14.0 Nm);反馈力矩
  超过它会进入零力矩故障状态。从从爪固件 1.2.9 起,型号的 `t_max` 同时也是固件对
  `0x700B` 启动上限的默认值和最大值(旧固件存下的值保留不动),所以用
  `ForcePositionConfig.for_spec(f.motor.get_spec())` 构造的配置和出厂设备是一致的。
- `hold_torque_limit_nm`:**长期**保持力矩的上限,最大为设备的**额定(标称)力矩**
  (EL05 1.8 Nm、RS00 5.0 Nm);`grasp_torque_nm` 和运行时传入的目标力矩都不能超过它。
  注意它在当前控制律里**不钳任何输出**:只用来校验 grasp 的上界。

构造时只做 (0, 20] N·m 的合理性检查。`start()` 对照设备检查,`hold_torque_limit_nm`
超过额定力矩、`motion_torque_limit_nm` 超过 `t_max`、或 `grasp_torque_nm` 超过持续
堵转额定(EL05 1.1、RS00 3.6)时直接**抛异常**。

`start()` 会读取 V2.2 的持久化 `0x700B limit_torque`,并要求设备值不高于配置的
运动上限。这里持久化的是**夹爪 MCU Flash 中的启动配置**,不是电机自身 Flash。
第一次配置设备上限后必须物理断电重启,让 MCU 在开机时把该值写入电机运行参数
0x700B:

```python
f = t.FollowerGripper.open()
f.motor.set_startup_limit_torque(f.motor.get_model().t_max_nm)   # 型号的 t_max;写 MCU Flash,只需配置一次
print(f.motor.get_startup_limit_torque())
# 此处退出并拔插夹爪;不要在同一次上电中直接继续运动
```

重启后使用控制器:

```python
f = t.FollowerGripper.open()
f.motor.clear_fault()

cfg = t.ForcePositionConfig.for_spec(f.motor.get_spec())   # 本机电机的额定
cfg.grasp_torque_nm = 0.35         # 力矩预算 Nm —— 就是夹持力设定值,被挡住时停在这里。
                                   #   for_spec:堵转额定(EL05 1.1 / RS00 3.6)
cfg.close_speed_radps = 1.1        # 闭合/张开斜坡速度 rad/s(for_spec:1.1)
# for_spec 还会设置(EL05 / RS00):
#   hold_torque_limit_nm   = 额定力矩  1.8 / 5.0  —— 无限期保持上限
#   motion_torque_limit_nm = t_max     6.0 / 14.0 —— 运动瞬态上限
#   close_preload_nm       = 0.25                  —— 闭合止点的压紧力
cfg.status_timeout_ms = 350        # 状态流断这么久 → FAULT + 零力矩 + 观测置 invalid
cfg.motor_stream_hz = 100          # 状态流速率 Hz;提交锁在这个相位
grasp = t.ForcePositionController(f, cfg)
grasp.start()                 # 先验证设备上限,尚不主动闭合
f.motor.enable()
try:
    grasp.set_target(0.0)     # 闭合;被挡住时命令饱和在 0.35 Nm 并保持
    grasp.set_target(0.35, 0.45)  # 运行时改为 35% 开度、0.45 Nm
    while running:
        print(grasp.snapshot())
    grasp.release()           # 有界速度张开
finally:
    grasp.stop()              # 先下发零力矩
    f.motor.disable()
```

示例:

```bash
# 两个控制器的用法
python python/examples/impedance_control.py right
python python/examples/force_position_control.py right --grasp-torque 0.35

# 运动安全包络(固件侧的力矩与热保护,默认不启用,每台设备配一次)
python python/examples/impedance_control.py --show-envelope
python python/examples/impedance_control.py --set-envelope
```

注意:`t_max`(EL05 6 Nm、RS00 14 Nm)是电机峰值、只允许运动阶段的**瞬时**力矩到
这个量级,并不把夹爪机构的安全额定值提高到这个数。如果机构本身不能承受高于额定力矩
(EL05 1.8、RS00 5.0 Nm)的瞬时负载,应把 `motion_torque_limit_nm` 和设备 `0x700B`
一并设低;软件只能保证保持阶段的命令力矩不超过 `grasp_torque_nm`(其上界是设备的
持续堵转额定)。

`FollowerGripper` 拒绝打开固件低于 1.2.5 的从爪;该控制器还要求从爪开合行程已经标定。运行期间它独占电机控制与状态流,不要并发使用 `ImpedanceController`
或直接发送其他运动命令。

**相位为什么重要**:主机帧只要在 MCU 发送期间落地,就会让它丢掉正在发的那一帧,
整帧作废。所以碰撞取决于**落在什么时刻**,不是发了多少 —— 两个控制器都
是每收到一帧状态提交一次,落在 MCU 已知空闲的窗口里,实测 6000 提交 : 6000 帧 :
0 丢失;同一条件下自由跑 100 Hz 每轮丢 156–308 帧。别把 500 Hz 当预算花。

**低层写法没有任何保护。** 不带 ACK 的裸原语 `motor.submit_impedance` /
`submit_position` / `submit_velocity` / `submit_torque` 暴露给了 Python;带 ACK 的
`set_impedance` / `set_position` / `set_velocity` / `set_torque` 以及归一化包装
`FollowerGripper.set_position` 只在 C++ 侧。它们都是把控制帧直接丢上总线:没有误差
钳位、没有力矩天花板。走 `ImpedanceController` 或 `ForcePositionController`(两个
控制器内部就在用它们);真要直接调 `submit_*`,用 `motor.control_stats()` 查固件丢了
什么。

**反馈频率**:电机 `actual_*` 遥测只有 ~50–100 Hz,读观测请走
`observation()` / `snapshot()`,别用 `read_status()` 轮询 —— 超过 ~100 Hz 会拖住
固件自己的刷新,而且控制期间的 ACK 往返会撞坏遥测帧。

> **抓取力**要用 `ForcePositionController`:它不做接触判定 —— PD 请求对
> `grasp_torque_nm` 做误差钳位,被挡住时命令恰好停在那个值,饱和本身就是接触。
> 位置模式的 `max_torque` 不是紧的力上限,拿它当夹持力会把软物压坏。

可跑的示例:`impedance_control.py`、`force_position_control.py`(见上)、
`gripper_console.py`(交互控制台,两种模式)。

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
- **控制回路要用 `ImpedanceController` 或 `ForcePositionController`。** 两者都把
  提交锁在状态流的相位上;上面的相位说明在满负载下
  尤其重要:实测就是在"所有相机都在流 + 电机在往复"的条件下做的。
- **日志。** 全 SDK 一个单例 logger(`xense.taccap.log`),控制台默认 INFO,
  文件 sink 恒为 DEBUG,落在 `~/.taccaplogs/session_*.log`(可用 `$TACCAP_LOG_DIR`
  改),最多留 10 份。出问题先翻这个文件,里面有控制台被过滤掉的那些行。
- **固件刷完必须重新插拔 —— USB 线与电源线同时拔下再一起插回。** bank-swap
  之后是软复位,设备看起来完全正常
  (版本对、流在跑、计数干净),但会静默丢状态帧。见 [docs/FIRMWARE.md](FIRMWARE.md)。

---

## 出问题时

| 现象 | 多半是 |
| --- | --- |
| `scan_grippers()` 返回空 | 串口权限(见 [INSTALL.md](INSTALL.md))或线没插好 |
| `fw_sn` 为空 / `Side.Unknown` | 固件没烧 SN,或固件早于 V1.6 |
| 矫正后画面全黑 | 用了 `read_fisheye()` 的全零记录 —— 改用 `resolve_fisheye()` |
| 矫正后画面偏心 | 未必是错的,主点本来就不一定在画面中心 |
| 录下来的颜色是反的 | 裸 `Camera` 是 BGR、`wrist_camera` 是 RGB,搞混了 |
| 电机状态帧成片丢失 | 提交相位(用两个控制器之一,别自己发帧),或刷完固件没断电重插 |
| 改了 C++/bindings 但 Python 里没生效 | 消费端的环境没重装,见 [INSTALL.md](INSTALL.md) |
| `import xense.taccap` 报 `libopencv_core.so` | 缺 `LD_LIBRARY_PATH=$CONDA_PREFIX/lib` |

更细的专题:[INSTALL.md](INSTALL.md) · [CALIBRATION.md](CALIBRATION.md) ·
[FIRMWARE.md](FIRMWARE.md) · [EXAMPLES.md](EXAMPLES.md) ·
[ARCHITECTURE.md](ARCHITECTURE.md)

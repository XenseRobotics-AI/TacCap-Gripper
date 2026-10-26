# 从爪 IMU 与电机控制完整性、稳定性测试

本文是可重复执行的测试方案，不是通过报告。软件测试、当前机器上的 SDK 加载状态、设备上实际运行的固件，以及机械运动验收必须分别记录结果。未执行的项目标为 `NOT RUN`，设备或条件不具备时标为 `BLOCKED`。

方案针对支持 `FollowerGripper.start_streaming(motor_hz=100, imu_hz=...)` 的 SDK，以及配套固件的从爪 IMU 功能。编写时核对的基线为 SDK `c06bc342ba0c534f0e830e5ef8574e4fc0ba5d1f`、固件仓库 `XenseRobotics-AI/tc-gu-01` 的 `codex/gripper-imu-support@1d43fcaa423aca515ca4ef666e6e605fc5d4a5`；执行时记录实际 SHA 和未提交修改，不把这两个历史核对值当作“最新”。

## 1. 能确认什么，仍须测量什么

当前设计默认只上报电机状态；显式设置 `imu_hz=1..100` 才请求 IMU。SDK 先读 IMU 探测能力，再协商时间格式和修改流；旧固件拒绝时不会先改写流。实现见 [follower_gripper.cpp](../cpp/src/follower_gripper.cpp)、[imu.cpp](../cpp/src/components/imu.cpp)。

配套固件 `App/tasks/task_imu.c`、`task_slave_motor_control.c`、`task_data_stream.c`、`slave_imu.c` 中，电机控制、数据流、从爪 IMU 的线程优先级分别为 8、11、12；电机控制周期为 2 ms，电机状态最高 100 Hz；同时到期时先发电机状态。IMU 采样失败不将旧包伪装成新样本，也不主动停止电机控制。这些设计降低了干扰，但共享 CPU、UART 和主机处理仍可能产生时延。

SDK 的 motor 与 IMU 回调共享 Transport 分发线程。慢 IMU 回调会拖延电机状态回调和控制器目标更新，即使串口读取线程仍在接收。当前控制器按分发时刻刷新状态年龄，积压的旧反馈仍可能使 `snapshot().observation.age_ms` 很小、`valid` 为真；这些字段与 TRACKING 状态不能单独用于实时性验收。必须同时测量设备反馈时间、主机接收/分发积压和目标更新节奏。软件替身测试不能证明真实 RTOS 抢占、SPI/CAN/UART 时序、电源稳定或夹爪运动质量。

## 2. 设备、环境与运行身份

每次运行记录以下内容，EL05 与 RS00、左右爪分别建档，不用一种设备结果替代另一种：

| 类别 | 必须记录 |
| --- | --- |
| SDK | 仓库分支、完整 SHA、dirty 状态、构建配置、Python 可执行文件、Python 包与 `_taccap_native` 实际加载路径、版本 |
| 固件 | MCU 返回版本、固件构建 SHA、烧录 BIN 的 SHA-256、烧录记录；电机型号及其固件版本来源 |
| 设备 | 精确 firmware SN、角色和左右侧、CH343 序列号、MCU 的 by-id 路径、硬件版本 |
| 台架 | 电源电压/限流、线缆与 USB 拓扑、夹持物/治具、环境温度、传感器安装方向、外部急停与独立测量仪器 |
| 控制 | 控制器类型、完整配置、行程标定、安全包络、实际轨迹、循环次数、驻留时间、负载和温度停止条件 |

配套试验固件曾沿用从爪 `1.2.16`，因此相同版本号不代表相同程序，也不能证明 IMU 或时间扩展已刷入。能力探测只能证明被探测接口可用；没有烧录证据时将固件 SHA 记为 `unknown`，不能用本机源码 SHA 填充。

运行前核对独立构建/测试环境。构建源码不等于当前 Python 已加载它；尤其要检查原生扩展路径，避免旧 `.so` 与新 Python 代码混用。构建与安装操作按 [INSTALL.md](INSTALL.md) 执行；本文采集片段不自动安装或刷机。

### 串口与发现副作用

`scan_grippers()` 不是被动 USB 枚举：它只选择 `/dev/serial/by-id/usb-1a86_USB_Dual_Serial_*-if02`，以 3,000,000 baud 打开控制口，发送 `StopStream`、`GetDevType`、`GetSn` 后关闭。不要根据 `/dev/ttyACM0`、`ttyACM1` 编号猜测 MCU 接口，也不要试开另一接口。实现见 [discovery.cpp](../cpp/src/discovery.cpp)。

SDK 使用 `flock` 与 `TIOCEXCL` 独占串口，`in_use` 必须视为占用并退出，不能抢占、杀死现有控制进程或绕过锁。发现和构造 `FollowerGripper` 都可能停止遗留流，应在设备原控制任务已正常结束、机械状态安全后进行；独占锁不能代替现场确认。

`FollowerGripper` 默认构造会停流、读取版本/SN/电机型号/行程配置，部分设备还读取 CAN ID；默认不开相机，不使能电机、不发运动目标、不写 Flash。它不凭构造参数证明设备角色，仍须根据发现结果确认唯一 SN 的 `Role.Follower`。上电自动回零属于固件行为，不能因采集脚本没有运动命令就忽略正在进行的回零。

## 3. 分阶段测试矩阵

各阶段先记录预期、门槛和停止条件，再执行；表中时长是建议的采样预算，可按产品要求调整，不是已完成次数。

| 阶段 | 场景与步骤 | 主要证据与进入下一阶段条件 |
| --- | --- | --- |
| S0 软件回归 | 运行第 8 节 C++/Python 测试；有固件源码时运行其主机替身测试 | 通过接口、混合数据流、时间、并发和退出检查；缺依赖/失败保留原始结果 |
| H0 硬件预检 | 核实唯一设备、启动终态、方向与行程标定、型号、包络、实际已失能 | `home_diag()` 无 ACTIVE，状态/故障可读且新鲜；角色、固件或机械状态未知时不运动 |
| H1 静止上报 | motor=100 Hz；IMU=0、10、50、100 Hz，5 s 预热后每段采集 60 s，A/B/A 重复至少 3 次 | 有效电机/IMU 速率、时延分位数、时间连续性、错误计数；确认开 IMU 与恢复 motor-only 后均稳定 |
| H2 空载小行程 | 采用已确认的行程内部范围，先少量慢速往返，再每条件不少于 10 次；两类控制器分别做同轨迹 A/B/A | 到位误差、轨迹完成时间、速度纹波、反向帧、峰值力矩、温升、控制 deadline 和目标年龄 |
| H3 代表性负载 | 明确夹持物、接触位置、摩擦/柔顺条件；重复 H2，再测试规定保持时长 | 力矩/温升、滑移或掉落、包络行为；需要夹持力结论时使用标定的力传感器 |
| H4 长时间稳定性 | 初筛通过后 100 Hz IMU 连续 30–60 min；产品验收再按任务周期做 8 h 或规定时长；前后追加 IMU=0 对照 | 每段数据量、错误计数、温度、RSS/CPU、重连/复位/意外失能、轨迹完成率；长稳通过仅覆盖实测时长 |
| H5 生命周期 | 停止后切换 IMU 频率、重复开/关和连接/退出；安全停机后冷启动，分别测工具提前连接和启动后连接 | 启动时间、回零结果、IMU 就绪时间、端口释放、重复会话无回调残留；不能将 MCU 软复位代替整机断电 |
| H6 故障/保护 | 独立急停、人员在场、无坠落危险的专用台架上测试 host 中断/断链、IMU 故障、限位/堵转 | 故障可见、停机时间与最终状态、恢复需明确动作且无意外运动；不在普通长稳循环自动注入 |
| H7 校准维护 | 运动中请求校准应拒绝且不停止控制；停止失能后在指定姿态单独执行校准、保存、重启恢复 | BUSY 响应、控制连续性、偏置复测和参数恢复；持久化/断电故障另列步骤，不自动混入采集 |

H6/H7 的断电、传感器断开、堵转、校准保存、OTA 等操作具有实际设备副作用。执行前需明确设备、动作、机械条件和恢复方式；普通数据采集流程不执行这些动作。**慢回调/分发积压只在 PTY 或已安全失能的场景注入；真机运动不做该注入，除非独立安全机制和运动范围已经明确。** 记录预期触发的故障与非预期故障，不能用正常阶段的“错误必须为零”否定有意故障注入，也不能把注入阶段的错误掩盖到正常基线里。

### A/B/A 控制变量

A 为同一候选固件、同一 SDK 的 `imu_hz=0`；B 分别为 10、50、100 Hz；再次 A 检查温升和时间漂移。建议 `0→10→0→50→0→100→0`，重复时交替 B 的顺序。若还要评估固件改动本身，需要另测已知基线固件，刷机与冷启动应单独授权和记录。

保持 motor_hz=100、控制器参数、轨迹、负载、电源、主机负载、USB 拓扑、日志配置和诊断查询时点一致。改变 IMU 频率时先安全停止控制器、停流，再重新配置。不要在运动中反复 `start_streaming()`，也不要把切流停顿算作稳态抖动。

## 4. API、控制顺序与退出

推荐接口如下，定义见 [Python bindings](../python/bindings/bind_gripper.cpp)、[motor bindings](../python/bindings/bind_motor.cpp)、[control bindings](../python/bindings/bind_control.cpp)：

| 用途 | API |
| --- | --- |
| 发现与打开 | `scan_grippers()`；`FollowerGripper(mcu_device=endpoint.mcu_device, open_cameras=False)` |
| 预检 | `g.home_diag()`、`g.get_gripper_config()`、`g.motor.get_model()`、`g.motor.get_spec()`、`g.audit_envelope()` |
| 订阅 | `g.motor.on_status(callback)`、`g.imu.on_data(callback)`；保存订阅 ID 并使用对应 `off(id)` |
| 配置流 | `g.start_streaming(motor_hz=100, imu_hz=rate)`；改频率前 `g.stop_streaming()` |
| 主机控制 | `ForcePositionConfig.for_spec(g.motor.get_spec())` / `ImpedanceConfig.for_spec(...)`，对应 Controller 的 `start()`、`set_target()`、`snapshot()`、`stop()` |
| 显式使能 | 控制器 `start()` 成功后，由调用者执行 `g.motor.enable()`；`start()` 本身不使能 |
| 诊断 | `g.motor.control_stats()`、`g.diagnostics.uart_stats()`、`g.transport.stats` |
| 退出 | `controller.stop()` → `g.stop_streaming()` → 注销回调 → `g.close()` |

**先开 motor+IMU 共流，再 `controller.start()`，成功后显式 `g.motor.enable()`。** 两种控制器会借用已开启的流；若先启动控制器，默认是 motor-only，之后不能直接新增 IMU。控制器启动会读取并校验电机限制、以当前位置初始化目标，但不使能电机；应在使能前完成这些启动检查。控制时不得预开 IMU-only：控制器需要电机状态反馈。借用的流需由调用者最终关闭。

回调只做有界、快速的数据复制/入队。不要在 IMU 或电机回调中 `sleep`、同步写磁盘、绘图、打印、获取长期持有的锁或发送 ACK 查询；另设消费线程批量保存，并记录采集队列溢出。`queue_dropped=0` 也不证明回调无害，必须同时测量 motor 间隔和控制状态。

运动中从流和 `controller.snapshot()` 获取状态；不要频繁 `read_status()`、`read_status_ext()`、`fault_report()` 或重新 `scan_grippers()`。诊断 ACK 同样占用链路，应放在统一区段边界；需要运行中诊断时采用一致、已记录的低频查询，并区分查询附近窗口。

`close()` 释放端口但不失能电机。控制器 `stop()` 会尝试零输出和失能，但失能失败可能只记录警告；断链时不能以函数返回证明物理停机。运动测试退出后在链路仍可用时确认已失能/实际反馈模式，再关闭；断链使用台架独立急停及恢复流程。无运动采集不应为了“清理”而擅自对正在夹持的设备发失能。

### 默认无运动的采集片段

以下片段在确认设备原任务已经结束后使用。它不会使能、设目标、校准、写 Flash 或刷机；发现/构造和开停流具有前述副作用。先把代码保存为本次运行脚本，再传入精确 SN；没有 SN 时会在发现设备前退出。为便于核对只输出段摘要，正式验收应按第 6 节保存原始数据与完整诊断差分。

```python
import collections
import sys
import time
from xense.taccap import FollowerGripper, Role, scan_grippers

if len(sys.argv) != 2:
    raise SystemExit("usage: python capture.py <exact-firmware-SN>")
endpoints = scan_grippers()
matches = [e for e in endpoints if e.firmware_sn == sys.argv[1]]
if len(matches) != 1 or matches[0].role != Role.Follower or matches[0].in_use:
    raise SystemExit("Need exactly one available follower with the requested SN")

g = FollowerGripper(mcu_device=matches[0].mcu_device, open_cameras=False)
motor_sub = imu_sub = None
motor_rows = collections.deque(maxlen=65536)
imu_rows = collections.deque(maxlen=65536)
try:
    print("firmware", g.firmware_version, "SN", g.firmware_sn)
    if g.home_diag().flags & 0x01:  # HomeDiagFlag.Active
        raise RuntimeError("Power-on calibration is active; end this run")
    initial = g.motor.read_status()
    if initial.status & 0x01:  # MotorStatusBit.Enabled
        raise RuntimeError("Motor already enabled; require a safe stopped session")
    motor_sub = g.motor.on_status(lambda s: motor_rows.append((
        time.monotonic(), s.host_time, s.raw.status_timestamp_ms,
        s.actual_pos, s.actual_vel, s.actual_torque, s.motor_temp_c,
        s.status, s.fault_code, s.latched_fault_code)))
    imu_sub = g.imu.on_data(lambda s: imu_rows.append((
        time.monotonic(), s.host_time, s.mcu_timestamp_us, s.seq,
        s.boot_session_id, s.timestamp_is_64bit, s.timestamp_valid,
        s.timestamp_discontinuity, s.valid_flag)))
    for rate in (0, 10, 0, 50, 0, 100, 0):
        before_uart = g.diagnostics.uart_stats()
        before_transport = g.transport.stats
        g.start_streaming(motor_hz=100, imu_hz=rate)
        time.sleep(5.0)
        begin = time.monotonic()
        time.sleep(60.0)
        end = time.monotonic()
        g.stop_streaming()
        # A timestamp window excludes preheat and any late-dispatched samples.
        motor = [r for r in list(motor_rows) if begin <= r[0] <= end]
        imu = [r for r in list(imu_rows) if begin <= r[0] <= end]
        after_uart = g.diagnostics.uart_stats()
        after_transport = g.transport.stats
        print({"imu_hz": rate, "seconds": end - begin,
               "motor_samples": len(motor), "imu_samples": len(imu),
               "motor_hz_observed": len(motor) / (end - begin),
               "imu_hz_observed": len(imu) / (end - begin)})
        # Serialize the rows and before/after counters outside the callbacks.
        # This short example's 65536-row buffers cover its planned duration;
        # a long run needs a draining queue plus an explicit overflow counter.
finally:
    try:
        g.stop_streaming()
    finally:
        try:
            if motor_sub is not None:
                g.motor.off(motor_sub)
            if imu_sub is not None:
                g.imu.off(imu_sub)
        finally:
            g.close()
```

### 运动配置必须先填写

行程用归一化 `[0, 1]` 表示，原始轴角为 rad、轴角速度为 rad/s、力矩为 N·m。归一化开度不直接等于 mm；须使用该装置的机构/标定映射。电机力矩不能未经传动比、有效力臂、效率/摩擦、接触点与实测标定直接换算成指尖夹持力 N。空载初筛采用行程内部范围，避免把端点预载或硬限位作为普通轨迹。

运行记录中先填写下表；空值表示不能开始该项运动。`for_spec()` 的默认值反映电机型号，不等于本次台架已批准的负载、速度或力矩。

| 参数 | 待填写的本次值 |
| --- | --- |
| 精确 SN、左右侧、EL05/RS00、控制器 | `待填写` |
| 起点/终点归一化开度，允许 raw rad 范围 | `待填写` |
| 起止姿态、夹爪固定方式、运动区域和负载 | `待填写` |
| 速度 rad/s、ForcePosition.grasp_torque_nm 或 Impedance.max_position_torque_nm | `待填写` |
| Impedance 的 kp/kd/feedforward 或 ForcePosition 的预载、保护配置 | `待填写完整配置` |
| 每段驻留、往返次数、完成时限、允许位置误差 | `待填写` |
| 最大温度、力矩、电流及独立急停条件 | `待填写` |

运动时顺序为：确认已失能，读取/审核型号与包络，配置控制器，订阅轻量采集回调，`g.start_streaming(motor_hz=100, imu_hz=rate)`，再 `controller.start()`；启动成功且所有运动前提满足后，显式调用 `g.motor.enable()`，随后在规定范围内 `set_target()`，以 `snapshot()` 和流检验轨迹。两类控制器的 `start()` 都不调用电机使能，不能省略或提前执行这个独立步骤；对应示例为 [impedance_control.py](../python/examples/impedance_control.py) 和 [force_position_control.py](../python/examples/force_position_control.py)。每段停止前取得结束控制统计，再执行本节退出顺序；相邻段恢复同一初始条件。

## 5. 时间、帧率和序列号的解释

- IMU 当前固件开流后约每 10 ms 采样，上报设置为 10/50/100 Hz 时，采样线程仍约 100 Hz。`seq` 是 16 位采样序号，低上报频率自然会跨号；先按设备时间和抽样倍率解释，不能直接把所有跨号加成丢包数。遇到回绕用无符号模差，遇到重启先切分会话。
- SDK 会滤除相邻完整相同的 IMU 载荷。回调计数是被交付的新样本数，不是所有线上帧数；必要时结合 Transport 帧数和固件 UART 计数区分重复包、丢包及采样变慢。
- `mcu_timestamp_us` 表示设备侧时间；`timestamp_is_64bit=True` 是协商后的原生 V2，legacy 展开时间仍有歧义。保存 `boot_session_id`、`timestamp_valid`、`timestamp_discontinuity`，会话首帧的 discontinuity 与运行中意外跳变分别处理。
- `host_time` 与 callback 中的 `time.monotonic()` 属于主机单调时钟；当前解码位于分发路径，不能把它当作精确 UART 到达或硬件采样时刻。它们可反映应用接收/分发节奏，不能和设备启动时基直接相减声称端到端延迟，也不证明多设备同步。
- 电机 `s.raw.status_timestamp_ms` 是设备最近一次实际状态反馈时刻；结合 `monitor_version`/有效标志检查其推进，避免“主机持续收到旧状态”假稳定。`monitor_version=0` 表示扩展缺失，零填充字段不能当作已测零故障。
- `control_stats.actual_hz` 的窗口由统计查询推进；查询时控制已停止会返回 0。运动段应在 `controller.stop()` 前取末次统计，并记录 `sample_ms`。还应使用 `loop_count` 差分与窗口时长独立交叉核对，避免启动/停止部分污染稳态窗口。
- 当前 SDK 文档将 `timeout_count` 描述为 reserved，但上述候选固件已在 host watchdog 首次进入 T1 时递增。按已核实固件语义解释，不把旧固件的零值当作无超时证明。当前固件控制周期是 500 Hz；SDK 状态驱动的目标提交通常是 100 Hz，这是不同指标。

## 6. 量化记录与判据

| 记录域 | 字段/计算 |
| --- | --- |
| 区段 | run_id、A/B/A 次序、设定频率、预热/测量起止、墙钟时间与单调时长、诊断查询时点、完整参数 |
| Motor | host_time、callback 单调时间、monitor_version、status_timestamp_ms、actual/target pos/vel/torque、温度、status、stop_reason、fault/latched_fault_code；速率、帧间隔 p50/p95/p99/max、反馈新鲜度 |
| IMU | host_time、callback 单调时间、mcu_timestamp_us、seq、boot_session_id、所有 timestamp 标志、valid_flag、accel m/s²、gyro rad/s、mag μT、温度；新样本率、设备时间间隔、有效位缺失率 |
| 固件控制 | running/mode、target_hz/actual_hz、period_ms/sample_ms、target_seq/applied_seq、loop/error/deadline_miss/timeout 差分、last_error、target_age_ms、target_update_hz |
| 固件 UART | tx_bytes_ok/tx_calls_ok、tx_fail_timeout/tx_fail_other、rx_bytes/rx_errors/rx_rearms/rx_overflow、debug_tx_bytes/log_dropped 的差分 |
| SDK Transport | bytes/frames、crc_errors/resync_bytes/parser_overflow_bytes、queue_dropped/queue_high_water、callback_max_us、ack_timeouts/retries/callback_exceptions；采集自身丢样另计 |
| 运动质量 | 每程起止时间、目标和实际位置、末端误差、完成率、速度均值/标准差/峰峰值、反向帧、力矩/温度峰值；保持阶段滑移、外部力或位移测量 |

所有累计计数按真实位宽做无符号差分，检测设备重启后开启新段，不跨重启直接相减。`callback_max_us` 等历史最大值和高水位不能直接当作区段差值。保存原始时间序列，摘要同时报告分位数和最坏间隔，不能只报告平均帧率。

正常稳定段的硬失败包括：非预期运动/夹持失效/复位/失能、控制器 FAULT、控制错误或 host timeout 增量、UART 发送失败或 RX 溢出、CRC/parser 错误或分发/采集队列丢样、运行中无法解释的时间回退/会话切换、反馈长期不推进、应有流消失。旧故障与新故障分开；在初始故障来源未解释前不能直接继续运动。

`deadline_miss_count` 增加说明发生了周期未按期完成，应列为实时性异常并保留上下文，不能因轨迹最终到位忽略。频率偏差、p99/最大间隔、deadline miss 比例、位置误差、速度纹波、力矩/温升、最大无数据时间、停止时间和长期完成率的产品验收上限必须在运行前由需求/台架负责人给定。尚无阈值时报告 A/B/A 原始值与差异，结论为“已测量、验收阈值待定”，不能自行指定通用百分比后宣称不影响电机。

即便应用数据完全有效，静态加速度幅值、陀螺零偏、磁场合理性与轴向仍须独立验证。电机通断电、不同力矩和姿态会影响磁力计；不把磁航向当作电机附近的绝对真值。

## 7. 源码可复用测试与覆盖边界

| 测试 | 证明范围 |
| --- | --- |
| [test_follower_imu_pty.cpp](../cpp/tests/test_follower_imu_pty.cpp) | motor-only 默认、旧固件拒绝不改流、双回调、非法频率和校准协议流程 |
| [test_imu_motor_concurrency.cpp](../cpp/tests/test_imu_motor_concurrency.cpp) | 两类控制器保留预先开启的混合流；特定慢回调时序下故障/零输出；活动回调期间关闭的生命周期，共 5 项。不能据此推断持续积压时能及时识别旧反馈 |
| [test_imu_time.cpp](../cpp/tests/test_imu_time.cpp)、[test_imu_time.py](../python/tests/test_imu_time.py) | 时间协商、回绕/歧义、会话、重复样本等协议与绑定行为 |
| [test_transport_concurrent_write.cpp](../cpp/tests/test_transport_concurrent_write.cpp)、[test_dispatch_decoupling.py](../python/tests/test_dispatch_decoupling.py)、[test_port_release.py](../python/tests/test_port_release.py) | 并发写入、读/分发分离、端口释放相关行为 |
| [control_ripple.py](../python/examples/control_ripple.py) | 实机速度纹波统计可供复用；现有脚本包含实际运动且无 IMU A/B 参数，不能直接等同本文完整验收 |

固件仓库的 `tools/slave_imu_test/run.sh` 运行生产 service/stream/寄存器/启动代码的主机替身测试，`tools/timebase_test/run.sh` 运行时间、双角色协议及主爪流回归。替身中的 RTOS、SPI、CAN、Flash 不覆盖真实抢占、电气故障或电机运动。对固件建立独立源码身份和结果记录，不能将 SDK 测试 SHA 当作固件验证身份。

## 8. 软件验证命令

以下从本 SDK 仓库根目录执行，使用已配置好依赖的测试环境。CMake 与现有 CI 同样构建核心、Python、示例和测试；生成的本地扩展不代表已安装到其他业务环境。不要在运行硬件控制的进程旁并行运行会占用相同端口的测试。

```bash
cmake -S . -B build -G Ninja \
  -DTACCAP_BUILD_TESTS=ON -DTACCAP_BUILD_EXAMPLES=ON \
  -DTACCAP_BUILD_PYTHON=ON \
  -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build

# 有关功能先单独检查；随后执行完整软件回归。
./build/cpp/tests/taccap_unit_tests \
  --gtest_filter='FollowerImu.*:ImuMotorConcurrency.*'
./build/cpp/tests/taccap_unit_tests
env -u PYTHONPATH python -m pytest python/tests -q

# 或使用 CTest 运行已注册的 C++ 测试。
ctest --test-dir build --output-on-failure
```

若另有对应固件仓库 checkout，从该仓库根目录执行，不依赖某台开发机的绝对路径：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
bash tools/slave_imu_test/run.sh
bash tools/timebase_test/run.sh
```

将实际命令、退出码、原始日志、通过/失败/跳过项和环境写入本次运行证据。软件 PASS 与硬件 PASS 分开，CI 未查询时不声称 CI 通过；完整性结论必须列出尚未具备的电机型号、负载、故障场景和长期运行覆盖。

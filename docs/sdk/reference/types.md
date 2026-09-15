# 数据类型

以下类型由 `xense.taccap` 导出，均为不可变 dataclass。
可用 `dataclasses.asdict()` 转换为字典。不要通过修改状态对象控制设备。

## GripperDevice {#gripperdevice}

`Gripper.discover()` 返回的设备描述。

| 字段 | 类型 | 含义 |
|---|---|---|
| `device` | str | MCU 串口路径 |
| `firmware_sn` | str | 固件序列号 |
| `side` | str | left / right / unknown |

## GripperInfo {#gripperinfo}

连接建立时读取的信息，通过 `gripper.info` 获取。

| 字段 | 类型 | 含义 |
|---|---|---|
| `device` | str | MCU 串口路径 |
| `firmware_version` | tuple[int, int, int] | 固件主 / 次 / 补丁版本 |
| `motor_travel_rad` | float | 标定电机行程 rad |
| `reversed` | bool | 原始电机方向是否与开口方向相反 |
| `control_modes` | tuple[str, ...] | position、grasp、impedance |

## GripperLimits {#gripperlimits}

客户接口的输入边界，不是设备的动态可用额度。

| 字段 | 默认值 | 单位 |
|---|---|---|
| `position_min` / `position_max` | 0.0 / 1.0 | 归一化开口 |
| `max_torque_nm` | 1.2 | N·m |
| `max_speed_rad_s` | 50.0 | rad/s |
| `max_kp` | 500.0 | N·m/rad |
| `max_kd` | 5.0 | N·m·s/rad |

## GripperResult {#gripperresult}

有限操作返回的结果；结果返回后控制仍保持活跃。

| 字段 | 类型 / 取值 | 含义 |
|---|---|---|
| `operation` | move / release / grasp / hold | 操作名称 |
| `status` | reached / contact / no_object / blocked | 完成状态 |
| `state` | GripperState | 返回结果时的观测 |

`reached` 为位置到位；`contact/no_object` 用于 grasp 的接触 / 近闭合判断。
`blocked` 保留兼容，当前位置策略不会因障碍自动进入该结果。
很薄物体可能被归为 no_object；contact 不代表独立验证的稳定夹持。

## GripperState {#gripperstate}

`gripper.state` 在活动控制时读缓存；未启动 / 已停止控制时会同步查询串口，
返回无活动目标的 idle / fault 状态。字段不可修改。

| 字段 | 类型 / 含义 |
|---|---|
| `position` | float，按原标定归一化并截断到 `[0,1]` 的反馈开口 |
| `velocity_rad_s` | float，电机原始坐标带符号速度 |
| `torque_nm` | float，电机估计扭矩 |
| `temperature_c` | float，电机温度反馈 ℃ |
| `target` | float 或 None，最近操作的归一化目标；没有活动目标时 None |
| `control_mode` | `idle` / `position` / `grasp` / `impedance`，客户控制意图 |
| `mode` | str，原生执行阶段，兼容字段，如 `moving_position`、`holding_position`、`closing`、`holding_force`、`fault` |
| `age_ms` | float，观测年龄；大不代表静止，代表数据不够新 |
| `fault` | str 或 None，SDK / 电机 / 固件错误信息 |
| `blocked` | bool，旧结果兼容；当前位置跟随策略不锁存受阻 |
| `seq` | int，主机接收的反馈计数；用来去重，不是模型动作 ID |
| `command_id` | int，原生目标代次；与固件 target/applied 序号不是同一个计数器 |
| `ready` | bool，当前代次已完成主机提交且看到后续反馈；不是精确的电机执行 ACK |
| `position_tolerance` | float，归一化到位容差 |
| `torque_limit_nm` | float 或 None，当前 SDK 请求预算 |
| `effective_torque_limit_nm` | float 或 None，固件包络额度；不含 SDK / 电机全部限制的合成结果 |
| `target_limited` | bool 或 None，固件变换了提交目标；不是专指热降额 |
| `target_error_rad` | float 或 None，位置模式中含补偿的原始电机目标误差 |
| `motor_tolerance_rad` | float，原始电机误差的到位容差 |

执行遥测尚不可用时，相应可选字段为 None，不能当作 0。
在高频更新时，`ready` 可能不断被更新的目标代次清除；不要单凭它统计丢包。

计算属性：

| 属性 | 含义 |
|---|---|
| `position_error` | `target-position`，无目标时 None；不含补偿，单位为归一化行程 |
| `reached` | 无故障、年龄 < 350 ms、有目标、非阻抗 / blocked、低速且目标误差达标 |
| `contact` | `mode==holding_force` 且无故障、年龄 < 350 ms；接触推断，不是夹稳 |
| `moving` | 无故障、年龄 < 350 ms 且速度绝对值 > 0.05 rad/s |

`reached` 在有 `target_error_rad` 时使用该原始误差与 `motor_tolerance_rad`；
没有时使用归一化误差。
单次 `reached` 不包含等待方法的稳定时间、ready 或最终执行查询。

## GripperTiming {#grippertiming}

`gripper.timing` 不查询串口；控制未启动 / 停止后为 None。统计范围为当前原生运行会话。

| 字段 | 含义 |
|---|---|
| `configured_hz` | 配置的 SDK / 反馈频率 |
| `feedback_hz`、`command_hz` | 会话累计帧数除以运行时间，非滑动窗口、非 Python 调用频率 |
| `feedback_frames`、`command_frames` | 实际反馈 / 提交计数 |
| `max_feedback_gap_ms`、`max_command_gap_ms` | 已观测最大帧间隔 |
| `last_submit_latency_ms`、`max_submit_latency_ms` | 目标被 SDK 接受至主机 UART 写完的耗时；被覆盖而未发送的目标不计 |
| `execution_telemetry` | 是否收到扩展执行状态遥测；1.1.7 兼容路径不具备该流 |

以上均不等于端到端运动延迟或硬实时保证。固件状态与电机反馈也不是原子事务快照。

## GripperDiagnostics {#gripperdiagnostics}

`gripper.diagnostics()` 同步查询固件 0x56，返回：

| 字段 | 含义 |
|---|---|
| `owner` | `idle` / `host` / `homing` / `ota` / `fault` / `unknown` |
| `error_code` | 固件错误码 |
| `effective_torque_limit_nm` | 固件包络额度，Nm |
| `target_limited` | 固件最近目标经过变换 |
| `output_failed` | 固件执行失败标志 |
| `target_sequence`、`applied_sequence` | MCU 接收 / 下发目标序号，非 SDK 的 command_id |

更完整的 requested/applied 五参数、归零和包络状态可读
`gripper.advanced.motor.execution_status()`；它也是同步查询。
在实时循环外执行，参见[状态与诊断](../guides/observations.md)。

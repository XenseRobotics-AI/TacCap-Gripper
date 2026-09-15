# 状态与诊断

## 读取控制状态

```python
# 在已启动的 gripper 会话中
state = gripper.state
print(state.position, state.target, state.torque_nm)
print(state.control_mode, state.reached, state.fault)
```

控制活跃时读取缓存，不额外查询串口；尚未启动或已经停止时，`state` 会同步查询设备。
完整字段见 [GripperState](../reference/types.md#gripperstate)。

| 常用字段 | 用途 |
|---|---|
| `position`、`target` | 反馈与请求开口；没有活动目标时 target 为 None |
| `target_error_rad` | 位置模式中含补偿的电机目标误差 |
| `velocity_rad_s`、`torque_nm`、`temperature_c` | 电机运动与输出反馈 |
| `control_mode` | position / grasp / impedance / idle |
| `fault`、`age_ms` | 故障信息与反馈新鲜度 |
| `seq` | 反馈计数，用于去重 |

`reached` 是一次观测的到位判断；`wait_result()` 还检查新反馈、稳定窗口和执行状态。
`ready` 表示当前目标代次已提交并收到后续反馈，不是精确的电机执行 ACK。
流式目标快速覆盖时它可能经常为 False，不宜单独用于统计丢帧。

## 请求与输出限值

`torque_limit_nm` 表示 SDK 当前请求预算。
`effective_torque_limit_nm` 表示 MCU 动态包络额度，不包含所有 SDK / 电机限制的合成结果。
`target_limited` 表示 MCU 变换过目标，不专指热降额。

可选字段为 None 时表示尚不可用，不表示零。
电机扭矩反馈是估计值，不是爪尖力传感器读数。

## 时间统计

```python
from dataclasses import asdict

timing = gripper.timing
if timing is not None:
    print(asdict(timing))
```

统计包含会话平均发送 / 反馈频率、累计帧数、最大间隔与提交耗时。
停止后为 None，重新启动会建立新会话统计。

`last_submit_latency_ms` 以 SDK 接受目标为起点，以主机 UART 写完为终点，
不等于电机起动或到位延迟。`feedback_hz` 也不等于 Python 读取属性的频率。

实际测量方法见[控制与读取频率测试](frequency-test.md)。

## 显式查询诊断

```python
diagnostics = gripper.diagnostics()
print(diagnostics.owner, diagnostics.error_code)
print(diagnostics.effective_torque_limit_nm)
```

此调用同步查询固件，宜在高频循环外执行。返回控制所有者、固件错误、限值与目标序号。
SDK `command_id`、反馈 `seq`、MCU 的目标 / 下发序号是不同计数器，不直接对应。

排障时同时记录设备信息、执行命令、状态、时间统计与诊断，避免仅依靠某一个布尔标志。

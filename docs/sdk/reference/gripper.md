# Gripper

```python
from xense.taccap import Gripper
```

`Gripper` 管理一台从爪的连接、控制与观测。支持上下文管理器；活动会话退出时请求失能并关闭连接。
本页示例中的 `gripper` 表示已经建立的连接，完整程序见[快速开始](../getting-started/quickstart.md)。

## 构造函数 {#constructor}

```text
Gripper(target=None, *, position_tolerance=0.01, closed_tolerance=0.03,
        settle_time=0.05, torque_limit_nm=0.35, speed_rad_s=0.5,
        update_hz=100, close_compensation_rad=0.0)
```

建立被动连接并读取版本与标定。不会自动标定、清错或启动运动。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `target` | str / GripperDevice / None | None | left、right、固件 SN、`/dev/...` 路径或发现结果；省略时要求唯一从爪 |
| `position_tolerance` | float | 0.01 | 归一化到位容差，范围 `(0,0.1]` |
| `closed_tolerance` | float | 0.03 | grasp 的近闭合判断，范围 `[position_tolerance,0.1]` |
| `settle_time` | float | 0.05 | 到位等待的稳定时间，秒，范围 `[0,5]` |
| `torque_limit_nm` | float | 0.35 | 位置控制默认预算，N·m，范围 `(0,1.2]` |
| `speed_rad_s` | float | 0.5 | 位置参考默认变化速率，rad/s，范围 `(0,50]` |
| `update_hz` | int | 100 | 20、25、40、50 或 100，不接受 float / bool |
| `close_compensation_rad` | float | 0.0 | 闭合补偿，rad，范围 `[0,0.05]`；非零要求固件 ≥ 1.1.8 |

数值必须有限。构造要求从爪固件 ≥ 1.1.7 以及有效行程标定。

**异常**：参数不合法时 `ValueError` / `TypeError`；设备选择、版本或标定不满足时
`GripperError`；底层连接失败时可能传播 `IoError` / `ProtocolError` / `TimeoutError`。

```python
with Gripper("left", torque_limit_nm=0.35, update_hz=100) as gripper:
    print(gripper.info)
```

## discover {#discover}

```text
Gripper.discover() -> tuple[GripperDevice, ...]
```

只读枚举从爪，不使能电机。无设备时返回空元组。

**返回**：[GripperDevice](types.md#gripperdevice) 元组。
**异常**：设备探测可能传播底层 I/O 错误。

```python
for device in Gripper.discover():
    print(device.firmware_sn, device.side, device.device)
```

## start {#start}

```text
start() -> None
```

初始化反馈与后台控制，使能并先保持当前位置。活动会话中再次调用不创建第二个控制器。
建议在定时循环前调用；首次运动方法也会自动启动。

**返回**：None。
**异常**：故障时 `FaultError`，控制被占用时 `BusyError`，以及底层 I/O / 配置错误。

```python
gripper.start()
```

## set_position {#set-position}

```text
set_position(position: float) -> None
```

使用构造默认预算和速度，持续跟随最新目标。新目标覆盖旧目标，不等待到位。

| 参数 | 说明 |
|---|---|
| `position` | 有限归一化开口 `[0,1]`，0 闭合、1 张开 |

**返回**：None。
**异常**：越界时 `ValueError`；启动 / 运行故障可能传播 `GripperError`、`FaultError`、`BusyError` 或底层错误。

```python
gripper.set_position(0.5)
```

## move {#move}

```text
move(position, *, torque_nm=None, torque_limit_nm=None,
     speed_rad_s=None, wait=False, timeout=5.0) -> GripperResult | None
```

设置位置目标，可覆盖预算与速度，并可等待到位。受阻时仍朝目标施加有界力矩。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `position` | 必填 | 有限开口 `[0,1]` |
| `torque_limit_nm` | None | 预算 `(0,1.2]` N·m；None 使用连接默认值 |
| `speed_rad_s` | None | 参考速率 `(0,50]` rad/s；None 使用连接默认值 |
| `wait` | False | 是否等待到位，必须是 bool |
| `timeout` | 5.0 | 有限正秒数；即使 wait=False 也校验 |
| `torque_nm` | None | torque_limit_nm 的兼容别名，两者不可同时指定 |

**返回**：非阻塞时 None；阻塞成功返回 [GripperResult](types.md#gripperresult)。
**异常**：参数错误、启动错误，以及等待期间的 `TimeoutError` / `FaultError` / `OperationInterrupted`。
等待超时后操作仍活跃。覆盖参数不改变连接默认值。

```python
result = gripper.move(0.8, torque_limit_nm=0.6, speed_rad_s=1.0,
                      wait=True, timeout=10.0)
```

## release {#release}

```text
release(position=1.0, *, torque_limit_nm=None, speed_rad_s=None,
        wait=False, timeout=5.0) -> GripperResult | None
```

使用位置控制移动到释放开口，默认最大张开。无需先调用 grasp。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `position` | 1.0 | `[0,1]`，应选择足以释放物体的开口 |
| `torque_limit_nm` | None | `(0,1.2]` N·m，None 使用连接默认值 |
| `speed_rad_s` | None | `(0,50]` rad/s，None 使用连接默认值 |
| `wait` | False | 是否等待到位 |
| `timeout` | 5.0 | 有限正秒数 |

**返回 / 异常**：与 move 相同，结果 operation 为 `release`。

```python
result = gripper.release(0.8, wait=True, timeout=10.0)
```

## grasp {#grasp}

```text
grasp(*, torque_nm=0.35, speed_rad_s=0.5,
      wait=False, timeout=5.0) -> GripperResult | None
```

朝闭合端运动，推断接触后保持有界前馈扭矩。不使用闭合补偿。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `torque_nm` | 0.35 | 正的闭合请求 `(0,1.2]` N·m，方向由 SDK 处理 |
| `speed_rad_s` | 0.5 | 闭合参考速率 `(0,50]` rad/s |
| `wait` | False | 是否等待接触 / 近闭合结果 |
| `timeout` | 5.0 | 有限正秒数 |

此方法的扭矩 / 速度默认值独立于构造默认值。
**返回**：None 或 GripperResult，状态为 `contact` / `no_object`；不保证稳定夹持。
**异常**：参数 / 启动错误，以及等待超时、故障或操作替换错误。

```python
result = gripper.grasp(torque_nm=0.35, wait=True, timeout=10.0)
```

## impedance {#impedance}

```text
impedance(position, *, kp, kd, feedforward_torque_nm=0.0,
          velocity_rad_s=0.0, torque_limit_nm=1.0) -> None
```

提交持续阻抗目标，无到位结果，不使用闭合补偿。

| 参数 | 默认值 | 单位 / 范围 |
|---|---|---|
| `position` | 必填 | 归一化 `[0,1]` |
| `kp` | 必填 | N·m/rad，`[0,500]` |
| `kd` | 必填 | N·m·s/rad，`[0,5]` |
| `feedforward_torque_nm` | 0.0 | 原始电机坐标 N·m，绝对值不超过本次预算 |
| `velocity_rad_s` | 0.0 | 原始电机坐标 rad/s，`[-50,50]` |
| `torque_limit_nm` | 1.0 | `(0,1.2]` N·m；独立于构造默认值 |

**返回**：None。
**异常**：参数错误、启动 / 运行错误。对此模式调用 wait_result 会抛 GripperError。

```python
gripper.impedance(0.5, kp=5.0, kd=0.5, torque_limit_nm=0.35)
```

## hold {#hold}

```text
hold(*, wait=False, timeout=5.0) -> GripperResult | None
```

取消旧目标，保持当前原始电机位置。不会重新叠加闭合补偿，也不保证维持原抓取力矩。
`wait` 为 bool；`timeout` 为有限正秒数。

**返回 / 异常**：与 move 的等待规则相同，结果 operation 为 `hold`。

```python
gripper.hold(wait=True)
```

## wait_result {#wait-result}

```text
wait_result(timeout=5.0) -> GripperResult
```

等待当前有限操作完成。检查提交后的反馈、位置 / 速度条件、稳定窗口及最终执行状态。
`timeout` 为有限正秒数。

**返回**：[GripperResult](types.md#gripperresult)。
**异常**：无可等待操作 / 阻抗模式时 GripperError；被替换时 OperationInterrupted；
故障时 FaultError；超时时 TimeoutError，附加 `state` 属性，原操作仍活跃。

```python
gripper.move(0.8)
result = gripper.wait_result(timeout=10.0)
```

## wait {#wait}

```text
wait(timeout=5.0) -> GripperState
```

兼容等待方法，与 wait_result 的参数、完成条件和异常相同，但只返回结果状态。
新代码需要区分操作结果时使用 wait_result。

```python
gripper.move(0.8)
state = gripper.wait(timeout=10.0)
```

## disable / stop {#disable}

```text
disable() -> None
stop() -> None
```

停止控制并请求电机失能；`stop` 为兼容别名。
即使本对象未启动控制，也可以显式发出失能请求。

**返回**：None。
**异常**：关闭后的连接抛 GripperError，失能 / 通讯失败会传播底层异常。
持物可能松开，失败时设备是否已停不能视为已确认。

```python
gripper.disable()
```

## clear_fault {#clear-fault}

```text
clear_fault() -> None
```

先失能，再清除并检查固件故障。保持空闲，不自动恢复旧目标。

**返回**：None。
**异常**：清错后仍有故障时 FaultError，或底层通讯 / 失能错误。

```python
gripper.clear_fault()
```

## close {#close}

```text
close() -> None
```

关闭连接；自身有活动控制或状态不确定时先请求失能。纯被动连接关闭不主动停止其他控制者。
重复关闭直接返回。

**返回**：None。
**异常**：可能传播失能错误；连接清理仍会执行。

```python
gripper.close()
```

## diagnostics {#diagnostics}

```text
diagnostics() -> GripperDiagnostics
```

同步查询固件执行状态，宜在高频循环外调用。

**返回**：[GripperDiagnostics](types.md#gripperdiagnostics)。
**异常**：关闭连接时 GripperError；查询失败时 ProtocolError / IoError / TimeoutError。

```python
diagnostics = gripper.diagnostics()
```

## 属性 {#properties}

| 属性 | 返回类型 | 行为 |
|---|---|---|
| `info` | [GripperInfo](types.md#gripperinfo) | 缓存的连接与行程信息 |
| `limits` | [GripperLimits](types.md#gripperlimits) | SDK 输入边界；亦可通过 Gripper.limits 访问 |
| `state` | [GripperState](types.md#gripperstate) | 活动时缓存，空闲时同步查询；关闭后报错 |
| `timing` | [GripperTiming](types.md#grippertiming) 或 None | 缓存会话统计；无活动运行时为 None，关闭后报错 |
| `advanced` | 原生 FollowerGripper | 设备维护入口；修改配置前停止客户控制；关闭后报错 |

## 兼容导入

`FollowerGripper`、`ControlLoop`、`ForcePositionController`、`OtaSession` 等旧导出继续可用。
新代码可通过 `xense.taccap.advanced` 显式访问。
原生旧 `set_target` 的接触策略与客户 `set_position` 不同，迁移时按使用指南选择行为。

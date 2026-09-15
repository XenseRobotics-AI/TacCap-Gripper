# 阻抗控制

`impedance()` 提供持续的位置、速度、增益与前馈扭矩目标，适用于需要自行设置柔顺性的应用。
它与位置控制共享连接和后台运行时。

## 设置柔顺目标

```python
import time
from xense.taccap import Gripper

with Gripper("left") as gripper:
    gripper.move(0.5, wait=True, timeout=10.0)
    gripper.impedance(0.5, kp=5.0, kd=0.5, torque_limit_nm=0.35)
    for _ in range(100):
        state = gripper.state
        if state.fault:
            raise RuntimeError(state.fault)
        time.sleep(0.02)
    gripper.hold()
```

此示例在中点保持柔顺目标约 2 秒，然后切换到当前位置保持。

## 参数

| 参数 | 含义 |
|---|---|
| `position` | 归一化位置目标 `[0,1]` |
| `kp` | 位置刚度，N·m/rad |
| `kd` | 速度阻尼，N·m·s/rad |
| `velocity_rad_s` | 带符号电机速度参考，默认 0 |
| `feedforward_torque_nm` | 带符号电机前馈扭矩，默认 0 |
| `torque_limit_nm` | 总体请求预算，默认 1.0 N·m |

未限幅时控制请求由位置项、速度项与前馈叠加。SDK 与固件对请求施加预算，
不能只把前馈参数当作总输出上限。所有参数范围见[API 参考](../reference/gripper.md#impedance)。

位置使用统一开口坐标；速度和前馈使用电机原始坐标系，方向参考 `info.reversed`。
`kp=kd=0` 可以表达纯前馈请求，仍受固件连续输出限制。

## 执行与结束

`impedance()` 非阻塞，后续调用覆盖之前的目标，没有到位完成结果。
不能对其调用 `wait_result()`。使用 `state` 观察，使用 `hold()` 切回当前位置保持，
或 `disable()` 失能。该模式不应用闭合补偿。

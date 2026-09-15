# 位置控制

位置控制适用于开口设定、遥操和模型推理。使用 `set_position()` 持续更新目标，
或使用 `move()` 为单次动作指定参数并等待到位。

## 提交目标

```python
import time
from xense.taccap import Gripper

with Gripper("left", torque_limit_nm=0.35, speed_rad_s=0.5) as gripper:
    gripper.start()
    gripper.set_position(0.5)
    for _ in range(100):
        state = gripper.state
        if state.fault:
            raise RuntimeError(state.fault)
        print(state.position)
        time.sleep(0.02)
```

`start()` 在定时循环前完成初始化；此后 `set_position()` 更新后台目标，
不等待到位。示例观察 2 秒后退出，不把观察时长解释为到位保证。

## 单次参数覆盖

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    result = gripper.move(
        0.8,
        torque_limit_nm=0.6,
        speed_rad_s=1.0,
        wait=True,
        timeout=10.0,
    )
    print(result.status)
```

参数仅作用于当前目标。后续 `set_position()` 仍使用构造时的默认预算和速度。
速度参数限制参考变化率，实际机械速度可能受负载、输出约束或瞬态影响。

## 位置夹持与释放

给出小于物体开口的目标，夹爪会接近物体并持续有界施力。无需切换抓取模式：

```python
import time
from xense.taccap import Gripper

with Gripper("left", torque_limit_nm=0.35) as gripper:
    gripper.release(wait=True, timeout=10.0)
    input("放好物体并移开手指，按 Enter 闭合：")
    gripper.set_position(0.0)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        state = gripper.state
        if state.fault:
            raise RuntimeError(state.fault)
        time.sleep(0.02)
    input("支撑物体，按 Enter 张开：")
    gripper.release(wait=True, timeout=10.0)
```

被物体阻挡时 `reached=False` 可以是正常持物状态，不能用 `move(wait=True)` 判断是否夹稳。
上述示例保持 5 秒，不执行物体检测。退出会话或失能会降低 / 撤去输出。

`hold()` 会取消旧目标并保持当前观测点，可能减少压紧力。
需要保持同一夹持意图时，保留原目标即可。

## 闭合补偿 {#closing-compensation}

固件 ≥ 1.1.8 时可设置闭合方向偏移：

```python
from xense.taccap import Gripper

with Gripper("left", close_compensation_rad=0.03) as gripper:
    gripper.move(0.0, wait=True, timeout=10.0)
```

`0.03 rad` 为配置示例，应按设备确认。默认 0，范围 `[0,0.05]`。
偏移按 `compensation × (1 - position)` 衰减：闭合使用全量，最大张开不补偿。
它不修改 Flash 标定，不是全行程双向背隙模型。

反馈开口仍按原标定归一化并截断到 0..1；补偿执行情况通过 `target_error_rad`
判断，到位检测使用补偿后的电机误差。不要发送负开口实现补偿。

相关接口：[set_position](../reference/gripper.md#set-position)、[move](../reference/gripper.md#move)、
[release](../reference/gripper.md#release)、[hold](../reference/gripper.md#hold)。

# 抓取与释放

`grasp()` 用于接近物体、推断接触并保持电机扭矩。
如果应用只输出目标开口，可直接采用[位置夹持](position-control.md)，无需切换本模式。

## 完成一次抓取

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    gripper.release(wait=True, timeout=10.0)
    input("放好物体并移开手指，按 Enter 抓取：")
    result = gripper.grasp(torque_nm=0.35, speed_rad_s=0.5,
                           wait=True, timeout=10.0)
    print(result.status)
    if result.status == "contact":
        input("保持中；支撑物体后按 Enter 释放：")
    gripper.release(wait=True, timeout=10.0)
```

返回结果后控制保持活跃；不要在持物任务完成前退出 `with`。

## 结果语义

| 状态 | 含义 |
|---|---|
| `contact` | 根据电机运动与扭矩推断接触，随后保持有界前馈扭矩 |
| `no_object` | 到达或接近闭合端；未确认有效物体接触 |

靠近闭合端的判断由 `closed_tolerance` 配置，默认 0.03。
很薄的物体可能被判为 `no_object`。接触推断不包含防滑或独立夹持力测量。

`torque_nm` 为正的闭合扭矩请求，方向由 SDK 处理；实际输出仍受固件包络限制。
静止 / 温度降额会降低允许输出。

## 与位置模式的区别

`grasp` 使用自己的接触状态机，默认 0.35 N·m / 0.5 rad/s，不继承构造时的所有默认值。
它不应用 `close_compensation_rad`。`release()` 使用位置控制路径，默认完全张开，
也可设置更小的释放开口。

调用 `set_position()`、`impedance()` 或 `hold()` 会替换抓取意图。
`hold()` 是当前位置保持，不等价于维持抓取扭矩。

相关接口：[grasp](../reference/gripper.md#grasp)、[release](../reference/gripper.md#release)。

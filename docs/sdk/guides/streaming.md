# 遥操与模型推理

应用负责产生目标开口，SDK 负责周期执行。推荐使用一个长生命周期 `Gripper`，
在循环前 `start()`，随后提交最新目标并读取状态缓存。

## 接口约定

- 动作使用归一化开口 `[0,1]`，0 闭合、1 张开。
- 新目标替换旧目标；应用不需要按状态流频率重复提交相同目标。
- 模型动作块由应用选择当前有效点；SDK 不保存动作队列。
- `set_position()` 非阻塞，流式控制不逐条调用 `wait_result()`。

例如模型以 20 Hz 产生动作，SDK 可用 100 Hz 刷新，在两条动作之间维持最后目标。
SDK 提供参考限速，不提供完整的加速度 / jerk 轨迹规划。

## 接入函数

下面的函数可直接嵌入应用。`read_latest()` 为调用者实现的非阻塞函数，返回
`(position, monotonic_timestamp)`；尚无目标时返回 `None`。
时间戳需与本进程 `time.monotonic()` 使用同一时钟，远端时钟应先由应用转换。

```python
import time
from xense.taccap import Gripper

def run_gripper(read_latest, should_stop, device="left"):
    with Gripper(device, torque_limit_nm=0.35, speed_rad_s=0.5,
                 update_hz=100) as gripper:
        gripper.start()
        previous_timestamp = None
        deadline = time.monotonic()
        while not should_stop():
            sample = read_latest()
            now = time.monotonic()
            if sample is None or not 0 <= now - sample[1] <= 0.5:
                raise RuntimeError("Target is missing or stale")
            position, timestamp = sample
            if timestamp != previous_timestamp:
                gripper.set_position(position)
                previous_timestamp = timestamp
            state = gripper.state
            if state.fault:
                raise RuntimeError(state.fault)
            deadline += 0.01
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
```

先准备第一条目标再启动该函数。示例在目标过期时退出并失能；
持物应用应自行决定过期后的行为。循环错过周期时跳过积压，不追赶补发旧目标。

## 目标过期与通讯超时

MCU 的 500 ms 超时监测 SDK 是否持续提交有效运动目标。
即使模型不再更新，只要 SDK 后台仍刷新旧目标，MCU 就不会触发该超时。

应用需检查动作时间戳。上例的检查只在应用循环仍能调度时有效；
SDK 当前没有独立监测 Python 生产者停更的原生目标 watchdog。
进程完全停止或通讯断开则由已有通讯 / 反馈保护处理。

## 读取反馈

活动控制中的 `gripper.state` 与 `gripper.timing` 读取缓存。
根据 `state.seq` 去重，不要将重复读取缓存计为新的硬件反馈。

不要在定时循环内执行设备发现、重新连接、标定、OTA 或同步 `diagnostics()`。
图像处理、模型推理和日志写盘宜在独立的生产 / 消费流程执行。

时间统计的具体定义见[状态与诊断](observations.md)。

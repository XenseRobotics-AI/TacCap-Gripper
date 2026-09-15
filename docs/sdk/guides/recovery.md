# 停止与故障恢复

## 停止行为

| 操作 | 行为 |
|---|---|
| `hold()` | 取消旧目标，保持当前观测位置 |
| `disable()` / `stop()` | 停止控制并请求电机失能 |
| 活动会话 `close()` / 退出 `with` | 请求失能并关闭连接 |
| 等待到位超时 | 抛异常，原操作仍活跃 |

失能可能释放夹持物体。失能请求若失败会传播异常，不能据此认定设备已经停止。

## 处理到位超时

```python
from xense.taccap import Gripper, TimeoutError

with Gripper("left") as gripper:
    try:
        gripper.move(0.8, wait=True, timeout=10.0)
    except TimeoutError as exc:
        print(exc.state)
        gripper.disable()
```

示例选择超时后停止。应用也可以发送新目标，但应先判断位置受阻、参考速度不足或反馈异常。
目标被物体挡住时无法到位，不应仅通过扩大等待时间判断是否夹稳。

## 清除故障

确认电源、通讯、机械障碍等故障原因已处理后，在现有连接内执行：

```python
gripper.clear_fault()
# 清错后仍为空闲；确认允许恢复运动时再给目标
gripper.move(0.8)
```

`clear_fault()` 会先失能，再清除和检查固件故障，不恢复旧动作。
不要以自动循环清错替代故障处理。

## 通讯与占用

MCU 500 ms 未收到有效目标会请求失能并锁存故障；读取状态不能续期。
SDK 活动反馈超过 350 ms 会报告 stale fault。
应用目标停更但 SDK 仍刷新旧值的行为，见[遥操与模型推理](streaming.md)。
当前主机通讯保护不覆盖 MCU 停摆或电机 CAN 断线后的所有情况。

`BusyError` 表示其他控制、归零或 OTA 正在占用；等待对应流程结束后再启动。
同一设备不要被多个进程或 GUI 同时控制。

异常类别与捕获方式见[异常参考](../reference/errors.md)。

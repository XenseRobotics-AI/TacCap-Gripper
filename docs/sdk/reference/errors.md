# 异常

| 异常 | 处理 |
|---|---|
| `ValueError` / `TypeError` | 输入范围、非有限值或类型不正确；不应靠重试解决 |
| `GripperError` | 设备选择不唯一、版本 / 标定不满足、已关闭、无可等待操作等 |
| `BusyError` | 其他控制、归零或 OTA 正在占用 |
| `FaultError` | 反馈 / 固件 / 电机故障；处理原因，显式 `clear_fault()` 后再给新目标 |
| `OperationInterrupted` | 等待的操作被新命令、停止或关闭替换 |
| `TimeoutError` | 原生 I/O 或操作等待超时；仅运动等待超时附 `exc.state`，此时操作仍运行 |
| `ProtocolError` | 固件拒绝 / 协议错误；检查版本、角色、配置和错误码 |
| `IoError` | 设备路径、权限、断连、串口等 I/O 问题 |

`FaultError`、`BusyError`、`OperationInterrupted` 继承 `GripperError`（再继承 `RuntimeError`）。
原生 I/O / 协议异常独立导出，不保证都能用 `except GripperError` 捕获。


## 捕获示例

```python
from xense.taccap import Gripper, FaultError, TimeoutError

with Gripper("left") as gripper:
    try:
        gripper.move(0.8, wait=True, timeout=10.0)
    except TimeoutError as exc:
        print(getattr(exc, "state", None))
        gripper.disable()
    except FaultError as exc:
        print(exc)
        # 处理故障原因后，才显式清错并重新提交目标
```

上下文退出会清理活动会话。等待超时本身不停止执行；
只有运动等待产生的 TimeoutError 保证附有 `state`，底层 I/O 超时不具有该保证。

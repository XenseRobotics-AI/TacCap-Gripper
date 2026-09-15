# 第一次控制夹爪

本节完成设备发现、一次位置移动和资源释放。需要一台已标定的从爪，固件版本 ≥ 1.1.7。

## 1. 发现设备

```python
from xense.taccap import Gripper

for device in Gripper.discover():
    print(device.firmware_sn, device.side, device.device)
```

发现不会使能电机。`side` 表示左右侧，`firmware_sn` 是固件序列号。
多台同侧从爪连接时，请使用序列号或设备路径。

## 2. 连接并检查信息

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    print(gripper.info)
```

连接时会检查固件与行程标定，不发运动命令。设备固件若启用了上电自动标定，
请等启动标定完成后再控制。

## 3. 移动到目标开口

清空夹爪运动范围后运行：

```python
from xense.taccap import Gripper

with Gripper("left", torque_limit_nm=0.35, speed_rad_s=0.5) as gripper:
    result = gripper.move(0.8, wait=True, timeout=10.0)
    print(result.status)
    print(result.state.position)
```

- `0.8` 表示 80% 标定开口，`0` 闭合，`1` 最大张开。
- 第一次运动自动使能；`wait=True` 等待到位并返回 `GripperResult`。
- 成功状态为 `reached`；超时会抛出 `TimeoutError`。
- 退出 `with` 时请求失能。如果正在持物，物体可能松开。

## 4. 连续修改目标

使用仓库交互示例：

```bash
python python/examples/position_control.py left
```

确认后输入 `0..1` 设置开口，输入 `s` 查看状态，输入 `q` 退出。
夹爪在两次输入之间继续执行最后目标。

程序接入使用 `set_position()`，参见[位置控制](../guides/position-control.md)；
模型或遥操输出参见[流式接入](../guides/streaming.md)。

# TacCap-Gripper SDK

TacCap-Gripper SDK 用于连接 TacCap 设备、控制从爪运动、读取主爪与电机状态，
以及采集腕部相机图像。Python 模块为 `xense.taccap`。

从爪控制使用 `Gripper`。应用提交目标开口，SDK 在后台持续执行；通过状态接口读取反馈，
通过上下文管理器释放控制资源。

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    result = gripper.move(0.8, wait=True, timeout=10.0)
    print(result.status, result.state.position)
```

此示例将夹爪移动至 80% 开口，结束时失能。首次运行请按照[快速开始](getting-started/quickstart.md)准备设备。

## 开始使用

| 任务 | 文档 |
|---|---|
| 安装 SDK 并连接设备 | [安装](getting-started/installation.md) · [快速开始](getting-started/quickstart.md) |
| 持续控制目标开口 | [位置控制](guides/position-control.md) |
| 接入遥操或模型输出 | [遥操与模型推理](guides/streaming.md) |
| 使用抓取或柔顺控制 | [抓取与释放](guides/grasp.md) · [阻抗控制](guides/impedance.md) |
| 查询方法与数据结构 | [Gripper](reference/gripper.md) · [数据类型](reference/types.md) · [异常](reference/errors.md) |
| 更新设备与标定 | [固件升级](maintenance/firmware.md) · [标定](maintenance/calibration.md) |

## 兼容性

本文档适用于 Python SDK 0.2.0b1 接口。推荐使用 Linux x86_64、Python 3.12。
`Gripper` 要求已标定的从爪及固件 1.1.7 或更高版本；闭合补偿要求固件 1.1.8 或更高版本。
具体安装包以交付文件的平台标签与版本说明为准。

主爪、腕相机和维护功能通过 `xense.taccap.advanced` 提供，已有顶层导入保持兼容。
OG 视触觉传感器使用独立的 `xensesdk`，参见[主爪与相机](guides/sensors.md)。

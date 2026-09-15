# TacCap-Gripper SDK

TacCap 夹爪的 Python / C++ SDK，提供从爪位置与阻抗控制、主爪状态采集、腕相机和设备维护功能。
Python 模块为 `xense.taccap`。当前候选版本为 **0.2.0b1**，配套从爪固件 **1.1.8**。
升级和命名对照见[beta 迁移指南](docs/sdk/guides/migration.md)。

## 安装

```bash
mamba env create -f environment.yml
mamba activate taccap
python -m pip install . --no-build-isolation
```

推荐 Linux x86_64、Python 3.12。wheel 安装、设备权限和环境检查见[安装文档](docs/sdk/getting-started/installation.md)。

## 快速开始

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    result = gripper.move(0.8, wait=True, timeout=10.0)
    print(result.status, result.state.position)
```

目标开口为 0..1，0 闭合、1 最大张开。示例会真实运动，退出时请求失能。
从爪需完成行程标定，固件版本 ≥ 1.1.7。
连续控制使用 `set_position()`，参见[位置控制](docs/sdk/guides/position-control.md)。

## 文档

- [SDK 文档首页](docs/sdk/index.md)
- [第一次控制夹爪](docs/sdk/getting-started/quickstart.md)
- [遥操与模型推理](docs/sdk/guides/streaming.md)
- [API 参考](docs/sdk/reference/gripper.md)
- [固件升级](docs/sdk/maintenance/firmware.md)
- [控制与读取频率测试](docs/sdk/guides/frequency-test.md)
- [示例](docs/sdk/examples.md)
- [更新日志](docs/sdk/changelog.md)

本地浏览与文档构建见[文档维护说明](docs/README.md)。
研发架构、协议约束和实验记录保留在[开发文档](docs/development/index.md)。

旧 `FollowerGripper`、`ControlLoop`、`OtaSession` 等导入继续兼容，
也可通过 `xense.taccap.advanced` 访问。OG 视触觉使用独立 `xensesdk`。

Apache-2.0 · Copyright (c) 2026 XenseRobotics Co., Ltd.

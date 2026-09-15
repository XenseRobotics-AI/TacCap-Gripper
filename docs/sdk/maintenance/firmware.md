# 固件升级

安装 SDK 与升级固件是两个独立操作。SDK 安装不会自动刷写夹爪。

## 选择固件

| 角色 / 功能 | 固件要求 |
|---|---|
| 新 `Gripper` 从爪控制 | slave ≥ 1.1.7 |
| 闭合补偿与扩展执行流 | slave ≥ 1.1.8 |
| 主爪采集 | master 镜像；不得刷入 slave 固件 |

按设备角色选择镜像：SN 尾缀 `s` 为 slave，`m` 为 master。
左右侧与镜像角色无关。使用交付包中明确标注版本与角色的文件，并核对其校验和。

仓库 `firmware/manifest.json` 描述 `firmware/` 中的镜像；其中 slave 1.1.6
不满足新 Gripper 的最低要求。不要将旧镜像与新的版本参数一起使用。

## OTA 升级

停止使用目标设备的运动程序 / GUI，支撑或移走物体。
在 SDK 仓库根目录执行，下列镜像路径替换为收到的实际文件：

```bash
python python/examples/ota_update.py \
  /path/to/tc-gu-01-slave-1.1.8.bin \
  slave --target-version 1.1.8
```

`slave` 要求连接唯一从爪。多台设备时使用完整固件 SN。
脚本会显示设备与镜像信息，确认后分块写入并校验，再重启设备。

`--target-version` 是升级元数据，不修改 bin 内的编译版本。
镜像角色不匹配时应更换文件，不绕过角色检查。
`--all` 使用仓库默认镜像选择，不适用于指定一个新交付镜像的流程。

只读 OTA 状态：

```bash
python python/examples/ota_update.py --get-status slave
```

## 升级后检查

1. 等待脚本完成并提示设备重启。
2. 完整断电再上电，使 MCU 和 USB 转串口桥都重新初始化。
3. 等待固件启动 / 自动标定结束。
4. 读取设备版本与行程，确认后再启动运动。

```python
from xense.taccap import Gripper

with Gripper("left") as gripper:
    print(gripper.info.firmware_version)
    print(gripper.info.motor_travel_rad)
```

旧固件不满足构造要求时，通过 OTA 维护脚本升级，不绕过 Gripper 版本检查。
如果 OTA 脚本无法导入 `OtaSession`，先按[安装说明](../getting-started/installation.md)修复 SDK 环境。

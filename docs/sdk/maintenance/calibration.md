# 标定

主爪编码器、从爪电机行程和腕相机内参使用独立的标定记录。

| 对象 | 标定内容 | 方式 |
|---|---|---|
| 主爪 | 编码器零点与最大开口角 | 手动闭合 / 张开引导 |
| 从爪 | 电机闭合端、张开端与方向 | 固件上电自动标定，取决于设备配置 |
| 腕相机 | 鱼眼内参 | 内参写入 / 读取 |

## 主爪编码器

在 SDK 仓库根目录运行，选择主爪的侧别或完整 SN：

```bash
python python/examples/calibrate.py left
```

按提示将主爪保持在完全闭合处设置零点，再手动打开到机械限位并记录行程。
结果写入 MCU Flash，供 `normalize_position=True` 使用。
同侧连接主爪和从爪时使用完整 SN；此脚本不用于从爪电机标定。

## 从爪行程

固件允许自动标定时，上电会执行闭合找零、张开测行程。
移除夹持物并空出行程后上电，等待完成后再接管控制。
`Gripper` 只读取已有标定，不会自动启动标定或写入替代行程。

标定无效时，维护人员可通过原生连接读取 `get_gripper_config()` 和
`get_auto_cal_config()`，检查行程、方向、有效位与上电设置。
不要用假定的固定行程绕过错误。重新标定后关闭并重建 Gripper，以更新缓存映射。

## 腕相机内参

只读显示已有记录：

```bash
python python/examples/fisheye_cal.py show left
```

程序使用 `calibration.resolve_fisheye()` 取得可用内参及是否为参考回退值。
未标定记录可能为空或全零，不能仅检查是否为 None。
图像去畸变的支持分辨率为 640×480；参考内参不等于该设备的测量标定。

## 闭合补偿与标定

`close_compensation_rad` 是本次 SDK 会话的位置偏移，不写入 Flash。
它适用于已正确标定但仍需要少量闭合修正的设备；不会替代行程或相机标定。
具体规则见[位置控制](../guides/position-control.md#closing-compensation)。

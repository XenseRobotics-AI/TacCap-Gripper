# 主爪与相机

`Gripper` 用于从爪运动。主爪与腕相机使用 `xense.taccap.advanced`，
可以独立于从爪控制运行。

## 主爪开口

主爪需先完成编码器零点与行程标定。下面选择唯一主爪并读取归一化开口：

```python
import time
from xense.taccap import advanced as t

devices = [d for d in t.scan_grippers() if d.role == t.Role.Leader]
if len(devices) != 1:
    raise RuntimeError("Select a leader by its firmware serial number")

with t.LeaderGripper(mcu_device=devices[0].mcu_device,
                     normalize_position=True) as leader:
    subscription = leader.encoder.on_data(lambda sample: print(sample.position))
    leader.start_streaming(encoder_hz=100, imu_hz=0)
    try:
        time.sleep(5.0)
    finally:
        leader.stop_streaming()
        leader.encoder.off(subscription)
```

`sample.position` 为归一化开口，`position_rad` 为编码器角度。
回调应保持简短。遥操时将最新开口交给应用目标缓冲区，再由从爪控制循环读取，
不要在采集回调中执行长时间阻塞操作。

IMU 同样可使用 `leader.imu.on_data(...)` 订阅；通过 `start_streaming` 的 `imu_hz`
配置推送。主爪与从爪的状态流参数不同，不要给从爪开启主爪 IMU / 编码器流。

## 腕相机

先查找 XC 相机路径：

```bash
python python/examples/wrist_camera.py --list
```

使用找到的稳定 `/dev/v4l/by-id/...` 路径：

```python
from xense.taccap import advanced as t

camera = t.Camera(device="/dev/v4l/by-id/实际腕相机路径",
                  width=640, height=480, fps=30.0,
                  use_mjpg=True, color_mode=t.ColorMode.RGB)
try:
    frame = camera.read(timeout_ms=500)
    if frame is not None:
        print(frame.frame_index, frame.image.shape)
finally:
    camera.stop()
```

独立 Camera 默认 BGR，聚合设备的 wrist_camera 默认 RGB。显式选择通道顺序可避免误用。
`Gripper` 不默认打开腕相机。需要去畸变时，可使用 `wrist_camera.py --undistort`，
或基于 `calibration.resolve_fisheye()` 构造 `FisheyeUndistorter`；支持的标定分辨率为 640×480。
内参可能回退到 SDK 参考值，定量图像测量应确认使用设备标定。

## 视触觉与多模态集成

OG / GSPS 视触觉模组由独立的 `xensesdk` 管理，不通过 Camera 或 Gripper 读取。
电机、腕图和触觉各有生命周期与时间戳；应用负责同步、缓存与数据融合。
关闭某一路采集时，仍需按各自 SDK 的资源释放流程处理。

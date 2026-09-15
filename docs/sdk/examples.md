# 示例

以下脚本位于 SDK 仓库的 `python/examples/`。在仓库根目录、已安装 SDK 的环境中运行。
`--help` 显示参数，不连接设备。

## 位置控制

```bash
python python/examples/gripper_control.py --list
python python/examples/position_control.py left
```

`--list` 只读发现。位置示例确认后使能，可输入开口，使用 `s` 查看状态、`q` 退出。

固定目标序列与反馈记录：

```bash
python python/examples/position_control.py left \
  --sequence 1,0,1 --dwell 5 --csv /tmp/gripper-position.csv
```

`--dwell` 是每个目标的保留时间，不是到位等待。脚本结束时请求失能。

## 控制模式

```bash
python python/examples/gripper_control.py left --grasp --torque 0.35 --hold
python python/examples/gripper_modes.py left
```

第一个演示装载、抓取、保持与释放；第二个演示同一会话中的模式切换。
都会真实运动。算法接入流程见[遥操与模型推理](guides/streaming.md)。

## 传感器与维护

| 脚本 | 用途 |
|---|---|
| `leader_normalized_position.py` | 主爪开口流 |
| `wrist_camera.py` | 腕相机查看、可选去畸变 |
| `read_intrinsics.py` | 内参 JSON 读取 |
| `calibrate.py` | 主爪编码器标定 |
| `fisheye_cal.py` | 相机内参与主爪行程维护 |
| `ota_update.py` | 固件 OTA |

传感器 / 维护脚本通常接受 `left/right/SN`。同侧存在多个角色时用完整 SN。
OTA 额外支持 `master/slave`；具体以各脚本 `--help` 为准。

## 轨迹记录与离线绘图

`frequency_benchmark.py` 测量目标提交、状态读取与原生控制 / 反馈频率，输出统计和图表。
命令及指标定义见[控制与读取频率测试](guides/frequency-test.md)。

`sine_sweep.py` 比较正弦轨迹频率、速度与扭矩预算；`plot_sine_sweep.py` 对记录离线绘图。
绘图依赖安装：

```bash
python -m pip install -r python/examples/requirements-plot.txt
python python/examples/sine_sweep.py --help
python python/examples/plot_sine_sweep.py --help
```

实验参数仅用于该脚本，不改变 SDK 默认设置。电机运动与反馈需使用同一时间定义分析，
不要将主机提交耗时当作机械到位时间。

## 原生兼容示例

`force_position_control.py`、`impedance_control.py`、`gripper_console.py` 演示原生控制器，
用于已有集成与参数调试。其接触 / 受阻策略不同于客户位置接口；新项目优先使用 `Gripper`。

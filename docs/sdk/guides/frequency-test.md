# 控制与读取频率测试

此测试用于验证遥操 / 模型推理程序的提交速度、反馈更新速度及主机调度表现。
使用 `Gripper` 位置模式，分别配置三个频率，测量它们实际达到的数值。

## 测量内容

| 指标 | 定义 |
|---|---|
| 应用目标提交频率 | Python `set_position()` 成功调用次数 / 测量时长 |
| 应用读取频率 | Python `state` 成功读取次数 / 测量时长 |
| 原生控制发送频率 | 测量窗口内 SDK UART 控制发送计数增量 / 时长 |
| 原生反馈频率 | 测量窗口内 SDK 收到的反馈计数增量 / 时长 |
| 观测到的新反馈频率 | Python 读到不同 `seq` 的次数 / 时长 |
| 调用耗时 | Python API 调用开始至返回的主机时间 |
| 调度延误 | 调用开始时间相对计划时刻的延误 |
| 反馈新鲜度 | SDK 提供的 `age_ms` |

调用耗时、调用周期、调度延误、新反馈观测周期与反馈新鲜度给出均值、标准差、p50 / p95 / p99 / 最大值。
三个速率组合逐组测试，每组新建控制会话；预热不进入计数差分测量窗口。

## 开始测试

在 SDK 仓库根目录、已安装 SDK 的环境中执行。绘图使用与正弦轨迹测试相同的可选依赖：

```bash
python -m pip install -r python/examples/requirements-plot.txt
python python/examples/frequency_benchmark.py left --output frequency-runs/baseline
```

默认执行 9 组：SDK 更新 20 / 50 / 100 Hz，应用提交 50 / 100 / 200 Hz，应用读取 500 Hz。
每组预热 1 秒、测量 10 秒，保持当前开口。脚本提示确认后使能，每组结束和退出时失能；
负载应有支撑，夹爪运动范围内不要放置手指。

只测试一个常用配置：

```bash
python python/examples/frequency_benchmark.py left \
  --sdk-hz 100 --command-hz 100 --read-hz 500 \
  --duration 30 --note "空载，独立 USB 端口" \
  --output frequency-runs/100hz
```

比较不同读取速度：

```bash
python python/examples/frequency_benchmark.py left \
  --sdk-hz 100 --command-hz 100 --read-hz 50,100,500,1000 \
  --output frequency-runs/read-rates
```

## 运动状态测试

保持测试通过后，使用正弦目标验证运动时的通信与调用表现：

```bash
python python/examples/frequency_benchmark.py left \
  --sdk-hz 100 --command-hz 50,100,200 --read-hz 500 \
  --motion sine --sine-hz 1 --center 0.5 --amplitude 0.2 \
  --torque 0.6 --speed 7.5 --duration 20 \
  --output frequency-runs/moving
```

此命令在开口 0.3..0.7 之间往复。正弦开始前先移动到起点；该定位时间不进入频率统计。
默认关闭闭合补偿，避免把端点压紧加入通信基线。温度达到默认 60°C、反馈过期或发生故障时停止试验并退出会话。

`--sine-hz` 是机械目标轨迹频率，`--sdk-hz` 是 SDK 更新频率，两者含义不同。
要比较轨迹跟踪误差、幅值衰减、相位滞后与扭矩因素，继续使用 `sine_sweep.py` 与 `plot_sine_sweep.py`。
本测试不把主机调用耗时解释为电机响应或到位时间。

## 输出与绘图

每次运行要求一个不存在的输出目录，避免覆盖实验。目录包含：

| 文件 | 内容 |
|---|---|
| `run.json` | 配置、SDK / 固件 / 主机信息、每组状态、指标、原生计数快照 |
| `trial-001-commands.csv` | 每次目标调用的时间、目标、耗时、调度延误与跳过时隙 |
| `trial-001-reads.csv` | 每次读取，包括重复缓存；序号、反馈新鲜度、开口、扭矩和温度 |
| `frequency-summary.png` / `.svg` | 频率对比、耗时 / 新鲜度、反馈观测周期分布、主机调度超期 |

默认采集完成并关闭设备后绘图。可用 `--no-plot` 只采集，再在离线环境执行：

```bash
python python/examples/frequency_benchmark.py --plot frequency-runs/100hz
```

异常或 Ctrl+C 会保存已采集数据，并将运行标记为 `aborted`。这类数据也可以离线绘图，
但不应当作完整试验结果。进程被强制杀死或主机断电时，不保证内存中的数据已保存。

## 如何判读

- **100 Hz 提交 + 500 Hz 读取不产生 500 Hz 反馈。** 活动会话的状态读取来自缓存，重复序号是正常情况。
- **高于 SDK 更新频率的目标可能被覆盖。** 后台发送最新目标；应用调用次数与控制帧数无需一一对应。
- **低频读取会跳过中间缓存。** `unobserved_cache_updates` 记录读者没有观察到的中间反馈，不能证明 UART 丢包。链路丢包定位需要固件发送统计与主机接收计数的专门对照。
- **读取和提交由同一个 Python 定时循环调度。** 慢调用会影响另一项任务；它代表单线程应用的组合负载，不是两个隔离线程的极限吞吐测试。超期时跳过时隙，不突发补发。
- **反馈周期图包含轮询抖动。** 原生帧率来自计数差分；原生最大间隔与提交延迟保存在 `timing_after`，覆盖本次会话（含预热），不是逐帧延迟分布。
- **不同固件路径应分别比较。** 1.1.7 使用兼容查询，1.1.8 可使用扩展遥测；运行记录保留固件版本和 `execution_telemetry`。

建议先比较空载基线，再在同样参数下加入实际模型推理、相机或 CPU 负载，并在 `--note` 中记录环境。
频率是否满足应用，应结合 p99 调度延误、最大反馈间隔和轨迹跟踪结果评估；本工具不预设未经实机验证的通过阈值。

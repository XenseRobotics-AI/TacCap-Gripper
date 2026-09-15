# 正弦位置跟随测试

用 [sine_sweep.py](../python/examples/sine_sweep.py) 比较扭矩预算、正弦频率、
参考速度对位置跟随的影响。使用现有 `Gripper.move(..., wait=False)`，每次替换最新目标，
同一控制会话完成所有试验；不改写电机限扭、固件包络或标定。需要当前 SDK 0.2.0b1 和
从爪固件 >= 1.1.8。脚本会真实运动，结束、异常或 Ctrl+C 时退出连接并请求失能。

## 运行

在 `TacCap-Gripper` 目录、已安装 SDK 的 `taccap` 环境中：

```bash
# 仅绘图需要这两个依赖，采集不会导入它们
python -m pip install -r python/examples/requirements-plot.txt

# 运动性能扫描：空夹爪，中间行程，20 组扭矩 × 频率，约 6 分钟加定位时间
python python/examples/sine_sweep.py left \
  --torques 0.35,0.6,0.9,1.2 --frequencies 0.25,0.5,1,2,3 --speeds 7.5 \
  --center 0.5 --amplitude 0.25 \
  --hz 100 --command-hz 100 --close-compensation-rad 0.03 \
  --cycles 8 --warmup-cycles 2 --rest 3 \
  --note "empty gripper, higher sine frequencies" --output sine-runs/empty-high-frequency
```

脚本显示试验范围和预计时间，按 Enter 开始。输出目录必须不存在，避免覆盖数据。
默认开口范围为 `0.25..0.75`，避开端点接触，便于分离运动跟随与闭合接触。
`0.03 rad` 沿用当前夹爪暂时可用的补偿值，不代表其他夹爪的标定值。

2026-09-11 的 `empty-baseline` 实测行程为 1.225934 rad：0.5 Hz、半幅 0.25 时需要
0.986408 rad/s，旧测试速度 0.5 rad/s 会把正弦参考削成近似三角波。
该频率下 0.35 和 0.5 Nm 的 RMSE 均约为行程的 12.03%，幅值比约 0.65，
因此提高扭矩前需先解除这个速度瓶颈。当前测试默认扫描 0.35、0.6、0.9、1.2 Nm，
正弦频率为 0.25、0.5、1、2、3 Hz，参考速度为 7.5 rad/s；该范围已有空夹爪测试，
结果与中心漂移问题见[当前状态](STATUS.md)，不表示已经确定通用最佳参数。
开始前会按连接设备的真实标定行程打印各频率的速度需求。

按上述实测行程、半幅 0.25 和补偿 0.03 rad 计算：

| 正弦频率 | 峰值参考速度需求 | 100 Hz 下每周期采样点数 |
|---|---|---|
| 0.25 Hz | 0.493 rad/s | 400 |
| 0.5 Hz | 0.986 rad/s | 200 |
| 1 Hz | 1.973 rad/s | 100 |
| 2 Hz | 3.946 rad/s | 50 |
| 3 Hz | 5.918 rad/s | 约 33 |

7.5 rad/s 给这台夹爪的 3 Hz 目标留出参考速度余量，不保证电机实际跟得上。
保持半幅不变时，加速度需求随频率平方增长；这轮可同时观察扭矩预算是否限制加减速。
默认每组预热 2 周期、测量 8 周期，3 Hz 下分别约 0.67 秒和 2.67 秒。
先让正在运行的测试结束，再启动新目录中的测试，避免两个进程争用同一夹爪。

每组开始前用单独的 `--positioning-torque 0.35`、参考速度 `0.5 rad/s` 定位到轨迹最大开口。
补偿后的电机误差 <= 0.015 rad 且速度 <= 0.05 rad/s 持续 0.2 秒后开始休息计时；
10 秒未能定位会终止，可通过 `--settle-timeout` 调整观察时间。
休息期间保持该位置，**不是失能冷却**，不会清空固件热历史。
因此很低的测试扭矩即使无法完成运动，也不会自动提升到定位扭矩；定位和正弦测量分开。

温度达到 `--max-temperature`（默认 60°C）、反馈过旧或故障时停止，保留已采数据。
这是试验停止条件，不是电机持续扭矩能力的认证。带物体测试时固定安装、支撑负载，
在 `--note` 记录物体、安装方式和环境温度；不要让自动往复松开动作导致物体掉落。

## 扫描其他因素

```bash
# 固定扭矩，比较速度限制和更高的正弦频率；每组重复 2 次
python python/examples/sine_sweep.py left \
  --torques 0.6 --frequencies 0.25,0.5,1 --speeds 0.5,1.5,2.5 \
  --repeats 2 --output sine-runs/speed-frequency

# 固定正弦与扭矩，模拟模型以 20 Hz 输出、SDK 以 100 Hz 跟随
python python/examples/sine_sweep.py left \
  --torques 0.35 --frequencies 0.5 --command-hz 20 --hz 100 \
  --output sine-runs/input-20hz

# 可选：5 Hz，缩小半幅到 0.1，开口范围 0.4..0.6
# 本机峰值参考速度需求约 3.95 rad/s；与半幅 0.25 的试验分开分析
python python/examples/sine_sweep.py left \
  --torques 0.6,0.9,1.2 --frequencies 1,2,3,5 --amplitude 0.1 --speeds 5 \
  --cycles 12 --warmup-cycles 3 --output sine-runs/small-amplitude-high-frequency
```

`--command-hz` 是 Python 更新目标的频率；`--hz` 是 SDK 配置的控制/上报频率，
可选 20、25、40、50、100。比较上报频率时分次运行，改变 `--hz` 并使用不同目录。
每个正弦周期至少配置 20 个命令和反馈采样点，否则参数检查拒绝运行。
所以配置 100 Hz 时脚本允许的正弦频率最高为 5 Hz；这是本工具的采样规则，不是电机带宽。
`--repeats` 控制重复次数；组合按 `--seed`（默认 7）随机排列，减少固定顺序和升温的混淆。
温度、负载和固件热历史仍需结合原始数据判断；不能仅凭随机顺序认为条件完全一致。

轨迹是带相位偏移的正弦，从最大开口以零斜率起步：

```text
p(t) = center + amplitude * cos(2*pi*frequency*t)
所需电机峰值参考速度 = 2*pi*frequency*amplitude*(标定行程_rad + 补偿_rad)
```

如果 `--speeds` 低于这个需求，输出和图中用 `reference speed LIMITED` / `x` 标记。
这类结果包含参考斜坡限速的影响，不能单独归因于扭矩不足。脚本不自动提高速度或扭矩。
默认 8 个测量周期，另有 2 个预热周期；各频率时长不同，温升不能直接当作等时热性能比较。
本工具不识别安全堵转时长；要评价长时间夹持，需要另行控制负载、持续时间和冷却条件。

## 输出与读图

完成全部试验并关闭连接后，自动绘图。没有绘图依赖时保留 CSV 并打印重绘命令说明。
也可以 `--no-plot` 只采集，再离线运行：

```bash
python python/examples/plot_sine_sweep.py sine-runs/empty-high-frequency
```

| 文件 | 内容 |
|---|---|
| `run.json` | SDK/固件、标定行程、参数、试验顺序、开始温度、每组状态与原生帧数/时间统计 |
| `commands.csv` | 实际 Python 调用的单调时间、目标、API 调用耗时、跳过的过期定时点 |
| `samples.csv` | 新反馈序号、主机观察时间、目标/实际位置、速度、扭矩、温度、采样年龄、固件降额 |
| `summary.csv` | 每组误差、幅值比、相位滞后、力矩 RMS、温度、实际采样/命令频率与质量标记 |
| `plots/trial-001.png` 等 | 各组目标与实际轨迹、误差、力矩、速度、温度；灰区为不计分的预热阶段 |
| `plots/comparison.png` / `.svg` | 扭矩和速度分组的频率对比；点为单次试验，线连接重复试验均值 |

主要读数：

- **跟踪 RMSE / 最大误差**：越小越接近期望的补偿后电机位置。CSV 使用 0..1 单位，
  图中乘以 100 表示标定行程的百分比；误差按观察时间间隔加权，包含偏差和滞后。
- **幅值比**：实际位置基频幅值 / 期望电机位置基频幅值，理想接近 1。
  卡住、波形畸变时仍需查看原始轨迹和 `sine_fit_r2`，单个比值不能代表控制带宽。
- **相位滞后**：用带偏置项的正弦最小二乘拟合计算；折算毫秒后标为 `apparent_phase_lag_ms`。
  只在完整、采样足够、未触及截断端点且输出拟合 R² >= 0.8 时报告，静止输出不报告相位。
- **速度、力矩与温度**：结合 `reference_speed_limited`、固件允许的扭矩和实际力矩判断原因。
  `effective_torque_limit_nm` 只是固件包络允许值，不是包含 SDK 和电机自身限值的最终值，
  `target_limited` 也不单独等于热降额。

相位采用 ±180° 的主值，没有跨频率解缠；周期性输入可能产生整周期歧义。
时间轴是 Python 读缓存的主机时间，未测量串口传输延迟、传感器之后的背隙和爪尖运动。
因此这里的相位毫秒数是**观察到的跟随滞后**，不是电机固有延迟或完整机械响应时间。
API 调用耗时、SDK 提交延迟也不能替代它。采集按单调时钟的绝对时间调度，主机卡顿时
跳过过期命令，保持实际时间上的正弦相位，不突发补发；Python 采集可能漏读反馈帧。
`run.json` / `summary.csv` 同时保留原生实际反馈/发送帧率与 Python 观察帧率，便于识别漏采。

## 补偿与误差坐标

本版反馈 `position` 沿用原标定的 0..1 坐标，补偿不会改变反馈零点。
因此正确跟随时 `position` 也可能与请求开口不同：

```text
expected_encoder_position = clip(p - compensation_rad/travel_rad * (1-p), 0, 1)
```

轨迹图分别画出原始请求、补偿后的理想电机轨迹、实际发出的阶梯目标和实测电机位置。
误差与幅值计算相对于补偿后的理想电机轨迹，避免把人为补偿误当作跟踪偏差。
`target_error_rad` 另外保留 SDK 的补偿后电机误差；它是异步快照，不能证明与某个命令逐帧对应。
反馈在闭合端截断，纯电机数据不能证明爪尖真正合拢；默认中间行程测试避免该问题。

## 不连接设备的绘图预览

```bash
python python/examples/plot_sine_sweep.py --demo /tmp/gripper-sine-demo
```

这会生成带 **SIMULATED DATA** 标识的示例 CSV 和图，用于验证绘图流程。
示例响应是人为设定的数学信号，不是 RS05 仿真模型或本夹爪实测结果。

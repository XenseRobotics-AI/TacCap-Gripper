# 升级到 0.2.0b1

0.2.0b1 是本轮客户接口的首个统一 beta，包含先前 0.1.10 / 0.1.11 开发快照及后续到位修复。
已有数据集与实验记录保留采集时的版本号，不修改为 beta。beta 不代表已完成持物、温升或断链验收。

## 版本与安装

| 对象 | 版本 / 规则 |
|---|---|
| Python SDK / wheel / 原生版本字符串 | `0.2.0b1`，从 `_version.py` 统一生成 |
| C++ 共享库数字版本 | `0.2.0`；完整 beta 标识见 `TACCAP_VERSION_STRING` |
| 从爪固件 | 推荐 `1.1.8`；客户 API 最低 `1.1.7` |
| 协议 | 保留旧命令，增加 `0x56` 执行诊断与可选 107B 状态流；不把 SDK 版本当作协议版本 |
| 旧镜像 | 随库历史从爪 `1.1.6` 不兼容客户 `Gripper` |

按照[安装说明](../getting-started/installation.md)重新构建或安装 beta wheel。核对：

```python
import xense.taccap as sdk
from xense.taccap import _taccap_native

print(sdk.__version__)
print(_taccap_native.__version__)
assert sdk.__version__ == _taccap_native.__version__
```

两者都应为 `0.2.0b1`。仅修改 Python 文件或查看源码版本不能证明已替换原生库。
固件使用设备协议中的数字版本，不添加 beta 字符串；SDK 安装不会更新设备。

## 接口命名

| 需求 | 推荐入口 | 兼容关系 |
|---|---|---|
| 客户连接与控制 | `from xense.taccap import Gripper` | 一个对象管理连接和控制生命周期 |
| 持续目标开口 | `set_position(position)` | 非阻塞，最新目标覆盖；使用连接默认参数 |
| 单次运动 / 参数覆盖 | `move(..., wait=True)` | `wait=False` 仍是默认；超时不取消目标 |
| 抓取与释放 | `grasp()` / `release()` | 接触检测不是可靠抓稳保证 |
| 停止输出 | `disable()` | `stop()` 保留同义兼容 |
| 等待结果 | `wait_result()` | 旧 `wait()` 继续只返回状态 |
| 位置力矩限值 | `torque_limit_nm` | `move(torque_nm=...)` 为兼容别名；不能同时给两者 |
| 原生控制 / 维护 / OTA | `xense.taccap.advanced` | 旧顶层导入和 wildcard 导入继续可用 |

`grasp(torque_nm=...)` 表示闭合请求，名称不等同于 `move` 的力矩限值；
`impedance` 的前馈是原始电机坐标。默认值和符号以[API 参考](../reference/gripper.md)为准。
`state.mode` 是执行阶段，`state.control_mode` 是控制方式；两者保留并明确区分。

相对 0.1.9，原始 Python `Motor.set_position/set_velocity/set_torque/set_impedance`、
对应的 `submit_*` 以及 `FollowerGripper.set_position` 已撤下。
这些写入绕过 SDK 控制策略；已有应用应迁移到客户 `Gripper` 或原生受限控制器。
`advanced` 不是恢复这些原始写入的入口。C++ 原始方法仍供原生层内部使用。

腕相机默认 RGB，OpenCV 消费者需要 BGR 时显式选择 `ColorMode.Bgr`。
原生控制示例的设备选择使用 `left/right/SN` 位置参数，旧 `--side` 不再支持。

## 文件命名与入口

| 当前入口 | 旧入口 / 说明 |
|---|---|
| `python/examples/gripper_control.py` | `gripper.py` 保留转发，日常动作命令 |
| `python/examples/position_control.py` | 交互 / 流式位置控制，保持原名 |
| `python/examples/gripper_modes.py` | 多模式示例，保持原名 |
| `python/examples/frequency_benchmark.py` | `frequency_test.py` 保留转发；性能采集工具，不是单元测试 |
| `python/examples/sine_sweep.py` / `plot_sine_sweep.py` | 轨迹扫描 / 离线绘图，保持原名 |
| `python/examples/requirements-plot.txt` | 绘图依赖；`requirements-sine.txt` 保留引用 |
| `python/tests/test_*.py` | 不驱动设备的单元测试 / 模拟 UART 回归，实机测试需明确排除 |
| `docs/sdk/` | 当前客户文档，进入网站 |
| `docs/development/` / `docs/history/` | 实现记录 / 历史证据；旧大写 Markdown 文件保留跳转 |

包名 `taccap-gripper`、导入路径 `xense.taccap`、原生模块 `_taccap_native` 保持不变。
原生头文件与控制器类名保留，避免无行为收益的 C++ 接口迁移。

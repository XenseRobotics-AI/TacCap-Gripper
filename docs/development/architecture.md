# SDK 架构与源码导航

本文面向维护者。用户使用流程见[使用手册](../USAGE.md)，API 签名见[参考手册](../API_REFERENCE.md)，
固件责任与保护数值见[控制契约](../CONTROL_LAYERING.md)。以当前工作分支为准。

## 1. 产品与分层

SDK 同时包含客户操作层与设备访问层。Python 用户通过 `Gripper` 控制从爪，
原生 C++ 控制器负责持续执行；主爪传感器、相机和维护保留独立入口。
ROS 2 / LeRobot、任务编排、模型推理、数据集由外部适配层实现。

```text
Python 用户 / 模型 / 遥操
  ├─ Gripper：客户生命周期、参数、动作、观测和结果
  │    └─ ForcePositionController：位置 / 抓取 / 阻抗策略
  │         └─ ControlRuntime：控制权、反馈订阅、线程、计时、停止
  │              └─ FollowerGripper / Motor
  └─ advanced：LeaderGripper、旧 ControlLoop、维护与传感器
       ├─ MCU components -> Transport -> SerialBus -> 串口 -> MCU -> RS05
       └─ Camera -> V4L2 / OpenCV -> XC 腕相机

OG 视触觉 -> 独立 xensesdk -> 应用融合
```

`advanced` 是现有原生绑定的显式导出，不是另一个通信栈。
顶层 `__getattr__` 保留旧导出，客户 facade 不增加 Python 控制线程。

## 2. 模块导航

| 层 | 文件 / 目录 | 职责 |
|---|---|---|
| Python API | [gripper.py](../../python/xense/taccap/gripper.py) | 发现、构造、模式切换、等待、恢复与关闭 |
| Python 类型 | [gripper_types.py](../../python/xense/taccap/gripper_types.py) | 不可变状态 / 结果 / 时间统计、派生状态判断 |
| 兼容导出 | [__init__.py](../../python/xense/taccap/__init__.py)、[advanced.py](../../python/xense/taccap/advanced.py) | 客户符号优先，旧导出继续解析 |
| 控制策略 | [force_position_controller.cpp](../../cpp/src/force_position_controller.cpp) | 位置参考、共享预算、接触策略、阻抗目标、补偿与快照 |
| 共享运行时 | [control_runtime.cpp](../../cpp/src/control_runtime.cpp) | 控制权 / 流订阅生命周期、调度与停止 |
| 旧控制器 | [control_loop.cpp](../../cpp/src/control_loop.cpp) | 兼容阻抗控制路径及其受阻策略 |
| 从爪聚合 | [follower_gripper.cpp](../../cpp/src/follower_gripper.cpp) | 连接、版本、标定映射、启动状态流与维护记录 |
| 电机组件 | [motor.cpp](../../cpp/src/components/motor.cpp) | 运动序列化、状态解码、固件执行诊断与控制权检查 |
| 头文件 | [cpp/include/taccap](../../cpp/include/taccap) | C++ 用户接口及配置定义 |
| 协议 | [cpp/src/protocol](../../cpp/src/protocol) | 命令、载荷、编码与解码 |
| Python 绑定 | [python/bindings](../../python/bindings) | pybind11 类型、方法、回调与 GIL 生命周期 |
| 示例 / 测试 | [examples](../../python/examples)、[Python tests](../../python/tests)、[C++ tests](../../cpp/tests) | 用户流程与软件回归 |

## 3. 一次位置更新

1. `Gripper.move/set_position` 校验开口、预算与速度；首次操作启动控制器。
2. `set_position_target` 设置最新目标、补偿与参考运动意图，增加原生代次，不排队。
3. 原生线程从反馈快照生成位置 / 速度参考与 MIT 参数，每条新反馈至多发送一次。
4. `Motor` 编码并通过无需 ACK 的控制提交路径写串口；这是主机提交完成，非机械到位。
5. MCU 接收最新目标，在名义 500 Hz 执行任务中检查反馈与包络，发送电机 CAN 指令。
6. 电机反馈通过 MCU 数据流回主机；原生控制器更新观测 / 执行状态 / 统计，Python 读取缓存。

预算裁剪在 SDK 与 MCU 两层都存在；各自基于观测，不保证采样间实际力矩。
算法异常和硬件极限必须区分；高频中心漂移列在[状态文档](../STATUS.md)，不是已解决能力。

## 4. 命令、流与时间

| 通路 | 行为 / 用途 |
|---|---|
| 同步命令 | `read_status`、配置、诊断等等待 ACK；用于低频维护，不在实时循环轮询 |
| 运动提交 | 原生运行时周期刷新最新目标；不等待每条 ACK，不承诺每条模型输入都发送 |
| MCU 状态流 | 1.1.8 可协商 107B 扩展流；旧 31B 流仍兼容 |
| Python state / timing | 控制活跃时缓存快照；未启动时 state 会改为同步查询 |
| wait_result | 在缓存观测上等稳定条件，返回前再查一次固件执行状态 |

`seq` 是主机反馈计数，`command_id` 是原生目标代次，固件 target/applied 是第三套计数。
不把这些数值相等当作端到端对应关系。1.1.8 的执行快照与电机快照也不是原子事务。
1.1.7 兼容监控含同步查询，可能影响间隔。

## 5. 生命周期与并发

连接是被动的；`start` 或首次运动使能并启动运行时。
同一个 Motor 对象只允许一个控制器持有控制权；第二个控制器拒绝启动。
直接调用 Motor.disable 会使已有控制权不能继续提交，恢复需重新建立运行会话。
这一机制不能替代跨进程串口独占。

Python facade 使用锁协调命令、状态、停止与等待代次；无需客户自己为每个目标建线程。
底层 Transport reader / dispatcher、控制运行时和 Camera capture 各自有生命周期。
退出时停止 / join，解绑状态订阅，避免回调访问已销毁对象。
`gil_safe.hpp` 管理原生线程到 Python 的回调；用户回调应短小，不在其中调用会等待同一调度器的操作。

客户 `close()` 只对自身活动 / 状态不确定会话请求失能；纯被动连接关闭不主动干扰其他控制者。
显式 `disable()` 是全局停止意图。旧原生 Follower 上下文不替代控制器 stop；
创建控制器的代码应在 finally 中停止控制器，再关闭设备。

## 6. 标定、视觉与接口边界

从爪的行程 / 方向来自 GripperConfig，主爪归一化来自编码器零点与 max-angle 记录。
两者不使用同一标定命令。SDK 闭合偏移仅属于运行时位置映射，不重写标定。

XC 腕相机通过 Camera 独立打开，可使用 FisheyeUndistorter；默认不开相机。
OG 视触觉由 xensesdk 管理，SDK 不保证电机、腕图与触觉的同步时间戳或共同成功。
融合、采样策略和记录由应用负责；详见[高级集成](../ADVANCED.md)。

## 7. 构建与验证边界

顶层 CMake 同时构建 C++ 核心与 pybind11，scikit-build-core 生成 Python wheel。
`_taccap_native` 和 `libtaccap_core` 共同安装，不能从不同构建随意混用。
构建 / 测试命令见[安装说明](../INSTALL.md)。

Python fake-device / PTY 测试验证客户流程与协议；C++ 测试验证策略 / 生命周期；
固件 host tests 验证纯执行 / 热策略，ARM 构建检查可编译性。
这些不等于实际机械性能、温度或断线停止验收。当前结果只在[状态文档](../STATUS.md)集中声明。

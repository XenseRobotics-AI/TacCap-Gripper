# 安装、更新与构建

返回[使用手册](../USAGE.md)。以下命令在 `TacCap-Gripper` 仓库根目录执行。
发行包名是 `taccap-gripper`，Python 导入名是 `xense.taccap`，环境名是 **`taccap`**。

## 1. 支持范围与依赖

- 当前构建 / 验证环境为 Linux x86_64、CPython 3.12。相机使用 V4L2 / UVC，
  当前不提供 Windows / macOS 使用保证。
- 包元数据声明 Python ≥ 3.10；现有本地 wheel 是 `cp312-cp312-linux_x86_64`，
  只适用于对应解释器 / 平台，其他版本需另行构建验证。
- 源码构建需要 C++ 工具链、CMake ≥ 3.20、Ninja、pkg-config、OpenCV、spdlog、
  pybind11 和 scikit-build-core。使用 `environment.yml` 的 GCC 14 配套依赖可减少 ABI 混用。
- wheel 包含 SDK 原生扩展和核心库，但未宣称是带齐系统依赖的 manylinux 包。
  OpenCV / C++ 运行库仍需匹配；新机器优先使用仓库环境，而不是只在空 venv 中安装 wheel。

仓库包含纯 Python 客户封装与 C++ 扩展，不能只复制 `python/xense/taccap/*.py` 就运行。

## 2. 从源码安装（当前工作分支）

```bash
mamba env create -f environment.yml
mamba activate taccap
python -m pip install -e . --no-build-isolation
```

已存在环境时，不要再次 create：

```bash
mamba activate taccap
mamba env update -f environment.yml -n taccap
python -m pip install -e . --no-build-isolation
```

没有 mamba 时可使用 conda 的对应命令。`-e` 为开发安装；修改 Python 源码可直接生效，
但修改 C++ / bindings 后仍需重新执行安装命令，让两部分一起构建。
生产部署可在相同构建环境用普通安装：

```bash
python -m pip install . --no-build-isolation
```

SDK 仓库独立构建，不依赖固件仓库或子模块。使用 `python -m pip` 确保操作当前解释器。

## 3. 安装已有 wheel

已经拿到匹配平台的 wheel 时，可在配好依赖的 `taccap` 环境安装，例如本地候选包：

```bash
python -m pip install --no-index --no-deps --force-reinstall \
  dist/taccap_gripper-0.1.11-cp312-cp312-linux_x86_64.whl
```

wheel 文件必须实际存在；交付时按收到的真实文件名替换。
`--no-deps` 假定运行依赖已准备好，不能代替环境创建。
wheel 安装后修改仓库源码不会更新 site-packages；同样，现有 `dist/` wheel 不会自动包含
后续的文档或到位判断修复。[状态表](../STATUS.md)区分这些版本。

在构建环境生成新的 wheel：

```bash
python -m pip wheel . --no-deps --no-build-isolation -w dist
```

正式交付应使用新版本号并附源提交、校验和、平台与验证记录；当前未发布新的正式版本。
不要用相同版本号覆盖后声称不同用户拿到的是同一份构建。

## 4. 验证 Python 与原生库来源

此步骤不连接设备：

```bash
env -u PYTHONPATH python - <<'PY'
import sys
import xense.taccap as sdk
from xense.taccap import _taccap_native as native
from xense.taccap import Gripper, OtaSession
print("python:", sys.executable)
print("python version:", sdk.__version__)
print("native version:", sdk.hello())
print("python package:", sdk.__file__)
print("native module:", native.__file__)
print("customer / OTA imports:", Gripper.__name__, OtaSession.__name__)
PY
```

Python 和原生版本应一致，路径应指向预期的当前 checkout（开发安装）或同一环境的
site-packages（wheel 安装）。旧 editable 安装、`PYTHONPATH` 或混用环境可能使导入转向另一份仓库。
按正确路径重新执行 `python -m pip install -e . --no-build-isolation`；不要通过手拷 `.so`
拼凑不同版本。原生库通常通过 `$ORIGIN` 找同目录核心库，不需要全局设置 `LD_LIBRARY_PATH`。

`ImportError: OtaSession`、构造参数不存在、新遥测字段不存在，先做这一检查；
同一 shell 中安装成功不证明正在用那份代码。

## 5. 设备权限与只读发现

MCU 通常在 `/dev/serial/by-id/...-if02`，底层可能是 ttyUSB 或 ttyACM；
腕相机在 `/dev/video*`。优先使用稳定的 by-id 路径或固件 SN。

```bash
sudo usermod -aG dialout,video "$USER"
```

重新登录后组权限生效。只用电机控制时主要需要串口权限；仅访问已授权设备，
不需要用 root 运行 SDK。

```bash
python python/examples/gripper.py --list
```

这只发现从爪，不使能。运行运动示例前等设备启动标定完成。

## 6. 常用可选依赖

| 需求 | 安装 / 来源 |
|---|---|
| 位置控制、状态、OTA | 核心 SDK；环境已提供构建与运行依赖 |
| 正弦 CSV 离线绘图 | `python -m pip install -r python/examples/requirements-sine.txt` |
| 腕相机 / Rerun 示例 | 仓库环境包含 OpenCV 和 rerun-sdk；见[高级集成](../ADVANCED.md) |
| OG 视触觉传感器 | 独立交付的 `xensesdk`，不随此 wheel 安装 |
| ROS 2 / LeRobot | 各自的适配仓库；核心 SDK 不要求安装它们 |

## 7. C++ 构建与集成

```bash
cmake -S . -B build/cpp -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DTACCAP_BUILD_PYTHON=OFF \
  -DTACCAP_BUILD_EXAMPLES=ON \
  -DTACCAP_BUILD_TESTS=ON
cmake --build build/cpp -j
ctest --test-dir build/cpp --output-on-failure
```

`TACCAP_BUILD_PYTHON` 默认 ON，另外两项默认 OFF。核心库位于构建目录的 `cpp/`。
Python 客户 `Gripper` 是 Python facade；C++ 继续用
`FollowerGripper`、`ForcePositionController` / `ControlLoop` 和现有头文件，
没有新增完全对等的 C++ `Gripper` 类。
在自己的 CMake 工程中通过 `add_subdirectory` 引入此仓库并链接 `taccap_core`。
原生使用方法见[高级集成](../ADVANCED.md)与[架构](../ARCHITECTURE.md)。

## 8. 不驱动硬件的软件验证

在已安装当前源码的开发环境：

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest python/tests -q \
  -k 'not test_imu_vectors_are_real_buffers and not test_accel_magnitude_is_about_one_g'
```

排除两个实机 IMU 用例；其余使用单元测试 / 模拟 UART，不作为机械或热性能验收。
CLI 帮助也不连接设备：

```bash
python python/examples/position_control.py --help
python python/examples/ota_update.py --help
```

更新 C++ 后重复安装并检查双版本；仅修改说明文档不需要刷固件。

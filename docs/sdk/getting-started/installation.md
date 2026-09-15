# 安装

## 系统要求

| 项目 | 要求 |
|---|---|
| 操作系统 | Linux；当前提供 Linux x86_64 构建 |
| Python | 推荐 CPython 3.12；wheel 必须匹配 Python 与平台标签 |
| 设备权限 | 串口访问权限；使用相机时还需视频设备权限 |
| 从爪固件 | `Gripper` 要求 ≥ 1.1.7；闭合补偿要求 ≥ 1.1.8 |

SDK 包含 C++ 扩展及 OpenCV 运行依赖。推荐使用仓库的 `environment.yml` 创建环境。
包元数据允许 Python ≥ 3.10，其他 Python / 平台组合需要单独构建与验证。

## 从源码安装

在 SDK 仓库根目录执行：

```bash
mamba env create -f environment.yml
mamba activate taccap
python -m pip install . --no-build-isolation
```

使用 conda 时，将 `mamba` 换为 `conda`。开发安装使用：

```bash
python -m pip install -e . --no-build-isolation
```

已有环境可先通过 `mamba env update -f environment.yml -n taccap` 补齐依赖。
修改 C++ 或 Python 绑定后需重新安装；开发安装不会自动重编译这些文件。

## 安装 wheel

在已准备好运行依赖的环境中，安装收到的 wheel。将下列路径替换为实际文件：

```bash
python -m pip install /path/to/taccap_gripper-VERSION-cp312-cp312-linux_x86_64.whl
```

`cp312` 表示 CPython 3.12。不要将不同 Python 或 CPU 架构的 wheel 混用。
wheel 安装与仓库源码相互独立；修改源码后，需重新安装才能更新已安装的包。

## 配置权限

```bash
sudo usermod -aG dialout,video "$USER"
```

重新登录后权限生效。仅使用夹爪控制时需要串口权限；相机需要 `video` 组权限。
SDK 不要求以 root 运行。

## 验证安装

以下代码不连接设备：

```bash
python - <<'PY'
import sys
import xense.taccap as sdk
from xense.taccap import Gripper, OtaSession, _taccap_native

print(sys.executable)
print(sdk.__version__)
print(sdk.hello())
print(sdk.__file__)
print(_taccap_native.__file__)
PY
```

Python 包与原生库应来自同一版本和预期的安装环境。

| 问题 | 处理 |
|---|---|
| 找不到 `xense.taccap` | 确认激活的环境；使用该环境的 `python -m pip` 安装 |
| 找不到 `OtaSession` 或新接口 | 检查 Python / 原生库版本及加载路径；重新安装完整 SDK |
| 加载到另一份源码 | 清理外部 `PYTHONPATH`，在正确仓库重新执行开发安装 |
| 缺少 OpenCV / C++ 动态库 | 使用仓库环境补齐匹配依赖；不要手动拼接不同版本的 `.so` |
| 串口 permission denied | 确认组权限已经通过重新登录生效 |

安装完成后继续[第一次控制夹爪](quickstart.md)。

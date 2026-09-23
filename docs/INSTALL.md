# 安装与构建

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

## Install

The SDK has two consumable surfaces — the C++ shared library
(`libtaccap_core.so`) and the Python extension (`xense.taccap`). Both are
produced by the **same** top-level CMake project; you choose which surface
to build.

### 1. Prerequisites

|                       | Required                                                                                            |
| --------------------- | --------------------------------------------------------------------------------------------------- |
| OS                    | Linux (Ubuntu 22.04+ tested). The capture path is V4L2 + UVC XU; macOS / Windows are not supported. |
| Toolchain             | gcc/g++ ≥ 13, CMake ≥ 3.20, Ninja, pkg-config                                                       |
| Python (for bindings) | CPython 3.12                                                                                        |
| Recommended           | `mamba` / `conda` — `environment.yml` pins the entire toolchain & C++ deps to a known-good set      |

> **Why mamba is recommended.** `environment.yml` ships gcc-14, OpenCV
> 4.12, spdlog, gtest, pybind11 and scikit-build-core at a known-good set
> of versions. If you build against system packages instead, you are on
> your own for ABI compatibility.

### 2. Clone

```bash
git clone <repo-url> taccap-gripper
cd taccap-gripper
```

There are no git submodules — the SDK builds standalone.

### 3. Create the development environment

```bash
mamba env create -f environment.yml
mamba activate taccap

# Or, if you already have a conda env you want to add this to:
mamba env update -f environment.yml -n <your-env>
```

This installs gcc-14, the C++ deps, Python 3.12, pybind11,
scikit-build-core, numpy, pyserial, opencv-python==4.12.0.88 and
rerun-sdk in one shot. After activation you should see:

```bash
which cmake     # → .../envs/taccap/bin/cmake
which python    # → .../envs/taccap/bin/python
gcc --version   # → 14.x
```

### 4. Device permissions (one-time)

Plugged-in TacCap devices appear as `/dev/ttyACM*` (MCU) and
`/dev/video*` (UVC cameras). Your user needs to be in the matching
groups:

```bash
sudo usermod -aG dialout,video "$USER"
# log out and back in (or `newgrp dialout && newgrp video`) for it to apply
```

### 5a. Python install (recommended for most users)

`pyproject.toml` uses **scikit-build-core** as the build backend, which
drives CMake under the hood with `TACCAP_BUILD_PYTHON=ON` and an explicit
`TACCAP_BUILD_EXAMPLES=OFF` — the option defaults to ON for source builds so CI
compiles the examples, but example binaries have no place in a wheel. A single
install invocation builds the C++ core and the pybind11 extension, then
co-locates them inside the wheel under `xense/taccap/`:

```bash
# Editable / development install (re-runs CMake on every install):
uv pip install -e . --no-build-isolation

# Or a regular install (builds a wheel, installs it):
uv pip install . --no-build-isolation
```

`uv` ships in `environment.yml` and picks up the activated conda env by itself.
`pip` accepts the same arguments if you prefer it.

**`--no-build-isolation` is not optional here in spirit.** The env pins
`pybind11` and `scikit-build-core` alongside `libopencv` and `spdlog`, and the
flag says "build against those". Drop it and the installer builds in a
throwaway environment against a `pybind11` downloaded from PyPI — measured,
`pybind11_DIR` then points into `~/.cache/uv/builds-v0/…` rather than the env.
pybind11 is header-only, so whatever copy is present at build time is compiled
into the extension; this codebase is version-sensitive there, which is what
`python/tests/test_numpy_views.py` guards.

What ends up where (editable build):

```
python/xense/taccap/
├── _taccap_native.cpython-312-x86_64-linux-gnu.so   # pybind11 module
└── libtaccap_core.so.<version>  (+ .so.0 symlink)   # SDK core
```

These two are co-located on purpose — the rpath is set to `$ORIGIN`,
so loading `xense.taccap` just works without `LD_LIBRARY_PATH`.

Build artefacts for editable installs land under `build/{wheel_tag}/`
(see `[tool.scikit-build] build-dir` in `pyproject.toml`). Delete that
directory if you want a clean rebuild; the next install regenerates it.

### 5b. C++-only build (no Python)

If you don't need the Python bindings — e.g. you are integrating
`libtaccap_core.so` into a ROS 2 package or another CMake project —
build directly with CMake/Ninja:

```bash
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DTACCAP_BUILD_PYTHON=OFF \
    -DTACCAP_BUILD_EXAMPLES=ON \
    -DTACCAP_BUILD_TESTS=ON

cmake --build build -j
```

Output:

```
build/
├── cpp/libtaccap_core.so(.0)(.<version>)
├── cpp/examples/leader_demo
└── cpp/tests/...                # gtest binaries; run via `ctest`
```

CMake options (top-level `CMakeLists.txt:19-21`):

| Option                  | Default | Effect                                          |
| ----------------------- | ------- | ----------------------------------------------- |
| `TACCAP_BUILD_PYTHON`   | `ON`    | Build the `_taccap_native` pybind11 module      |
| `TACCAP_BUILD_EXAMPLES` | `ON`    | Build `cpp/examples/` (leader stream demo, follower status, two controllers) |
| `TACCAP_BUILD_TESTS`    | `OFF`   | Build the gtest suite under `cpp/tests/`        |

### 6. Verify

```bash
# Python — note `env -u PYTHONPATH`, see the note below
env -u PYTHONPATH python -c "import xense.taccap as t; print(t.hello()); print(t.__version__)"
# → taccap-gripper OK; version <version>
# → <version>          # both lines match python/xense/taccap/_version.py

# Python tests (hardware-free cases always run; IMU cases skip without a gripper)
env -u PYTHONPATH pytest python/tests

# C++ tests (only if TACCAP_BUILD_TESTS=ON)
ctest --test-dir build --output-on-failure
```

> **If `xense.taccap` resolves somewhere unexpected, check `PYTHONPATH`.**
> Stacked conda activations can export another env's `site-packages` into
> *every* interpreter, which then shadows this repo with whatever editable
> install lives there — you end up testing a different checkout without any
> error. `env -u PYTHONPATH python -c "import xense.taccap as t; print(t.__file__)"`
> tells you which tree you are actually running.

### 7. Rebuild / clean

```bash
# Python: blow away scikit-build-core's build dir
rm -rf build/ && uv pip install -e . --no-build-isolation

# Pure C++: incremental rebuild is fine
cmake --build build -j

# Full reset
rm -rf build/
```

---


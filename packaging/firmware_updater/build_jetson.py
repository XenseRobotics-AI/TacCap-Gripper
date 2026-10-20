#!/usr/bin/env python3
"""Build on Ubuntu 22.04 ARM64, in a dedicated checkout (no CUDA needed)."""

import argparse
import os
import shutil
import subprocess
from pathlib import Path

from platform_config import check_elf, native_target

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    target = native_target()
    if target["machine"] != "aarch64":
        raise SystemExit("Run on Ubuntu 22.04 ARM64, not an x86_64 machine")
    if os.geteuid() == 0:
        raise SystemExit("Run as a normal user; install build dependencies separately")
    if args.jobs < 1:
        raise SystemExit("--jobs must be positive")
    for command in ("cmake", "ninja", "g++", "patchelf", "readelf", "dpkg-deb", "git"):
        if not shutil.which(command):
            raise SystemExit(
                f"Missing {command}; see docs/FIRMWARE_UPDATER.md Jetson section"
            )
    # Refuse to overwrite a native extension copied from another machine.
    for native in (ROOT / "python/xense/taccap").glob("_taccap_native*.so"):
        check_elf(native, target)
    build = ROOT / "build/jetson-updater"
    env_dir = build / "venv"
    env = os.environ.copy()
    for key in (
        "PYTHONPATH",
        "PYTHONHOME",
        "LD_LIBRARY_PATH",
        "LD_PRELOAD",
        "CMAKE_PREFIX_PATH",
    ):
        env.pop(key, None)
    # Do not use conda's Python or compiler on the Jetson.
    env["PATH"] = "/usr/bin:/bin:/usr/sbin:/sbin"
    subprocess.run(
        ["/usr/bin/python3", "-m", "venv", str(env_dir)], env=env, check=True
    )
    python = str(env_dir / "bin/python")
    subprocess.run(
        [
            python,
            "-m",
            "pip",
            "install",
            "PySide6==6.7.3",
            "PyInstaller==6.22.3",
            "pybind11==2.13.6",
        ],
        env=env,
        check=True,
    )
    pybind_dir = subprocess.check_output(
        [python, "-m", "pybind11", "--cmakedir"], env=env, text=True
    ).strip()
    subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(build / "sdk"),
            "-G",
            "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF",
            "-DTACCAP_BUILD_EXAMPLES=OFF",
            "-DTACCAP_BUILD_TESTS=OFF",
            "-DTACCAP_BUILD_PYTHON=ON",
            f"-DPYTHON_EXECUTABLE={python}",
            f"-Dpybind11_DIR={pybind_dir}",
        ],
        env=env,
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(build / "sdk"), "--parallel", str(args.jobs)],
        env=env,
        check=True,
    )
    subprocess.run(
        [python, str(HERE / "build_all.py"), "--output", str(args.output.resolve())],
        env=env,
        check=True,
    )


if __name__ == "__main__":
    main()

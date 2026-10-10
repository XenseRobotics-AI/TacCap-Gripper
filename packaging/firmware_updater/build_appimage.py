#!/usr/bin/env python3
"""Wrap a verified updater deb in an AppImage, preserving its build identity."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

PACKAGE = "taccap-firmware-updater"
# Official AppImage GitHub asset digests reviewed on 2026-10-11.
TOOL_SHA = "95cbe7cce9717fce90c484e34052ee7c7f1d7635b33c12525b4776826a7d29b6"
RUNTIME_SHA = "156f4bdbde9c52d01814600013e0a273f0118dc2de98975f3c8c63427ec79074"


def verify_file(path, expected):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f"Unreviewed build dependency: {path}")


def prepare_appdir(extracted, appdir):
    frozen = extracted / "opt" / PACKAGE
    metadata = json.loads((frozen / "_internal/build-info.json").read_text())
    if metadata["dirty"]:
        raise ValueError("Refusing a dirty source build")
    shutil.copytree(frozen, appdir / "usr/lib" / PACKAGE)
    shutil.copy2(extracted / "usr/share/pixmaps" / f"{PACKAGE}.png", appdir)
    shutil.copy2(appdir / f"{PACKAGE}.png", appdir / ".DirIcon")
    desktop = (extracted / "usr/share/applications" / f"{PACKAGE}.desktop").read_text()
    desktop = desktop.replace(f"Exec=/usr/bin/{PACKAGE}", "Exec=AppRun")
    (appdir / f"{PACKAGE}.desktop").write_text(desktop)
    launcher = appdir / "AppRun"
    launcher.write_text(
        "#!/bin/sh\nset -eu\n"
        "unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD\n"
        'if [ "$(id -u)" -eq 0 ]; then\n'
        '    echo "Run Xense TacCap as a normal user, not root." >&2\n'
        "    exit 1\nfi\n"
        'app_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)\n'
        f'exec "$app_dir/usr/lib/{PACKAGE}/{PACKAGE}" "$@"\n'
    )
    launcher.chmod(0o755)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("deb", type=Path)
    parser.add_argument("--tool", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("dist"))
    args = parser.parse_args()
    tool, runtime = args.tool.resolve(), args.runtime.resolve()
    verify_file(tool, TOOL_SHA)
    verify_file(runtime, RUNTIME_SHA)
    tool.chmod(0o755)
    args.output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        env.pop(key, None)
    env.update(ARCH="x86_64", APPIMAGE_EXTRACT_AND_RUN="1")
    with tempfile.TemporaryDirectory(prefix="taccap-appimage-") as temp:
        temp = Path(temp)
        extracted, appdir = temp / "deb", temp / "TacCap.AppDir"
        subprocess.run(
            ["dpkg-deb", "-x", str(args.deb.resolve()), str(extracted)], check=True
        )
        metadata = prepare_appdir(extracted, appdir)
        version = metadata["package_version"]
        output = args.output.resolve() / f"{PACKAGE}_{version}_x86_64.AppImage"
        if output.exists():
            raise FileExistsError(f"Preserve the existing artifact first: {output}")
        subprocess.run(
            [str(tool), "--runtime-file", str(runtime), str(appdir), str(output)],
            env=env,
            check=True,
        )
        output.chmod(0o755)
        print(output)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Build a self-contained frozen updater on Ubuntu 24.04 amd64.

Run with a build-only Python containing PyInstaller, PySide6 and a matching built
SDK. No build-environment Python is invoked by the installed application.
"""

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = "taccap-firmware-updater"
VERSION = "0.1.1"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    args = parser.parse_args()
    patcher = shutil.which("patchelf")
    if not patcher:
        raise SystemExit("Build dependency missing: patchelf")
    os_release = Path("/etc/os-release").read_text()
    if 'VERSION_ID="24.04"' not in os_release or platform.machine() != "x86_64":
        raise SystemExit("Build on Ubuntu 24.04 amd64, not a newer glibc host")
    if not list((ROOT / "python/xense/taccap").glob("_taccap_native*.so")):
        raise SystemExit("Build the current checkout's native SDK first")
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="taccap-updater-build-") as temp:
        temp = Path(temp)
        app = ROOT / "apps/firmware_updater"
        env = os.environ.copy()
        for key in ("PYTHONPATH", "PYTHONHOME", "LD_PRELOAD", "LD_LIBRARY_PATH"):
            env.pop(key, None)
        subprocess.run(
            [
                sys.executable,
                "-m",
                "PyInstaller",
                "--noconfirm",
                "--clean",
                "--onedir",
                "--name",
                PACKAGE,
                "--distpath",
                str(temp / "frozen"),
                "--workpath",
                str(temp / "work"),
                "--specpath",
                str(temp),
                "--paths",
                str(ROOT / "python"),
                "--paths",
                str(ROOT / "python/examples"),
                "--paths",
                str(app),
                "--hidden-import",
                "device",
                "--hidden-import",
                "xense.taccap._taccap_native",
                "--add-data",
                f"{app / 'catalog.json'}:.",
                "--add-data",
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:fonts",
                "--exclude-module",
                "tkinter",
                "--exclude-module",
                "PyQt5",
                str(app / "entry.py"),
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )
        frozen = temp / "frozen" / PACKAGE
        # Replace build-machine RPATHs in every bundled ELF, not only Python.
        internal = frozen / "_internal"
        native = next((ROOT / "python/xense/taccap").glob("_taccap_native*.so"))
        # Preserve the exact SDK dependency closure, rather than accidentally
        # selecting different OpenCV/BLAS builds from the freezing environment.
        linked = subprocess.check_output(["ldd", str(native)], text=True, env=env)
        if "not found" in linked:
            raise RuntimeError(f"SDK dependency missing: {linked}")
        for line in linked.splitlines():
            if " => /" not in line:
                continue
            name, resolved = line.strip().split(" => ", 1)
            source = Path(resolved.split(" (", 1)[0])
            if str(source).startswith(("/lib/", "/usr/lib/")):
                continue
            for alias in ("libblas.so.3", "libcblas.so.3", "liblapack.so.3"):
                candidate = source.parent / alias
                if name == "liblapack.so.3" and candidate.exists():
                    shutil.copy2(candidate, internal / alias)
            dest = internal / name
            if dest.is_symlink():
                dest.unlink()
            shutil.copy2(source, dest)
        for binary in frozen.rglob("*"):
            if not binary.is_file() or binary.is_symlink():
                continue
            with binary.open("rb") as stream:
                elf = stream.read(4) == b"\x7fELF"
            if elf:
                relative = os.path.relpath(internal, binary.parent)
                subprocess.run(
                    [
                        patcher,
                        "--set-rpath",
                        f"$ORIGIN:$ORIGIN/{relative}",
                        str(binary),
                    ],
                    check=True,
                )
        probe = subprocess.run(
            [str(frozen / PACKAGE), "--self-test"],
            env=env,
            cwd=temp,
            text=True,
            capture_output=True,
            check=False,
        )
        if probe.returncode:
            raise RuntimeError(
                f"Frozen self-test failed: {probe.stdout}\n{probe.stderr}"
            )
        report = json.loads(probe.stdout.strip())
        if (
            report["sdk"] != "0.4.3"
            or not report["frozen"]
            or str(frozen) not in report["native"]
        ):
            raise RuntimeError(f"Mismatched or non-isolated SDK: {report}")
        package = temp / "deb"
        installed = package / "opt" / PACKAGE
        installed.parent.mkdir(parents=True)
        shutil.copytree(frozen, installed)
        (installed / "BUILD-INFO.json").write_text(json.dumps(report, indent=2))
        control = package / "DEBIAN"
        control.mkdir()
        (control / "control").write_text(
            f"Package: {PACKAGE}\nVersion: {VERSION}\nArchitecture: amd64\n"
            "Maintainer: TacCap firmware updater maintainers\nSection: utils\nPriority: optional\n"
            "Depends: libc6 (>= 2.39), libx11-6, libxext6, libxrender1, libfontconfig1, libfreetype6, libxcb-cursor0, libxkbcommon-x11-0, libgl1, libegl1\n"
            "Description: TacCap follower firmware update and rollback assistant\n"
            " Bundled Python, Qt, CJK font and native SDK. Runs as a normal user.\n"
        )
        bin_dir = package / "usr/bin"
        bin_dir.mkdir(parents=True)
        launcher = bin_dir / PACKAGE
        launcher.write_text(
            "#!/bin/sh\n"
            "unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD\n"
            f'exec /opt/{PACKAGE}/{PACKAGE} "$@"\n'
        )
        launcher.chmod(0o755)
        desktop = package / "usr/share/applications"
        desktop.mkdir(parents=True)
        (desktop / f"{PACKAGE}.desktop").write_text(
            "[Desktop Entry]\nType=Application\nName=TacCap Firmware Updater\n"
            "Name[zh_CN]=TacCap 固件升级与回滚\n"
            f"Exec=/usr/bin/{PACKAGE}\nTerminal=false\nCategories=Utility;\n"
        )
        docs = package / "usr/share/doc" / PACKAGE
        docs.mkdir(parents=True)
        shutil.copy2(ROOT / "docs/FIRMWARE_UPDATER.md", docs / "README.md")
        shutil.copy2(ROOT / "LICENSE", docs / "copyright")
        shutil.copy2(
            "/usr/share/doc/fonts-noto-cjk/copyright", docs / "Noto-CJK-copyright"
        )
        out = args.output.resolve() / f"{PACKAGE}_{VERSION}_amd64.deb"
        subprocess.run(
            ["dpkg-deb", "--root-owner-group", "--build", str(package), str(out)],
            check=True,
        )
        print(out)


if __name__ == "__main__":
    main()

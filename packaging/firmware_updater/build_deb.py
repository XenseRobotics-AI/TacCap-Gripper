#!/usr/bin/env python3
"""Build a self-contained frozen updater for supported Ubuntu architectures.

Run with a build-only Python containing PyInstaller, PySide6 and a matching built
SDK. No build-environment Python is invoked by the installed application.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = "taccap-firmware-updater"
sys.path.insert(0, str(ROOT / "apps/firmware_updater"))
from build_info import PACKAGE_VERSION, VERSION  # noqa: E402
from platform_config import check_elf, native_target  # noqa: E402
from portable import build_portable  # noqa: E402


def install_icons(source, share):
    """Letterbox the original into square theme icons without stretching."""
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QImage, QPainter

    original = QImage(str(source))
    if original.isNull():
        raise ValueError(f"Cannot load application icon: {source}")
    for size in (16, 24, 32, 48, 64, 128, 256, 512):
        scaled = original.scaled(
            size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        canvas = QImage(size, size, QImage.Format_ARGB32)
        canvas.fill(Qt.transparent)
        painter = QPainter(canvas)
        painter.drawImage(
            (size - scaled.width()) // 2, (size - scaled.height()) // 2, scaled
        )
        painter.end()
        destination = share / "icons/hicolor" / f"{size}x{size}/apps"
        destination.mkdir(parents=True, exist_ok=True)
        if not canvas.save(str(destination / f"{PACKAGE}.png")):
            raise RuntimeError("Cannot save application icon")


def install_serial_rules(package):
    here = Path(__file__).resolve().parent
    rules = package / "usr/lib/udev/rules.d"
    rules.mkdir(parents=True, exist_ok=True)
    shutil.copy2(here / "69-taccap-firmware-updater.rules", rules)
    postinst = package / "DEBIAN/postinst"
    shutil.copy2(here / "postinst", postinst)
    postinst.chmod(0o755)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    args = parser.parse_args()
    patcher = shutil.which("patchelf")
    if not patcher:
        raise SystemExit("Build dependency missing: patchelf")
    target = native_target()
    natives = list((ROOT / "python/xense/taccap").glob("_taccap_native*.so"))
    if len(natives) != 1:
        raise SystemExit("Build exactly one current checkout native SDK first")
    check_elf(natives[0], target)
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="taccap-updater-build-") as temp:
        temp = Path(temp)
        app = ROOT / "apps/firmware_updater"
        sys.path.insert(0, str(app))
        import bundled
        import core

        versions = bundled.verify_all(
            core.catalog_at(app / "catalog.json"), ROOT / "firmware"
        )
        manifest = json.loads((ROOT / "firmware/manifest.json").read_text())
        for role in ("master", "slave"):
            entry = manifest["images"][role]
            if (
                versions[f"mcu/{role}"] != entry["version"]
                or bundled.FILES["mcu", role] != entry["file"]
            ):
                raise ValueError(
                    f"{role}: approved catalog differs from current manifest; review firmware before packaging"
                )

        sha = subprocess.check_output(
            ["git", "rev-parse", "--short=12", "HEAD"], cwd=ROOT, text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                [
                    "git",
                    "status",
                    "--porcelain",
                    "--",
                    "apps/firmware_updater",
                    "packaging/firmware_updater",
                    "python",
                    "cpp",
                    "docs/FIRMWARE_UPDATER.md",
                ],
                cwd=ROOT,
                text=True,
            ).strip()
        )
        metadata = temp / "build-info.json"
        metadata.write_text(
            json.dumps(
                {
                    "version": VERSION,
                    "package_version": PACKAGE_VERSION,
                    "sha": sha,
                    "dirty": dirty,
                    "architecture": target["machine"],
                    "ubuntu": target["ubuntu"],
                }
            )
        )
        firmware_args = ["--add-data", f"{metadata}:."]

        for relative in bundled.packaged_files():
            path = ROOT / "firmware" / relative
            firmware_args.extend(
                ["--add-data", f"{path}:firmware/{Path(relative).parent}"]
            )
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
                f"{app / 'icon.png'}:.",
                "--add-data",
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:fonts",
                "--exclude-module",
                "tkinter",
                "--exclude-module",
                "PyQt5",
                *firmware_args,
                str(app / "entry.py"),
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )
        frozen = temp / "frozen" / PACKAGE
        shutil.copy2(
            Path(__file__).parent / "69-taccap-firmware-updater.rules",
            frozen / "_internal/serial.rules",
        )
        # Replace build-machine RPATHs in every bundled ELF, not only Python.
        internal = frozen / "_internal"
        # The UI uses PNG only. Qt 6.7 ARM wheels ship a WebP plugin linked
        # against libwebp.so.6, unavailable on Ubuntu 22.04 (ABI 7).
        # Do not ship an unusable optional plugin or fake an ABI symlink.
        for plugin in internal.glob("PySide6/Qt/plugins/imageformats/libqwebp.so"):
            plugin.unlink()
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
                check_elf(binary, target)
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
        (frozen / "LICENSE").write_text((ROOT / "LICENSE").read_text())
        shutil.copy2(
            "/usr/share/doc/fonts-noto-cjk/copyright", frozen / "Noto-CJK-copyright"
        )
        arch = target["deb_arch"]
        portable = args.output.resolve() / f"{PACKAGE}_{PACKAGE_VERSION}_{arch}.run"
        build_portable(frozen, portable)
        print(portable)
        package = temp / "deb"
        installed = package / "opt" / PACKAGE
        installed.parent.mkdir(parents=True)
        shutil.copytree(frozen, installed)
        (installed / "BUILD-INFO.json").write_text(json.dumps(report, indent=2))
        control = package / "DEBIAN"
        control.mkdir()
        (control / "control").write_text(
            f"Package: {PACKAGE}\nVersion: {PACKAGE_VERSION}\nArchitecture: {arch}\n"
            "Maintainer: TacCap firmware updater maintainers\nSection: utils\nPriority: optional\n"
            f"Depends: udev, libc6 (>= {target['glibc']}), libx11-6, libxext6, libxrender1, libfontconfig1, libfreetype6, libxcb-cursor0, libxkbcommon-x11-0, libsm6, libice6, libxcb-shape0, libxcb-icccm4, libxcb-keysyms1, libgl1, libegl1\n"
            "Description: TacCap follower firmware update and rollback assistant\n"
            " Bundled Python, Qt, CJK font and native SDK. Runs as a normal user.\n"
        )
        install_serial_rules(package)
        bin_dir = package / "usr/bin"
        bin_dir.mkdir(parents=True)
        launcher = bin_dir / PACKAGE
        launcher.write_text(
            "#!/bin/sh\n"
            "unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD\n"
            f'exec /opt/{PACKAGE}/{PACKAGE} "$@"\n'
        )
        launcher.chmod(0o755)
        pixmaps = package / "usr/share/pixmaps"
        pixmaps.mkdir(parents=True)
        shutil.copy2(app / "icon.png", pixmaps / f"{PACKAGE}.png")
        install_icons(app / "icon.png", package / "usr/share")
        desktop = package / "usr/share/applications"
        desktop.mkdir(parents=True)
        (desktop / f"{PACKAGE}.desktop").write_text(
            "[Desktop Entry]\nType=Application\nName=Xense TacCap Firmware Updater\n"
            "Name[zh_CN]=Xense TacCap 固件更新\n"
            f"Exec=/usr/bin/{PACKAGE}\nIcon={PACKAGE}\nTerminal=false\nStartupWMClass={PACKAGE}\nCategories=Utility;\n"
        )
        docs = package / "usr/share/doc" / PACKAGE
        docs.mkdir(parents=True)
        shutil.copy2(ROOT / "docs/FIRMWARE_UPDATER.md", docs / "README.md")
        shutil.copy2(ROOT / "LICENSE", docs / "copyright")
        shutil.copy2(
            "/usr/share/doc/fonts-noto-cjk/copyright", docs / "Noto-CJK-copyright"
        )
        out = args.output.resolve() / f"{PACKAGE}_{PACKAGE_VERSION}_{arch}.deb"
        subprocess.run(
            [
                "dpkg-deb",
                "--root-owner-group",
                *(["-Zgzip", "-z3"] if arch == "arm64" else []),
                "--build",
                str(package),
                str(out),
            ],
            check=True,
        )
        print(out)


if __name__ == "__main__":
    main()

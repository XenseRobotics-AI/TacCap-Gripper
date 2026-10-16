#!/usr/bin/env python3
"""Entry point for the isolated Qt updater."""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if not getattr(sys, "frozen", False):
    sys.path.insert(0, str(HERE.parents[1] / "python" / "examples"))


def main():
    from fontconfig_runtime import configure_fontconfig

    configure_fontconfig()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--language", choices=("zh", "en"), help="UI language for this run"
    )
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="Read-only OS/SDK diagnostics; no serial commands",
    )
    parser.add_argument(
        "--install-desktop",
        action="store_true",
        help="Register AppImage icon without opening hardware",
    )
    parser.add_argument("--ui-smoke-test", action="store_true")
    parser.add_argument("--screenshot", help="Save a hardware-free UI preview")
    args = parser.parse_args()
    from i18n import initialize, language, tr

    initialize(args.language)
    if args.diagnose:
        from host_diagnostics import diagnostic_report

        report = diagnostic_report()
        report["scope"] = tr(report["scope"])
        for issue in report["issues"]:
            issue["message"] = tr(issue["message"])
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    if args.install_desktop:
        import os

        from desktop_integration import integrate_appimage

        if os.geteuid() == 0:
            raise SystemExit(tr("请以普通用户注册应用菜单"))
        if not integrate_appimage(HERE / "icon.png"):
            raise SystemExit(tr("此选项仅适用于 AppImage"))
        print(tr("AppImage 菜单入口与图标已注册"))
        return
    if not args.self_test:
        from qt_ui import main as gui

        gui(args)
        return
    import bundled
    import core
    import xense.taccap as sdk
    from build_info import read_build_info
    from PySide6.QtCore import qVersion
    from xense.taccap import _taccap_native as native

    assert hasattr(native, "expected_open_reverse")
    from control_parameters import FIELDS, config_type, values_of

    for mode in FIELDS:
        assert set(values_of(config_type(sdk, mode)(), mode)) == {
            f.name for f in FIELDS[mode]
        }
    assert core.catalog_at(HERE / "catalog.json")["schema"] == 1
    bundled_versions = bundled.verify_all(core.catalog_at(HERE / "catalog.json"))
    external = []
    if getattr(sys, "frozen", False):
        bundle = Path(sys._MEIPASS).resolve()
        for line in Path("/proc/self/maps").read_text().splitlines():
            candidate = line.split()[-1]
            if candidate.startswith("/") and ".so" in candidate:
                path = Path(candidate).resolve()
                if not path.is_relative_to(bundle) and not str(path).startswith(
                    ("/usr/lib/", "/lib/")
                ):
                    external.append(str(path))
        if external:
            raise RuntimeError(
                f"Non-isolated shared libraries: {sorted(set(external))}"
            )
    print(
        json.dumps(
            {
                "build": read_build_info(),
                "sdk": sdk.__version__,
                "native": native.__file__,
                "python": sys.version,
                "frozen": bool(getattr(sys, "frozen", False)),
                "qt": qVersion(),
                "language": language(),
                "controller_test_modes": list(FIELDS),
                "bundled_firmware": bundled_versions,
                "mcu_versions": bundled.verify_mcu_versions(
                    core.catalog_at(HERE / "catalog.json")
                ),
                "approved_mcu_versions": {
                    role: [
                        e["version"]
                        for e in bundled.mcu_entries(
                            core.catalog_at(HERE / "catalog.json"), role
                        )
                    ]
                    for role in ("master", "slave")
                },
                "external_non_system_libraries": external,
            }
        )
    )


if __name__ == "__main__":
    main()

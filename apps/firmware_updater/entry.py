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
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--ui-smoke-test", action="store_true")
    parser.add_argument("--screenshot", help="Save a hardware-free UI preview")
    args = parser.parse_args()
    if not args.self_test:
        from qt_ui import main as gui

        gui(args)
        return
    import core
    import xense.taccap as sdk
    from PySide6.QtCore import qVersion
    from xense.taccap import _taccap_native as native

    assert hasattr(native, "expected_open_reverse")
    assert core.catalog_at(HERE / "catalog.json")["schema"] == 1
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
                "sdk": sdk.__version__,
                "native": native.__file__,
                "python": sys.version,
                "frozen": bool(getattr(sys, "frozen", False)),
                "qt": qVersion(),
                "external_non_system_libraries": external,
            }
        )
    )


if __name__ == "__main__":
    main()

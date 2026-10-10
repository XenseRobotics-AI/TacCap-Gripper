"""Frozen build identity; source previews are explicitly marked development."""

import json
from pathlib import Path

VERSION = "0.1.1"
PACKAGE_REVISION = 9
PACKAGE_VERSION = f"{VERSION}-{PACKAGE_REVISION}"


def read_build_info():
    path = Path(__file__).with_name("build-info.json")
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"version": VERSION, "sha": "dev", "dirty": True}


def window_title():
    info = read_build_info()
    suffix = "-dirty" if info["dirty"] else ""
    return f"Xense TacCap 固件更新 · v{info['version']} · build {info['sha']}{suffix}"

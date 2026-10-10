"""Pinned defaults shipped in the installer, validated against the catalog."""

import sys
from pathlib import Path

import core

FILES = {
    ("mcu", "slave"): "gripper/tc-gu-01-slave-1.2.16.bin",
    ("motor", "EL05"): "motor/EL05_1.0.5.0.4.bin",
    ("motor", "RS00"): "motor/rs00-0.0.3.32.bin",
}


def firmware_root():
    here = Path(__file__).resolve().parent
    return (
        here / "firmware"
        if getattr(sys, "frozen", False)
        else here.parents[1] / "firmware"
    )


def default_image(kind, model, catalog, root=None):
    relative = FILES.get((kind, model))
    if relative is None:
        return None
    path = (root or firmware_root()) / relative
    image = core.load_image(path, kind, catalog)
    if image.model != model:
        raise ValueError("内置固件型号与清单不符")
    return path, image


def verify_all(catalog, root=None):
    return {
        f"{kind}/{model}": default_image(kind, model, catalog, root)[1].version
        for kind, model in FILES
    }

"""Newest approved defaults; frozen with each installer, never downloaded."""

import sys
from pathlib import Path

import core


def approved_defaults(catalog):
    selected = {}
    for entry in catalog["images"]:
        if not entry.get("file"):
            continue
        key = entry["kind"], entry["model"]
        if key not in selected or core.version_key(entry["version"]) > core.version_key(
            selected[key]["version"]
        ):
            selected[key] = entry
    return {key: entry["file"] for key, entry in selected.items()}


FILES = approved_defaults(core.catalog_at(Path(__file__).with_name("catalog.json")))


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


def mcu_entries(catalog, role):
    entries = [e for e in catalog["images"] if (e["kind"], e["model"]) == ("mcu", role)]
    return sorted(entries, key=lambda e: core.version_key(e["version"]), reverse=True)


def packaged_files():
    """Only latest defaults are payload; other approvals are metadata only."""
    return list(FILES.values())


def mcu_image(entry, catalog, root=None):
    path = (root or firmware_root()) / entry["file"]
    image = core.load_image(path, "mcu", catalog)
    core.validate_mcu_target(image, entry["model"])
    if image.version != entry["version"]:
        raise ValueError("Approved MCU version mismatch")
    return path, image


def verify_mcu_versions(catalog, root=None):
    return {
        role: [default_image("mcu", role, catalog, root)[1].version]
        for role in ("master", "slave")
    }

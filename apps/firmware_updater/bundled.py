"""Newest approved defaults; frozen with each installer, never downloaded."""

import hashlib
import subprocess
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


def materialize_mcu_images(catalog, repository, destination):
    """Build-only history recovery; frozen customers never need Git/network."""
    files = []
    payloads = []
    for role in ("master", "slave"):
        for entry in mcu_entries(catalog, role):
            relative = Path(entry["file"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("Invalid approved MCU path")
            source = repository / "firmware" / relative
            if source.is_file():
                data = source.read_bytes()
            else:
                history = catalog.get("history", {}).get(entry["file"])
                if not history:
                    raise ValueError(f"Missing approved MCU image/history: {relative}")
                commit = history["commit"]
                if len(commit) != 40 or any(
                    c not in "0123456789abcdef" for c in commit
                ):
                    raise ValueError("History must pin a full commit SHA")
                data = subprocess.check_output(
                    ["git", "show", f"{commit}:{history['path']}"],
                    cwd=repository,
                    timeout=20,
                )
            if hashlib.sha256(data).hexdigest() != entry["sha256"]:
                raise ValueError(f"Approved MCU SHA256 mismatch: {relative}")
            files.append(entry["file"])
            payloads.append((relative, data))
    # Do not leave a partial reviewed set when any history/hash check fails.
    for relative, data in payloads:
        path = destination / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return files


def mcu_image(entry, catalog, root=None):
    path = (root or firmware_root()) / entry["file"]
    if not path.is_file() and root is None and not getattr(sys, "frozen", False):
        repository = Path(__file__).resolve().parents[2]
        cache = repository / "build/updater-mcu"
        materialize_mcu_images(catalog, repository, cache)
        path = cache / entry["file"]
    image = core.load_image(path, "mcu", catalog)
    core.validate_mcu_target(image, entry["model"])
    if image.version != entry["version"]:
        raise ValueError("Approved MCU version mismatch")
    return path, image


def verify_mcu_versions(catalog, root=None):
    return {
        role: [
            mcu_image(e, catalog, root)[1].version for e in mcu_entries(catalog, role)
        ]
        for role in ("master", "slave")
    }

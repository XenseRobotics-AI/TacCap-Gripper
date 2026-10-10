"""Hardware-free AppDir metadata and dependency verification checks."""

import importlib.util
import json
from pathlib import Path

import pytest

SOURCE = (
    Path(__file__).resolve().parents[2] / "packaging/firmware_updater/build_appimage.py"
)
spec = importlib.util.spec_from_file_location("updater_appimage", SOURCE)
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def test_dependency_hash_rejects_unreviewed_tool(tmp_path):
    tool = tmp_path / "tool"
    tool.write_bytes(b"untrusted")
    with pytest.raises(ValueError, match="Unreviewed"):
        builder.verify_file(tool, builder.TOOL_SHA)


def test_appdir_preserves_identity_and_launcher(tmp_path):
    package = builder.PACKAGE
    extracted = tmp_path / "deb"
    internal = extracted / "opt" / package / "_internal"
    internal.mkdir(parents=True)
    info = dict(version="0.1.0", package_version="0.1.0", sha="abc123", dirty=False)
    (internal / "build-info.json").write_text(json.dumps(info))
    for folder, name, content in (
        ("pixmaps", f"{package}.png", "icon"),
        (
            "applications",
            f"{package}.desktop",
            f"[Desktop Entry]\nExec=/usr/bin/{package}\n",
        ),
    ):
        path = extracted / "usr/share" / folder / name
        path.parent.mkdir(parents=True)
        path.write_text(content)
    appdir = tmp_path / "AppDir"
    assert builder.prepare_appdir(extracted, appdir) == info
    assert "Exec=AppRun" in (appdir / f"{package}.desktop").read_text()
    assert (appdir / ".DirIcon").read_text() == "icon"
    launcher = (appdir / "AppRun").read_text()
    assert '"$@"' in launcher and "unset PYTHONPATH" in launcher
    assert (appdir / "AppRun").stat().st_mode & 0o111

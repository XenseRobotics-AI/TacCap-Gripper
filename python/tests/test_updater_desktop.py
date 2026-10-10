"""User-local AppImage desktop integration without modifying real user files."""

import importlib.util
from pathlib import Path

import pytest

SOURCE = (
    Path(__file__).resolve().parents[2] / "apps/firmware_updater/desktop_integration.py"
)
spec = importlib.util.spec_from_file_location("updater_desktop", SOURCE)
desktop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(desktop)


def test_appimage_launcher_and_icon(tmp_path, monkeypatch):
    image = tmp_path / "Tac Cap%tool.AppImage"
    image.touch()
    icon = tmp_path / "source.png"
    icon.write_bytes(b"icon")
    root = tmp_path / "data"
    monkeypatch.setenv("APPIMAGE", str(image))
    assert desktop.integrate_appimage(icon, root)
    entry = root / "applications" / f"{desktop.APPIMAGE_ID}.desktop"
    assert "Tac Cap%%tool.AppImage" in entry.read_text()
    assert f"StartupWMClass={desktop.desktop_id()}" in entry.read_text()
    assert (
        root / "icons/hicolor/512x512/apps" / f"{desktop.APPIMAGE_ID}.png"
    ).read_bytes() == b"icon"
    assert desktop.integrate_appimage(icon, root)


def test_preserve_user_entry_and_skip_non_appimage(tmp_path, monkeypatch):
    monkeypatch.delenv("APPIMAGE", raising=False)
    assert not desktop.integrate_appimage(tmp_path / "absent", tmp_path)
    image = tmp_path / "test.AppImage"
    image.touch()
    monkeypatch.setenv("APPIMAGE", str(image))
    entry = tmp_path / "applications" / f"{desktop.APPIMAGE_ID}.desktop"
    entry.parent.mkdir()
    entry.write_text("user owned")
    with pytest.raises(ValueError, match="not owned"):
        desktop.integrate_appimage(tmp_path / "absent", tmp_path)
    assert entry.read_text() == "user owned"

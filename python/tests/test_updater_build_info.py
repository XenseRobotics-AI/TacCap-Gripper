"""Application and package versions must share the suffix-free version."""

import importlib.util
import json
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "apps/firmware_updater/build_info.py"
spec = importlib.util.spec_from_file_location("updater_build_info", SOURCE)
build_info = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_info)


def test_application_and_package_versions_match_without_revision():
    assert build_info.VERSION == "0.1.2"
    assert build_info.PACKAGE_VERSION == build_info.VERSION
    assert "-" not in build_info.PACKAGE_VERSION


def test_frozen_title_uses_application_version(tmp_path, monkeypatch):
    monkeypatch.setattr(build_info, "__file__", str(tmp_path / "build_info.py"))
    (tmp_path / "build-info.json").write_text(
        json.dumps(
            {
                "version": "0.1.2",
                "package_version": "0.1.2",
                "sha": "abcdef123456",
                "dirty": False,
            }
        )
    )
    assert build_info.window_title() == (
        "Xense TacCap 固件更新 · v0.1.2 · build abcdef123456"
    )


def test_source_preview_keeps_development_marker(tmp_path, monkeypatch):
    monkeypatch.setattr(build_info, "__file__", str(tmp_path / "build_info.py"))
    assert build_info.window_title().endswith("v0.1.2 · build dev-dirty")

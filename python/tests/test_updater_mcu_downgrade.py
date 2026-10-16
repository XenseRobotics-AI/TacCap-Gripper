"""Approved MCU selection and downgrade safety, without opening hardware."""

import hashlib
import importlib.util
import sys
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import bundled  # noqa: E402
import core  # noqa: E402
from test_updater_policy import FakeDevice, FakeMaster, image, run  # noqa: E402


def test_cancel_mcu_downgrade_before_any_motor_or_configuration_write():
    device = FakeDevice()
    prompts = []

    def ask(title, body):
        prompts.append(title)
        return title != "确认夹爪 MCU回滚"

    with pytest.raises(RuntimeError, match="用户取消"):
        run(device, image("mcu", "1.2.14"), image("motor", "1.0.5.0.2"), ask)
    assert device.events == []
    assert prompts == ["开始升级/回滚", "确认夹爪 MCU回滚"]


def test_mcu_version_change_after_motor_work_invalidates_downgrade_confirmation():
    device = FakeDevice()
    device.mcu_version = Mock(side_effect=["1.2.16", "1.2.15"])
    with pytest.raises(RuntimeError, match="版本.*变化"):
        run(device, image("mcu", "1.2.14"), image("motor", "1.0.5.0.2"))
    assert any(e[0] == "flash_motor" for e in device.events)
    assert not any(e[0] == "flash_mcu" for e in device.events)


@pytest.mark.parametrize("actual", ["1.2.14", "1.2.13", None])
def test_stale_inspection_does_not_allow_any_downgrade_or_motor_write(actual):
    device = FakeDevice()
    device.mcu_version = Mock(return_value=actual)
    with pytest.raises(RuntimeError, match="版本.*变化"):
        run(device, image("mcu", "1.2.14"), image("motor", "1.0.5.0.2"))
    assert not device.events


def test_rs00_cancel_downgrade_does_not_start_can_id_configuration():
    device = FakeDevice()
    state = device.inspect()
    state["model"] = "RS00"
    device.inspect = Mock(return_value=state)
    device.ensure_can_id = Mock()
    with pytest.raises(RuntimeError, match="用户取消"):
        run(
            device,
            mcu=image("mcu", "1.2.14"),
            ask=lambda title, _: "MCU回滚" not in title,
        )
    device.ensure_can_id.assert_not_called()
    assert not device.events


@pytest.mark.parametrize("role,version", [("master", "1.2.5"), ("slave", "1.2.14")])
def test_mcu_only_downgrade_requires_red_confirmation_and_actual_version(role, version):
    device = FakeMaster() if role == "master" else FakeDevice()
    device.mcu = "1.2.6" if role == "master" else "1.2.16"
    prompts = []
    events = run(
        device,
        mcu=image("mcu", version, role),
        ask=lambda title, body: prompts.append((title, body)) or True,
    )
    assert device.mcu == version
    assert len([e for e in device.events if e[0] == "flash_mcu"]) == 1
    assert not any(e[0] in ("flash_motor", "record") for e in device.events)
    warning = next(body for title, body in prompts if "MCU回滚" in title)
    assert "配置" in warning and "恢复" in warning
    assert events[-1][0] == "success"
    if role == "master":
        assert not any(e[0] in ("protocol", "homing") for e in device.events)


@pytest.mark.parametrize("role,version", [("master", "1.2.4"), ("slave", "1.2.13")])
def test_unsupported_mcu_target_rejected_before_any_write(role, version):
    device = FakeMaster() if role == "master" else FakeDevice()
    with pytest.raises((ValueError, RuntimeError)):
        run(device, mcu=image("mcu", version, role))
    assert not device.events


def test_approved_mcu_lists_are_role_specific_and_keep_latest_default():
    catalog = core.catalog_at(APP / "catalog.json")
    assert [e["version"] for e in bundled.mcu_entries(catalog, "master")] == [
        "1.2.6",
        "1.2.5",
    ]
    assert [e["version"] for e in bundled.mcu_entries(catalog, "slave")] == [
        "1.2.16",
        "1.2.14",
    ]
    assert bundled.FILES["mcu", "master"].endswith("1.2.6.bin")
    assert bundled.FILES["mcu", "slave"].endswith("1.2.16.bin")


def test_history_images_match_approved_hashes_and_are_not_added_to_source(tmp_path):
    repository = APP.parents[1]
    catalog = core.catalog_at(APP / "catalog.json")
    for history in catalog["history"].values():
        probe = bundled.subprocess.run(
            ["git", "cat-file", "-e", f"{history['commit']}:{history['path']}"],
            cwd=repository,
            capture_output=True,
        )
        if probe.returncode:
            pytest.skip(
                "Package history integration requires the pinned Git objects; package builds still fail closed"
            )
    files = bundled.materialize_mcu_images(catalog, repository, tmp_path)
    assert len(files) == 4
    assert bundled.verify_mcu_versions(catalog, tmp_path) == {
        "master": ["1.2.6", "1.2.5"],
        "slave": ["1.2.16", "1.2.14"],
    }
    for entry in catalog["images"]:
        if entry["kind"] == "mcu":
            assert (
                hashlib.sha256((tmp_path / entry["file"]).read_bytes()).hexdigest()
                == entry["sha256"]
            )
    assert not (repository / "firmware/gripper/tc-gu-01-master-1.2.5.bin").exists()


def test_bad_history_payload_is_rejected_before_materialization(tmp_path):
    repository = tmp_path / "repo"
    repository.mkdir()
    catalog = core.catalog_at(APP / "catalog.json")
    catalog["images"] = [
        e for e in catalog["images"] if e["file"] in catalog["history"]
    ]
    with patch.object(bundled.subprocess, "check_output", return_value=b"tampered"):
        with pytest.raises(ValueError, match="SHA256"):
            bundled.materialize_mcu_images(catalog, repository, tmp_path / "output")
    assert not list((tmp_path / "output").rglob("*.bin"))


@pytest.fixture
def ui():
    if importlib.util.find_spec("PySide6") is None:
        pytest.skip("Qt UI checks require the packaging environment")
    import i18n
    import qt_ui
    from PySide6.QtCore import QCoreApplication, QEvent
    from PySide6.QtWidgets import QApplication

    previous = i18n.language()
    i18n.set_language("zh")
    app = QApplication.instance() or QApplication([])
    qt_ui.configure(app)
    window = qt_ui.App(auto_connect=False)
    window.show()
    app.processEvents()
    with patch.object(i18n, "settings", return_value=Mock()):
        yield window, app
    window.busy = False
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    i18n.set_language(previous)


@pytest.mark.parametrize("role,version", [("master", "1.2.5"), ("slave", "1.2.14")])
def test_role_menu_selects_verified_old_mcu_without_changing_motor(ui, role, version):
    window, app = ui
    identity = ("TESTm" if role == "master" else "TESTs", "usb")
    window.selected_identity = identity
    window.current_info = dict(
        recorded=True, model="EL05", mcu="1.2.6" if role == "master" else "1.2.16"
    )
    window.apply_defaults()
    motor_before = window.paths["motor"]
    menu = window.make_mcu_menu()
    try:
        actions = [a for a in menu.actions() if a.data()]
        assert [a.data() for a in actions] == (
            ["1.2.6", "1.2.5", "file"]
            if role == "master"
            else ["1.2.16", "1.2.14", "file"]
        )
        with patch.object(window, "launch") as launch:
            actions[1].trigger()
            launch.assert_not_called()
        app.processEvents()
        assert window.mcu_target.version == version
        assert core.load_image(window.paths["mcu"], "mcu", window.catalog).model == role
        assert window.paths["motor"] == motor_before
        assert window.mcu_hint.isVisible()
        assert "不恢复旧配置" in window.mcu_hint.text()
        assert "mcu" in window.overrides
        assert (
            window.info.geometry().bottom() < window.file_labels["mcu"].geometry().top()
        )
        assert window.width() <= 700
        window.use_defaults()
        assert not window.mcu_hint.isVisible()
    finally:
        menu.deleteLater()


def test_menu_manual_file_route_and_busy_guard(ui):
    window, _ = ui
    menu = window.make_mcu_menu()
    try:
        with patch.object(window, "choose") as choose:
            next(a for a in menu.actions() if a.data() == "file").trigger()
            choose.assert_called_once_with("mcu")
        window.busy = True
        before = window.paths.copy()
        window.select_mcu_version(bundled.mcu_entries(window.catalog, "slave")[1])
        window.clear("mcu")
        assert window.paths == before
    finally:
        menu.deleteLater()


def test_downgrade_selection_does_not_lose_hint_on_readback_and_translates(ui):
    import i18n

    window, app = ui
    entry = bundled.mcu_entries(window.catalog, "slave")[1]
    window.select_mcu_version(entry)
    assert not window.mcu_hint.isVisible()  # No actual current version yet.
    window.selected_identity = ("TESTs", "usb")
    window.current_info = dict(recorded=True, model="EL05", mcu="1.2.16")
    window.apply_defaults()
    assert window.mcu_hint.isVisible()
    window.language_combo.setCurrentIndex(1)
    app.processEvents()
    assert "MCU downgrade" in window.mcu_hint.text()
    menu = window.make_mcu_menu()
    try:
        assert all(not i18n.CJK.search(a.text()) for a in menu.actions())
        assert "historical" in menu.actions()[1].text()
    finally:
        menu.deleteLater()
    window.clear("mcu")
    assert not window.mcu_hint.isVisible()


@pytest.mark.parametrize("role", ["master", "slave"])
def test_mcu_readback_failure_after_downgrade_never_reports_success(role):
    device = FakeMaster() if role == "master" else FakeDevice()
    target = image("mcu", "1.2.5" if role == "master" else "1.2.14", role)
    device.flash_mcu = Mock()  # Simulate a transfer which did not apply.
    events = []
    with pytest.raises(RuntimeError, match="版本"):
        core.execute(device, target, None, lambda *_: True, lambda *e: events.append(e))
    assert not any(e[0] == "success" for e in events)


def test_frozen_missing_old_image_fails_without_using_git(tmp_path):
    catalog = core.catalog_at(APP / "catalog.json")
    entry = bundled.mcu_entries(catalog, "master")[1]
    with (
        patch.object(sys, "frozen", True, create=True),
        patch.object(bundled.subprocess, "check_output") as git,
    ):
        with pytest.raises(FileNotFoundError):
            bundled.mcu_image(entry, catalog, root=tmp_path)
        git.assert_not_called()

"""Legacy MCU-only upgrade/downgrade; no motor/model writes or control claims."""

import sys
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import bundled  # noqa: E402
import core  # noqa: E402
import test_updater_mcu_downgrade as history_cases  # noqa: E402
import test_updater_ota_indicator as adapter_cases  # noqa: E402
from test_updater_policy import FakeDevice, image, run  # noqa: E402

ui = history_cases.ui
rig = adapter_cases.rig


class LegacyDevice(FakeDevice):
    def prepare_mcu_only(self):
        self.events.append(("prepare_mcu_only",))

    def inspect(self):
        info = super().inspect()
        if core.version_key(self.mcu) < core.version_key("1.2.14"):
            info.update(model="未知", recorded=False, legacy_mcu_only=True)
        return info


@pytest.mark.parametrize(
    "current,target", [("1.2.16", "1.1.6"), ("1.1.6", "1.2.16"), ("1.1.6", "1.1.6")]
)
def test_legacy_round_trip_uses_only_mcu_and_actual_readback(current, target):
    device = LegacyDevice()
    device.mcu = current
    prompts = []
    events = run(
        device,
        mcu=image("mcu", target),
        ask=lambda title, body: prompts.append((title, body)) or True,
    )
    assert device.mcu == target and events[-1][0] == "success"
    assert [e[0] for e in device.events] == ["prepare_mcu_only", "flash_mcu", "cycle"]
    assert any("旧版" in title for title, _ in prompts)


def test_legacy_target_refuses_rs00_before_any_write():
    device = LegacyDevice()
    inspect = device.inspect
    device.inspect = lambda: {**inspect(), "model": "RS00"}
    with pytest.raises(RuntimeError, match="EL05"):
        run(device, mcu=image("mcu", "1.1.6"))
    assert not device.events


@pytest.mark.parametrize("current,target", [("1.2.16", "1.1.6"), ("1.1.6", "1.2.16")])
def test_legacy_combined_motor_update_is_rejected(current, target):
    device = LegacyDevice()
    device.mcu = current
    with pytest.raises(RuntimeError, match="仅.*MCU"):
        run(device, mcu=image("mcu", target), motor=image("motor", "1.0.5.0.4"))
    assert not device.events


def test_legacy_cancel_and_same_version_skip_do_not_prepare_motor():
    device = LegacyDevice()
    device.mcu = "1.1.6"
    with pytest.raises(RuntimeError, match="取消"):
        run(device, mcu=image("mcu", "1.2.16"), ask=lambda *_: False)
    assert not device.events
    events = run(
        device, mcu=image("mcu", "1.1.6"), ask=lambda title, _: "重复刷写" not in title
    )
    assert not device.events and events[-1][0] == "success"


@pytest.mark.parametrize("failure", ["mcu", "cycle", "wrong-version"])
def test_legacy_failure_never_reports_success(failure):
    device = LegacyDevice()
    device.fail = failure
    if failure == "wrong-version":
        device.flash_mcu = lambda _: device.events.append(("flash_mcu",))
    events = []
    with pytest.raises(RuntimeError):
        core.execute(
            device,
            image("mcu", "1.1.6"),
            None,
            lambda *_: True,
            lambda *e: events.append(e),
        )
    assert not any(e[0] == "success" for e in events)


def test_legacy_preparation_version_change_stops_before_flash():
    device = LegacyDevice()
    device.prepare_mcu_only = lambda: setattr(device, "mcu", "1.2.15")
    with pytest.raises(RuntimeError, match="版本.*变化"):
        run(device, mcu=image("mcu", "1.1.6"))
    assert not any(e[0] == "flash_mcu" for e in device.events)


def test_legacy_approval_matches_real_historical_bytes(tmp_path):
    catalog = core.catalog_at(APP / "catalog.json")
    path = history_cases.external_mcu_file(
        catalog, "slave", "1.1.6", tmp_path / "legacy.bin"
    )
    img = core.load_image(path, "mcu", catalog)
    core.validate_mcu_target(img, "slave")
    assert img.version == "1.1.6" and "EL05" in img.warning
    assert len(bundled.packaged_files()) == 4


def test_legacy_discovery_uses_identity_and_version_only(rig):
    rig.flow._version_tuple.return_value = (1, 1, 6, 0)
    rig.flow._version_text.return_value = "1.1.6"
    info = rig.device.inspect()
    assert info["legacy_mcu_only"] and not info["recorded"] and info["model"] == "未知"
    rig.sdk.FollowerGripper.assert_not_called()
    rig.leader.motor.get_model.assert_not_called()
    rig.leader.motor.motor_version.assert_not_called()


def test_old_follower_open_override_is_admin_only_and_explicit(rig):
    with rig.device.opened(allow_outdated=True):
        pass
    rig.sdk.FollowerGripper.assert_called_once_with(
        mcu_device="/dev/test-verified", allow_outdated_firmware=True
    )


def test_prepare_legacy_mcu_only_selects_mit_admin_path(rig):
    rig.device.protocol = Mock()
    rig.device.prepare_mcu_only()
    rig.device.protocol.assert_called_once_with("Mit", allow_outdated=True)


def test_legacy_admin_reads_no_new_model_or_home_diagnostics(rig):
    rig.flow._version_text.return_value = "1.1.6"
    rig.follower.motor.get_protocol.return_value = rig.sdk.MotorProtocol.Mit
    rig.device.prepare_mcu_only()
    assert rig.sdk.FollowerGripper.call_count == 2
    assert all(
        c.kwargs.get("allow_outdated_firmware") is True
        for c in rig.sdk.FollowerGripper.call_args_list
    )
    rig.follower.motor.get_model.assert_not_called()
    rig.flow._prepare_motor_admin.assert_called_once_with(rig.follower)
    rig.flow._wait_motor_protocol.assert_called_once_with(
        rig.follower.motor, rig.sdk.MotorProtocol.Mit
    )


def test_legacy_admin_cannot_request_private(rig):
    with pytest.raises(RuntimeError, match="MIT 管理"):
        rig.device.protocol("Private", allow_outdated=True)
    rig.sdk.FollowerGripper.assert_not_called()


def test_model_changed_to_rs00_before_preparation_stops_writes(rig):
    rig.flow._version_text.return_value = "1.2.16"
    rig.follower.motor.get_model.return_value = Mock(name="model", from_flash=True)
    rig.follower.motor.get_model.return_value.name = "RS00"
    with pytest.raises(RuntimeError, match="EL05"):
        rig.device.prepare_mcu_only()
    rig.flow._prepare_motor_admin.assert_not_called()
    rig.follower.motor.switch_protocol.assert_not_called()


def test_legacy_ui_selection_hides_motor_and_allows_return_upgrade(ui, tmp_path):
    window, app = ui
    window.selected_identity = ("TESTs", "usb")
    window.current_info = dict(model="EL05", recorded=True, mcu="1.2.16")
    window.apply_defaults()
    old = next(
        e
        for e in bundled.mcu_entries(window.catalog, "slave")
        if e["version"] == "1.1.6"
    )
    path = history_cases.external_mcu_file(
        window.catalog, "slave", "1.1.6", tmp_path / "legacy.bin"
    )
    with patch("qt_ui.QFileDialog.getOpenFileName", return_value=(str(path), "")):
        window.select_mcu_version(old)
    assert window.mcu_target.version == "1.1.6" and not window.paths["motor"]
    assert all(not widget.isVisible() for widget in window.motor_widgets)
    assert window.start.isEnabled()
    window.current_info = dict(
        model="未知", recorded=False, legacy_mcu_only=True, mcu="1.1.6", motor="未知"
    )
    window.use_defaults()
    assert window.mcu_target.version == "1.2.16" and not window.paths["motor"]
    assert window.start.isEnabled() and not window.model_row.isVisible()


def test_legacy_prompts_are_translated_without_chinese_escapes():
    import i18n

    previous = i18n.language()
    i18n.set_language("en")
    try:
        device = LegacyDevice()
        messages = []
        run(
            device,
            mcu=image("mcu", "1.1.6"),
            ask=lambda t, b: messages.extend((t, b)) or True,
        )
        for text in messages:
            translated = i18n.tr(text)
            assert not i18n.CJK.search(translated) and "\\u" not in translated
    finally:
        i18n.set_language(previous)


def test_legacy_actual_status_is_not_reported_as_model_missing():
    from version_status import device_summary

    info = dict(
        model="未知", recorded=False, legacy_mcu_only=True, mcu="1.1.6", motor="未知"
    )
    html, text, tone, status = device_summary(
        info, "slave", core.catalog_at(APP / "catalog.json")
    )
    assert "旧版 MCU" in text and "EL05" in status and tone == "warning"
    assert "已是内置最新" not in text


def test_legacy_english_summary_and_actions_fit_compact_window(ui):
    import i18n
    from PySide6.QtCore import QPoint

    window, app = ui
    i18n.set_language("en")
    identity = ("TESTs", "usb")
    window.selected_identity = identity
    info = dict(
        model="未知", recorded=False, legacy_mcu_only=True, mcu="1.1.6", motor="未知"
    )
    window.emit("info", (identity, info))
    window.drain()
    app.processEvents()
    assert window.info.wordWrap()
    assert window.info.heightForWidth(window.info.width()) <= window.info.height()
    for widget in (window.start, window.refresh, window.info, window.mcu_choose):
        assert widget.mapTo(window, QPoint()).x() + widget.width() <= window.width()

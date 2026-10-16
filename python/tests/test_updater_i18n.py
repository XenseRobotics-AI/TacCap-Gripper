"""Language switching is presentation-only and must not touch devices."""

import ast
import importlib.util
import json
import sys
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import i18n  # noqa: E402


@pytest.fixture(autouse=True)
def isolate_language():
    previous = i18n.language()
    i18n.set_language("zh")
    yield
    i18n.set_language(previous)


def test_saved_preference_and_command_override_do_not_overwrite_each_other():
    store = Mock()
    store.value.return_value = "en"
    assert i18n.initialize(store=store) == "en"
    assert i18n.initialize("zh", store=store) == "zh"
    store.setValue.assert_not_called()
    i18n.set_language("en", persist=True, store=store)
    store.setValue.assert_called_once_with("ui/language", "en")
    store.sync.assert_called_once()
    store.value.return_value = "invalid"
    assert i18n.initialize(store=store) == "zh"
    with pytest.raises(ValueError):
        i18n.set_language("invalid", persist=True, store=store)


def test_real_settings_round_trip_is_user_scoped(tmp_path):
    from PySide6.QtCore import QSettings

    path = str(tmp_path / "updater.ini")
    i18n.set_language("en", persist=True, store=QSettings(path, QSettings.IniFormat))
    i18n.set_language("zh")
    assert i18n.initialize(store=QSettings(path, QSettings.IniFormat)) == "en"


def test_actual_activity_transfer_wait_and_runtime_diagnostics_are_english():
    from activity import Activity

    i18n.set_language("en")
    clock = [0.0]
    activity = Activity(clock=lambda: clock[0])
    activity.begin("准备切换/核验 Private 协议")
    activity.update(500, 1000, "包")
    clock[0] = 10.0
    for text in (activity.detail(), activity.heartbeat()):
        translated = i18n.tr(text)
        assert not i18n.CJK.search(translated) and "\\u" not in translated, translated
        assert "transfer" in translated
    activity.update(1000, 1000, "包")
    assert "verification still required" in i18n.tr(activity.detail())
    error = "等待电机协议 MotorProtocol.Private 稳定超时，最后读回 MotorProtocol.Mit。停止后续步骤；确认断开的是 24 V（仅拔 USB 不够），保持至少 2 秒，再重新运行。若仍失败，保留日志排查协议切换。"
    assert "USB alone is insufficient" in i18n.tr(error)
    assert "\\u" not in i18n.tr(error)


@pytest.mark.parametrize("skip_same", [True, False])
def test_real_policy_plan_confirmations_and_events_stay_english(skip_same):
    import core

    device = Mock()
    device.inspect.return_value = dict(
        sn="TESTs",
        model="EL05",
        recorded=True,
        mcu="1.2.16",
        motor="1.0.5.0.4（Flash 存档，非实时）",
    )
    device.mcu_version.return_value = "1.2.16"
    device.live_motor_version.return_value = "1.0.5.0.4"
    mcu = core.Image("mcu", "slave", "1.2.16", "digest", "", b"image")
    motor = core.Image("motor", "EL05", "1.0.5.0.4", "digest", "", b"image")
    messages = []

    def ask(title, body):
        messages.extend((title, body))
        return not (skip_same and "重复刷写" in title)

    i18n.set_language("en")
    core.execute(device, mcu, motor, ask, lambda kind, text: messages.append(text))
    for text in messages:
        translated = i18n.tr(text)
        assert not i18n.CJK.search(translated) and "\\u" not in translated, translated
    if skip_same:
        device.flash_mcu.assert_not_called()
        device.flash_motor.assert_not_called()
    else:
        device.flash_mcu.assert_called_once_with(mcu)
        device.flash_motor.assert_called_once_with(motor)


def test_technical_ids_and_unknown_native_diagnostics_are_lossless():
    i18n.set_language("en")
    value = "SN TCGU01A28Z0094s CAN ID 18 / 0.0.3.32 / 0x4A8CD360"
    assert i18n.tr(value) == value
    value = "ProtocolError: 外部驱动未知错误 0x42"
    translated = i18n.tr(value)
    assert not i18n.CJK.search(translated)
    assert "0x42" in translated
    import re

    assert (
        re.sub(r"\\u([0-9a-f]{4})", lambda m: chr(int(m[1], 16)), translated) == value
    )


def messages():
    for name in (
        "compact_layout",
        "qt_ui",
        "dialogs",
        "core",
        "device",
        "activity",
        "version_status",
        "build_info",
        "bundled",
        "entry",
    ):
        tree = ast.parse((APP / f"{name}.py").read_text())
        parents = {c: n for n in ast.walk(tree) for c in ast.iter_child_nodes(n)}
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                if isinstance(parents.get(node), ast.JoinedStr):
                    continue
                value = node.value
            elif isinstance(node, ast.JoinedStr):
                value = "".join(
                    part.value if isinstance(part, ast.Constant) else "VALUE"
                    for part in node.values
                )
            else:
                continue
            if i18n.CJK.search(value):
                yield name, node.lineno, value


def test_every_owned_message_has_an_english_translation_without_escape_fallback():
    i18n.set_language("en")
    missing = []
    for name, line, value in messages():
        result = i18n.tr(value)
        if i18n.CJK.search(result) or "\\u" in result:
            missing.append(f"{name}:{line} {value!r} -> {result!r}")
    assert not missing, "\n".join(missing)


def test_catalog_downgrade_warnings_are_translated_without_changing_policy():
    import core

    i18n.set_language("en")
    catalog = json.loads((APP / "catalog.json").read_text())
    for entry in catalog["images"]:
        if not entry["warning"]:
            continue
        warning = i18n.tr(entry["warning"])
        assert not i18n.CJK.search(warning) and "\\u" not in warning, warning
        img = core.Image(
            **{k: v for k, v in entry.items() if k != "file"}, data=b"test"
        )
        ask = Mock(return_value=False)
        with pytest.raises(RuntimeError, match="用户取消"):
            core.confirm_change(
                ask,
                "电机" if entry["kind"] == "motor" else "夹爪 MCU",
                "1.2.16" if entry["kind"] == "mcu" else "1.0.5.0.4",
                img,
            )
        title, body = ask.call_args.args
        assert "downgrade" in i18n.tr(title)
        assert "\\u" not in i18n.tr(body)
        assert entry["warning"] in body  # no policy mutation


HAS_QT = importlib.util.find_spec("PySide6") is not None


@pytest.fixture
def window():
    if not HAS_QT:
        pytest.skip("Qt required")
    import qt_ui
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    qt_ui.configure(app)
    w = qt_ui.App(auto_connect=False)
    w.show()
    app.processEvents()
    yield w, app
    if w.popup:
        w.popup.accept()
    w.busy = False
    w.close()
    app.processEvents()


def visible_texts(root):
    from PySide6.QtWidgets import QComboBox, QLabel, QPushButton

    values = [root.windowTitle()]
    for widget in root.findChildren(QLabel) + root.findChildren(QPushButton):
        values.extend((widget.text(), widget.toolTip()))
    for widget in root.findChildren(QComboBox):
        values.extend(widget.itemText(i) for i in range(widget.count()))
        values.append(widget.placeholderText())
    return "\n".join(values)


def test_live_switch_round_trip_keeps_selection_images_logs_and_no_worker(window):
    w, app = window
    identity = ("TCGU01A28Z0094s", "USBTEST")
    w.devices = [identity]
    w.combo.addItem("TCGU01A28Z0094s / USBTEST")
    w.combo.setCurrentIndex(0)
    w.selected_identity = identity
    current_info = w.current_info = dict(model="RS00", recorded=True)
    w.overrides = {"motor"}
    w.paths["motor"] = "/tmp/selected-version.bin"
    original_paths = w.paths.copy()
    w.log("MCU 实际版本已确认：1.2.16")
    raw_log = list(w.log_lines)
    with (
        patch("i18n.settings", return_value=Mock()),
        patch.object(w, "launch") as launch,
    ):
        w.language_combo.setCurrentIndex(1)
        app.processEvents()
        assert i18n.language() == "en"
        text = visible_texts(w)
        assert not i18n.CJK.search(text), text
        assert "\\u" not in text, text
        assert "Firmware Updater" in w.windowTitle()
        assert "MCU running version confirmed: 1.2.16" in w.text.toPlainText()
        assert w.paths == original_paths and w.log_lines == raw_log
        assert w.selected_identity == identity and w.current_info is current_info
        assert w.combo.currentIndex() == 0 and w.overrides == {"motor"}
        w.language_combo.setCurrentIndex(0)
        app.processEvents()
        assert w.start.text() == "开始更新"
        assert "MCU 实际版本已确认" in w.text.toPlainText()
        launch.assert_not_called()


def test_english_file_picker_does_not_inherit_chinese_native_buttons(window):
    import qt_ui

    w, _ = window
    i18n.set_language("en")
    with patch("qt_ui.QFileDialog.getOpenFileName", return_value=("", "")) as picker:
        w.choose("mcu")
    assert picker.call_args.args[1] == "Select firmware"
    assert picker.call_args.kwargs["options"] & qt_ui.QFileDialog.DontUseNativeDialog


def test_switch_during_operation_is_rejected_without_persisting(window):
    w, _ = window
    w.busy = True
    with patch("i18n.settings") as store:
        w.language_combo.setCurrentIndex(1)
        assert i18n.language() == "zh"
        assert w.language_combo.currentData() == "zh"
        store.assert_not_called()


def test_cleared_view_stays_clear_after_switch_and_export_is_english(window, tmp_path):
    w, _ = window
    w.log("准备切换/核验 Private 协议")
    w.clear_log_view()
    with patch("i18n.settings", return_value=Mock()):
        w.language_combo.setCurrentIndex(1)
    assert w.text.toPlainText() == ""
    path = tmp_path / "export.log"
    with patch("qt_ui.QFileDialog.getSaveFileName", return_value=(str(path), "")):
        w.export()
    exported = path.read_text()
    assert not i18n.CJK.search(exported)
    assert "\\u" not in exported, exported
    assert "Private protocol" in exported
    assert "准备切换" in "\n".join(w.log_lines)


@pytest.mark.parametrize("role", ["slave", "master"])
def test_english_device_summary_and_power_instructions(window, role):
    w, app = window
    sn = "TCGU01A28Z0094s" if role == "slave" else "TCGU01A25Z0017m"
    identity = (sn, "USBTEST")
    w.selected_identity = identity
    with patch("i18n.settings", return_value=Mock()):
        w.language_combo.setCurrentIndex(1)
    w.emit(
        "info",
        (
            identity,
            dict(
                model="RS00",
                recorded=True,
                mcu="1.2.15",
                motor="0.0.3.32（Flash 存档，非实时）",
                can_id=18,
                can_id_expected=18,
            ),
        ),
    )
    w.drain()
    app.processEvents()
    text = visible_texts(w)
    assert not i18n.CJK.search(text) and "\\u" not in text, text
    assert "update to 1.2.16" in text if role == "slave" else "Leader" in text
    w.show_power(dict(role=role, reason="MCU 刷写完成，必须硬重启"))
    app.processEvents()
    text = visible_texts(w.popup)
    assert not i18n.CJK.search(text) and "\\u" not in text, text
    assert "2 seconds" in text
    if role == "slave":
        assert "Unplugging USB alone" in text and "24 V" in text
    w.popup.accept()


@pytest.mark.parametrize(
    "title,phrase",
    [
        ("确认夹爪 MCU重复刷写", "Reflash anyway"),
        ("确认电机回滚", "Confirm downgrade"),
        ("确认机械标定", "Calibration is correct"),
    ],
)
def test_english_confirmations_keep_colors_and_safe_defaults(window, title, phrase):
    import dialogs

    w, app = window
    i18n.set_language("en")
    box, yes = dialogs.confirmation_box(
        w, title, "刷写过程中不可断电；重新上电可能触发自动标定。"
    )
    box.show()
    app.processEvents()
    assert yes.text() == phrase
    assert not i18n.CJK.search(visible_texts(box))
    assert yes is not box.defaultButton() and yes is not box.escapeButton()
    assert yes.property("actionTone") == (
        "danger" if "回滚" in title else "success" if "标定" in title else "warning"
    )
    assert yes.width() >= yes.fontMetrics().horizontalAdvance(yes.text()) + 20
    box.close()

"""Hardware-free Qt UI checks; run with the packaging Python."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP_DIR))
HAS_QT = importlib.util.find_spec("PySide6") is not None
if HAS_QT:
    import qt_ui
    from PySide6.QtWidgets import QApplication


@unittest.skipUnless(HAS_QT, "Qt UI tests require the packaging environment")
class UpdaterUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.family = qt_ui.configure(cls.app)

    def setUp(self):
        self.window = qt_ui.App(auto_connect=False)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        if self.window.popup:
            self.window.popup.accept()
        self.window.busy = False
        self.window.close()

    def test_clear_logs_preserves_export_history_and_layout(self):
        self.window.toggle.setChecked(True)
        self.window.log("before clear")
        saved = list(self.window.log_lines)
        height = self.window.height()
        self.window.clear_logs.click()
        self.app.processEvents()
        self.assertEqual(self.window.text.toPlainText(), "")
        self.assertEqual(self.window.log_lines, saved)
        self.assertEqual(self.window.height(), height)
        self.window.log("after clear")
        self.assertIn("after clear", self.window.text.toPlainText())
        self.assertNotIn("before clear", self.window.text.toPlainText())
        self.assertLessEqual(self.window.width(), 700)

    def test_initial_state_and_font(self):
        self.assertEqual(self.family, "Noto Sans CJK SC")
        self.assertFalse(self.app.windowIcon().isNull())
        self.assertEqual(self.app.desktopFileName(), "taccap-firmware-updater")
        self.assertFalse(self.window.text.isVisible())
        self.assertEqual(self.window.combo.currentIndex(), -1)
        self.assertFalse(self.window.busy)
        self.assertGreaterEqual(self.window.start.height(), 34)
        self.assertLessEqual(self.window.height(), 460)
        self.assertLessEqual(self.window.width(), 700)

    def test_environment_dialog_is_reusable_and_does_not_block_idle(self):
        import host_diagnostics as health
        from environment_guidance import enrich

        report = health.collect(
            sys_root=Path("/nonexistent/sys"), dev_root=Path("/nonexistent/dev")
        )
        report["runtime"] = {"ok": False}
        enrich(report)
        self.window.busy = True
        self.window.emit("diagnostics", report)
        self.window.emit("idle", "")
        self.window.drain()
        dialog = self.window.environment_dialog
        self.assertFalse(self.window.busy)
        self.assertTrue(dialog.retry.isEnabled())
        self.assertFalse(dialog.copy.isEnabled())
        with patch.object(self.window, "launch") as launch:
            dialog.retry.click()
            launch.assert_called_once()
        self.window.show_diagnostics(report)
        self.assertIs(dialog, self.window.environment_dialog)
        dialog.accept()

    def test_startup_precheck_blocks_serial_scan_when_runtime_is_broken(self):
        from unittest.mock import Mock

        import host_diagnostics as health

        module = Mock()
        with (
            patch.object(health, "diagnostic_report", return_value={"blocking": True}),
            patch.dict(sys.modules, {"device": module}),
            patch.object(self.window, "launch", side_effect=lambda fn: fn()),
        ):
            self.window.scan()
        module.discover.assert_not_called()
        self.assertEqual(self.window.events.get_nowait()[0], "environment")

    def test_diagnostic_copy_and_export_preserve_commands_and_evidence(self):
        import json
        import tempfile

        import host_diagnostics as health

        report = health.collect(
            sys_root=Path("/nonexistent/sys"), dev_root=Path("/nonexistent/dev")
        )
        report["actions"] = [
            {"message": "environment", "commands": 'id\nprintf "test"'}
        ]
        self.window.show_diagnostics(report)
        dialog = self.window.environment_dialog
        dialog.copy.click()
        self.assertEqual(self.app.clipboard().text(), 'id\nprintf "test"')
        with tempfile.TemporaryDirectory() as folder:
            target = str(Path(folder) / "report.json")
            with patch.object(
                qt_ui.QFileDialog, "getSaveFileName", return_value=(target, "")
            ):
                dialog.export.click()
            self.assertEqual(json.loads(Path(target).read_text())["ports"], [])
        dialog.accept()

    def test_environment_issue_not_repeated_on_automatic_scan(self):
        report = {
            "issues": [{"code": "permission_denied"}],
            "ports": [],
            "blocking": True,
        }
        with patch.object(self.window, "show_diagnostics") as show:
            for _ in range(2):
                self.window.emit("environment", report)
                self.window.drain()
            show.assert_called_once_with(report)

    def test_environment_refresh_updates_disconnected_to_connected_without_reopening(
        self,
    ):
        from test_updater_environment_status import environment

        self.window.show_diagnostics(environment())
        dialog = self.window.environment_dialog
        self.assertEqual(dialog.state["tone"], "warning")
        self.assertFalse(dialog.connection.isEnabled())
        self.window.emit("diagnostics_auto", environment("new-usb"))
        self.window.drain()
        self.assertIs(dialog, self.window.environment_dialog)
        self.assertTrue(dialog.connection.isEnabled())
        self.assertIn("尚需连接验证", dialog.result._i18n_bindings["setText"])
        self.window.selected_identity = ("TESTs", "new-usb")
        self.window.current_info = {"recorded": True}
        self.window.emit("diagnostics_auto", environment("new-usb"))
        self.window.drain()
        self.assertEqual(dialog.state["tone"], "success")
        self.window.emit("diagnostics_auto", environment())
        self.window.drain()
        self.assertIsNone(self.window.current_info)
        self.assertFalse(self.window.start.isEnabled())
        self.assertEqual(dialog.state["tone"], "warning")
        dialog.accept()

    def test_environment_auto_refresh_is_read_only_idle_only_and_stops_when_hidden(
        self,
    ):
        from test_updater_environment_status import environment

        self.window.show_diagnostics(environment())
        dialog = self.window.environment_dialog
        with patch.object(self.window, "diagnose") as diagnose:
            dialog.refresh()
            diagnose.assert_called_once_with(automatic=True)
            self.window.busy = True
            dialog.refresh()
            diagnose.assert_called_once()
            self.window.busy = False
            dialog.accept()
            dialog.refresh()
            diagnose.assert_called_once()
        self.assertFalse(dialog.refresh_timer.isActive())

    def test_environment_details_collapse_compacts_window_and_colors_results(self):
        from test_updater_environment_status import environment

        report = environment("usb")
        report["ports"][0]["read_write"] = False
        self.window.show_diagnostics(report)
        dialog = self.window.environment_dialog
        self.app.processEvents()
        self.assertEqual(dialog.result.property("tone"), "error")
        self.assertEqual(
            dialog.checks.topLevelItem(3).foreground(1).color().name(), "#9a2836"
        )
        compact = dialog.height()
        dialog.toggle.setChecked(True)
        self.app.processEvents()
        self.assertGreater(dialog.height(), compact)
        dialog.toggle.setChecked(False)
        self.app.processEvents()
        self.assertLessEqual(dialog.height(), compact + 2)
        dialog.accept()

    def test_silent_environment_refresh_does_not_discover_or_open_serial(self):
        from unittest.mock import Mock

        from test_updater_environment_status import environment

        device = Mock()
        with (
            patch("host_diagnostics.diagnostic_report", return_value=environment()),
            patch.dict(sys.modules, {"device": device}),
            patch.object(self.window, "launch", side_effect=lambda fn, **kw: fn()),
        ):
            self.window.diagnose(automatic=True)
        device.discover.assert_not_called()
        device.Device.assert_not_called()
        self.assertEqual(self.window.events.get_nowait()[0], "diagnostics_auto")

    def test_ownership_hint_is_neutral_and_clear_in_both_languages(self):
        import i18n
        from test_updater_environment_status import environment

        previous = i18n.language()
        try:
            for language in ("zh", "en"):
                i18n.set_language(language)
                report = environment("usb")
                report["ownership_scope"] = "best_effort_visible_processes_only"
                self.window.selected_identity = ("TESTs", "usb")
                self.window.current_info = {"model": "EL05", "recorded": True}
                self.window.show_diagnostics(report)
                dialog = self.window.environment_dialog
                self.app.processEvents()
                row = dialog.checks.topLevelItem(4)
                self.assertEqual(row.foreground(2).color().name(), "#626b78")
                self.assertIn(
                    "非故障" if language == "zh" else "not a fault", row.text(2)
                )
                self.assertEqual(row.toolTip(2), row.text(2))
                self.assertEqual(dialog.result.property("tone"), "success")
                detail = dialog.detail.toPlainText()
                self.assertIn(
                    "本身不阻止" if language == "zh" else "does not itself block",
                    detail,
                )
                self.assertIn(
                    "当前用户可见"
                    if language == "zh"
                    else "visible to the current user",
                    detail,
                )
                self.assertIn(
                    "再次验证" if language == "zh" else "verified again", detail
                )
                if language == "en":
                    self.assertNotIn("\\u", detail)
                dialog.accept()
        finally:
            i18n.set_language(previous)

    def test_user_tool_is_update_only_for_both_roles_and_languages(self):
        import i18n
        from PySide6.QtWidgets import QPushButton, QSlider

        previous = i18n.language()
        try:
            for language in ("zh", "en"):
                i18n.set_language(language)
                i18n.retranslate(self.window)
                for role in ("m", "s"):
                    with self.subTest(language=language, role=role):
                        self.window.selected_identity = (
                            f"TCGU01A25Z0017{role}",
                            "offline-scope",
                        )
                        self.window.current_info = {"model": "EL05", "recorded": True}
                        self.window.apply_defaults()
                        self.window.update_actions()
                        self.app.processEvents()
                        self.assertTrue(self.window.start.isEnabled())
                        self.assertTrue(self.window.refresh.isEnabled())
                        self.assertFalse(self.window.findChildren(QSlider))
                        forbidden = {
                            "功能测试",
                            "Function test",
                            "启用测试",
                            "Enable test",
                        }
                        self.assertFalse(
                            forbidden.intersection(
                                b.text() for b in self.window.findChildren(QPushButton)
                            )
                        )
                        self.assertFalse(hasattr(self.window, "open_control_test"))
                        for module in (
                            "control_test",
                            "control_test_ui",
                            "control_parameters",
                            "control_parameters_ui",
                        ):
                            self.assertIsNone(importlib.util.find_spec(module))
        finally:
            i18n.set_language(previous)

    def test_connected_information_does_not_overlap_firmware_rows(self):
        self.window.info.setText(
            "型号 EL05（已记录）   MCU 1.2.16\\n电机 1.0.5.0.4（Flash 存档，非实时）"
        )
        self.app.processEvents()
        info = self.window.info
        self.assertGreaterEqual(info.height(), info.sizeHint().height())
        self.assertLess(
            info.geometry().bottom(),
            self.window.file_labels["mcu"].geometry().top(),
        )
        self.assertTrue(
            self.window.windowTitle().startswith("Xense TacCap 固件更新 · v")
        )
        self.assertIn("build ", self.window.windowTitle())

    def test_version_colors_and_compact_layout(self):
        identity = ("TCGU01A25Z0017s", "test")
        self.window.selected_identity = identity
        self.window.emit(
            "info",
            (
                identity,
                dict(
                    model="EL05",
                    recorded=True,
                    mcu="1.2.14",
                    motor="1.0.5.0.4（Flash 存档，非实时）",
                ),
            ),
        )
        self.window.drain()
        self.app.processEvents()
        self.assertIn("#B42318", self.window.info.text())
        self.assertIn("#146448", self.window.info.text())
        self.assertIn("请更新至", self.window.info.text())
        self.assertIn("建议", self.window.status.text())
        self.assertLessEqual(self.window.width(), 700)
        self.assertLess(
            self.window.info.geometry().bottom(),
            self.window.file_labels["mcu"].geometry().top(),
        )

    def test_rs00_can_id_status_fits_without_overlap(self):
        identity = ("TCGU01A00Z0001s", "test")
        self.window.selected_identity = identity
        self.window.emit(
            "info",
            (
                identity,
                dict(
                    model="RS00",
                    recorded=True,
                    mcu="1.2.16",
                    motor="0.0.3.32（Flash 存档，非实时）",
                    can_id=127,
                    can_id_expected=17,
                    can_id_side="左",
                ),
            ),
        )
        self.window.drain()
        self.app.processEvents()
        self.assertIn("CAN ID 127", self.window.info.text())
        self.assertIn("确认修正", self.window.status.text())
        self.assertLessEqual(self.window.width(), 700)
        self.assertGreaterEqual(
            self.window.info.height(), self.window.info.sizeHint().height()
        )
        self.assertLess(
            self.window.info.geometry().bottom(),
            self.window.file_labels["mcu"].geometry().top(),
        )

    def test_dark_system_palette_is_overridden(self):
        from PySide6.QtGui import QColor, QPalette

        dark = QPalette()
        dark.setColor(QPalette.Window, QColor("#202020"))
        dark.setColor(QPalette.Text, QColor("#ffffff"))
        self.app.setPalette(dark)
        qt_ui.configure(self.app)
        self.assertEqual(self.app.palette().color(QPalette.Window).name(), "#ffffff")
        self.assertEqual(self.app.palette().color(QPalette.Text).name(), "#252a32")
        self.assertEqual(self.window.centralWidget().objectName(), "surface")

    def test_expand_logs_and_clear_image(self):
        self.window.toggle.setChecked(True)
        self.assertTrue(self.window.text.isVisible())
        self.window.paths["motor"] = "example.bin"
        self.window.clear("motor")
        self.assertEqual(self.window.paths["motor"], "")

    def test_missing_selection_does_not_launch(self):
        with (
            patch.object(qt_ui, "notice"),
            patch.object(self.window, "launch") as launch,
        ):
            self.window.run()
        launch.assert_not_called()

    def test_update_worker_owns_indicator_context_on_success_and_failure(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock, Mock

        identity = ("test-s", "usb")
        self.window.devices = [identity]
        self.window.combo.addItem("test-s")
        self.window.combo.setCurrentIndex(0)
        self.window.current_info = {"recorded": True}
        self.window.paths = {"mcu": "test.bin", "motor": ""}
        for error in (None, RuntimeError("original failure")):
            with self.subTest(error=error):
                device = MagicMock()
                context = device.update_indication.return_value
                context.__exit__.return_value = False
                with (
                    patch.dict(
                        sys.modules,
                        {"device": SimpleNamespace(Device=Mock(return_value=device))},
                    ),
                    patch.object(qt_ui.core, "load_image", return_value="image"),
                    patch.object(qt_ui.core, "execute", side_effect=error) as execute,
                    patch.object(
                        self.window, "launch", side_effect=lambda fn, **_: fn()
                    ),
                ):
                    if error:
                        with self.assertRaisesRegex(RuntimeError, "original failure"):
                            self.window.run()
                    else:
                        self.window.run()
                context.__enter__.assert_called_once()
                context.__exit__.assert_called_once()
                execute.assert_called_once_with(
                    device, "image", None, self.window.ask, self.window.emit
                )

    def test_power_popup_cannot_acknowledge_restart(self):
        self.window.emit("power", "等待真实断电")
        self.window.drain()
        self.window.popup.reject()
        self.assertTrue(self.window.popup.isVisible())
        self.window.emit("power_done", "")
        self.window.drain()
        self.assertIsNone(self.window.popup)

    def test_auto_scan_only_while_idle_and_without_devices(self):
        self.window.auto_connect = True
        with patch.object(self.window, "scan") as scan:
            self.window.auto_scan()
            scan.assert_called_once()
            self.window.busy = True
            self.window.auto_scan()
            self.window.busy = False
            self.window.devices = [("unit-s", "usb")]
            self.window.auto_scan()
            scan.assert_called_once()

    def test_multiple_devices_never_auto_select(self):
        with patch.object(qt_ui.QTimer, "singleShot") as later:
            self.window.emit("devices", [("one-s", "usb1"), ("two-s", "usb2")])
            self.window.drain()
            self.assertEqual(self.window.combo.currentIndex(), -1)
            later.assert_not_called()

    def test_single_device_schedules_connection_not_flash(self):
        with (
            patch.object(qt_ui.QTimer, "singleShot") as later,
            patch.object(self.window, "run") as run,
        ):
            self.window.emit("devices", [("one-s", "usb1")])
            self.window.drain()
            self.assertEqual(self.window.combo.currentIndex(), 0)
            later.assert_called_once()
            self.assertEqual(later.call_args.args[1], self.window.inspect)
            run.assert_not_called()

    def test_unrecorded_model_is_not_used_for_motor_default(self):
        identity = ("one-s", "usb1")
        self.window.selected_identity = identity
        self.window.emit(
            "info",
            (identity, dict(model="EL05", recorded=False, mcu="1.2.16", motor="未知")),
        )
        self.window.drain()
        self.assertTrue(self.window.model_row.isVisible())
        self.assertEqual(self.window.model_combo.currentIndex(), 0)
        self.assertEqual(self.window.paths["motor"], "")

    def test_manual_choice_and_clear_survive_default_refresh(self):
        self.window.paths["mcu"] = "/manual/old.bin"
        self.window.overrides.add("mcu")
        self.window.apply_defaults()
        self.assertEqual(self.window.paths["mcu"], "/manual/old.bin")
        self.window.clear("mcu")
        self.window.apply_defaults()
        self.assertEqual(self.window.paths["mcu"], "")
        self.window.use_defaults()
        self.assertTrue(self.window.paths["mcu"].endswith("1.2.16.bin"))

    def test_model_configuration_cancel_writes_nothing(self):
        self.window.current_info = {"recorded": False}
        self.window.selected_identity = ("one-s", "usb1")
        self.window.model_combo.setCurrentText("RS00")
        with (
            patch.object(qt_ui, "confirm", return_value=False),
            patch.object(self.window, "launch") as launch,
        ):
            self.window.set_model()
            launch.assert_not_called()

    def test_stage_switches_from_percentage_to_wait_animation(self):
        self.window.activity.begin("传输")
        self.window.emit("progress", {"done": 25, "total": 100, "unit": "B"})
        self.window.drain()
        self.assertEqual(self.window.progress.value(), 25)
        self.window.emit("stage", "等待重启")
        self.window.drain()
        self.assertEqual(self.window.progress.maximum(), 0)
        self.assertIn("等待重启", self.window.text.toPlainText())
        self.assertNotIn("25%", self.window.activity_detail.text())

    def test_live_worker_progress_reaches_ui_before_worker_finishes(self):
        import threading
        import time

        release = threading.Event()

        def work():
            self.window.emit("stage", "模拟传输")
            self.window.emit("progress", {"done": 40, "total": 100, "unit": "包"})
            release.wait(2)

        self.window.launch(work, updating=True)
        try:
            deadline = time.monotonic() + 1
            while self.window.progress.value() != 40 and time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.01)
            self.assertEqual(self.window.progress.value(), 40)
            self.assertTrue(self.window.busy)
            self.assertTrue(self.window.text.isVisible())
            self.assertIn("模拟传输", self.window.text.toPlainText())
        finally:
            release.set()
            deadline = time.monotonic() + 2
            while self.window.busy and time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.01)
        self.assertFalse(self.window.busy)
        self.assertFalse(self.window.activity.active)

    def test_power_status_and_semantic_color(self):
        self.window.emit("power", "请断开 24 V")
        self.window.drain()
        self.assertEqual(self.window.status.property("tone"), "warning")
        self.window.emit("power_status", "设备在线：uptime=8000 ms")
        self.window.drain()
        self.assertIn("8000", self.window.popup.connection_label.text())
        self.window.emit("power_done", "")
        self.window.drain()

    def test_repeated_log_toggle_recovers_compact_height(self):
        self.window.info.setText("型号 EL05（已记录）   MCU 1.2.16\\n电机 1.0.5.0.4")
        self.window.fit_contents()
        self.app.processEvents()
        compact = self.window.height()
        for _ in range(4):
            self.window.toggle.setChecked(True)
            self.app.processEvents()
            self.assertGreater(self.window.height(), compact)
            self.window.toggle.setChecked(False)
            self.app.processEvents()
            self.assertLessEqual(self.window.height(), compact + 2)
            self.assertLess(
                self.window.status.geometry().top()
                - self.window.safety.geometry().bottom(),
                20,
            )

    def test_master_defaults_and_usb_prompt(self):
        identity = ("TESTm", "usb")
        self.window.selected_identity = identity
        self.window.emit(
            "info",
            (identity, dict(model="主爪", recorded=True, mcu="1.2.6", motor="不适用")),
        )
        self.window.drain()
        self.app.processEvents()
        self.assertIn("master", self.window.paths["mcu"])
        self.assertEqual(self.window.paths["motor"], "")
        self.assertFalse(self.window.file_labels["motor"].isVisible())
        self.window.emit("power", {"role": "master", "reason": "更新完成"})
        self.window.drain()
        self.assertIn("USB", self.window.popup.windowTitle())

    def test_master_filter_without_device_never_shows_follower_firmware(self):
        with patch.object(self.window, "scan"):
            self.window.role_filter.setCurrentIndex(2)
        self.app.processEvents()
        self.assertIsNone(self.window.selected_identity)
        self.assertIn("master", self.window.paths["mcu"])
        self.assertEqual(self.window.paths["motor"], "")
        self.assertFalse(self.window.file_labels["motor"].isVisible())
        self.assertFalse(self.window.start.isEnabled())
        self.window.emit("devices", [])
        self.window.drain()
        self.app.processEvents()
        self.assertIn("master", self.window.paths["mcu"])
        self.assertFalse(self.window.file_labels["motor"].isVisible())
        self.assertNotEqual(self.window.status.property("tone"), "success")
        with patch.object(self.window, "scan"):
            self.window.role_filter.setCurrentIndex(1)
        self.app.processEvents()
        self.assertIn("slave", self.window.paths["mcu"])
        self.assertTrue(self.window.file_labels["motor"].isVisible())

    def test_default_refresh_does_not_duplicate_hash_logs(self):
        before = len(self.window.log_lines)
        self.window.apply_defaults()
        self.window.apply_defaults()
        self.assertEqual(len(self.window.log_lines), before)

    def test_matching_hint_alignment_and_font(self):
        from PySide6.QtCore import Qt

        hint = self.window.match_hint
        self.assertTrue(hint.alignment() & Qt.AlignRight)
        self.assertEqual(hint.font(), self.window.role_label.font())
        self.assertEqual(hint.font().family(), self.window.role_filter.font().family())
        self.assertEqual(
            hint.geometry().right(), self.window.refresh.geometry().right()
        )

    def test_busy_blocks_close(self):
        self.window.busy = True
        with patch.object(qt_ui, "notice"):
            self.window.close()
        self.assertTrue(self.window.isVisible())

    def wait_for_ui(self, condition, timeout=2):
        import time

        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.01)
        self.app.processEvents()

    def fake_devices(self, devices, info=None):
        from types import SimpleNamespace
        from unittest.mock import Mock

        return SimpleNamespace(
            discover=Mock(return_value=devices),
            Device=Mock(return_value=SimpleNamespace(inspect=Mock(return_value=info))),
        )

    def test_refresh_follows_single_follower_after_master_unplug(self):
        from test_updater_environment_status import environment

        old = ("TCGU01A00Z0001m", "old-usb")
        new = ("TCGU01A00Z0002s", "new-usb")
        info = dict(model="EL05", recorded=True, mcu="1.2.16", motor="未知")
        with patch.object(self.window, "scan"):
            self.window.role_filter.setCurrentIndex(2)
        self.window.devices = [old]
        self.window.combo.addItem("old master")
        self.window.selected_identity = old
        self.window.current_info = dict(model="主爪", recorded=True)
        with (
            patch.dict(sys.modules, {"device": self.fake_devices([new], info)}),
            patch(
                "host_diagnostics.diagnostic_report", return_value=environment(new[1])
            ),
        ):
            self.window.scan()
            self.wait_for_ui(lambda: self.window.current_info == info)
        self.assertEqual(self.window.selected_identity, new)
        self.assertEqual(self.window.role_filter.currentIndex(), 1)
        self.assertEqual(self.window.current_info, info)
        self.assertTrue(self.window.file_labels["motor"].isVisible())
        self.assertTrue(self.window.start.isEnabled())

    def test_refresh_queues_after_old_read_and_discards_its_error(self):
        import threading

        from test_updater_environment_status import environment

        old = ("TCGU01A00Z0001m", "old-usb")
        new = ("TCGU01A00Z0002s", "new-usb")
        info = dict(model="EL05", recorded=True, mcu="1.2.16", motor="未知")
        release = threading.Event()
        self.window.devices = [old]
        self.window.selected_identity = old
        self.window.current_info = dict(model="主爪", recorded=True)
        self.window.auto_connect = True
        self.window.scan_timer.start(5000)

        def old_read():
            release.wait(2)
            self.window.emit("connection_error", "old device disconnected")
            self.window.emit(
                "info",
                (old, dict(model="主爪", recorded=True, mcu="1.2.6", motor="不适用")),
            )

        with (
            patch.dict(sys.modules, {"device": self.fake_devices([new], info)}),
            patch(
                "host_diagnostics.diagnostic_report", return_value=environment(new[1])
            ),
        ):
            self.window.launch(old_read)
            try:
                self.assertTrue(self.window.refresh.isEnabled())
                self.window.scan()
                self.assertIsNone(self.window.current_info)
                self.assertIsNone(self.window.selected_identity)
                self.assertEqual(self.window.devices, [])
                self.assertFalse(self.window.start.isEnabled())
            finally:
                release.set()
                self.wait_for_ui(lambda: self.window.current_info == info)
        self.assertEqual(self.window.selected_identity, new)
        self.assertEqual(self.window.current_info, info)
        self.assertTrue(self.window.scan_timer.isActive())
        self.assertNotIn("old device disconnected", self.window.text.toPlainText())
        self.assertFalse(self.window.busy)

    def test_refresh_during_update_preserves_target_and_blocks_refresh(self):
        import threading

        identity = ("TCGU01A00Z0002s", "target-usb")
        info = dict(model="EL05", recorded=True)
        release = threading.Event()
        self.window.devices = [identity]
        self.window.selected_identity = identity
        self.window.current_info = info
        module = self.fake_devices([identity], info)
        with patch.dict(sys.modules, {"device": module}):
            self.window.launch(lambda: release.wait(2), updating=True)
            try:
                self.assertFalse(self.window.refresh.isEnabled())
                self.window.scan()
                self.assertEqual(self.window.selected_identity, identity)
                self.assertIs(self.window.current_info, info)
                module.discover.assert_not_called()
            finally:
                release.set()
                self.wait_for_ui(lambda: not self.window.busy)

    def test_failed_read_clears_cache_and_restores_auto_discovery(self):
        identity = ("TCGU01A00Z0001m", "old-usb")
        self.window.auto_connect = True
        self.window.scan_timer.stop()
        self.window.devices = [identity]
        self.window.selected_identity = identity
        self.window.current_info = dict(model="主爪", recorded=True)
        self.window.emit("connection_error", "device disappeared")
        self.window.drain()
        self.assertEqual(self.window.devices, [])
        self.assertIsNone(self.window.selected_identity)
        self.assertIsNone(self.window.current_info)
        self.assertTrue(self.window.scan_timer.isActive())
        self.assertFalse(self.window.start.isEnabled())
        with patch.object(self.window, "scan") as scan:
            self.window.auto_scan()
            scan.assert_called_once()


if __name__ == "__main__":
    unittest.main()

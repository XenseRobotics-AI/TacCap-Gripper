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
    from PySide6.QtWidgets import QApplication, QMessageBox


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
        self.assertEqual(self.window.windowTitle(), "Xense TacCap 固件更新")

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
            patch.object(QMessageBox, "information"),
            patch.object(self.window, "launch") as launch,
        ):
            self.window.run()
        launch.assert_not_called()

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
            patch.object(QMessageBox, "question", return_value=QMessageBox.No),
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

    def test_busy_blocks_close(self):
        self.window.busy = True
        with patch.object(QMessageBox, "warning"):
            self.window.close()
        self.assertTrue(self.window.isVisible())


if __name__ == "__main__":
    unittest.main()

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
        self.assertFalse(self.window.text.isVisible())
        self.assertEqual(self.window.combo.currentIndex(), -1)
        self.assertFalse(self.window.busy)
        self.assertGreaterEqual(self.window.start.height(), 34)
        self.assertLessEqual(self.window.height(), 460)
        self.assertLessEqual(self.window.width(), 700)

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

    def test_busy_blocks_close(self):
        self.window.busy = True
        with patch.object(QMessageBox, "warning"):
            self.window.close()
        self.assertTrue(self.window.isVisible())


if __name__ == "__main__":
    unittest.main()

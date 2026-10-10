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
        self.window = qt_ui.App()
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
        self.assertGreaterEqual(self.window.start.height(), 42)

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

    def test_busy_blocks_close(self):
        self.window.busy = True
        with patch.object(QMessageBox, "warning"):
            self.window.close()
        self.assertTrue(self.window.isVisible())


if __name__ == "__main__":
    unittest.main()

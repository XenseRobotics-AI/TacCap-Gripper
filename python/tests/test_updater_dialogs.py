"""Hardware-free dialog colors, labels and keyboard safety checks."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/firmware_updater"))
HAS_QT = importlib.util.find_spec("PySide6") is not None
if HAS_QT:
    import dialogs
    import qt_ui
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QPalette
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QMessageBox


@unittest.skipUnless(HAS_QT, "Qt checks require the packaging environment")
class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        qt_ui.configure(cls.app)

    def test_actions_have_distinct_rendered_colors_and_clear_labels(self):
        cases = (
            ("确认机械标定", "方向与标定正常", "success", "#15803d", "danger"),
            ("确认电机回滚", "确认回滚", "danger", "#b42318", "neutral"),
            ("确认电机重复刷写", "仍要重刷", "warning", "#fbbf24", "success"),
            ("确认夹爪 MCU升级", "确认继续", "warning", "#fbbf24", "neutral"),
            ("确认电机型号", "确认继续", "warning", "#fbbf24", "neutral"),
            ("确认修正 RS00 CAN ID", "确认继续", "warning", "#fbbf24", "neutral"),
            ("开始升级/回滚", "确认继续", "warning", "#fbbf24", "neutral"),
        )
        for title, text, tone, color, no_tone in cases:
            with self.subTest(title=title):
                box, yes = dialogs.confirmation_box(None, title, "请核对设备与操作。")
                try:
                    box.show()
                    self.app.processEvents()
                    no = box.defaultButton()
                    self.assertEqual(yes.text(), text)
                    self.assertEqual(yes.property("actionTone"), tone)
                    self.assertEqual(yes.palette().color(QPalette.Button).name(), color)
                    self.assertEqual(
                        yes.grab().toImage().pixelColor(8, 8).name(), color
                    )
                    self.assertEqual(no.property("actionTone"), no_tone)
                    self.assertEqual(box.buttonRole(no), QMessageBox.NoRole)
                    self.assertIs(box.escapeButton(), no)
                    self.assertFalse(yes.isDefault())
                    self.assertFalse(yes.geometry().intersects(no.geometry()))
                    self.assertGreaterEqual(yes.width(), yes.sizeHint().width())
                    for button in box.buttons():
                        colors = [
                            button.palette().color(role)
                            for role in (QPalette.Button, QPalette.ButtonText)
                        ]
                        luminance = []
                        for value in colors:
                            rgb = [value.redF(), value.greenF(), value.blueF()]
                            linear = [
                                c / 12.92
                                if c <= 0.04045
                                else ((c + 0.055) / 1.055) ** 2.4
                                for c in rgb
                            ]
                            luminance.append(
                                sum(
                                    a * b
                                    for a, b in zip(linear, (0.2126, 0.7152, 0.0722))
                                )
                            )
                        self.assertGreaterEqual(
                            (max(luminance) + 0.05) / (min(luminance) + 0.05), 4.5
                        )
                finally:
                    box.done(0)
                    box.deleteLater()

    def test_enter_escape_and_window_close_never_confirm_a_write(self):
        for action in ("enter", "escape", "close"):
            with self.subTest(action=action):
                box, yes = dialogs.confirmation_box(None, "确认电机回滚", "谨慎操作")
                with patch.object(dialogs, "confirmation_box", return_value=(box, yes)):
                    if action == "close":
                        QTimer.singleShot(0, box.close)
                    else:
                        key = Qt.Key_Return if action == "enter" else Qt.Key_Escape
                        QTimer.singleShot(0, lambda k=key: QTest.keyClick(box, k))
                    self.assertFalse(dialogs.confirm(None, "ignored", "ignored"))
                box.deleteLater()

    def test_explicit_affirmative_click_still_confirms(self):
        box, yes = dialogs.confirmation_box(None, "确认电机重复刷写", "谨慎操作")
        with patch.object(dialogs, "confirmation_box", return_value=(box, yes)):
            QTimer.singleShot(0, yes.click)
            self.assertTrue(dialogs.confirm(None, "ignored", "ignored"))
        box.deleteLater()

    def test_success_and_warning_notices_have_semantic_buttons(self):
        for success, icon, expected in (
            (True, QMessageBox.Information, "success"),
            (False, QMessageBox.Warning, "warning"),
            (False, QMessageBox.Critical, "neutral"),
        ):
            observed = []

            def check():
                box = self.app.activeModalWidget()
                button = box.defaultButton()
                observed.append((button.property("actionTone"), button.text()))
                button.click()

            QTimer.singleShot(0, check)
            dialogs.notice(None, "提示", "无需连接设备", icon, success=success)
            self.assertEqual(observed, [(expected, "完成" if success else "知道了")])

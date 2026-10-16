"""Restricted post-update slider test. Creating the dialog never opens hardware."""

import queue
import threading

from control_test import ControlSession
from dialogs import confirm
from i18n import set_text, set_title, tr
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
)


class ControlTestDialog(QDialog):
    def __init__(self, identity, parent=None):
        super().__init__(parent)
        self.identity = tuple(identity)
        self.events = queue.Queue()
        self.session = None
        self.worker = None
        self.result = None
        self.closing = False
        self.active = False
        set_title(self, "从爪功能测试")
        self.setMinimumWidth(520)
        box = QVBoxLayout(self)
        box.setContentsMargins(24, 20, 24, 20)
        box.setSpacing(12)
        box.addWidget(QLabel(self.identity[0]))
        self.warning = QLabel()
        set_text(
            self.warning,
            "真实运动：先空载测试，清空活动范围并确认实物型号与开合方向。软件停止不是急停；失联时请断开 24 V。",
        )
        self.warning.setWordWrap(True)
        self.warning.setObjectName("notice")
        box.addWidget(self.warning)
        hint = QLabel()
        set_text(hint, "SDK 力位控制 · 低速测试 · 力矩预算不超过 0.5 N·m · 最长 120 秒")
        hint.setWordWrap(True)
        box.addWidget(hint)
        self.target = QLabel()
        set_text(self.target, "目标开度：等待启用")
        box.addWidget(self.target)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.move_target)
        box.addWidget(self.slider)
        endpoints = QHBoxLayout()
        for text in ("0% 闭合", "100% 张开"):
            item = QLabel()
            set_text(item, text)
            endpoints.addWidget(item)
            if endpoints.count() == 1:
                endpoints.addStretch(1)
        box.addLayout(endpoints)
        self.feedback = QLabel("—")
        self.feedback.setWordWrap(True)
        box.addWidget(self.feedback)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setObjectName("status")
        set_text(self.status, "尚未启用；滑条不会发送命令")
        box.addWidget(self.status)
        buttons = QHBoxLayout()
        self.enable = QPushButton()
        set_text(self.enable, "启用测试")
        self.enable.setProperty("actionTone", "success")
        self.enable.clicked.connect(self.start_test)
        buttons.addWidget(self.enable)
        self.stop_button = QPushButton()
        set_text(self.stop_button, "停止并失能")
        self.stop_button.setProperty("actionTone", "danger")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_test)
        buttons.addWidget(self.stop_button)
        buttons.addStretch(1)
        self.close_button = QPushButton()
        set_text(self.close_button, "关闭")
        self.close_button.clicked.connect(self.reject)
        buttons.addWidget(self.close_button)
        for button in (self.enable, self.stop_button, self.close_button):
            button.setMinimumHeight(34)
            button.setAutoDefault(False)
        box.addLayout(buttons)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.drain)
        self.timer.start(50)

    def start_test(self):
        if self.worker is not None:
            return
        if not confirm(self, "确认启用测试", self.warning._i18n_bindings["setText"]):
            return
        self.enable.setEnabled(False)
        self.stop_button.setEnabled(True)
        set_text(self.status, "正在检查测试条件；尚未启用电机")
        self.session = ControlSession(self.identity, self.emit)
        self.worker = threading.Thread(target=self.session.run, daemon=False)
        self.worker.start()

    def emit(self, kind, value):
        self.events.put((kind, value))

    def move_target(self, value):
        if self.active:
            set_text(self.target, tr("目标开度：") + f"{value}%")
            self.session.set_target(value / 100)

    def stop_test(self):
        self.active = False
        self.slider.setEnabled(False)
        if self.session:
            self.session.stop()
        self.stop_button.setEnabled(False)
        set_text(self.status, "正在停止控制器并失能，请稍候")

    def drain(self):
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind in ("stage", "log"):
                set_text(self.status, value)
            elif kind in ("sample", "ready"):
                if kind == "ready" and not self.session.stopping.is_set():
                    self.slider.blockSignals(True)
                    self.slider.setValue(round(value["position"] * 100))
                    self.slider.blockSignals(False)
                    self.active = True
                    self.slider.setEnabled(True)
                    set_text(self.target, tr("目标开度：") + f"{self.slider.value()}%")
                    set_text(self.status, "测试已启用；拖动滑条控制开度")
                self.feedback.setText(
                    tr("实际开度：")
                    + f"{value['position']:.0%}    {value['torque']:+.2f} N·m    "
                    + f"{value['velocity']:+.2f} rad/s    {value['temperature']:.0f} °C"
                )
            elif kind == "finished":
                self.result = (value,)
                self.active = False
                self.slider.setEnabled(False)
                self.stop_button.setEnabled(False)
        # The completion event precedes Thread.run returning. Never reopen the
        # device or release the parent lock until the owning thread has exited.
        if self.result is not None and not self.worker.is_alive():
            self.worker.join()
            result = self.result[0]
            error = result["error"]
            message = error or (
                "测试已停止，电机失能已确认"
                if result["disabled"]
                else "测试未启用；未发送运动命令"
            )
            self.worker = None
            self.session = None
            self.result = None
            self.enable.setEnabled(not error)
            self.status.setProperty("tone", "error" if error else "success")
            self.status.style().unpolish(self.status)
            self.status.style().polish(self.status)
            set_text(self.status, message)
            parent = self.parent()
            if parent is not None:
                parent.log(message)
            if self.closing and not error:
                super().reject()
            else:
                self.closing = False  # Keep an unconfirmed shutdown visible.

    def reject(self):
        if self.worker is not None:
            self.closing = True
            self.stop_test()
        else:
            super().reject()

    def accept(self):
        self.reject()

    def closeEvent(self, event):
        if self.worker is not None:
            self.closing = True
            self.stop_test()
            event.ignore()
        else:
            event.accept()

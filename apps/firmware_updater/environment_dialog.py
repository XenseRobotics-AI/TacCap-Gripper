"""Modeless worker result dialog: the main event queue must keep draining."""

import json
import time
from pathlib import Path

from environment_guidance import commands
from environment_status import assess
from host_diagnostics import summary
from i18n import set_text, set_title, tr
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
)


class EnvironmentDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.owner = parent
        self.report = None
        set_title(self, "环境诊断")
        self.setModal(True)
        self.setMinimumWidth(680)
        box = QVBoxLayout(self)
        self.result = QLabel()
        self.result.setObjectName("status")
        self.result.setWordWrap(True)
        box.addWidget(self.result)
        self.legend = QLabel()
        set_text(self.legend, "绿色：通过　黄色：待处理　红色：阻断　灰色：参考信息")
        box.addWidget(self.legend)
        self.checks = QTreeWidget()
        self.checks.setRootIsDecorated(False)
        self.checks.setFocusPolicy(Qt.NoFocus)
        self.checks.setColumnCount(3)
        self.checks.setColumnWidth(0, 195)
        self.checks.setColumnWidth(1, 115)
        self.checks.setFixedHeight(222)
        box.addWidget(self.checks)
        self.next_step = QLabel()
        self.next_step.setWordWrap(True)
        box.addWidget(self.next_step)
        self.connection = QPushButton()
        set_text(self.connection, "返回并连接验证")
        self.connection.setObjectName("primary")
        self.connection.clicked.connect(self.connect_device)
        box.addWidget(self.connection)
        self.updated = QLabel()
        set_text(self.updated, "窗口打开时每 3 秒自动复检；只检查环境，不自动刷写。")
        self.updated.setWordWrap(True)
        self.updated.setObjectName("muted")
        box.addWidget(self.updated)
        self.toggle = QPushButton()
        self.toggle.setCheckable(True)
        set_text(self.toggle, "展开诊断详情")
        box.addWidget(self.toggle)
        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setFixedHeight(150)
        self.detail.hide()
        self.toggle.toggled.connect(self.toggle_details)
        box.addWidget(self.detail)
        row = QHBoxLayout()
        self.copy = QPushButton()
        self.retry = QPushButton()
        self.export = QPushButton()
        close = QPushButton()
        for button, text, tone in (
            (self.copy, "复制命令", "neutral"),
            (self.retry, "重新检测", "success"),
            (self.export, "导出报告", "neutral"),
            (close, "关闭", "neutral"),
        ):
            set_text(button, text)
            button.setProperty("actionTone", tone)
            row.addWidget(button)
        box.addLayout(row)
        self.copy.clicked.connect(self.copy_commands)
        self.retry.clicked.connect(parent.diagnose)
        self.export.clicked.connect(self.export_report)
        close.clicked.connect(self.accept)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(3000)
        self.refresh_timer.timeout.connect(self.refresh)

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_timer.start()

    def hideEvent(self, event):
        self.refresh_timer.stop()
        super().hideEvent(event)

    def refresh(self):
        if self.isVisible() and not self.owner.busy and not self.owner.updating:
            self.owner.diagnose(automatic=True)

    def toggle_details(self, visible):
        self.detail.setVisible(visible)
        set_text(self.toggle, "收起诊断详情" if visible else "展开诊断详情")
        self.adjustSize()

    def connect_device(self):
        if self.owner.busy or self.owner.updating:
            return
        if not self.state["system_ready"]:
            return
        self.accept()
        if not self.state["connection_verified"]:
            self.owner.scan()

    def set_busy(self, busy):
        self.retry.setEnabled(not busy)
        self.connection.setEnabled(
            not busy
            and bool(
                self.report
                and assess(
                    self.report, self.owner.selected_identity, self.owner.current_info
                )["system_ready"]
            )
        )

    def update_report(self, report):
        self.report = report
        self.checked_at = time.strftime("%H:%M:%S")
        set_text(
            self.updated,
            f"最近环境检查：{self.checked_at} · 空闲时每 3 秒复检，不自动刷写。",
        )
        state = assess(report, self.owner.selected_identity, self.owner.current_info)
        self.state = state
        set_text(
            self.connection,
            "返回主界面" if state["connection_verified"] else "返回并连接验证",
        )
        set_text(self.result, state["title"])
        self.result.setProperty("tone", state["tone"])
        self.result.style().unpolish(self.result)
        self.result.style().polish(self.result)
        self.checks.setHeaderLabels([tr("检查项"), tr("结果"), tr("说明")])
        self.checks.clear()
        colors = {
            "success": "#146448",
            "warning": "#815000",
            "error": "#9A2836",
            "neutral": "#626B78",
        }
        names = {
            "success": "通过",
            "warning": "待处理",
            "error": "阻断",
            "neutral": "参考",
        }
        for check in state["checks"]:
            item = QTreeWidgetItem(
                [tr(check["name"]), tr(names[check["tone"]]), tr(check["value"])]
            )
            item.setToolTip(0, tr(check["name"]))
            for column in (1, 2):
                item.setForeground(column, QColor(colors[check["tone"]]))
                item.setToolTip(column, tr(check["value"]))
            item.setFlags(item.flags() & ~Qt.ItemIsSelectable)
            self.checks.addTopLevelItem(item)
        set_text(self.next_step, state["next_step"])
        # Translate prose separately: executable commands must remain byte-exact.
        self.detail.setPlainText(tr(summary(report)))
        self.copy.setEnabled(bool(commands(report)))
        self.set_busy(self.owner.busy)
        if not self.isMaximized():
            self.adjustSize()

    def copy_commands(self):
        QApplication.clipboard().setText(commands(self.report))

    def export_report(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            tr("导出报告"),
            "taccap-environment.json",
            "JSON (*.json)",
            options=QFileDialog.DontUseNativeDialog,
        )
        if path:
            try:
                # Raw evidence is lossless, with a localized readable summary.
                data = dict(
                    self.report,
                    summary=tr(summary(self.report)),
                    readiness=self.state,
                    checked_at=self.checked_at,
                )
                Path(path).write_text(
                    json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
            except OSError as exc:
                from dialogs import notice

                notice(self, "导出失败", str(exc))

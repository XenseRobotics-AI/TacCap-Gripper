"""Modeless worker result dialog: the main event queue must keep draining."""

import json
from pathlib import Path

from environment_guidance import commands
from host_diagnostics import summary
from i18n import set_text, set_title, tr
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
)


class EnvironmentDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.owner = parent
        self.report = None
        set_title(self, "环境诊断")
        self.setModal(True)
        self.resize(680, 440)
        box = QVBoxLayout(self)
        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
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

    def update_report(self, report):
        self.report = report
        # Translate prose separately: executable commands must remain byte-exact.
        self.detail.setPlainText(tr(summary(report)))
        self.copy.setEnabled(bool(commands(report)))

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
                data = dict(self.report, summary=tr(summary(self.report)))
                Path(path).write_text(
                    json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
            except OSError as exc:
                from dialogs import notice

                notice(self, "导出失败", str(exc))

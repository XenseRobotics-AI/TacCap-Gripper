"""Compact updater form with explicit, theme-independent surface colors."""

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLayout,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

STYLE = """
QWidget { color: #252A32; font-size: 13px; }
QWidget#surface, QDialog, QMessageBox { background: #FFFFFF; }
QLabel { background: transparent; }
QLabel#title { font-size: 18px; font-weight: 600; }
QLabel#muted { color: #626B78; }
QLabel#notice { color: #626B78; padding: 0; background: transparent; }
QLabel#heading { font-size: 15px; font-weight: 600; }
QPushButton { background: #FFFFFF; border: 1px solid #D7DBE1; border-radius: 5px; padding: 5px 12px; }
QPushButton:hover { background: #F3F5F8; border-color: #AAB2BF; }
QPushButton:pressed { background: #E8ECF2; }
QPushButton:disabled { color: #9CA3AD; background: #F5F6F8; }
QPushButton#primary { background: #2563EB; color: #FFFFFF; border: 1px solid #2563EB; font-weight: 600; }
QPushButton#primary:hover { background: #1D4ED8; }
QPushButton#primary:disabled { background: #A6BAE5; border-color: #A6BAE5; }
QPushButton#link { border: none; background: transparent; color: #626B78; padding: 4px 0; }
QPushButton#link:hover { color: #2563EB; }
QComboBox { background: #FAFBFC; color: #252A32; border: 1px solid #D7DBE1; border-radius: 5px; padding: 6px 10px; }
QComboBox QAbstractItemView { background: white; color: #252A32; selection-background-color: #E8EFFD; selection-color: #252A32; }
QLabel#file { background: #FAFBFC; border: 1px solid #E1E5EB; border-radius: 5px; padding: 6px 10px; color: #626B78; }
QProgressBar { background: #EDF0F4; color: #172B46; border: none; border-radius: 2px; text-align: center; }
QProgressBar::chunk { background: #A9C5FF; border-radius: 2px; }
QPlainTextEdit { background: #FAFBFC; color: #252A32; border: 1px solid #E1E5EB; border-radius: 5px; padding: 6px; }
QToolTip { background: #FFFFFF; color: #252A32; border: 1px solid #D7DBE1; }
"""


def palette():
    result = QPalette()
    for role, color in (
        (QPalette.Window, "#FFFFFF"),
        (QPalette.WindowText, "#252A32"),
        (QPalette.Base, "#FFFFFF"),
        (QPalette.AlternateBase, "#FAFBFC"),
        (QPalette.Text, "#252A32"),
        (QPalette.Button, "#FFFFFF"),
        (QPalette.ButtonText, "#252A32"),
        (QPalette.Highlight, "#2563EB"),
        (QPalette.HighlightedText, "#FFFFFF"),
        (QPalette.ToolTipBase, "#FFFFFF"),
        (QPalette.ToolTipText, "#252A32"),
        (QPalette.PlaceholderText, "#626B78"),
    ):
        result.setColor(role, QColor(color))
    return result


def text(value, name="muted"):
    widget = QLabel(value)
    widget.setObjectName(name)
    widget.setWordWrap(True)
    return widget


def build(window):
    window.setWindowTitle("Xense TacCap 固件更新")
    window.resize(680, 430)
    window.setMinimumWidth(640)
    surface = QWidget()
    surface.setObjectName("surface")
    window.setCentralWidget(surface)
    box = QVBoxLayout(surface)
    box.setSizeConstraint(QLayout.SetMinimumSize)
    box.setContentsMargins(24, 20, 24, 16)
    box.setSpacing(8)
    header = QHBoxLayout()
    header.addWidget(text("固件更新", "title"))
    header.addStretch()
    header.addWidget(text("Xense TacCap"))
    box.addLayout(header)
    window.controls = []
    form = QGridLayout()
    form.setHorizontalSpacing(10)
    form.setVerticalSpacing(10)
    form.setColumnStretch(1, 1)
    form.addWidget(text("设备", "label"), 0, 0)
    window.combo = QComboBox()
    window.combo.setMinimumHeight(34)
    window.combo.setPlaceholderText("选择已连接的从爪")
    window.combo.activated.connect(window.inspect)
    form.addWidget(window.combo, 0, 1, 1, 2)
    window.refresh = window.button("刷新", window.scan)
    form.addWidget(window.refresh, 0, 3)
    window.controls.append(window.combo)
    window.info = text("正在自动扫描，请关闭其他控制程序。")
    window.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
    window.info.setWordWrap(False)
    window.info.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
    window.info.ensurePolished()
    window.info.setMinimumHeight(window.info.fontMetrics().lineSpacing() * 2 + 8)
    form.addWidget(window.info, 1, 1, 1, 3)
    window.file_labels = {}
    for row, kind, title in ((2, "mcu", "夹爪固件"), (3, "motor", "电机固件")):
        form.addWidget(text(title, "label"), row, 0)
        field = text("未选择", "file")
        field.setMinimumHeight(34)
        form.addWidget(field, row, 1)
        window.file_labels[kind] = field
        form.addWidget(
            window.button("选择文件", lambda checked=False, k=kind: window.choose(k)),
            row,
            2,
        )
        clear = window.button("清除", lambda checked=False, k=kind: window.clear(k))
        clear.setObjectName("link")
        form.addWidget(clear, row, 3)
    box.addLayout(form)
    window.model_row = QWidget()
    model_box = QHBoxLayout(window.model_row)
    model_box.setContentsMargins(0, 0, 0, 0)
    model_box.addWidget(text("型号未记录，请确认实物："))
    window.model_combo = QComboBox()
    window.model_combo.addItems(["选择电机型号", "EL05", "RS00"])
    model_box.addWidget(window.model_combo, 1)
    window.model_save = window.button("设置型号", window.set_model)
    model_box.addWidget(window.model_save)
    window.controls.append(window.model_combo)
    window.model_row.hide()
    box.addWidget(window.model_row)
    options = QHBoxLayout()
    options.addWidget(text("默认使用内置固件；可手动选择其他版本。"), 1)
    defaults = window.button("恢复默认", window.use_defaults)
    defaults.setObjectName("link")
    options.addWidget(defaults)
    box.addLayout(options)
    line = QFrame()
    line.setFixedHeight(1)
    line.setStyleSheet("background: #E8EBEF;")
    box.addWidget(line)
    box.addWidget(text("请清空夹爪活动范围，仅在提示时断开 24 V；刷写中勿拔线。"))
    box.addStretch(1)
    window.status = text("等待选择设备和固件")
    box.addWidget(window.status)
    window.progress = QProgressBar()
    window.progress.setRange(0, 100)
    window.progress.setValue(0)
    window.progress.setTextVisible(True)
    window.progress.setFormat("%p%")
    window.progress.setFixedHeight(18)
    box.addWidget(window.progress)
    window.activity_detail = text("百分比仅表示当前固件传输；完成后还需重启和校验。")
    box.addWidget(window.activity_detail)
    footer = QHBoxLayout()
    window.toggle = QPushButton("查看日志")
    window.toggle.setObjectName("link")
    window.toggle.setCheckable(True)
    window.toggle.toggled.connect(window.toggle_log)
    footer.addWidget(window.toggle)
    export = QPushButton("导出日志")
    export.setObjectName("link")
    export.clicked.connect(window.export)
    footer.addWidget(export)
    footer.addStretch(1)
    window.start = window.button("开始更新", window.run)
    window.start.setObjectName("primary")
    window.start.setMinimumWidth(110)
    footer.addWidget(window.start)
    box.addLayout(footer)
    window.text = QPlainTextEdit()
    window.text.setReadOnly(True)
    window.text.setFixedHeight(160)
    window.text.hide()
    box.addWidget(window.text)

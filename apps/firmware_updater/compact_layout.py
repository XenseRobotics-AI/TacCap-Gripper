"""Compact updater form with explicit, theme-independent surface colors."""

from build_info import window_title
from i18n import (
    add_items,
    language,
    set_format,
    set_placeholder,
    set_text,
    set_title,
    set_tooltip,
)
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
QWidget#surface { background: #F5F8FC; }
QDialog QPushButton[actionTone="success"]:enabled { background: #15803D; color: #FFFFFF; border-color: #15803D; }
QDialog QPushButton[actionTone="danger"]:enabled { background: #B42318; color: #FFFFFF; border-color: #B42318; }
QSlider::groove:horizontal { height: 6px; background: #DCE5F2; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #2563EB; border-radius: 3px; }
QSlider::handle:horizontal { background: #FFFFFF; border: 2px solid #2563EB; width: 16px; margin: -6px 0; border-radius: 8px; }
QSlider:disabled { background: #EDF0F4; }
QDialog, QMessageBox { background: #FFFFFF; }
QLabel { background: transparent; }
QLabel#title { font-size: 18px; font-weight: 600; color: #163C65; }
QLabel#brand { color: #236C87; font-weight: 600; }
QLabel#status { border-radius: 5px; padding: 5px 8px; color: #214E85; background: #E5EEFC; }
QLabel#status[tone="success"] { color: #146448; background: #DDF3E9; }
QLabel#status[tone="warning"] { color: #815000; background: #FFF0CE; }
QLabel#status[tone="error"] { color: #9A2836; background: #FCE4E8; }
QLabel#muted { color: #626B78; }
QLabel#notice { color: #815000; padding: 4px 6px; background: #FFF0CE; border-radius: 4px; }
QLabel#rollback { color: #B42318; padding: 4px 6px; background: #FCE4E8; border-radius: 4px; }
QMenu { background: #FFFFFF; border: 1px solid #D7DBE1; padding: 4px; }
QMenu::item { padding: 7px 14px; }
QMenu::item:selected { background: #E8EFFD; }
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
QMessageBox QPushButton[actionTone="neutral"]:enabled { background: #F3F4F6; color: #374151; border-color: #CBD5E1; }
QMessageBox QPushButton[actionTone="neutral"]:hover { background: #E5E7EB; }
QMessageBox QPushButton[actionTone="success"]:enabled { background: #15803D; color: #FFFFFF; border-color: #15803D; font-weight: 600; }
QMessageBox QPushButton[actionTone="success"]:hover { background: #166534; border-color: #166534; }
QMessageBox QPushButton[actionTone="success"]:pressed { background: #14532D; }
QMessageBox QPushButton[actionTone="warning"]:enabled { background: #FBBF24; color: #422006; border-color: #D99A0B; font-weight: 600; }
QMessageBox QPushButton[actionTone="warning"]:hover { background: #F59E0B; border-color: #B45309; }
QMessageBox QPushButton[actionTone="warning"]:pressed { background: #D99A0B; }
QMessageBox QPushButton[actionTone="danger"]:enabled { background: #B42318; color: #FFFFFF; border-color: #B42318; font-weight: 600; }
QMessageBox QPushButton[actionTone="danger"]:hover { background: #912018; border-color: #912018; }
QMessageBox QPushButton[actionTone="danger"]:pressed { background: #7A1B16; }
QMessageBox QPushButton[actionTone]:focus { border: 2px solid #172B46; padding: 4px 11px; }
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
    widget = QLabel()
    set_text(widget, value)
    widget.setObjectName(name)
    widget.setWordWrap(True)
    return widget


def build(window):
    set_title(window, window_title())
    window.resize(680, 430)
    window.setMinimumWidth(680)
    surface = QWidget()
    surface.setObjectName("surface")
    window.setCentralWidget(surface)
    box = QVBoxLayout(surface)
    box.setSizeConstraint(QLayout.SetMinimumSize)
    box.setContentsMargins(24, 12, 24, 12)
    box.setSpacing(4)
    box.setAlignment(Qt.AlignTop)
    window.role_filter = QComboBox()
    add_items(window.role_filter, ["全部夹爪", "从爪", "主爪"])
    window.role_filter.setMinimumHeight(36)
    window.role_filter.setMaximumWidth(160)
    window.role_filter.currentIndexChanged.connect(window.change_role)
    window.controls = [window.role_filter]
    form = QGridLayout()
    form.setHorizontalSpacing(10)
    form.setVerticalSpacing(9)
    form.setColumnMinimumWidth(0, 72)
    form.setColumnMinimumWidth(2, 90)
    form.setColumnMinimumWidth(3, 44)
    form.setColumnStretch(1, 1)
    window.role_label = text("设备类型", "label")
    form.addWidget(window.role_label, 0, 0)
    type_controls = QHBoxLayout()
    type_controls.setSpacing(8)
    type_controls.addWidget(window.role_filter)
    type_controls.addWidget(text("语言", "label"))
    form.addLayout(type_controls, 0, 1)
    window.match_hint = text("自动匹配固件", "label")
    window.match_hint.setWordWrap(False)
    window.match_hint.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    form.addWidget(window.match_hint, 0, 2, 1, 2)
    window.language_combo = QComboBox()
    add_items(window.language_combo, ["简体中文", "English"])
    window.language_combo.setItemData(0, "zh")
    window.language_combo.setItemData(1, "en")
    window.language_combo.setCurrentIndex(1 if language() == "en" else 0)
    window.language_combo.currentIndexChanged.connect(window.change_language)
    window.controls.append(window.language_combo)
    type_controls.addWidget(window.language_combo)
    type_controls.addStretch(1)
    form.addWidget(text("连接设备", "label"), 1, 0)
    window.combo = QComboBox()
    window.combo.setMinimumHeight(36)
    set_placeholder(window.combo, "选择已连接的夹爪")
    window.combo.activated.connect(window.inspect)
    form.addWidget(window.combo, 1, 1, 1, 2)
    window.refresh = window.button("刷新", window.scan)
    window.refresh.setFixedHeight(36)
    form.addWidget(window.refresh, 1, 3)
    window.controls.append(window.combo)
    window.info = text("正在自动扫描，请关闭其他控制程序。")
    window.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
    window.info.setWordWrap(False)
    window.info.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
    window.info.ensurePolished()
    window.info.setMinimumHeight(window.info.fontMetrics().lineSpacing() * 2 + 8)
    form.addWidget(window.info, 2, 1, 1, 3)
    window.file_labels = {}
    window.motor_widgets = []
    for row, kind, title in ((3, "mcu", "夹爪固件"), (4, "motor", "电机固件")):
        title_label = text(title, "label")
        form.addWidget(title_label, row, 0)
        field = text("未选择", "file")
        field.setFixedHeight(36)
        form.addWidget(field, row, 1)
        window.file_labels[kind] = field
        choose = window.button(
            "选择版本" if kind == "mcu" else "选择文件",
            window.open_mcu_menu if kind == "mcu" else lambda: window.choose("motor"),
        )
        choose.setFixedHeight(36)
        form.addWidget(choose, row, 2)
        if kind == "mcu":
            window.mcu_choose = choose
        clear = window.button("清除", lambda checked=False, k=kind: window.clear(k))
        clear.setObjectName("link")
        form.addWidget(clear, row, 3)
        if kind == "motor":
            window.motor_widgets = [title_label, field, choose, clear]
    box.addLayout(form)
    window.model_row = QWidget()
    model_box = QHBoxLayout(window.model_row)
    model_box.setContentsMargins(0, 0, 0, 0)
    model_box.addWidget(text("型号未记录，请确认实物："))
    window.model_combo = QComboBox()
    add_items(window.model_combo, ["选择电机型号", "EL05", "RS00"])
    model_box.addWidget(window.model_combo, 1)
    window.model_save = window.button("设置型号", window.set_model)
    model_box.addWidget(window.model_save)
    window.controls.append(window.model_combo)
    window.model_row.hide()
    box.addWidget(window.model_row)
    options = QHBoxLayout()
    options.addWidget(text("默认使用内置固件；可手动选择其他版本。"), 1)
    window.diagnostics = window.button("环境诊断", window.diagnose)
    window.diagnostics.setObjectName("link")
    options.addWidget(window.diagnostics)
    defaults = window.button("恢复默认", window.use_defaults)
    defaults.setObjectName("link")
    options.addWidget(defaults)
    box.addLayout(options)
    window.mcu_hint = text("", "rollback")
    window.mcu_hint.hide()
    box.addWidget(window.mcu_hint)
    line = QFrame()
    line.setFixedHeight(1)
    line.setStyleSheet("background: #E8EBEF;")
    box.addWidget(line)
    window.safety = text(
        "请清空活动范围；仅在提示时断开 24 V，刷写中勿拔线。", "notice"
    )
    box.addWidget(window.safety)
    window.status = text("等待选择设备和固件", "status")
    box.addWidget(window.status)
    window.progress = QProgressBar()
    window.progress.setRange(0, 100)
    window.progress.setValue(0)
    window.progress.setTextVisible(True)
    set_format(window.progress, "%p%")
    window.progress.setFixedHeight(18)
    box.addWidget(window.progress)
    window.activity_detail = text("百分比仅表示当前固件传输；完成后还需重启和校验。")
    box.addWidget(window.activity_detail)
    footer = QHBoxLayout()
    footer.setSpacing(16)
    window.toggle = QPushButton()
    set_text(window.toggle, "查看日志")
    window.toggle.setObjectName("link")
    window.toggle.setCheckable(True)
    window.toggle.toggled.connect(window.toggle_log)
    footer.addWidget(window.toggle)
    export = QPushButton()
    set_text(export, "导出日志")
    export.setObjectName("link")
    export.clicked.connect(window.export)
    footer.addWidget(export)
    window.clear_logs = QPushButton()
    set_text(window.clear_logs, "清空日志")
    window.clear_logs.setObjectName("link")
    set_tooltip(window.clear_logs, "仅清空窗口显示；导出日志仍包含完整记录")
    window.clear_logs.clicked.connect(window.clear_log_view)
    footer.addWidget(window.clear_logs)
    footer.addStretch(1)
    window.test_button = window.button("功能测试", window.open_control_test)
    footer.addWidget(window.test_button)
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

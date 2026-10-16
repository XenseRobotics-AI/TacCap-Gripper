"""Mode-specific SDK inputs, editable only between serial-owning sessions."""

from control_parameters import FIELDS
from i18n import add_items, set_text, set_tooltip, tr
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


class CurrentPageStack(QStackedWidget):
    def sizeHint(self):
        return (
            self.currentWidget().sizeHint()
            if self.currentWidget()
            else super().sizeHint()
        )

    def minimumSizeHint(self):
        return (
            self.currentWidget().minimumSizeHint()
            if self.currentWidget()
            else super().minimumSizeHint()
        )


class ParameterPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.defaults = None
        self.limits = None
        self.inputs = {}
        box = QVBoxLayout(self)
        box.setContentsMargins(0, 0, 0, 0)
        box.setSpacing(6)
        row = QHBoxLayout()
        self.mode = QComboBox()
        add_items(self.mode, ["力位控制", "阻抗控制"])
        self.mode.setItemData(0, "force_position")
        self.mode.setItemData(1, "impedance")
        row.addWidget(self.mode, 1)
        self.restore = QPushButton()
        set_text(self.restore, "恢复保守参数")
        self.restore.clicked.connect(self.reset)
        row.addWidget(self.restore)
        box.addLayout(row)
        self.detail = QLabel()
        self.detail.setWordWrap(True)
        box.addWidget(self.detail)
        self.stack = CurrentPageStack()
        self.advanced = []
        for mode, fields in FIELDS.items():
            page = QWidget()
            grid = QGridLayout(page)
            grid.setContentsMargins(0, 0, 0, 0)
            self.inputs[mode] = {}
            advanced = []
            for row_index, field in enumerate(fields):
                label = QLabel()
                set_text(label, field.label)
                spin = QSpinBox() if field.integer else QDoubleSpinBox()
                if not field.integer:
                    spin.setDecimals(3)
                    spin.setSingleStep(
                        0.05 if "torque" in field.name or "speed" in field.name else 0.1
                    )
                spin.setRange(field.minimum, field.maximum)
                spin.setSuffix(" " + field.unit)
                spin.setKeyboardTracking(False)
                spin.setMinimumWidth(160)
                set_tooltip(spin, field.name)
                spin.valueChanged.connect(self.refresh_hint)
                grid.addWidget(label, row_index, 0)
                grid.addWidget(spin, row_index, 1)
                self.inputs[mode][field.name] = spin
                if field.advanced:
                    advanced.extend((label, spin))
            self.advanced.append(advanced)
            self.stack.addWidget(page)
        box.addWidget(self.stack)
        self.more = QPushButton()
        set_text(self.more, "高级参数")
        self.more.setCheckable(True)
        self.more.toggled.connect(self.show_advanced)
        box.addWidget(self.more)
        self.speed_hint = QLabel()
        self.speed_hint.setWordWrap(True)
        box.addWidget(self.speed_hint)
        self.mode.currentIndexChanged.connect(self.change_mode)
        self.show_advanced(False)
        self.lock(True)
        self.refresh_hint()

    def selected_mode(self):
        return self.mode.currentData()

    def values(self):
        return {
            name: spin.value()
            for name, spin in self.inputs[self.selected_mode()].items()
        }

    def lock(self, locked):
        for control in (self.mode, self.restore, self.more, self.stack):
            control.setEnabled(not locked and self.defaults is not None)

    def load(self, settings):
        self.defaults = settings["defaults"]
        self.limits = settings["limits"]
        for mode in FIELDS:
            for key, value in self.defaults[mode].items():
                self.inputs[mode][key].setValue(value)
        self.refresh_hint()

    def reset(self):
        if self.defaults is not None:
            for key, value in self.defaults[self.selected_mode()].items():
                self.inputs[self.selected_mode()][key].setValue(value)

    def show_advanced(self, show):
        for group in self.advanced:
            for widget in group:
                widget.setVisible(show)
        if self.window().layout():
            self.window().adjustSize()

    def change_mode(self, index):
        self.stack.setCurrentIndex(index)
        self.refresh_hint()
        self.window().adjustSize()

    def refresh_hint(self):
        mode = self.selected_mode()
        set_text(
            self.detail,
            "参数仅用于本次 SDK 会话，不写入设备；最长测试 120 秒。"
            if mode == "force_position"
            else "阻抗控制没有独立速度输入；接近速度约为位置预算 / Kd，不是速度保证。",
        )
        if self.limits is None:
            set_text(self.speed_hint, "请先读取设备参数，再修改并启用测试。")
        else:
            limit = min(self.limits["continuous"], self.limits["envelope_cont"])
            hint = tr("当前持续预算上限：") + f"{limit:.3f} N·m"
            if mode == "impedance":
                values = self.values()
                hint += (
                    " · "
                    + tr("估算接近速度：")
                    + f"{values['max_position_torque_nm'] / values['kd']:.3f} rad/s"
                )
            self.speed_hint.setText(hint)

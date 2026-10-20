"""Semantic dialog actions; colors never replace labels or safe defaults."""

from i18n import tr
from PySide6.QtWidgets import QMessageBox


def action_button(box, text, role, tone):
    text = tr(text)
    button = box.addButton(text, role)
    button.setProperty("actionTone", tone)
    button.setMinimumHeight(34)
    button.setAutoDefault(False)
    # addButton may polish before the semantic property has been assigned.
    button.style().unpolish(button)
    button.style().polish(button)
    button.setMinimumWidth(max(88, button.fontMetrics().horizontalAdvance(text) + 32))
    return button


def confirmation_box(parent, title, text):
    same = "重复刷写" in title
    calibration = title == "确认机械标定"
    downgrade = "回滚" in title and title != "开始升级/回滚"
    tone = "success" if calibration else "danger" if downgrade else "warning"
    icon = QMessageBox.Question if calibration else QMessageBox.Warning
    box = QMessageBox(icon, tr(title), tr(text), parent=parent)
    yes_text = (
        "方向与标定正常"
        if calibration
        else "仍要重刷"
        if same
        else "确认回滚"
        if downgrade
        else "确认继续"
    )
    no_text = "标定异常，停止" if calibration else "跳过此项" if same else "取消"
    yes = action_button(box, yes_text, QMessageBox.YesRole, tone)
    no = action_button(
        box,
        no_text,
        QMessageBox.NoRole,
        "danger" if calibration else "success" if same else "neutral",
    )
    box.setDefaultButton(no)
    box.setEscapeButton(no)
    return box, yes


def confirm(parent, title, text):
    box, yes = confirmation_box(parent, title, text)
    box.exec()
    return box.clickedButton() == yes


def notice(parent, title, text, icon=QMessageBox.Information, *, success=False):
    box = QMessageBox(icon, tr(title), tr(text), parent=parent)
    tone = (
        "success"
        if success
        else "warning"
        if icon == QMessageBox.Warning
        else "neutral"
    )
    button = action_button(
        box, "完成" if success else "知道了", QMessageBox.AcceptRole, tone
    )
    box.setDefaultButton(button)
    box.setEscapeButton(button)
    box.exec()

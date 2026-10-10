"""Restricted Qt front end. Hardware work stays off the GUI thread."""

import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

import core
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

HERE = Path(__file__).resolve().parent
STYLE = """
QWidget { color: #172B46; font-size: 14px; }
QMainWindow, QDialog { background: #F3F6FA; }
QFrame#card { background: white; border: 1px solid #DFE6EF; border-radius: 12px; }
QLabel#title { font-size: 28px; font-weight: 700; }
QLabel#heading { font-size: 17px; font-weight: 600; }
QLabel#muted { color: #52657D; }
QLabel#eyebrow { color: #2169D5; font-size: 12px; font-weight: 600; }
QLabel#notice { background: #E8F0FC; color: #285387; padding: 12px; border-radius: 8px; }
QPushButton { background: white; border: 1px solid #CBD5E1; border-radius: 7px; padding: 9px 17px; }
QPushButton:hover { background: #EDF3FD; border-color: #7CA5E0; }
QPushButton:disabled { color: #8794A5; background: #EFF2F6; }
QPushButton#primary { background: #2169D5; border: 0; color: white; font-weight: 600; padding: 12px 24px; }
QPushButton#primary:hover { background: #1756B5; }
QPushButton#primary:disabled { background: #A5BBD9; }
QComboBox { background: white; border: 1px solid #CBD5E1; border-radius: 7px; padding: 9px; }
QComboBox QAbstractItemView { background: white; selection-background-color: #DBEAFE; selection-color: #172B46; }
QProgressBar { background: #E8EDF4; border: none; border-radius: 4px; height: 8px; }
QProgressBar::chunk { background: #2169D5; border-radius: 4px; }
QPlainTextEdit { background: #F8FAFC; border: 1px solid #DFE6EF; border-radius: 8px; padding: 8px; }
QMessageBox { background: #F3F6FA; }
"""


def configure(app):
    app.setStyle("Fusion")
    font_file = HERE / "fonts/NotoSansCJK-Regular.ttc"
    if not font_file.is_file():
        font_file = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    font_id = QFontDatabase.addApplicationFont(str(font_file))
    families = QFontDatabase.applicationFontFamilies(font_id)
    if font_id < 0:
        raise RuntimeError("中文字体加载失败，请重新安装升级工具")
    family = next((f for f in families if f.endswith(" SC")), families[0])
    app.setFont(QFont(family, 11))
    app.setStyleSheet(STYLE)
    return family


def label(text, name=None):
    item = QLabel(text)
    item.setWordWrap(True)
    if name:
        item.setObjectName(name)
    return item


class PowerDialog(QDialog):
    def reject(self):
        # Closing an instruction must not masquerade as a physical power cycle.
        pass


class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.events = queue.Queue()
        self.busy = False
        self.popup = None
        self.devices = []
        self.paths = {"mcu": "", "motor": ""}
        self.log_lines = []
        self.catalog = core.catalog_at(HERE / "catalog.json")
        self.setWindowTitle("TacCap 固件升级与回滚")
        self.resize(920, 790)
        self.setMinimumSize(800, 720)
        body = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(body)
        self.setCentralWidget(scroll)
        layout = QVBoxLayout(body)
        layout.setContentsMargins(30, 24, 30, 24)
        layout.setSpacing(14)
        layout.addWidget(label("TACCAP  /  DEVICE CARE", "eyebrow"))
        layout.addWidget(label("让夹爪更新，就这么简单", "title"))
        layout.addWidget(label("选择设备和固件，工具会引导你完成升级或回滚。", "muted"))
        self.steps = label(
            "① 选择夹爪    →    ② 选择固件    →    ③ 确认与更新    →    ④ 完成",
            "notice",
        )
        layout.addWidget(self.steps)
        self.controls = []
        device = self.card(layout)
        device.addWidget(label("01   连接的夹爪", "heading"))
        row = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.setPlaceholderText("点击刷新，查找已连接的从爪")
        self.combo.activated.connect(self.inspect)
        row.addWidget(self.combo, 1)
        self.refresh = self.button("刷新设备", self.scan)
        row.addWidget(self.refresh)
        device.addLayout(row)
        self.info = label("连接 USB 和 24 V 电源后刷新。请关闭其他控制程序。", "muted")
        self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        device.addWidget(self.info)
        self.controls.append(self.combo)
        files = self.card(layout)
        files.addWidget(label("02   选择要更新的固件", "heading"))
        files.addWidget(
            label("可只更新其中一项。相同版本重刷和旧版本回滚都会再次确认。", "muted")
        )
        self.file_labels = {}
        for kind, title in (("mcu", "夹爪 MCU"), ("motor", "电机")):
            row = QHBoxLayout()
            row.addWidget(label(title))
            field = label("未选择 · 保持当前固件", "muted")
            row.addWidget(field, 1)
            self.file_labels[kind] = field
            row.addWidget(
                self.button("选择 .bin", lambda checked=False, k=kind: self.choose(k))
            )
            row.addWidget(
                self.button("清除", lambda checked=False, k=kind: self.clear(k))
            )
            files.addLayout(row)
        layout.addWidget(
            label(
                "安全提示：先放下物体并清空夹爪活动范围。仅在提示时断开 24 V；刷写中请勿拔线。",
                "notice",
            )
        )
        row = QHBoxLayout()
        self.status = label("准备就绪 · 请先选择设备和固件", "muted")
        row.addWidget(self.status, 1)
        self.start = self.button("检查并开始更新", self.run)
        self.start.setObjectName("primary")
        row.addWidget(self.start)
        layout.addLayout(row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        layout.addWidget(self.progress)
        row = QHBoxLayout()
        self.toggle = QPushButton("展开详细日志")
        self.toggle.setCheckable(True)
        self.toggle.toggled.connect(self.toggle_log)
        row.addWidget(self.toggle)
        row.addStretch()
        export = QPushButton("导出日志")
        export.clicked.connect(self.export)
        row.addWidget(export)
        layout.addLayout(row)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMinimumHeight(140)
        self.text.hide()
        layout.addWidget(self.text)
        layout.addStretch()
        layout.addWidget(
            label("仅支持从爪 · 不修改型号、方向或力矩参数 · 回滚不恢复旧配置", "muted")
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.drain)
        self.timer.start(100)

    def card(self, parent):
        frame = QFrame()
        frame.setObjectName("card")
        box = QVBoxLayout(frame)
        box.setContentsMargins(18, 16, 18, 16)
        box.setSpacing(12)
        parent.addWidget(frame)
        return box

    def button(self, text, callback):
        button = QPushButton(text)
        button.setMinimumHeight(42)
        button.clicked.connect(callback)
        self.controls.append(button)
        return button

    def toggle_log(self, visible):
        self.text.setVisible(visible)
        self.toggle.setText("收起详细日志" if visible else "展开详细日志")

    def log(self, value):
        line = time.strftime("%Y-%m-%d %H:%M:%S ") + str(value)
        self.log_lines.append(line)
        self.text.appendPlainText(line)

    def emit(self, kind, value):
        self.events.put((kind, value))

    def ask(self, title, text):
        reply = queue.Queue()
        self.emit("ask", (title, text, reply))
        return reply.get()

    def launch(self, fn):
        if self.busy:
            return
        self.busy = True
        for control in self.controls:
            control.setEnabled(False)

        def work():
            try:
                fn()
            except Exception as exc:
                self.emit(
                    "error",
                    f"{type(exc).__name__}: {exc}\n\n已停止，不会自动重试或回滚。设备可能已部分更新或仍处于 Private。请导出日志联系支持。串口权限不足请配置 dialout 后重新登录，勿用 sudo 启动。",
                )
            finally:
                self.emit("idle", "")

        threading.Thread(target=work, daemon=False).start()

    def scan(self):
        def work():
            from device import discover

            self.emit("devices", discover())

        self.launch(work)

    def inspect(self, index=None):
        index = self.combo.currentIndex()
        if index < 0:
            return
        identity = self.devices[index]

        def work():
            from device import Device

            info = Device(identity, self.emit).inspect()
            self.emit(
                "info",
                f"型号  {info['model']}（{'已记录' if info['recorded'] else '未配置'}）    MCU  {info['mcu']}\n电机版本  {info['motor']}",
            )

        self.launch(work)

    def clear(self, kind):
        self.paths[kind] = ""
        self.file_labels[kind].setText("未选择 · 保持当前固件")
        self.file_labels[kind].setToolTip("")

    def choose(self, kind):
        path, _ = QFileDialog.getOpenFileName(self, "选择固件", "", "固件 (*.bin)")
        if not path:
            return
        try:
            image = core.load_image(path, kind, self.catalog)
            self.paths[kind] = path
            self.file_labels[kind].setText(
                f"{image.model}  ·  {image.version}\n{Path(path).name}"
            )
            self.file_labels[kind].setToolTip(path)
            self.log(
                f"选择 {kind}: {image.model} {image.version} SHA256={image.sha256}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "文件校验失败", str(exc))

    def run(self):
        index = self.combo.currentIndex()
        if index < 0 or not any(self.paths.values()):
            QMessageBox.information(
                self, "还差一步", "请先选择夹爪，并至少选择一个固件文件。"
            )
            return
        identity, paths = self.devices[index], self.paths.copy()
        self.progress.setValue(0)

        def work():
            from device import Device

            images = {
                k: core.load_image(p, k, self.catalog) if p else None
                for k, p in paths.items()
            }
            core.execute(
                Device(identity, self.emit),
                images["mcu"],
                images["motor"],
                self.ask,
                self.emit,
            )

        self.launch(work)

    def show_power(self, value):
        self.popup = PowerDialog(self)
        self.popup.setWindowTitle("请按步骤断开 24 V")
        self.popup.setMinimumWidth(580)
        box = QVBoxLayout(self.popup)
        box.setContentsMargins(28, 24, 28, 24)
        box.setSpacing(18)
        box.addWidget(label("需要你操作电源", "title"))
        box.addWidget(
            label(
                "①  拔掉 24 V 电源\n\n②  等待至少 2 秒\n\n③  重新插回 24 V 电源",
                "heading",
            )
        )
        box.addWidget(label("USB 保持连接。仅拔 USB 不算断电。", "notice"))
        box.addWidget(label(value, "muted"))
        box.addWidget(label("正在自动检测重启，确认后将继续，无需点击按钮。", "muted"))
        self.popup.setModal(True)
        self.popup.show()

    def drain(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                return
            if kind == "progress":
                self.progress.setValue(int(value))
            elif kind == "ask":
                title, text, reply = value
                self.log(title + ": " + text)
                box = QMessageBox(QMessageBox.Question, title, text, parent=self)
                yes = box.addButton("确认继续", QMessageBox.YesRole)
                no = box.addButton("取消", QMessageBox.NoRole)
                box.setDefaultButton(no)
                box.exec()
                answer = box.clickedButton() == yes
                self.log("用户确认: " + str(answer))
                reply.put(answer)
            elif kind == "power":
                self.status.setText("等待断开 24 V 并重新上电…")
                self.log(value)
                self.show_power(value)
            elif kind == "power_done":
                if self.popup:
                    self.popup.accept()
                    self.popup = None
                self.status.setText("正在校验设备…")
            elif kind == "devices":
                self.devices = value
                self.combo.clear()
                self.combo.addItems([f"{sn}  ·  USB {usb}" for sn, usb in value])
                self.combo.setCurrentIndex(-1)
                self.info.setText(
                    "请选择设备查看版本"
                    if value
                    else "未发现从爪，请检查 USB、24 V 和串口权限。"
                )
                if len(value) == 1:
                    self.combo.setCurrentIndex(0)
                    QTimer.singleShot(150, self.inspect)
            elif kind == "info":
                self.info.setText(value)
                self.log(value)
            elif kind == "idle":
                self.busy = False
                for control in self.controls:
                    control.setEnabled(True)
            else:
                self.log(value)
                if kind == "stage":
                    self.status.setText(value)
                elif kind == "error":
                    self.status.setText("操作已停止 · 请查看详细日志")
                    self.toggle.setChecked(True)
                    QMessageBox.critical(self, "需要处理", value)
                elif kind == "success":
                    self.status.setText("更新完成 · 所选固件已核验")
                    self.progress.setValue(100)
                    QMessageBox.information(self, "更新完成", value)

    def export(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出日志", "taccap-update.log", "日志 (*.log)"
        )
        if path:
            try:
                Path(path).write_text(
                    "\n".join(self.log_lines) + "\n", encoding="utf-8"
                )
            except OSError as exc:
                QMessageBox.critical(self, "导出失败", str(exc))

    def closeEvent(self, event):
        if self.busy:
            event.ignore()
            QMessageBox.warning(
                self, "操作尚未结束", "请等待完成或超时，刷写中请勿拔线。"
            )
        else:
            event.accept()


def main(args):
    if os.geteuid() == 0:
        raise SystemExit("请以普通用户运行，禁止 sudo 启动升级界面。")
    app = QApplication(sys.argv[:1])
    family = configure(app)
    window = App()
    window.show()
    if args.ui_smoke_test:

        def smoke():
            assert not window.text.isVisible()
            assert window.combo.currentIndex() == -1
            assert family.startswith("Noto Sans CJK")
            if args.screenshot:
                if not window.grab().save(args.screenshot):
                    raise RuntimeError("截图保存失败")
            print(json.dumps({"ui": "ok", "font": family}))
            app.quit()

        QTimer.singleShot(400, smoke)
    app.exec()

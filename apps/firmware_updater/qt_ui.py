"""Restricted Qt front end. Hardware work stays off the GUI thread."""

import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

import bundled
import core
from compact_layout import STYLE, build, palette
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

HERE = Path(__file__).resolve().parent


def configure(app):
    app.setStyle("Fusion")
    app.setPalette(palette())
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
    def __init__(self, auto_connect=True):
        super().__init__()
        self.events = queue.Queue()
        self.busy = False
        self.popup = None
        self.devices = []
        self.paths = {"mcu": "", "motor": ""}
        self.overrides = set()
        self.current_info = None
        self.selected_identity = None
        self.auto_connect = auto_connect
        self.log_lines = []
        self.catalog = core.catalog_at(HERE / "catalog.json")
        build(self)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.drain)
        self.timer.start(100)
        self.use_defaults()
        self.scan_timer = QTimer(self)
        self.scan_timer.timeout.connect(self.auto_scan)
        if auto_connect:
            self.scan_timer.start(5000)
            QTimer.singleShot(0, self.auto_scan)

    def auto_scan(self):
        if self.auto_connect and not self.busy and not self.devices:
            self.scan()

    def use_defaults(self):
        self.overrides.clear()
        self.apply_defaults()

    def apply_defaults(self):
        for kind in ("mcu", "motor"):
            if kind in self.overrides:
                continue
            model = (
                "slave"
                if kind == "mcu"
                else (
                    self.current_info["model"]
                    if self.current_info and self.current_info["recorded"]
                    else None
                )
            )
            selected = bundled.default_image(kind, model, self.catalog)
            if selected:
                path, image = selected
                self.set_image(kind, path, image, builtin=True)
            else:
                self.paths[kind] = ""
                self.file_labels[kind].setText("等待识别型号")
                self.file_labels[kind].setToolTip("型号未确认，不自动选择电机固件")

    def set_image(self, kind, path, image, builtin=False):
        self.paths[kind] = str(path)
        source = "内置" if builtin else "手动"
        self.file_labels[kind].setText(f"{image.model} · {image.version}（{source}）")
        self.file_labels[kind].setToolTip(str(path))
        self.log(
            f"{source} {kind}: {image.model} {image.version} SHA256={image.sha256}"
        )

    def button(self, text, callback):
        button = QPushButton(text)
        button.setMinimumHeight(34)
        button.clicked.connect(callback)
        self.controls.append(button)
        return button

    def toggle_log(self, visible):
        self.text.setVisible(visible)
        self.toggle.setText("收起日志" if visible else "查看日志")
        self.resize(self.width(), 602 if visible else 430)

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

    def launch(self, fn, updating=False):
        if self.busy:
            return
        self.busy = True
        for control in self.controls:
            control.setEnabled(False)

        def work():
            try:
                fn()
            except Exception as exc:
                if updating:
                    self.emit(
                        "error",
                        f"{type(exc).__name__}: {exc}\n\n已停止，不会自动重试或回滚。设备可能已部分更新或仍处于 Private。请导出日志联系支持。",
                    )
                else:
                    self.emit("connection_error", f"{type(exc).__name__}: {exc}")
            finally:
                self.emit("idle", "")

        threading.Thread(target=work, daemon=False).start()

    def scan(self):
        if self.busy:
            return
        self.status.setText("正在扫描设备…")

        def work():
            from device import discover

            self.emit("devices", discover())

        self.launch(work)

    def inspect(self, index=None):
        index = self.combo.currentIndex()
        if index < 0:
            return
        identity = self.devices[index]
        if identity != self.selected_identity:
            self.selected_identity = identity
            self.current_info = None
            self.overrides.clear()
            self.apply_defaults()
        self.status.setText("正在连接并读取设备信息…")

        def work():
            from device import Device

            info = Device(identity, self.emit).inspect()
            self.emit("info", (identity, info))

        self.launch(work)

    def set_model(self):
        if self.busy or not self.current_info or self.current_info["recorded"]:
            return
        name = self.model_combo.currentText()
        if name not in ("EL05", "RS00"):
            QMessageBox.information(
                self,
                "请选择实物型号",
                "请根据电机标签确认 EL05 或 RS00，不要按固件默认值猜测。",
            )
            return
        identity = self.selected_identity
        answer = QMessageBox.question(
            self,
            "确认电机型号",
            f"设备 {identity[0]}\n实物电机确认为 {name} 吗？\n\n选错型号会导致力矩/速度解释错误。请放下物体并清空活动范围。写入后必须按提示断开 24 V。已有行程配置不代表重新标定正确。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return

        def work():
            from device import Device

            info = Device(identity, self.emit).record_model(name)
            self.emit("info", (identity, info))
            self.emit("log", f"电机型号已写入并经重启核验：{name}")

        self.launch(work, updating=True)

    def clear(self, kind):
        self.overrides.add(kind)
        self.paths[kind] = ""
        self.file_labels[kind].setText("未选择")
        self.file_labels[kind].setToolTip("")

    def choose(self, kind):
        path, _ = QFileDialog.getOpenFileName(self, "选择固件", "", "固件 (*.bin)")
        if not path:
            return
        try:
            image = core.load_image(path, kind, self.catalog)
            self.overrides.add(kind)
            self.set_image(kind, path, image)
        except Exception as exc:
            QMessageBox.critical(self, "文件校验失败", str(exc))

    def run(self):
        index = self.combo.currentIndex()
        if index < 0 or not any(self.paths.values()):
            QMessageBox.information(
                self, "还差一步", "请先选择夹爪，并至少选择一个固件文件。"
            )
            return
        if not self.current_info or not self.current_info["recorded"]:
            QMessageBox.information(
                self, "尚未就绪", "请等待连接；型号未记录时，先确认实物并设置型号。"
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

        self.launch(work, updating=True)

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
                previous = self.selected_identity
                self.devices = value
                self.combo.clear()
                self.combo.addItems([f"{sn}  ·  USB {usb}" for sn, usb in value])
                self.combo.setCurrentIndex(-1)
                self.current_info = None
                self.model_row.hide()
                self.info.setText(
                    "请选择设备查看版本"
                    if value
                    else "未发现从爪，等待连接；请关闭其他控制程序。"
                )
                self.status.setText(
                    "请选择目标设备" if value else "等待设备 · 自动扫描中"
                )
                target = (
                    value.index(previous)
                    if previous in value
                    else (0 if len(value) == 1 else -1)
                )
                if target >= 0:
                    self.combo.setCurrentIndex(target)
                    QTimer.singleShot(150, self.inspect)
                else:
                    self.selected_identity = None
                    self.overrides.clear()
                    self.apply_defaults()
            elif kind == "info":
                identity, info = value
                if identity != self.selected_identity:
                    continue
                self.current_info = info
                self.model_row.setVisible(not info["recorded"])
                self.model_combo.setCurrentIndex(0)
                text = f"型号 {info['model']}（{'已记录' if info['recorded'] else '未配置'}）  MCU {info['mcu']}\n电机 {info['motor']}"
                self.info.setText(text)
                self.log(text)
                self.apply_defaults()
                self.status.setText("已连接 · 核对固件后点击开始更新")
            elif kind == "connection_error":
                self.scan_timer.stop()
                self.model_row.hide()
                self.current_info = None
                if "motor" not in self.overrides:
                    self.paths["motor"] = ""
                    self.file_labels["motor"].setText("读取设备失败")
                self.info.setText(
                    "连接失败，请检查 USB、24 V、串口权限或设备占用，然后点击刷新。"
                )
                self.status.setText("未连接 · 详情见日志")
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
    window = App(auto_connect=not args.ui_smoke_test)
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

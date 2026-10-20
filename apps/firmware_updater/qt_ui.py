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
from activity import Activity
from compact_layout import STYLE, build, palette
from desktop_integration import desktop_id, integrate_appimage
from dialogs import confirm, notice
from fontconfig_runtime import configure_fontconfig
from i18n import (
    language,
    retranslate,
    set_format,
    set_language,
    set_text,
    set_title,
    set_tooltip,
    tr,
)
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont, QFontDatabase, QIcon
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
)
from version_status import device_summary

HERE = Path(__file__).resolve().parent


def configure(app):
    app.setApplicationName(desktop_id())
    app.setDesktopFileName(desktop_id())
    app.setWindowIcon(QIcon(str(HERE / "icon.png")))
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
    app.setStyleSheet(
        f'QWidget {{ font-family: "{family}"; font-weight: 400; }}\n' + STYLE
    )
    return family


def label(text, name=None):
    item = QLabel()
    set_text(item, text)
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
        self.updating = False
        self.task_serial = 0
        self.active_task = None
        self.discard_read = False
        self.pending_scan = None
        self.previous_identity = None
        self.emit_context = threading.local()
        self.activity = Activity()
        self.popup = None
        self.devices = []
        self.paths = {"mcu": "", "motor": ""}
        self.mcu_target = None
        self.overrides = set()
        self.current_info = None
        self.selected_identity = None
        self.auto_connect = auto_connect
        self.log_lines = []
        self.visible_log_lines = []
        self.catalog = core.catalog_at(HERE / "catalog.json")
        build(self)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.drain)
        self.timer.start(100)
        self.activity_timer = QTimer(self)
        self.activity_timer.timeout.connect(self.refresh_activity)
        self.activity_timer.start(250)
        self.use_defaults()
        self.scan_timer = QTimer(self)
        self.scan_timer.timeout.connect(self.auto_scan)
        if auto_connect:
            self.scan_timer.start(5000)
            QTimer.singleShot(0, self.auto_scan)

    def change_language(self, *_):
        code = self.language_combo.currentData()
        if self.busy:
            self.language_combo.blockSignals(True)
            self.language_combo.setCurrentIndex(1 if language() == "en" else 0)
            self.language_combo.blockSignals(False)
            return
        set_language(code, persist=True)
        retranslate(self)
        self.text.setPlainText("\n".join(tr(line) for line in self.visible_log_lines))
        self.fit_contents()

    def auto_scan(self):
        if self.auto_connect and not self.busy and not self.devices:
            self.scan(follow_single=False)

    def effective_role(self):
        if self.selected_identity:
            return core.role_from_sn(self.selected_identity[0])
        return "master" if self.role_filter.currentIndex() == 2 else "slave"

    def change_role(self, *_):
        if self.busy:
            return
        self.selected_identity = None
        self.current_info = None
        self.devices = []
        self.combo.clear()
        self.model_row.hide()
        self.overrides.clear()
        self.apply_defaults()
        self.set_tone("running")
        set_text(self.info, "正在查找匹配设备…")
        self.fit_contents()
        self.scan(follow_single=False)

    def update_actions(self):
        self.start.setEnabled(
            not self.busy
            and self.selected_identity is not None
            and bool(self.current_info and self.current_info["recorded"])
            and any(self.paths.values())
        )
        self.refresh_mcu_hint()

    def use_defaults(self):
        self.overrides.clear()
        self.apply_defaults()

    def apply_defaults(self):
        role = self.effective_role()
        master = role == "master"
        for widget in self.motor_widgets:
            widget.setVisible(not master)
        for kind in ("mcu", "motor"):
            if master and kind == "motor":
                self.paths[kind] = ""
                set_text(self.file_labels[kind], "主爪不适用")
                continue
            if kind in self.overrides:
                continue
            model = (
                role
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
                set_text(self.file_labels[kind], "等待识别型号")
                set_tooltip(self.file_labels[kind], "型号未确认，不自动选择电机固件")

        set_text(
            self.safety,
            "主爪：刷写时勿拔 USB，完成后按提示拔插 USB。"
            if master
            else "请清空活动范围；仅在提示时断开 24 V，刷写中勿拔线。",
        )
        self.update_actions()
        self.fit_contents()

    def set_image(self, kind, path, image, builtin=False):
        changed = self.paths[kind] != str(path)
        self.paths[kind] = str(path)
        source = "内置" if builtin else "手动"
        model_name = {"master": "主爪 MCU", "slave": "从爪 MCU"}.get(
            image.model, image.model
        )
        set_text(self.file_labels[kind], f"{model_name} · {image.version}（{source}）")
        set_tooltip(self.file_labels[kind], str(path))
        if kind == "mcu":
            self.mcu_target = image
        if changed:
            self.log(
                f"{source} {kind}: {image.model} {image.version} SHA256={image.sha256}"
            )
        self.update_actions()

    def refresh_mcu_hint(self):
        current = self.current_info.get("mcu") if self.current_info else None
        target = self.mcu_target
        try:
            downgrade = bool(
                current
                and target
                and self.paths["mcu"]
                and target.model == self.effective_role()
                and core.version_key(target.version) < core.version_key(current)
            )
        except (ValueError, TypeError):
            downgrade = (
                False  # An unknown current version never proves downgrade safety.
            )
        self.mcu_hint.setVisible(downgrade)
        if downgrade:
            set_text(
                self.mcu_hint,
                f"MCU 降级：{current} → {target.version}。只更换固件，不恢复旧配置；刷写前需再次确认。",
            )
        self.fit_contents()

    def make_mcu_menu(self):
        menu = QMenu(self)
        role = self.effective_role()
        entries = bundled.mcu_entries(self.catalog, role, packaged_only=True)
        for entry in entries:
            action = menu.addAction(f"{entry['version']} ({tr('内置最新')})")
            action.setData(entry["version"])
            if entry["warning"]:
                action.setToolTip(tr(entry["warning"]))
            action.triggered.connect(
                lambda _=False, e=entry: self.select_mcu_version(e)
            )
        menu.setToolTipsVisible(True)
        menu.addSeparator()
        action = menu.addAction(tr("选择 .bin 文件…"))
        action.setData("file")
        action.triggered.connect(lambda: self.choose("mcu"))
        return menu

    def open_mcu_menu(self):
        if self.busy:
            return
        menu = self.make_mcu_menu()
        try:
            menu.exec(self.mcu_choose.mapToGlobal(self.mcu_choose.rect().bottomLeft()))
        finally:
            menu.deleteLater()

    def select_mcu_version(self, entry):
        if self.busy:
            return
        try:
            if entry["model"] != self.effective_role():
                raise ValueError("镜像主/从角色与当前设备不匹配")
            try:
                path, image = bundled.mcu_image(entry, self.catalog)
            except FileNotFoundError:
                # Historical approvals are metadata, not bundled images.
                # The selected local bytes must match this exact approval.
                self.choose("mcu", expected=entry)
                return
            self.overrides.add("mcu")
            self.set_image("mcu", path, image, builtin=True)
        except Exception as exc:
            notice(self, "文件校验失败", str(exc), QMessageBox.Critical)

    def refresh_activity(self):
        if not self.activity.active:
            return
        detail = self.activity.detail()
        set_text(self.activity_detail, detail)
        if self.popup and hasattr(self.popup, "detail_label"):
            set_text(self.popup.detail_label, detail)
        line = self.activity.heartbeat()
        if line:
            self.log(line)

    def set_tone(self, tone):
        self.status.setProperty("tone", tone)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)

    def set_stage(self, title):
        self.set_tone("running")
        self.activity.set_stage(title)
        set_text(self.status, title)
        self.progress.setRange(0, 0)
        self.log(title)
        self.refresh_activity()

    def button(self, text, callback):
        button = QPushButton()
        set_text(button, text)
        button.setMinimumHeight(34)
        button.clicked.connect(callback)
        self.controls.append(button)
        return button

    def clear_log_view(self):
        self.visible_log_lines.clear()
        self.text.clear()

    def toggle_log(self, visible):
        self.text.setVisible(visible)
        set_text(self.toggle, "收起日志" if visible else "查看日志")
        self.fit_contents()
        QTimer.singleShot(0, self.fit_contents)

    def fit_contents(self):
        layout = self.centralWidget().layout()
        layout.invalidate()
        layout.activate()
        self.layout().invalidate()
        self.layout().activate()
        if not self.isMaximized():
            self.resize(self.width(), self.sizeHint().height())

    def log(self, value):
        line = time.strftime("%Y-%m-%d %H:%M:%S ") + str(value)
        self.log_lines.append(line)
        self.visible_log_lines.append(line)
        self.text.appendPlainText(tr(line))

    def emit(self, kind, value):
        self.events.put((kind, value, getattr(self.emit_context, "task", None)))

    def ask(self, title, text):
        reply = queue.Queue()
        self.emit("ask", (title, text, reply))
        return reply.get()

    def launch(self, fn, updating=False):
        if self.busy:
            return
        self.busy = True
        self.updating = updating
        self.task_serial += 1
        task = self.active_task = self.task_serial
        self.discard_read = False
        self.activity.begin(
            "准备更新，请稍候" if updating else self.status._i18n_bindings["setText"]
        )
        self.progress.setRange(0, 0)
        set_text(self.status, self.activity.stage)
        self.log(self.activity.stage)
        if updating:
            self.toggle.setChecked(True)
        self.refresh_activity()
        for control in self.controls:
            control.setEnabled(False)
        # Read-only refresh can be queued; a firmware/model write keeps it locked.
        self.refresh.setEnabled(not updating)

        def work():
            self.emit_context.task = task
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

    def scan(self, *_, follow_single=True):
        if self.updating:
            return
        if self.selected_identity is not None:
            self.previous_identity = self.selected_identity
        self.selected_identity = None
        self.current_info = None
        self.devices = []
        self.combo.clear()
        self.model_row.hide()
        set_text(self.info, "重新扫描设备，请稍候…")
        self.update_actions()
        if self.auto_connect:
            self.scan_timer.start(5000)
        if self.busy:
            # Finish/close the old handle before opening another; drop its results.
            self.discard_read = True
            self.pending_scan = follow_single
            set_text(self.status, "等待旧读取结束，随后重新扫描设备…")
            return
        self.set_tone("running")
        set_text(self.status, "正在扫描设备…")

        role = self.role_filter.currentIndex()

        def work():
            from device import discover

            devices = discover()
            if role:
                matched = [
                    e
                    for e in devices
                    if core.role_from_sn(e[0]) == ("slave" if role == 1 else "master")
                ]
                if not matched and follow_single and len(devices) == 1:
                    self.emit("detected_role", core.role_from_sn(devices[0][0]))
                else:
                    devices = matched
            self.emit("devices", devices)

        self.launch(work)

    def inspect(self, index=None):
        if self.busy:
            return
        index = self.combo.currentIndex()
        if index < 0 or index >= len(self.devices):
            return
        identity = self.devices[index]
        if identity != self.selected_identity:
            self.selected_identity = identity
            self.current_info = None
            if identity != self.previous_identity:
                self.overrides.clear()
            self.previous_identity = None
            self.apply_defaults()
        set_text(self.status, "正在连接并读取设备信息…")

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
            notice(
                self,
                "请选择实物型号",
                "请根据电机标签确认 EL05 或 RS00，不要按固件默认值猜测。",
            )
            return
        identity = self.selected_identity
        answer = confirm(
            self,
            "确认电机型号",
            f"设备 {identity[0]}\n实物电机确认为 {name} 吗？\n\n选错型号会导致力矩/速度解释错误。请放下物体并清空活动范围。写入后必须按提示断开 24 V。已有行程配置不代表重新标定正确。",
        )
        if not answer:
            return

        def work():
            from device import Device

            info = Device(identity, self.emit).record_model(name)
            self.emit("info", (identity, info))
            self.emit("log", f"电机型号已写入并经重启核验：{name}")

        self.launch(work, updating=True)

    def clear(self, kind):
        if self.busy:
            return
        self.overrides.add(kind)
        self.paths[kind] = ""
        set_text(self.file_labels[kind], "未选择")
        set_tooltip(self.file_labels[kind], "")
        if kind == "mcu":
            self.mcu_target = None
        self.update_actions()

    def choose(self, kind, expected=None):
        if self.busy:
            return
        if (
            kind == "motor"
            and self.selected_identity
            and core.role_from_sn(self.selected_identity[0]) == "master"
        ):
            notice(self, "主爪固件", "主爪仅支持 MCU 更新，无需选择电机固件。")
            return
        path, _ = QFileDialog.getOpenFileName(
            self,
            tr("选择固件"),
            Path(expected["file"]).name if expected else "",
            tr("固件 (*.bin)"),
            options=QFileDialog.DontUseNativeDialog
            if language() == "en"
            else QFileDialog.Options(),
        )
        if not path:
            return
        try:
            image = core.load_image(path, kind, self.catalog)
            if expected and image.sha256 != expected["sha256"]:
                raise ValueError(
                    f"所选文件不是指定目标 {expected['model']} {expected['version']}，请重新选择"
                )
            if (
                kind == "mcu"
                and self.selected_identity
                and image.model != core.role_from_sn(self.selected_identity[0])
            ):
                raise ValueError("镜像主/从角色与当前设备不匹配")
            if kind == "mcu":
                core.validate_mcu_target(image, image.model)
            self.overrides.add(kind)
            self.set_image(kind, path, image)
        except Exception as exc:
            notice(self, "文件校验失败", str(exc), QMessageBox.Critical)

    def run(self):
        if self.busy:
            return
        index = self.combo.currentIndex()
        if index < 0 or not any(self.paths.values()):
            notice(self, "还差一步", "请先选择夹爪，并至少选择一个固件文件。")
            return
        if not self.current_info or not self.current_info["recorded"]:
            notice(self, "尚未就绪", "请等待连接；型号未记录时，先确认实物并设置型号。")
            return
        identity, paths = self.devices[index], self.paths.copy()
        self.progress.setValue(0)

        def work():
            from device import Device

            images = {
                k: core.load_image(p, k, self.catalog) if p else None
                for k, p in paths.items()
            }
            device = Device(identity, self.emit, ask=self.ask)
            with device.update_indication():
                core.execute(
                    device,
                    images["mcu"],
                    images["motor"],
                    self.ask,
                    self.emit,
                )

        self.launch(work, updating=True)

    def show_power(self, value):
        self.popup = PowerDialog(self)
        master = isinstance(value, dict) and value["role"] == "master"
        reason = value["reason"] if isinstance(value, dict) else value
        set_title(self.popup, "请拔插主爪 USB" if master else "请按步骤断开 24 V")
        self.popup.setMinimumWidth(580)
        box = QVBoxLayout(self.popup)
        box.setContentsMargins(28, 24, 28, 24)
        box.setSpacing(18)
        box.addWidget(label("需要你操作电源", "title"))
        box.addWidget(
            label(
                (
                    "①  拔掉主爪 USB\n\n②  等待至少 2 秒\n\n③  重新插回 USB"
                    if master
                    else "①  拔掉 24 V 电源\n\n②  等待至少 2 秒\n\n③  重新插回 24 V 电源"
                ),
                "heading",
            )
        )
        box.addWidget(
            label(
                "主爪通过 USB 供电，拔插 USB 即可。"
                if master
                else "USB 保持连接。仅拔 USB 不算断电。",
                "notice",
            )
        )
        box.addWidget(label(reason, "muted"))
        self.popup.connection_label = label(
            "正在检测 MCU 重启；随后仍需校验实际协议。", "muted"
        )
        box.addWidget(self.popup.connection_label)
        self.popup.detail_label = label(self.activity.detail(), "muted")
        box.addWidget(self.popup.detail_label)
        self.popup.setModal(True)
        self.popup.show()

    def drain(self):
        for _ in range(200):
            try:
                kind, value, task = self.events.get_nowait()
            except queue.Empty:
                return
            if task is not None and (
                task != self.active_task or (self.discard_read and kind != "idle")
            ):
                continue
            if kind == "diagnostics":
                self.show_diagnostics(value)
            elif kind == "progress":
                self.activity.update(value["done"], value["total"], value["unit"])
                if value["total"] > 0:
                    self.progress.setRange(0, 100)
                    set_format(self.progress, "当前传输 %p%")
                    self.progress.setValue(
                        min(100, int(100 * value["done"] / value["total"]))
                    )
                self.refresh_activity()
            elif kind == "ask":
                title, text, reply = value
                self.set_stage("等待确认：" + title)
                self.log(title + ": " + text)
                answer = confirm(self, title, text)
                self.log("用户确认: " + str(answer))
                self.set_stage("继续处理确认结果")
                reply.put(answer)
            elif kind == "power":
                self.set_stage(
                    "等待主爪 USB 重新插入（上限 180 秒）"
                    if isinstance(value, dict) and value["role"] == "master"
                    else "等待用户断开 24 V 并重新上电（上限 180 秒）"
                )
                self.set_tone("warning")
                self.log(value["reason"] if isinstance(value, dict) else value)
                self.show_power(value)
            elif kind == "power_status":
                self.log(value)
                if self.popup:
                    set_text(self.popup.connection_label, value)
            elif kind == "power_done":
                if self.popup:
                    self.popup.accept()
                    self.popup = None
                self.set_stage("断电检测结束，继续检查结果")
            elif kind == "detected_role":
                self.role_filter.blockSignals(True)
                self.role_filter.setCurrentIndex(2 if value == "master" else 1)
                self.role_filter.blockSignals(False)
                self.log(
                    "检测到新连接的"
                    + ("主爪" if value == "master" else "从爪")
                    + "，已切换设备类型"
                )
                self.apply_defaults()
            elif kind == "devices":
                previous = self.previous_identity or self.selected_identity
                self.devices = value
                self.combo.clear()
                self.combo.addItems([f"{sn}  ·  USB {usb}" for sn, usb in value])
                self.combo.setCurrentIndex(-1)
                self.current_info = None
                self.model_row.hide()
                set_text(
                    self.info,
                    "请选择设备查看版本"
                    if value
                    else "未发现匹配夹爪，等待连接；请关闭其他控制程序。",
                )
                self.set_tone("running")
                set_text(
                    self.status, "请选择目标设备" if value else "等待设备 · 自动扫描中"
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
                role = core.role_from_sn(identity[0])
                html, text, tone, status = device_summary(info, role, self.catalog)
                set_text(self.info, html)
                set_tooltip(
                    self.info,
                    "最新以本安装包内置审核固件为准；Flash 存档不等于实时验证。",
                )
                set_text(
                    self.safety,
                    "主爪：刷写时勿拔 USB，完成后按提示拔插 USB。"
                    if role == "master"
                    else "请清空活动范围；仅在提示时断开 24 V，刷写中勿拔线。",
                )
                self.fit_contents()
                self.log(text)
                self.apply_defaults()
                self.set_tone(tone)
                set_text(self.status, status)
            elif kind == "connection_error":
                self.previous_identity = self.selected_identity
                self.selected_identity = None
                self.devices = []
                self.combo.clear()
                if self.auto_connect:
                    self.scan_timer.start(5000)
                self.model_row.hide()
                self.current_info = None
                if "motor" not in self.overrides:
                    self.paths["motor"] = ""
                    set_text(self.file_labels["motor"], "读取设备失败")
                set_text(
                    self.info,
                    "连接失败，请检查 USB、24 V、串口权限或设备占用，然后点击刷新。",
                )
                self.set_tone("error")
                set_text(self.status, "未连接 · 详情见日志")
                self.log(value)
            elif kind == "idle":
                if self.activity.active:
                    set_text(self.activity_detail, self.activity.detail())
                self.activity.finish()
                if self.progress.maximum() == 0:
                    self.progress.setRange(0, 100)
                    self.progress.setValue(0)
                    set_format(self.progress, "等待下一步")
                self.busy = False
                self.updating = False
                self.active_task = None
                self.discard_read = False
                for control in self.controls:
                    control.setEnabled(True)
                self.update_actions()
                if self.pending_scan is not None:
                    follow_single = self.pending_scan
                    self.pending_scan = None
                    QTimer.singleShot(0, lambda: self.scan(follow_single=follow_single))
            else:
                if kind != "stage":
                    self.log(value)
                if kind == "stage":
                    self.set_stage(value)
                elif kind == "error":
                    self.set_tone("error")
                    set_text(self.activity_detail, self.activity.detail())
                    self.activity.finish()
                    self.progress.setRange(0, 100)
                    self.progress.setValue(0)
                    set_format(self.progress, "已停止")
                    set_text(self.status, "操作已停止 · 请查看详细日志")
                    self.toggle.setChecked(True)
                    notice(self, "需要处理", value, QMessageBox.Critical)
                elif kind == "success":
                    self.set_tone("success")
                    set_text(self.activity_detail, self.activity.detail())
                    self.activity.finish()
                    self.progress.setRange(0, 100)
                    set_format(self.progress, "完成")
                    set_text(self.status, "更新完成 · 所选固件已核验")
                    self.progress.setValue(100)
                    notice(self, "更新完成", value, success=True)

    def export(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            tr("导出日志"),
            "taccap-update.log",
            tr("日志 (*.log)"),
            options=QFileDialog.DontUseNativeDialog
            if language() == "en"
            else QFileDialog.Options(),
        )
        if path:
            try:
                Path(path).write_text(
                    "\n".join(tr(line) for line in self.log_lines) + "\n",
                    encoding="utf-8",
                )
            except OSError as exc:
                notice(self, "导出失败", str(exc), QMessageBox.Critical)

    def diagnose(self):
        if self.busy:
            return
        set_text(self.status, "正在检查环境；不发送夹爪命令…")

        def work():
            from host_diagnostics import diagnostic_report

            self.emit("diagnostics", diagnostic_report())

        self.launch(work)

    def show_diagnostics(self, report):
        from host_diagnostics import summary

        message = summary(report)
        self.log(message)
        self.log(json.dumps(report, ensure_ascii=False))
        dialog = QDialog(self)
        set_title(dialog, "环境诊断")
        dialog.setMinimumWidth(600)
        box = QVBoxLayout(dialog)
        detail = QPlainTextEdit()
        detail.setReadOnly(True)
        detail.setPlainText(tr(message))
        detail.setMinimumHeight(280)
        box.addWidget(detail)
        close = QPushButton()
        set_text(close, "关闭")
        close.clicked.connect(dialog.accept)
        box.addWidget(close)
        set_text(self.status, "诊断完成 · 详情已写入日志")
        dialog.exec()
        dialog.deleteLater()

    def closeEvent(self, event):
        if self.busy:
            event.ignore()
            notice(
                self,
                "操作尚未结束",
                "请等待完成或超时，刷写中请勿拔线。",
                QMessageBox.Warning,
            )
        else:
            event.accept()


def main(args):
    if os.geteuid() == 0:
        raise SystemExit(tr("请以普通用户运行，禁止 sudo 启动升级界面。"))
    configure_fontconfig()
    app = QApplication(sys.argv[:1])
    family = configure(app)
    integration_error = None
    if not args.ui_smoke_test:
        try:
            integrate_appimage(HERE / "icon.png")
        except (OSError, ValueError) as exc:
            integration_error = str(exc)
    window = App(auto_connect=not args.ui_smoke_test)
    if integration_error:
        window.log(f"应用菜单图标注册失败（不影响升级）：{integration_error}")
    window.show()
    if args.ui_smoke_test:

        def smoke():
            assert not window.text.isVisible()
            assert window.combo.currentIndex() == -1
            assert family.startswith("Noto Sans CJK")
            assert not app.windowIcon().isNull()
            assert not hasattr(window, "open_control_test")
            assert not hasattr(window, "test_button")
            if args.screenshot:
                if not window.grab().save(args.screenshot):
                    raise RuntimeError("截图保存失败")
            if language() == "en":
                import re

                for widget in window.findChildren(QLabel) + window.findChildren(
                    QPushButton
                ):
                    assert not re.search(r"[\u3400-\u9fff]", widget.text()), (
                        widget.text()
                    )
            print(
                json.dumps(
                    {
                        "ui": "ok",
                        "purpose": "firmware_update",
                        "hardware": "not opened",
                        "font": family,
                        "language": language(),
                    }
                )
            )
            app.quit()

        QTimer.singleShot(400, smoke)
    app.exec()

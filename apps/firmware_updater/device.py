"""Hardware adapter. Imported only by the worker, never by policy tests."""

import time
from contextlib import contextmanager

import core
import full_ota_update as flow
import motor_ota_update as motor_tools
from xense.taccap import (
    FollowerGripper,
    LeaderGripper,
    MotorOtaSession,
    MotorProtocol,
    OtaTargetVersion,
    Ws2812EffectType,
    scan_grippers,
)


def discover():
    return [
        (e.firmware_sn, e.mcu_serial)
        for e in scan_grippers()
        if e.firmware_sn and e.firmware_sn.endswith(("s", "m"))
    ]


class Device:
    def __init__(self, identity, emit, ask=None):
        self.identity = tuple(identity)
        self.role = core.role_from_sn(self.identity[0])
        self.emit = emit
        self.ask = ask
        self.transfer_unit = "B"
        self.last_progress_emit = 0.0
        self.last_progress_log = -1
        self._ota_indicator_active = False
        self._ota_indicator_pending = False
        self._ota_indicator_disabled = False
        self._last_mcu_device = None

    def endpoint(self):
        return flow._wait_ready(self.identity, 12, "查找所选夹爪")

    @contextmanager
    def opened(self, leader=False):
        ep = self.endpoint()
        cls = LeaderGripper if leader or self.role == "master" else FollowerGripper
        with cls(mcu_device=ep.mcu_device) as g:
            if g.device.get_sn() != self.identity[0]:
                raise RuntimeError("串口设备身份变化，停止")
            self._last_mcu_device = ep.mcu_device
            self._restore_ota_indicator(g)
            try:
                yield g
            except Exception:
                if self._ota_indicator_active:
                    self._indicator(g, "FaultBlink")
                raise

    def _indicator(self, g, name):
        if self.role != "slave" or self._ota_indicator_disabled:
            return False
        try:
            # Firmware presets reset old effects; a generic additive blue
            # effect could otherwise leave a white base looking white.
            g.led.effect(getattr(Ws2812EffectType, name), 0, 0, 0)
        except Exception as exc:
            self._ota_indicator_disabled = True
            self.emit("log", f"警告：灯效 {name} 设置失败：{exc}；不代表固件升级结果")
            return False
        if name == "OtaBlink":
            self._ota_indicator_pending = False
        return True

    def _restore_ota_indicator(self, g):
        if self._ota_indicator_active and self._ota_indicator_pending:
            if self._indicator(g, "OtaBlink"):
                self.emit(
                    "log", "MCU 重启后已请求恢复 OTA 蓝闪；后续状态查询不重置节奏"
                )

    def _begin_ota_indicator(self, g):
        if self.role != "slave" or self._ota_indicator_active:
            return
        self._ota_indicator_active = True
        if self._indicator(g, "OtaBlink"):
            self.emit("log", "已请求 OTA 蓝灯闪烁；刷写期间请勿拔线")
            # Existing MCU firmware renders effects in a 50 ms monitor tick.
            # Give the first blue frame time to reach the LEDs before the
            # high-priority flash transfer can occupy the MCU.
            time.sleep(0.06)

    @contextmanager
    def update_indication(self):
        try:
            yield
        except Exception:
            if self._ota_indicator_active:
                self.emit("log", "升级未完成：不会请求正常白灯，请以错误日志为准")
            raise
        else:
            if (
                self._ota_indicator_active
                and not self._ota_indicator_disabled
                and self._last_mcu_device
            ):
                try:
                    # Cosmetic cleanup must not launch another discovery loop.
                    # Recheck SN even on the last verified path before writing.
                    with LeaderGripper(mcu_device=self._last_mcu_device) as g:
                        if g.device.get_sn() != self.identity[0]:
                            raise RuntimeError("设备身份变化，未恢复灯效")
                        if self._indicator(g, "NormalSolid"):
                            self.emit("log", "固件与标定核验完成，已请求恢复正常白灯")
                except Exception as exc:
                    self.emit("log", f"警告：升级已核验，但正常灯效未恢复：{exc}")
        finally:
            self._ota_indicator_active = False
            self._ota_indicator_pending = False

    def inspect(self):
        self.emit("stage", "读取设备身份、型号与固件版本（版本查询最多等待 3 秒）")
        with self.opened() as g:
            if self.role == "master":
                return {
                    "sn": self.identity[0],
                    "role": "master",
                    "model": "主爪",
                    "recorded": True,
                    "motor": "不适用",
                    "mcu": flow._version_text(flow._version_tuple(g.firmware_version)),
                }
            m = g.motor.get_model()
            v = g.motor.motor_version(3000)
            label = "未知"
            if v.valid:
                label = v.vendor_str + (
                    "（Flash 存档，非实时）" if v.from_flash else "（实时）"
                )
            return {
                "sn": self.identity[0],
                "model": m.name,
                "recorded": m.from_flash,
                "mcu": flow._version_text(flow._version_tuple(g.firmware_version)),
                "motor": label,
                **self.can_id_state(g),
            }

    def can_id_state(self, g):
        # Use the same SDK mapping as the production calibration tool.
        from xense.taccap import expected_motor_can_id, parse_serial

        model = g.motor.get_model()
        if self.role != "slave" or model.name != "RS00":
            return {}
        parsed = parse_serial(self.identity[0])
        side = getattr(parsed.side, "name", "")
        expected = None
        if (
            model.from_flash
            and model.id == 1
            and getattr(parsed.role, "name", "") == "Follower"
            and side in ("Left", "Right")
        ):
            expected = expected_motor_can_id(model.id, parsed.side)
        return {
            "can_id": g.motor.get_can_id(),
            "can_id_expected": int(expected) if expected is not None else None,
            "can_id_side": {"Left": "左", "Right": "右"}.get(side, "未知"),
        }

    def required_can_id_state(self, g):
        model = g.motor.get_model()
        if model.name != "RS00" or not model.from_flash or model.id != 1:
            raise RuntimeError("RS00 型号记录变化或未确认，停止 CAN ID 操作")
        state = self.can_id_state(g)
        if state.get("can_id_expected") is None:
            raise RuntimeError("无法从固件 SN 确认 RS00 左右手，拒绝猜测 CAN ID")
        return state

    def check_can_id(self):
        with self.opened() as g:
            state = self.required_can_id_state(g)
        if state["can_id"] != state["can_id_expected"]:
            raise RuntimeError(
                f"CAN ID 读回不符：期望 {state['can_id_expected']}，"
                f"实际 {state['can_id']}；停止，不自动重写，请联系支持"
            )
        self.emit(
            "log", f"RS00 CAN ID 已核验：{state['can_id']}（{state['can_id_side']}爪）"
        )

    def ensure_can_id(self):
        if self.role != "slave":
            raise RuntimeError("主爪不提供电机 CAN ID 设置")
        with self.opened() as g:
            state = self.required_can_id_state(g)
        current, expected = state["can_id"], state["can_id_expected"]
        if current == expected:
            self.emit("log", f"CAN ID 已是 {expected}，跳过写入和断电")
            return
        if not self.ask or not self.ask(
            "确认修正 RS00 CAN ID",
            f"设备 {self.identity[0]}\n{state['can_id_side']}爪 RS00："
            f"CAN ID {current} → {expected}。\n"
            "目标由固件 SN 和 SDK 规则确定，不允许任意填写。\n"
            "写入后必须按提示断开整只从爪 24 V 至少 2 秒再上电；"
            "重新上电可能触发标定，请清空活动范围。取消将停止更新。",
        ):
            raise RuntimeError("用户取消 CAN ID 修正，未开始固件刷写")
        # Re-open and revalidate after the confirmation; never trust a stale UI.
        with self.opened() as g:
            if self.required_can_id_state(g) != state:
                raise RuntimeError("确认期间 CAN ID 或型号状态变化，停止，请刷新设备")
            self.emit("stage", "等待设备空闲并失能电机，准备修正 CAN ID")
            flow._prepare_motor_admin(g)
            g.motor.set_can_id(expected)
            self.emit("log", f"CAN ID {expected} 已请求写入，待断开 24 V 生效")
        self.power_cycle(f"应用 RS00 CAN ID {expected}，重连后必须读回核验")
        self.check_can_id()

    def mcu_version(self):
        # Do not spend up to three seconds querying an unrelated motor version.
        with self.opened(leader=True) as g:
            return flow._version_text(flow._version_tuple(g.firmware_version))

    def record_model(self, name):
        if self.role != "slave":
            raise RuntimeError("主爪不提供从爪电机型号设置")
        ids = {"EL05": 0, "RS00": 1}
        if name not in ids:
            raise ValueError("不支持的电机型号")
        with self.opened() as g:
            if g.motor.get_model().from_flash:
                raise RuntimeError("设备已有型号记录，用户版不覆盖已有型号")
            self.emit("stage", "等待设备空闲，准备写入电机型号")
            flow._prepare_motor_admin(g)
            self.emit("log", f"写入电机型号：{name}")
            g.motor.set_model(ids[name])
        self.power_cycle("应用电机型号；重新上电可能触发自动标定")
        info = self.inspect()
        if not info["recorded"] or info["model"] != name:
            raise RuntimeError("重启后电机型号读回不一致，停止更新")
        return info

    def power_cycle(self, reason):
        # Observe a baseline before displaying the nonblocking instructions.
        self.emit("stage", "准备断电检测：读取 MCU 启动时钟基线")
        ep = self.endpoint()
        before = flow._read_uptime(ep, self.identity)
        self.emit("log", f"断电检测基线：uptime={before} ms；等待物理断电，上限 180 秒")
        detector = flow._RestartDetector(before)
        self.emit(
            "power",
            {"role": self.role, "reason": reason},
        )
        deadline = time.monotonic() + 180
        last_report = 0.0
        detail = "等待 MCU 时钟回退并连续增长"
        try:
            while time.monotonic() < deadline:
                try:
                    ep = flow._scan_for(self.identity)
                    if ep is None:
                        # A vanished USB node is a sampling gap, not an
                        # observed low uptime. Do not carry a pre-gap rollback
                        # candidate into a newly enumerated port.
                        detector.reset_from = None
                        detail = "未发现目标 USB 设备，请检查 USB 连接和设备供电"
                    else:
                        uptime = flow._read_uptime(ep, self.identity)
                        if detector.observe(uptime):
                            # A serial reopen alone does not reset MCU effects.
                            # Reapply once only after the existing restart
                            # detector confirms a reboot; no extra clock reads.
                            self._ota_indicator_pending = self._ota_indicator_active
                            self.emit(
                                "log",
                                f"MCU 时钟重置已确认：uptime={uptime} ms；"
                                "这不能证明电机已断电，仍须校验实际协议",
                            )
                            return
                        detail = (
                            f"设备在线：uptime={uptime} ms；等待有效时钟回退及连续增长"
                        )
                except Exception as exc:
                    # Cable removal can interrupt an in-flight read, but an
                    # error alone must never count as proof of restart.
                    detector.reset_from = None
                    detail = f"设备暂时无法通信：{type(exc).__name__}: {exc}"
                if time.monotonic() - last_report >= 3:
                    self.emit("power_status", detail)
                    last_report = time.monotonic()
                time.sleep(0.5)
            raise RuntimeError(f"等待断电重启超时；{detail}。未绕过校验，请导出日志")
        finally:
            self.emit("power_done", "")

    def protocol(self, name, force_cycle=False, *, allow_recovery=True):
        if self.role != "slave":
            raise RuntimeError("主爪不能切换从爪电机协议")
        self.emit("stage", f"准备切换/核验 {name} 协议")
        expected = getattr(MotorProtocol, name)
        if force_cycle:
            time.sleep(3)
        with self.opened() as g:
            self.emit("stage", "等待标定结束并失能电机（单项等待上限 45 秒）")
            flow._prepare_motor_admin(g)
            changed = g.motor.get_protocol() != expected
            self.emit(
                "log", f"{name} 协议：{'需要切换并断电' if changed else '无需切换'}"
            )
            if changed:
                self.emit("stage", f"写入 {name} 协议启动配置")
                flow._switch_protocol(g, expected)
        if changed or force_cycle:
            self.power_cycle(f"应用 {name} 协议并重启电机")
        self.emit("stage", f"等待 {name} 协议连续两次读回一致（上限 20 秒）")
        try:
            with self.opened() as g:
                flow._wait_motor_protocol(g.motor, expected)
        except RuntimeError as exc:
            self.emit("log", f"协议校验失败：{exc}")
            # Never retry a flash, a failed connection, or the post-flash cycle.
            if name != "Private" or force_cycle or not allow_recovery or not self.ask:
                raise
            with self.opened() as g:
                actual = g.motor.get_protocol()
            self.emit("log", f"恢复前实际协议读回：{actual}")
            if actual != MotorProtocol.Mit:
                raise
            if not self.ask(
                "Private 协议未生效：是否恢复一次？",
                "当前仍读回 MIT，电机尚未开始本次刷写。\\n"
                "MCU 重启不能证明电机已断电；也可能是切换或参数保存未生效。\\n"
                "确认后仅重新发送一次协议切换请求，再按弹窗断开整只夹爪的 "
                "24 V 至少 2 秒（USB 保持连接）。\\n"
                "不会重刷 MCU、不会绕过协议校验；再次失败将停止并保留日志。",
            ):
                raise RuntimeError(
                    "用户取消协议恢复；未开始电机刷写，最后读回 MIT"
                ) from exc
            self.emit("log", "用户确认一次协议恢复；重新核对设备、等待空闲后发送")
            return self.protocol("Private", allow_recovery=False)
        self.emit("log", f"{name} 协议已核验")

    def begin_transfer(self, label, unit):
        self.transfer_unit = unit
        self.last_progress_emit = 0.0
        self.last_progress_log = -1
        self.emit("stage", label)

    def progress(self, done, total):
        # Coalesce frequent MCU callbacks so the GUI does not starve on logs.
        now = time.monotonic()
        percent = min(100, int(100 * done / total)) if total else 0
        if now - self.last_progress_emit >= 0.1 or done >= total:
            self.emit(
                "progress", {"done": done, "total": total, "unit": self.transfer_unit}
            )
            self.last_progress_emit = now
        bucket = percent // 10
        if bucket != self.last_progress_log or done >= total:
            self.emit(
                "log", f"传输 {percent}%：{done:,}/{total:,} {self.transfer_unit}"
            )
            self.last_progress_log = bucket

    def flash_mcu(self, image):
        if image.kind != "mcu" or image.model != self.role:
            raise ValueError("MCU 镜像与设备主/从角色不匹配")
        with self.opened(leader=True) as g:
            target = OtaTargetVersion(*map(int, image.version.split(".")), 0)
            self._begin_ota_indicator(g)
            self.begin_transfer("MCU 刷写：启动会话并传输固件，请勿拔线", "B")
            g.ota.update_from_bytes(image.data, target, self.progress)
            # Applying MCU firmware resets its LED state. Arm one restore on
            # the next identity-verified connection, not every status query.
            self._ota_indicator_pending = self._ota_indicator_active
        self.emit(
            "stage",
            "MCU 已提交，正在重启（可能短暂白灯/熄灭）；版本读回后恢复蓝闪（上限约 33 秒）",
        )
        time.sleep(3)
        flow._wait_follower_version(
            self.identity,
            flow._parse_version_text(image.version),
            30,
            on_ready=self._restore_ota_indicator if self.role == "slave" else None,
        )
        self.emit(
            "log", "MCU 版本已读回；接下来仍须按提示完成物理断电，不代表升级全流程结束"
        )

    def flash_motor(self, image):
        if self.role != "slave":
            raise RuntimeError("主爪不支持电机 OTA")
        with self.opened() as g:
            session = MotorOtaSession(g.motor, g.motor.get_can_id())
            self.emit("stage", "电机 OTA 预检：核对协议、型号与 UID")
            session.preflight(image.model)
            uid = session.read_uid()
            self.emit("log", f"电机 UID: {uid.hex()}；镜像 SHA256: {image.sha256}")
            self._begin_ota_indicator(g)
            self.begin_transfer("电机刷写：等待握手并传输固件，请勿拔线", "包")
            session.update_from_bytes(image.data, self.progress)
        self.emit("log", "电机结束帧已确认；仍需断电、版本核验和恢复协议")

    def live_motor_version(self):
        self.emit("stage", "等待 Private 协议及电机实时版本（版本轮询上限 20 秒）")
        with self.opened() as g:
            flow._wait_motor_protocol(g.motor, MotorProtocol.Private)
            return motor_tools.read_version(g.motor, 20)

    def record_motor(self, actual):
        self.emit("stage", f"保存已核验的电机版本记录：{actual}")
        with self.opened() as g:
            g.motor.set_motor_fw_version(actual)

    def check_homing(self):
        self.emit("stage", "最终检查：等待 MIT 协议和自动标定完成")
        with self.opened() as g:
            flow._wait_motor_protocol(g.motor, MotorProtocol.Mit)
            flow._admin_call(g, "等待最终标定", lambda: None)
            d = g.home_diag()
            cfg = g.get_gripper_config()
            if (
                d.fail_reason
                or not d.flags & 2
                or not cfg.flags & 1
                or cfg.max_open_rad <= cfg.min_open_rad
            ):
                raise RuntimeError("最终标定未通过；固件可能已更新，请勿直接夹取")

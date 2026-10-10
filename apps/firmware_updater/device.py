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
    scan_grippers,
)


def discover():
    return [
        (e.firmware_sn, e.mcu_serial)
        for e in scan_grippers()
        if e.firmware_sn and e.firmware_sn.endswith(("s", "m"))
    ]


class Device:
    def __init__(self, identity, emit):
        self.identity = tuple(identity)
        self.role = core.role_from_sn(self.identity[0])
        self.emit = emit
        self.transfer_unit = "B"
        self.last_progress_emit = 0.0
        self.last_progress_log = -1

    def endpoint(self):
        return flow._wait_ready(self.identity, 12, "查找所选夹爪")

    @contextmanager
    def opened(self, leader=False):
        ep = self.endpoint()
        cls = LeaderGripper if leader or self.role == "master" else FollowerGripper
        with cls(mcu_device=ep.mcu_device) as g:
            if g.device.get_sn() != self.identity[0]:
                raise RuntimeError("串口设备身份变化，停止")
            yield g

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
            }

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
                        detail = "未发现目标 USB 设备，请检查 USB 连接和设备供电"
                    else:
                        uptime = flow._read_uptime(ep, self.identity)
                        if detector.observe(uptime):
                            self.emit("log", f"目标设备重启已确认：uptime={uptime} ms")
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

    def protocol(self, name, force_cycle=False):
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
        with self.opened() as g:
            flow._wait_motor_protocol(g.motor, expected)
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
            self.begin_transfer("MCU 刷写：启动会话并传输固件，请勿拔线", "B")
            g.ota.update_from_bytes(image.data, target, self.progress)
        self.emit("stage", "MCU 已提交，等待设备重启和版本读回（上限约 33 秒）")
        time.sleep(3)
        flow._wait_follower_version(
            self.identity, flow._parse_version_text(image.version), 30
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

"""Hardware adapter. Imported only by the worker, never by policy tests."""

import time
from contextlib import contextmanager

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
        if e.firmware_sn and e.firmware_sn.endswith("s")
    ]


class Device:
    def __init__(self, identity, emit):
        self.identity = tuple(identity)
        self.emit = emit

    def endpoint(self):
        return flow._wait_ready(self.identity, 12, "查找所选从爪")

    @contextmanager
    def opened(self, leader=False):
        ep = self.endpoint()
        cls = LeaderGripper if leader else FollowerGripper
        with cls(mcu_device=ep.mcu_device) as g:
            if g.device.get_sn() != self.identity[0]:
                raise RuntimeError("串口设备身份变化，停止")
            yield g

    def inspect(self):
        with self.opened() as g:
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

    def power_cycle(self, reason):
        # Observe a baseline before displaying the nonblocking instructions.
        ep = self.endpoint()
        before = flow._read_uptime(ep, self.identity)
        deadline = time.monotonic() + 30
        while before < 5000 and time.monotonic() < deadline:
            time.sleep(0.5)
            before = flow._read_uptime(ep, self.identity)
        if before < 5000:
            raise RuntimeError("设备启动时钟尚未稳定，无法可靠检测断电；请稍后重试")
        detector = flow._RestartDetector(before)
        self.emit(
            "power",
            reason
            + "\n拔掉这只夹爪的 24 V，保持至少 2 秒再插回；USB 保留。\n仅拔 USB 不算断电。重新上电可能标定，请清空活动范围。\n程序自动检测，无需点击确认。心跳重启不能替代真实物理断电。",
        )
        deadline = time.monotonic() + 180
        try:
            while time.monotonic() < deadline:
                try:
                    ep = flow._scan_for(self.identity)
                    if ep is not None and detector.observe(
                        flow._read_uptime(ep, self.identity)
                    ):
                        self.emit("log", "目标设备重启已确认")
                        return
                except Exception:
                    # Cable removal can interrupt an in-flight read, but an
                    # error alone must never count as proof of restart.
                    detector.reset_from = None
                time.sleep(1)
            raise RuntimeError(
                "等待 24 V 断电重启超时；未绕过校验，请检查供电并导出日志"
            )
        finally:
            self.emit("power_done", "")

    def protocol(self, name, force_cycle=False):
        expected = getattr(MotorProtocol, name)
        if force_cycle:
            time.sleep(3)
        with self.opened() as g:
            flow._prepare_motor_admin(g)
            changed = g.motor.get_protocol() != expected
            if changed:
                flow._switch_protocol(g, expected)
        if changed or force_cycle:
            self.power_cycle(f"应用 {name} 协议并重启电机")
        with self.opened() as g:
            flow._wait_motor_protocol(g.motor, expected)

    def progress(self, done, total):
        self.emit("progress", int(100 * done / total) if total else 0)

    def flash_mcu(self, image):
        with self.opened(leader=True) as g:
            target = OtaTargetVersion(*map(int, image.version.split(".")), 0)
            g.ota.update_from_bytes(image.data, target, self.progress)
        time.sleep(3)
        flow._wait_follower_version(
            self.identity, flow._parse_version_text(image.version), 30
        )

    def flash_motor(self, image):
        with self.opened() as g:
            session = MotorOtaSession(g.motor, g.motor.get_can_id())
            session.preflight(image.model)
            uid = session.read_uid()
            self.emit("log", f"电机 UID: {uid.hex()}；镜像 SHA256: {image.sha256}")
            session.update_from_bytes(image.data, self.progress)

    def live_motor_version(self):
        with self.opened() as g:
            flow._wait_motor_protocol(g.motor, MotorProtocol.Private)
            return motor_tools.read_version(g.motor, 20)

    def record_motor(self, actual):
        with self.opened() as g:
            g.motor.set_motor_fw_version(actual)

    def check_homing(self):
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

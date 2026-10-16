"""One serial-owning SDK session; no Qt, flash writes, or raw control frames."""

import math
import threading
import time

# MotorStatusBit faults, excluding Enabled and Stalled (contact is expected).
FAULT_MASK = 0x0FFA
SESSION_SECONDS = 120


def preflight(g, sn, sdk):
    version = g.firmware_version
    if (version.major, version.minor, version.patch) < (1, 2, 14):
        raise RuntimeError("请先升级从爪 MCU 至 1.2.14 或更新版本")
    model = g.motor.get_model()
    if not model.from_flash or model.name not in ("EL05", "RS00"):
        raise RuntimeError("请先确认并记录电机型号")
    if g.motor.get_protocol() != sdk.MotorProtocol.Mit:
        raise RuntimeError("测试需要 MIT 协议，请先完成升级流程")
    home = g.home_diag()
    if home.flags & 1 or not home.flags & 2 or home.fail_reason:
        raise RuntimeError("自动标定未成功结束，请检查设备后重试")
    cfg = g.get_gripper_config()
    if (
        not cfg.flags & 1
        or not math.isfinite(cfg.max_open_rad)
        or not math.isfinite(cfg.min_open_rad)
        or cfg.max_open_rad <= cfg.min_open_rad
    ):
        raise RuntimeError("行程配置无效，禁止测试")
    parsed = sdk.parse_serial(sn)
    if model.name == "RS00":
        if not parsed.valid or parsed.side is None:
            raise RuntimeError("无法确认 RS00 左右爪身份，禁止测试")
        reverse = sdk.expected_open_reverse(model.id, parsed.side)
        can_id = sdk.expected_motor_can_id(model.id, parsed.side)
        if (
            reverse is None
            or can_id is None
            or bool(cfg.flags & 2) != reverse
            or g.motor.get_can_id() != can_id
        ):
            raise RuntimeError("RS00 方向或 CAN ID 不匹配，请先完成升级检查")
    audit = g.audit_envelope()
    env = audit.effective
    spec = g.motor.get_spec()
    if (
        env is None
        or env.flags & 3 != 3
        or not all(
            math.isfinite(v) and v > 0
            for v in (env.cont_torque_nm, env.peak_torque_nm, spec.t_max_nm)
        )
        or env.cont_torque_nm > env.peak_torque_nm
        or env.peak_torque_nm > spec.t_max_nm
    ):
        raise RuntimeError("安全包络未生效或超出范围，请联系支持")
    if g.motor.read_status().status & FAULT_MASK:
        raise RuntimeError("电机存在故障，禁止测试；不会自动清除故障")
    config = sdk.ForcePositionConfig.for_spec(spec)
    config.grasp_torque_nm = min(config.grasp_torque_nm, env.cont_torque_nm, 0.5)
    config.close_speed_radps = 0.25
    config.close_preload_nm = 0.0
    config.status_timeout_ms = 350
    return config


def observation(snapshot):
    obs = snapshot.observation
    if (
        not snapshot.running
        or snapshot.state.name == "FAULT"
        or snapshot.fault_reason
        or not obs.valid
        or not all(
            math.isfinite(v)
            for v in (
                obs.position,
                obs.torque,
                obs.velocity,
                obs.motor_temp_c,
                obs.age_ms,
            )
        )
        or not 0 <= obs.position <= 1
        or not 0 <= obs.age_ms <= 350
        or obs.status & FAULT_MASK
    ):
        raise RuntimeError("控制反馈失效或电机故障，正在停止测试")
    return {
        "position": obs.position,
        "torque": obs.torque,
        "velocity": obs.velocity,
        "temperature": obs.motor_temp_c,
    }


class ControlSession:
    """Commands are coalesced at 10 Hz; stop always outranks a pending target."""

    def __init__(self, identity, emit, *, opener=None, sdk=None):
        self.identity = tuple(identity)
        self.emit = emit
        self.opener = opener
        self.sdk = sdk
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.pending = None

    def set_target(self, value):
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError("Target must be finite and within [0, 1]")
        with self.lock:
            if not self.stopping.is_set():
                self.pending = value

    def stop(self):
        self.stopping.set()

    def run(self):
        error = None
        disabled = False
        try:
            if self.identity[0][-1:] != "s":
                raise RuntimeError("功能测试仅支持从爪")
            if self.stopping.is_set():
                return
            sdk = self.sdk
            opener = self.opener
            if sdk is None:
                import xense.taccap as sdk
            if opener is None:
                from device import Device

                opener = Device(self.identity, self.emit).opened
            self.emit("stage", "正在检查测试条件；尚未启用电机")
            with opener() as g:
                config = preflight(g, self.identity[0], sdk)
                ctl = None
                try:
                    if self.stopping.is_set():
                        return
                    g.motor.disable()
                    ctl = sdk.ForcePositionController(g, config)
                    ctl.start()  # SDK seeds the target from the measured position.
                    initial = observation(ctl.snapshot())
                    if self.stopping.is_set():
                        return
                    g.motor.enable()  # Only after the explicit GUI confirmation.
                    # Never replay a slider value from before the initial sample.
                    with self.lock:
                        self.pending = None
                    self.emit("ready", initial)
                    started = time.monotonic()
                    while not self.stopping.wait(0.1):
                        if time.monotonic() - started >= SESSION_SECONDS:
                            self.emit("stage", "已达到 120 秒测试上限，正在停止")
                            break
                        sample = observation(ctl.snapshot())
                        with self.lock:
                            target, self.pending = self.pending, None
                        if target is not None and not self.stopping.is_set():
                            ctl.set_target(target)
                        self.emit("sample", sample)
                finally:
                    # stop() stops the producer first. A bare disable() can race
                    # the controller's next frame and re-enable the motor.
                    try:
                        if ctl is not None:
                            ctl.stop()
                    finally:
                        try:
                            g.motor.disable()  # Explicit ACK; SDK stop swallows it.
                            disabled = True
                        except Exception as exc:
                            raise RuntimeError(
                                "未确认电机失能；请清空活动范围并断开 24 V"
                            ) from exc
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            self.stopping.set()
            self.emit("finished", {"error": error, "disabled": disabled})

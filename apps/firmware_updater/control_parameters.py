"""Public SDK config fields and stricter, non-persistent test-tool validation."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Field:
    name: str
    label: str
    unit: str = ""
    minimum: float = 0
    maximum: float = 20
    integer: bool = False
    advanced: bool = False


FIELDS = {
    "force_position": (
        Field("grasp_torque_nm", "夹持力矩预算", "N·m", 0.001),
        Field("close_speed_radps", "目标斜坡速度", "rad/s", 0.001, 4),
        Field("close_preload_nm", "闭合预载", "N·m", advanced=True),
        Field("hold_torque_limit_nm", "夹持预算上限", "N·m", 0.001, advanced=True),
        Field("motion_torque_limit_nm", "运动故障阈值", "N·m", 0.001, advanced=True),
        Field("status_timeout_ms", "反馈超时", "ms", 100, 350, True, True),
        Field("motor_stream_hz", "状态流频率", "Hz", 1, 100, True, True),
    ),
    "impedance": (
        Field("max_position_torque_nm", "位置力矩预算", "N·m", 0.001),
        Field("kp", "刚度 Kp", "N·m/rad", 0.001, 1000),
        Field("kd", "阻尼 Kd", "N·m·s/rad", 0.001, 100),
        Field("feedforward_torque", "前馈力矩", "N·m", -20),
        Field("rated_torque_nm", "实测力矩后备阈值", "N·m", 0.001, advanced=True),
        Field("peak_torque_nm", "反馈异常阈值", "N·m", 0.001, advanced=True),
        Field("status_timeout_ms", "反馈超时", "ms", 100, 350, True, True),
        Field("motor_stream_hz", "状态流频率", "Hz", 1, 100, True, True),
    ),
}


def config_type(sdk, mode):
    if mode not in FIELDS:
        raise ValueError("控制器类型无效")
    return sdk.ForcePositionConfig if mode == "force_position" else sdk.ImpedanceConfig


def values_of(config, mode):
    return {field.name: getattr(config, field.name) for field in FIELDS[mode]}


def limits_of(spec, env, startup, sdk):
    limits = {
        "continuous": spec.stall_cont_torque_nm,
        "rated": spec.rated_torque_nm,
        "range": spec.t_max_nm,
        "kp": spec.kp_max,
        "kd": spec.kd_max,
        "speed": min(spec.v_max_rad_s, sdk.MOTOR_MAX_APPROACH_SPEED_RADPS),
        "envelope_cont": env.cont_torque_nm,
        "envelope_peak": env.peak_torque_nm,
        "startup": startup,
    }
    if not all(math.isfinite(v) and v > 0 for v in limits.values()):
        raise ValueError("设备规格或安全限值无效，禁止测试")
    if startup > limits["range"] + 1e-6:
        raise ValueError("设备规格或安全限值无效，禁止测试")
    return limits


def validate(mode, values, limits):
    if mode not in FIELDS or set(values) != {f.name for f in FIELDS[mode]}:
        raise ValueError("控制器参数不完整或包含未知字段")
    for field in FIELDS[mode]:
        value = values[field.name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            or not field.minimum <= value <= field.maximum
            or (field.integer and value != int(value))
        ):
            raise ValueError("参数超出允许范围：" + field.name)
    if values["status_timeout_ms"] * values["motor_stream_hz"] < 3000:
        raise ValueError("反馈超时必须容纳至少三帧；请提高状态流频率")
    budget_limit = min(limits["continuous"], limits["envelope_cont"])
    if mode == "force_position":
        if values["grasp_torque_nm"] > budget_limit + 1e-6:
            raise ValueError("夹持预算超过连续堵转额定或固件安全包络")
        if values["close_speed_radps"] > limits["speed"] + 1e-6:
            raise ValueError("目标速度超过 SDK 或电机规格上限")
        if not (
            values["close_preload_nm"]
            <= values["grasp_torque_nm"]
            <= values["hold_torque_limit_nm"]
            <= values["motion_torque_limit_nm"]
        ):
            raise ValueError("需要满足：预载 ≤ 夹持预算 ≤ 夹持上限 ≤ 运动阈值")
        if (
            values["hold_torque_limit_nm"] > limits["rated"] + 1e-6
            or values["motion_torque_limit_nm"] > limits["range"] + 1e-6
            or values["motion_torque_limit_nm"] + 1e-6 < limits["startup"]
        ):
            raise ValueError("夹持或运动阈值与电机额定、量程或启动限矩不兼容")
    else:
        sustained = values["max_position_torque_nm"] + abs(values["feedforward_torque"])
        if sustained > budget_limit + 1e-6:
            raise ValueError("位置预算加前馈幅值超过连续额定或固件安全包络")
        if (
            not (
                sustained
                < values["rated_torque_nm"]
                <= values["peak_torque_nm"]
                <= limits["range"] + 1e-6
            )
            or values["rated_torque_nm"] > limits["rated"] + 1e-6
        ):
            raise ValueError("后备阈值必须高于预算加前馈幅值，且不超过电机额定")
        if values["kp"] > limits["kp"] or values["kd"] > limits["kd"]:
            raise ValueError("Kp 或 Kd 超过电机规格上限")
        if values["max_position_torque_nm"] / values["kd"] > limits["speed"] + 1e-6:
            raise ValueError("阻抗接近速度过高；请增大 Kd 或减小力矩预算")


def make_config(spec, env, startup, sdk, mode, parameters=None):
    limits = limits_of(spec, env, startup, sdk)
    config = config_type(sdk, mode).for_spec(spec)
    budget = min(0.5, limits["continuous"], limits["envelope_cont"])
    if mode == "force_position":
        config.grasp_torque_nm = budget
        config.close_speed_radps = 0.25
        config.close_preload_nm = 0
    else:
        config.max_position_torque_nm = budget
        config.kd = budget / 0.25
        config.feedforward_torque = 0
    config.status_timeout_ms = 350
    values = values_of(config, mode) if parameters is None else dict(parameters)
    validate(mode, values, limits)
    for field in FIELDS[mode]:
        setattr(
            config,
            field.name,
            int(values[field.name]) if field.integer else values[field.name],
        )
    return config

"""UI-independent update policy. No hardware or GUI imports here."""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Image:
    kind: str
    model: str
    version: str
    sha256: str
    warning: str
    data: bytes


def load_image(path, kind, catalog):
    path = Path(path)
    if path.stat().st_size > 4 * 1024 * 1024:
        raise ValueError("固件文件过大，拒绝读取")
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    matches = [
        r for r in catalog["images"] if r["sha256"] == digest and r["kind"] == kind
    ]
    if len(matches) != 1:
        raise ValueError(
            "文件不在内置校验清单中或类型错误，请向技术支持获取匹配的固件/工具。改名不能绕过校验。"
        )
    return Image(**{k: v for k, v in matches[0].items() if k != "file"}, data=data)


def catalog_at(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def version_key(value):
    parts = tuple(int(p) for p in value.split("."))
    if len(parts) == 5 and parts[0] == 1 and 0 <= parts[1] <= 9:
        parts = (10 + parts[1], *parts[2:])
    if len(parts) not in (3, 4) or any(p < 0 or p > 255 for p in parts):
        raise ValueError("版本格式无效")
    return parts


def confirm_change(ask, label, current, image):
    if not current:
        raise RuntimeError(f"无法确认{label}当前版本，停止，不能凭存档推断电机实际版本")
    a, b = version_key(current), version_key(image.version)
    if b == a:
        return ask(
            f"确认{label}重复刷写",
            f"当前已是 {current}。通常无需重刷。\n选择跳过可减少刷写和断电步骤；需要重刷时请确认。",
        )
    action = "回滚" if b < a else "升级"
    warning = image.warning or "刷写过程中不可断电；重新上电可能触发自动标定。"
    if not ask(
        f"确认{label}{action}",
        f"{current} → {image.version}\n\n{warning}\n\n确认继续？",
    ):
        raise RuntimeError(
            "用户取消；如已切换协议，设备可能仍处于 Private，请联系支持或重新完成升级流程。"
        )

    return True


def role_from_sn(sn):
    if sn.endswith("m"):
        return "master"
    if sn.endswith("s"):
        return "slave"
    raise ValueError("设备 SN 未标明主/从角色，拒绝猜测")


def execute_master(device, info, mcu, motor, ask, emit):
    if motor or not mcu or mcu.model != "master":
        raise ValueError("主爪只允许主爪 MCU 镜像，不支持从爪或电机固件")
    if not ask(
        "开始主爪更新",
        f"设备 {info['sn']}\nMCU {info['mcu']} → {mcu.version}\n更新后需拔插 USB。",
    ):
        raise RuntimeError("用户取消，未修改设备")
    if confirm_change(ask, "主爪 MCU", device.mcu_version(), mcu):
        device.flash_mcu(mcu)
        device.power_cycle("主爪 MCU 刷写完成，请拔插 USB")
        if device.mcu_version() != mcu.version:
            raise RuntimeError("主爪 MCU 实际版本不符")
        emit("success", f"主爪 MCU {mcu.version} 已更新并核验，无需电机标定。")
    else:
        emit("success", "已核验主爪 MCU 为所选版本，跳过重复刷写；未修改设备。")


def execute(device, mcu, motor, ask, emit):
    """The driver owns identity checks, busy guards and actual transfers.

    Every irreversible transfer has its own version/risk confirmation. A failed
    transfer or verification never causes an automatic retry/rollback.
    """
    if not mcu and not motor:
        raise ValueError("至少选择一种固件")
    emit("stage", "更新预检：重新核对设备与固件")
    info = device.inspect()
    role = role_from_sn(info["sn"])
    if role == "master":
        return execute_master(device, info, mcu, motor, ask, emit)
    if not info["recorded"]:
        raise RuntimeError("电机型号尚未配置，请联系技术支持；用户版不覆盖型号")
    if version_key(info["mcu"]) < (1, 2, 14):
        raise RuntimeError("此用户版仅支持从爪 MCU >= 1.2.14，旧设备请由技术支持初始化")
    if motor and motor.model != info["model"]:
        raise RuntimeError("电机型号与镜像不匹配，禁止刷写")
    if mcu and (mcu.model != "slave" or version_key(mcu.version) < (1, 2, 14)):
        raise RuntimeError("不支持此 MCU 回滚目标")
    plan = f"设备 {info['sn']}\n型号 {info['model']}\n"
    plan += f"MCU: {info['mcu']} → {mcu.version if mcu else '不修改'}\n"
    plan += f"电机: {info['motor']} → {motor.version if motor else '不修改'}\n"
    plan += "\n请放下物体、清空活动范围并关闭其他控制程序。不会修改型号、方向、CAN ID 或力矩参数。"
    if not ask("开始升级/回滚", plan):
        raise RuntimeError("用户取消，未修改设备")
    mcu_first = not mcu or version_key(mcu.version) >= version_key(info["mcu"])

    def flash_mcu():
        if not mcu:
            return
        if not confirm_change(ask, "夹爪 MCU", device.mcu_version(), mcu):
            emit("log", f"跳过同版本 MCU：{mcu.version}；无需刷写或为它断电")
            return
        device.protocol("Mit")
        emit("stage", "正在刷写 MCU：不要断开电源或 USB")
        device.flash_mcu(mcu)
        device.power_cycle("MCU 刷写完成，必须硬重启")
        if device.mcu_version() != mcu.version:
            raise RuntimeError("MCU 运行版本不符，停止后续步骤")
        emit("log", f"MCU 实际版本已确认：{mcu.version}")

    if mcu_first:
        flash_mcu()
    if motor:
        device.protocol("Private")
        before = device.live_motor_version()
        if confirm_change(ask, "电机", before, motor):
            emit("stage", "正在刷写电机：不要断开电源或 USB")
            device.flash_motor(motor)
            # Keep the mandatory physical cycle after an actual transfer.
            device.protocol("Private", force_cycle=True)
            actual = device.live_motor_version()
        else:
            emit("log", f"跳过同版本电机：{before}；仍需恢复 MIT 协议")
            actual = before
        if not actual or version_key(actual) != version_key(motor.version):
            raise RuntimeError(
                f"电机实时版本核验失败：目标 {motor.version}，读回 {actual}；未写入版本记录"
            )
        device.record_motor(actual)
        emit("log", f"电机实时版本已确认：{actual}")
    if not mcu_first:
        flash_mcu()
    device.protocol("Mit")
    device.check_homing()
    if not ask(
        "确认机械标定",
        "请确认本次上电标定先闭合再张开、无异常撞击或卡滞。\n方向正确且活动正常吗？\n选择否将报告需要技术支持，不会自动改变方向。",
    ):
        raise RuntimeError(
            "固件可能已写入，但机械标定未确认。不要运行夹取，请导出日志联系支持。"
        )
    emit(
        "success",
        "所选固件已逐项核验，电机已恢复 MIT，用户已确认标定。回滚设备请先完成对比验收，再恢复正常使用。",
    )

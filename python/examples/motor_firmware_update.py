#!/usr/bin/env python3
"""Motor-only guided OTA; manual 24 V power cycles remain mandatory."""

import argparse
from pathlib import Path

import _target
import full_ota_update as flow
import motor_ota_update as motor_ota
from xense.taccap import FollowerGripper, MotorProtocol


def check_version_policy(current, target, allow_downgrade):
    if current is None:
        raise RuntimeError("未读到实时电机版本，拒绝刷写（Flash 存档不能代替）")
    if motor_ota.version_bytes(target) < motor_ota.version_bytes(current):
        if not allow_downgrade:
            raise RuntimeError("目标版本较旧；对比测试需显式传 --allow-downgrade")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("image", help="指定型号和版本的电机 .bin 文件")
    _target.add_target_argument(p)
    p.add_argument("--target-version", required=True, help="例如 1.0.5.0.2")
    p.add_argument("--allow-downgrade", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--power-cycle-timeout", type=float, default=180)
    return p


def run(args):
    if args.power_cycle_timeout < 10:
        raise ValueError("断电等待超时至少为 10 秒")
    data = Path(args.image).read_bytes()
    model, version = motor_ota.image_facts(args.image, data, None)
    target = motor_ota.version_text(motor_ota.version_bytes(args.target_version))
    if version != target:
        raise ValueError(f"镜像版本 {version} 与指定版本 {target} 不一致")
    # Require an embedded match for this explicit-version workflow.
    if target.encode() not in data:
        raise ValueError("镜像没有指定版本的内嵌标识，拒绝仅凭文件名刷写")
    ep, _, _ = _target.resolve_target(args.target)
    if not ep.firmware_sn.endswith("s"):
        raise RuntimeError("仅支持从爪电机")
    identity = flow._identity(ep)
    with FollowerGripper(mcu_device=ep.mcu_device) as g:
        if flow._version_tuple(g.firmware_version) < (1, 2, 14):
            raise RuntimeError("此引导脚本要求从爪固件 >= 1.2.14")
        actual_model = g.motor.get_model()
        if actual_model.name != model or not actual_model.from_flash:
            raise RuntimeError("已记录的电机型号与镜像不符；不会自动覆盖型号")
        print(f"目标: {identity[0]} / {identity[1]}；电机: {model} -> {target}")
        print("只刷电机，不刷 MCU、不修改方向/力矩。需要手动断开 24 V。")
        if args.dry_run:
            print("dry-run: 未切换协议、未读取实时电机版本、未写入设备")
            return 0
        if model == "EL05" and motor_ota.version_bytes(target) < (10, 5, 0, 4):
            print(
                "警告：低于项目 EL05 支持下限；旧版存在速度反馈异常，仅用于对比测试。"
            )
            if not args.allow_downgrade:
                raise RuntimeError("低于支持下限需要 --allow-downgrade")
        if (
            input(f"清空夹爪活动范围，输入完整 SN {identity[0]} 继续: ").strip()
            != identity[0]
        ):
            raise RuntimeError("已取消，未写入设备")
        flow._prepare_motor_admin(g)
        switch = g.motor.get_protocol() != MotorProtocol.Private
        if switch:
            flow._switch_protocol(g, MotorProtocol.Private)
    if switch:
        ep = flow._wait_power_cycle(
            identity, args.power_cycle_timeout, "应用 Private 协议"
        )
    with FollowerGripper(mcu_device=ep.mcu_device) as g:
        flow._wait_motor_protocol(g.motor, MotorProtocol.Private)
        before = motor_ota.read_version(g.motor, 10)
    print(f"实时版本: {before} -> {target}")
    check_version_policy(before, target, args.allow_downgrade)
    answer = input("确认刷写电机？（同版本也会重刷）输入 yes: ").strip()
    if answer != "yes":
        raise RuntimeError("已取消刷写；电机可能仍在 Private，未自动恢复运动")
    flow._flash_motor(
        ep, args.image, data, model, target, reflash=True, no_progress=False
    )
    actual = flow._verify_motor_version_after_flash(
        identity, args.power_cycle_timeout, target
    )
    ep = flow._wait_ready(identity, args.power_cycle_timeout, "记录已核验的电机版本")
    with FollowerGripper(mcu_device=ep.mcu_device) as g:
        g.motor.set_motor_fw_version(actual)
    ep = flow._switch_back_to_mit(identity, args.power_cycle_timeout, "keep")
    with FollowerGripper(mcu_device=ep.mcu_device) as g:
        flow._wait_motor_protocol(g.motor, MotorProtocol.Mit)
    print(f"完成：电机实时核验 {before} -> {actual}，已记录并恢复 MIT。")
    print("未验证夹取性能或标定端点；请观察标定方向后再进行对比测试。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(run(parser().parse_args()))
    except (Exception, KeyboardInterrupt) as exc:
        print(f"停止: {exc}\n未自动重试刷写；可能仍为 Private，请勿直接启动夹取。")
        raise SystemExit(1)

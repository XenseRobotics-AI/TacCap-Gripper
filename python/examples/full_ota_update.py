#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""One-command follower MCU + RobStride motor firmware upgrade.

The command owns the ordering that is easy to get wrong when the two existing
OTA examples are run by hand:

1. put the motor on MIT before the follower MCU OTA;
2. flash the released follower image and verify its compiled-in version;
3. require a real 24 V power cycle;
4. record the motor model, its startup torque range, and switch to PRIVATE;
5. flash and verify the motor image;
6. switch back to MIT, power-cycle, and validate the final auto-calibration.

It cannot switch the 24 V rail itself.  At every required cycle it prints one
instruction and waits for the selected MCU's uptime to restart; no
second shell command is needed.

Typical RS00 invocation::

    python python/examples/full_ota_update.py rs00-0.0.3.32.bin TCGU01A28Z0085s

The follower image defaults to the released slave image named by
``firmware/manifest.json``.  The motor image is deliberately explicit because
RobStride images are not distributed in this repository and their bootloader
does not reject an image for the wrong motor model.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _target  # noqa: E402
import motor_ota_update  # noqa: E402
import ota_update  # noqa: E402
from xense.taccap import (  # noqa: E402
    FollowerGripper,
    LeaderGripper,
    MotorOtaSession,
    MotorProtocol,
    ProtocolError,
    log,
)

MODEL_IDS = {"EL05": 0, "RS00": 1}
MODEL_STARTUP_LIMITS = {"EL05": 6.0, "RS00": 14.0}
MIN_MOTOR_OTA_FOLLOWER = (1, 2, 8)


def _version_tuple(version) -> tuple[int, int, int]:
    return int(version.major), int(version.minor), int(version.patch)


def _parse_version_text(version: str) -> tuple[int, int, int]:
    parts = version.split(".")
    if len(parts) != 3:
        raise ValueError(f"expected MAJOR.MINOR.PATCH, got {version!r}")
    return tuple(int(part) for part in parts)


def _version_text(version: tuple[int, int, int]) -> str:
    return ".".join(str(part) for part in version)


def _should_flash_follower(
    current: tuple[int, int, int],
    target: tuple[int, int, int],
    *,
    reflash: bool,
    allow_downgrade: bool,
) -> bool:
    if current > target and not allow_downgrade:
        raise ValueError(
            f"refusing follower downgrade {_version_text(current)} -> "
            f"{_version_text(target)}; pass --allow-downgrade only if intentional"
        )
    return current != target or reflash


def _apply_direction(cfg, direction: str) -> bool:
    """Patch only GripperConfig.Reverse; preserve travel and the envelope."""
    if direction == "keep":
        return False
    before = int(cfg.flags)
    if direction == "negative":
        cfg.flags |= 0x0002
    else:
        cfg.flags &= ~0x0002
    return int(cfg.flags) != before


class _RestartDetector:
    """Observe MCU clock reset, not physical proof of a 24 V power cycle."""

    def __init__(self, uptime_ms: int) -> None:
        self.uptime_ms = uptime_ms
        self.reset_from = None

    def observe(self, uptime_ms: int) -> bool:
        previous = self.uptime_ms
        self.uptime_ms = uptime_ms
        # Never compare HAL ticks to host wall time: ticks may advance slowly
        # or pause. Require an observed rollback, then an advancing low reading.
        if self.reset_from is not None:
            if previous < uptime_ms < self.reset_from - 1000:
                return True
            self.reset_from = None
        forward_delta = (uptime_ms - previous) % (1 << 32)
        if uptime_ms + 1000 < previous and forward_delta > (1 << 31):
            self.reset_from = previous
        return False


def _identity(ep) -> tuple[str, str]:
    return ep.firmware_sn, ep.mcu_serial


def _find_endpoint(identity, endpoints):
    firmware_sn, mcu_serial = identity
    return next(
        (
            ep
            for ep in endpoints
            if ep.firmware_sn == firmware_sn and ep.mcu_serial == mcu_serial
        ),
        None,
    )


def _scan_for(identity):
    return _find_endpoint(identity, _target.scan_grippers())


def _wait_ready(identity, timeout_s: float, what: str):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            ep = _scan_for(identity)
            if ep is not None:
                return ep
        except Exception as exc:  # USB is expected to be unstable during reboot
            last_error = exc
        time.sleep(0.5)
    detail = f"; last scan error: {last_error}" if last_error is not None else ""
    raise RuntimeError(f"timed out waiting for {what}{detail}")


def _read_uptime(ep, identity):
    with LeaderGripper(mcu_device=ep.mcu_device) as gripper:
        if gripper.device.get_sn() != identity[0]:
            raise RuntimeError("心跳检查时设备 SN 已变化，拒绝继续")
        return int(gripper.device.heartbeat().uptime_ms)


def _wait_power_cycle(identity, timeout_s: float, reason: str):
    firmware_sn, _mcu_serial = identity
    ep = _wait_ready(identity, timeout_s, "the follower before power cycling")
    before = _read_uptime(ep, identity)
    detector = _RestartDetector(before)
    print()
    print(f"[需要断电] {reason}")
    print("  现在拔掉这只从爪的 24 V，保持至少 2 秒，再插回。")
    print("  USB 保持连接；仅拔 USB 不算断电。", flush=True)
    print(f"  MCU uptime 基线 {before} ms；等待心跳确认 MCU 重新计时。", flush=True)

    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            ep = _scan_for(identity)
            if ep is not None:
                current = _read_uptime(ep, identity)
                if detector.observe(current):
                    print(f"  {firmware_sn} MCU 重启已确认，uptime={current} ms。")
                    return ep
            else:
                detector.reset_from = None
        except Exception as exc:  # an unplug can interrupt an in-flight probe
            last_error = exc
            detector.reset_from = None
        time.sleep(1.0)

    detail = f"; last scan error: {last_error}" if last_error is not None else ""
    raise RuntimeError(
        f"未确认 {firmware_sn} MCU 重启 ({timeout_s:.0f}s)。"
        f"未读到可靠的 uptime 回退；基线={before} ms，最后={detector.uptime_ms} ms。"
        f"请按提示断开整只从爪的 24 V，再接回；仅拔 USB 不够{detail}"
    )


def _wait_motor_protocol(motor, expected, timeout_s: float = 20.0):
    """Allow boot-time discovery to settle; never resend a switch or bypass OTA checks."""
    deadline = time.monotonic() + timeout_s
    consecutive = 0
    last = None
    last_error = None
    print(f"[协议校验] 等待 {expected} 连续两次读回一致……", flush=True)
    while time.monotonic() < deadline:
        try:
            last = motor.get_protocol()
            last_error = None
            consecutive = consecutive + 1 if last == expected else 0
            if consecutive >= 2:
                return last
        except Exception as exc:
            last_error = exc
            consecutive = 0
        time.sleep(0.5)
    detail = f"; last error={last_error}" if last_error is not None else ""
    raise RuntimeError(
        f"等待电机协议 {expected} 稳定超时，最后读回 {last}{detail}。"
        "停止后续步骤；确认断开的是 24 V（仅拔 USB 不够），"
        "保持至少 2 秒，再重新运行。若仍失败，保留日志排查协议切换。"
    )


def _admin_call(gripper, name, operation, timeout_s: float = 45.0):
    """Wait for homing; retry only explicit SysBusy rejection of this command."""
    deadline = time.monotonic() + timeout_s
    last_detail = ""
    announced = False
    while time.monotonic() < deadline:
        try:
            # Old followers may not expose HomeDiag. Their command NACK
            # remains authoritative; do not require new diagnostics for MCU OTA.
            if _version_tuple(gripper.firmware_version) >= MIN_MOTOR_OTA_FOLLOWER:
                home = gripper.home_diag()
                if home.flags & 0x01:
                    last_detail = str(home)
                    if not announced:
                        print(f"[等待标定] {name}: {home}", flush=True)
                        announced = True
                    time.sleep(0.5)
                    continue
            return operation()
        except ProtocolError as exc:
            if not str(exc).endswith("NACK: SysBusy"):
                raise RuntimeError(f"{name} 失败: {exc}") from exc
            last_detail = str(exc)
            if not announced:
                print(f"[等待设备空闲] {name}: {exc}", flush=True)
                announced = True
            time.sleep(0.5)
    raise RuntimeError(
        f"{name} 等待设备空闲超时: {last_detail}；"
        "停止后续升级，请检查标定状态及是否有其他控制程序运行。"
    )


def _prepare_motor_admin(gripper):
    _admin_call(gripper, "清除电机故障", gripper.motor.clear_fault)
    _admin_call(gripper, "电机失能", gripper.motor.disable)


def _switch_protocol(gripper, protocol):
    _admin_call(
        gripper,
        f"切换电机协议到 {protocol}",
        lambda: gripper.motor.switch_protocol(protocol),
    )


def _resolve_follower_image(path: str | None):
    candidate = path or ota_update._default_firmware_for_role("slave")
    resolved = ota_update._resolve_or_report(candidate)
    if resolved is None:
        raise RuntimeError("follower image not found")
    with open(resolved, "rb") as stream:
        data = stream.read()
    role, meta = ota_update._identify_image(data)
    if role != "slave" or meta is None:
        raise RuntimeError(
            "the combined updater accepts only the released follower image "
            "identified as slave by firmware/manifest.json"
        )
    return resolved, data, meta


def _read_initial_state(ep):
    with FollowerGripper(
        mcu_device=ep.mcu_device,
        allow_outdated_firmware=True,
    ) as gripper:
        state = {
            "firmware": _version_tuple(gripper.firmware_version),
            "protocol": gripper.motor.get_protocol(),
            "model": None,
            "startup_limit": None,
            "config": None,
        }
        try:
            state["model"] = gripper.motor.get_model()
            state["startup_limit"] = gripper.motor.get_startup_limit_torque()
            state["config"] = gripper.get_gripper_config()
        except Exception as exc:
            # Old firmware may not expose the newer diagnostics.  It is being
            # upgraded before those values become prerequisites.
            state["detail_error"] = f"{type(exc).__name__}: {exc}"
        return state


def _confirm_plan(
    args, ep, initial, follower_path, follower_meta, motor_model, motor_ver
):
    print("=== 一键升级计划 ===")
    print(f"  target           : {ep.firmware_sn}")
    print(f"  CH343            : {ep.mcu_serial}")
    print(f"  device           : {ep.mcu_device}")
    print(f"  follower now     : {_version_text(initial['firmware'])}")
    print(f"  follower image   : {follower_path} ({follower_meta['version']})")
    print(f"  motor protocol   : {initial['protocol']}")
    print(f"  recorded model   : {initial['model'] or 'unavailable'}")
    print(f"  startup limit    : {initial['startup_limit']}")
    print(f"  motor image      : {args.motor_image}")
    print(f"  motor target     : {motor_model} {motor_ver or '(version unknown)'}")
    print(f"  open direction   : {args.direction}")
    print(f"  minimum travel   : {args.min_travel_rad:.3f} rad")
    if initial.get("detail_error"):
        print(f"  old-fw detail    : {initial['detail_error']}")
    print()
    print("过程按需刷写 MCU 与电机；版本相同默认跳过。需要时请手动断开 24 V。")
    if args.yes or args.dry_run:
        return
    try:
        answer = input(f"输入完整固件 SN {ep.firmware_sn} 继续: ").strip()
    except EOFError:
        answer = ""
    if answer != ep.firmware_sn:
        raise RuntimeError("确认失败，未写入设备")


def _ensure_mit_before_follower_ota(identity, timeout_s: float):
    ep = _wait_ready(identity, timeout_s, "the follower before MCU OTA")
    changed = False
    with FollowerGripper(
        mcu_device=ep.mcu_device,
        allow_outdated_firmware=True,
    ) as gripper:
        protocol = gripper.motor.get_protocol()
        _prepare_motor_admin(gripper)
        if protocol != MotorProtocol.Mit:
            print(f"[准备 MCU OTA] 电机协议 {protocol} -> MIT")
            _switch_protocol(gripper, MotorProtocol.Mit)
            changed = True
    if changed:
        _wait_power_cycle(identity, timeout_s, "让 MIT 协议在 MCU OTA 前生效")
        ep = _wait_ready(identity, timeout_s, "the follower on MIT")
        with FollowerGripper(
            mcu_device=ep.mcu_device,
            allow_outdated_firmware=True,
        ) as gripper:
            _wait_motor_protocol(gripper.motor, MotorProtocol.Mit)
            _prepare_motor_admin(gripper)


def _flash_follower(ep, image_path: str, no_progress: bool) -> None:
    update_args = SimpleNamespace(
        target_version=None,
        force=False,
        yes=True,
        no_progress=no_progress,
    )
    gripper = LeaderGripper(mcu_device=ep.mcu_device)
    try:
        rc = ota_update._cmd_update(update_args, gripper, ep, image_path)
        if rc != 0:
            raise RuntimeError(f"follower OTA failed with exit code {rc}")
    finally:
        try:
            gripper.close()
        except Exception:
            # OtaApply deliberately reboots underneath the transport.
            pass


def _wait_follower_version(identity, expected, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    last_version = None
    last_error = None
    while time.monotonic() < deadline:
        try:
            ep = _scan_for(identity)
            if ep is None:
                time.sleep(0.5)
                continue
            with LeaderGripper(mcu_device=ep.mcu_device) as gripper:
                last_version = _version_tuple(gripper.firmware_version)
            if last_version == expected:
                print(f"[MCU 校验] follower firmware {_version_text(last_version)}")
                return ep
        except Exception as exc:
            last_error = exc
        time.sleep(0.5)
    extra = f", last error={last_error}" if last_error is not None else ""
    raise RuntimeError(
        f"MCU rebooted but version did not become {_version_text(expected)}; "
        f"last={last_version}{extra}"
    )


def _configure_motor_ota_boot(gripper, model_name: str, direction: str):
    changes = []
    motor = gripper.motor
    _prepare_motor_admin(gripper)

    model = motor.get_model()
    if model.name != model_name or not model.from_flash:
        _admin_call(
            gripper, "设置电机型号", lambda: motor.set_model(MODEL_IDS[model_name])
        )
        changes.append(f"model={model_name}")

    desired_limit = MODEL_STARTUP_LIMITS[model_name]
    current_limit = _admin_call(gripper, "读取启动限矩", motor.get_startup_limit_torque)
    if abs(current_limit - desired_limit) > 0.01:
        _admin_call(
            gripper,
            "设置启动限矩",
            lambda: motor.set_startup_limit_torque(desired_limit),
        )
        changes.append(f"startup_limit={desired_limit:.1f}Nm")

    cfg = _admin_call(gripper, "读取行程配置", gripper.get_gripper_config)
    if _apply_direction(cfg, direction):
        _admin_call(gripper, "设置行程方向", lambda: gripper.set_gripper_config(cfg))
        changes.append(f"direction={direction}")

    if motor.get_protocol() != MotorProtocol.Private:
        _switch_protocol(gripper, MotorProtocol.Private)
        changes.append("protocol=Private")
    return changes


def _verify_private_boot(ep, model_name: str):
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        protocol = _wait_motor_protocol(gripper.motor, MotorProtocol.Private)
        model = gripper.motor.get_model()
        limit = gripper.motor.get_startup_limit_torque()
        print("[Private 预检]")
        print(f"  protocol         : {protocol}")
        print(f"  model            : {model}")
        print(f"  startup limit    : {limit}")
        if protocol != MotorProtocol.Private:
            raise RuntimeError("电机没有进入 Private，拒绝电机 OTA")
        if model.name != model_name or not model.from_flash:
            raise RuntimeError(
                f"记录型号不是 from-flash {model_name}，拒绝电机 OTA: {model}"
            )
        expected_limit = MODEL_STARTUP_LIMITS[model_name]
        if abs(limit - expected_limit) > 0.01:
            raise RuntimeError(
                f"启动限矩 {limit} 与 {model_name} 的 {expected_limit} 不一致"
            )


def _motor_progress(no_progress: bool):
    def progress(done, total):
        if no_progress:
            return
        pct = 100.0 * done / total
        bar = "#" * int(pct / 2.5)
        print(
            f"\r  [{bar:<40}] {pct:5.1f}%  {done:,}/{total:,}",
            end="",
            flush=True,
        )

    return progress


def _flash_motor(
    ep,
    image_path: str,
    image_data: bytes,
    model_name: str,
    image_version: str | None,
    *,
    reflash: bool,
    no_progress: bool,
):
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        motor = gripper.motor
        session = MotorOtaSession(motor, motor.get_can_id())
        session.preflight(model_name)
        before = motor_ota_update.read_version(motor, 5.0)
        uid = session.read_uid()

        print("=== motor OTA ===")
        print(f"  gripper      : {ep.firmware_sn}  firmware {gripper.firmware_version}")
        print(
            f"  motor        : {model_name}  can_id {motor.get_can_id()}  uid {uid.hex()}"
        )
        print(f"  motor fw now : {before or 'unknown'}")
        print(f"  image        : {image_path}")
        print(
            f"                 {len(image_data):,} B, "
            f"{(len(image_data) + 7) // 8:,} packs, "
            f"version {image_version or 'unknown'}"
        )

        if before and image_version and before == image_version and not reflash:
            print(f"  result       : 电机读回已是 {before}，本次未刷写")
            print("                 如需同版本重刷，使用 --reflash-motor")
            return False, before

        started = time.monotonic()
        session.update_from_bytes(image_data, _motor_progress(no_progress))
        if not no_progress:
            print()
        print(f"=== motor OTA done in {time.monotonic() - started:.1f}s ===")
        return True, before


def _verify_motor_version_after_flash(
    identity,
    timeout_s: float,
    expected_version: str | None,
):
    # The motor normally restarts on MIT.  Give it time to boot, then persist
    # PRIVATE and use the mandatory post-OTA power cycle as the apply step.
    time.sleep(3.0)
    ep = _wait_ready(identity, timeout_s, "the follower after motor OTA")
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        _prepare_motor_admin(gripper)
        if gripper.motor.get_protocol() != MotorProtocol.Private:
            _switch_protocol(gripper, MotorProtocol.Private)

    ep = _wait_power_cycle(
        identity, timeout_s, "应用电机 OTA 后的 Private 协议并校验版本"
    )
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        _wait_motor_protocol(gripper.motor, MotorProtocol.Private)
        actual = motor_ota_update.read_version(gripper.motor, 20.0)
        print(f"[电机版本校验] {actual or 'no answer'}")
        if expected_version and actual != expected_version:
            raise RuntimeError(
                f"motor version mismatch: expected {expected_version}, got {actual}"
            )
        return actual


def _switch_back_to_mit(identity, timeout_s: float):
    ep = _wait_ready(identity, timeout_s, "the follower before final MIT switch")
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        _prepare_motor_admin(gripper)
        if gripper.motor.get_protocol() != MotorProtocol.Mit:
            _switch_protocol(gripper, MotorProtocol.Mit)
    return _wait_power_cycle(identity, timeout_s, "切回 MIT 并执行最终自动标定")


def _final_validation(ep, model_name: str, direction: str, min_travel: float):
    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        protocol = _wait_motor_protocol(gripper.motor, MotorProtocol.Mit)
        deadline = time.monotonic() + 45.0
        home = gripper.home_diag()
        while home.flags & 0x01:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"等待自动标定结束超时: {home}")
            time.sleep(0.5)
            home = gripper.home_diag()

        model = gripper.motor.get_model()
        limit = gripper.motor.get_startup_limit_torque()
        cfg = gripper.get_gripper_config()
        auto = gripper.get_auto_cal_config()

        print("=== 最终校验 ===")
        print(f"  follower fw     : {gripper.firmware_version}")
        print(f"  protocol        : {protocol}")
        print(f"  model           : {model}")
        print(f"  startup limit   : {limit}")
        print(f"  gripper config  : {cfg}")
        print(f"  auto-cal config : {auto}")
        print(f"  home            : {home}")
        print(f"  open_sign       : {home.open_sign:+.1f}")
        print(f"  envelope        : {gripper.audit_envelope()}")

        errors = []
        if protocol != MotorProtocol.Mit:
            errors.append("motor protocol is not MIT")
        if model.name != model_name or not model.from_flash:
            errors.append(f"motor model is not from-flash {model_name}")
        expected_limit = MODEL_STARTUP_LIMITS[model_name]
        if abs(limit - expected_limit) > 0.01:
            errors.append(f"startup limit is {limit}, expected {expected_limit}")
        if not (auto.flags & 0x0002):
            errors.append("power-on auto-calibration is disabled")
        if home.fail_reason != 0 or not (home.flags & 0x02):
            errors.append(f"auto-calibration did not complete: {home}")
        if not (cfg.flags & 0x0001):
            errors.append("GripperConfig is not valid")
        if cfg.max_open_rad < min_travel:
            errors.append(
                f"calibrated travel {cfg.max_open_rad:.4f}rad is below "
                f"{min_travel:.4f}rad"
            )
        if direction != "keep":
            expected_reverse = direction == "negative"
            actual_reverse = bool(cfg.flags & 0x0002)
            expected_sign = -1.0 if expected_reverse else 1.0
            if actual_reverse != expected_reverse or home.open_sign != expected_sign:
                errors.append(
                    f"direction mismatch: reverse={actual_reverse}, "
                    f"open_sign={home.open_sign:+.1f}"
                )

        if errors:
            raise RuntimeError("final validation failed:\n  - " + "\n  - ".join(errors))

        print(f"  position        : {gripper.position():.4f}")
        return _version_text(_version_tuple(gripper.firmware_version))


def _print_result(follower_version, flashed_follower, motor_version, flashed_motor):
    print("\n=== 本次执行结果 ===")
    follower_action = "已刷写并校验" if flashed_follower else "版本相同，未刷写"
    motor_action = "已刷写并校验" if flashed_motor else "版本相同，未刷写"
    print(f"  从爪 MCU : {follower_version} — {follower_action}")
    print(f"  电机固件 : {motor_version or '未读到版本'} — {motor_action}")
    print("  电机已回到 MIT，标定检查通过。")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="一条命令升级从爪 MCU 与 RobStride 电机固件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("motor_image", help="RobStride motor firmware .bin")
    _target.add_target_argument(parser)
    parser.add_argument(
        "--follower-image",
        help="released slave image; default comes from firmware/manifest.json",
    )
    parser.add_argument(
        "--direction",
        choices=("keep", "positive", "negative"),
        default="keep",
        help="opening motor direction; default keeps the stored Reverse bit",
    )
    parser.add_argument(
        "--min-travel-rad",
        type=float,
        default=0.8,
        help="reject a false-success auto-calibration below this travel (default: 0.8)",
    )
    parser.add_argument(
        "--power-cycle-timeout",
        type=float,
        default=180.0,
        help="seconds to wait at each manual 24 V cycle (default: 180)",
    )
    parser.add_argument("--reflash-follower", action="store_true")
    parser.add_argument("--reflash-motor", action="store_true")
    parser.add_argument("--allow-downgrade", action="store_true")
    parser.add_argument("--no-progress", action="store_true")
    parser.add_argument("--yes", "-y", action="store_true", help="skip SN confirmation")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="resolve images and print the device plan without writing",
    )
    return parser


def run(args) -> int:
    if args.min_travel_rad <= 0.05:
        raise RuntimeError(
            "--min-travel-rad must be greater than firmware's 0.05rad floor"
        )
    if args.power_cycle_timeout < 10.0:
        raise RuntimeError("--power-cycle-timeout must be at least 10 seconds")

    if not os.path.isfile(args.motor_image):
        raise RuntimeError(f"motor image not found: {args.motor_image}")
    with open(args.motor_image, "rb") as stream:
        motor_data = stream.read()
    motor_model, motor_version = motor_ota_update.image_facts(
        args.motor_image, motor_data, None
    )

    follower_path, _follower_data, follower_meta = _resolve_follower_image(
        args.follower_image
    )
    follower_target = _parse_version_text(follower_meta["version"])

    ep, _by_side, _all = _target.resolve_target(args.target)
    if not ep.firmware_sn.endswith("s"):
        raise RuntimeError(
            f"{ep.firmware_sn} is not a follower (firmware SN must end in 's')"
        )
    identity = _identity(ep)
    initial = _read_initial_state(ep)
    flash_follower = _should_flash_follower(
        initial["firmware"],
        follower_target,
        reflash=args.reflash_follower,
        allow_downgrade=args.allow_downgrade,
    )

    _confirm_plan(
        args,
        ep,
        initial,
        follower_path,
        follower_meta,
        motor_model,
        motor_version,
    )
    if args.dry_run:
        print("dry-run: preflight complete; no device state was changed")
        return 0

    if flash_follower:
        _ensure_mit_before_follower_ota(identity, args.power_cycle_timeout)
        ep = _wait_ready(identity, args.power_cycle_timeout, "the follower for MCU OTA")
        _flash_follower(ep, follower_path, args.no_progress)
        _wait_follower_version(identity, follower_target, 30.0)
        ep = _wait_power_cycle(
            identity,
            args.power_cycle_timeout,
            "完成 MCU OTA 后的强制硬重启",
        )
    else:
        print(
            f"[MCU OTA] 已是 {_version_text(follower_target)}；"
            "使用 --reflash-follower 可强制重刷。"
        )
        ep = _wait_ready(identity, args.power_cycle_timeout, "the current follower")

    if follower_target < MIN_MOTOR_OTA_FOLLOWER:
        raise RuntimeError(
            f"follower target {_version_text(follower_target)} lacks motor OTA relay; "
            f"need >= {_version_text(MIN_MOTOR_OTA_FOLLOWER)}"
        )

    with FollowerGripper(mcu_device=ep.mcu_device) as gripper:
        changes = _configure_motor_ota_boot(gripper, motor_model, args.direction)
    if changes:
        print("[电机 OTA 启动配置] " + ", ".join(changes))
        ep = _wait_power_cycle(
            identity,
            args.power_cycle_timeout,
            "应用电机型号、启动限矩和 Private 协议",
        )
    else:
        print("[电机 OTA 启动配置] 已满足，无需额外写入")

    _verify_private_boot(ep, motor_model)
    flashed_motor, before_motor = _flash_motor(
        ep,
        args.motor_image,
        motor_data,
        motor_model,
        motor_version,
        reflash=args.reflash_motor,
        no_progress=args.no_progress,
    )

    actual_motor = before_motor
    if flashed_motor:
        actual_motor = _verify_motor_version_after_flash(
            identity,
            args.power_cycle_timeout,
            motor_version,
        )
        print(f"[电机 OTA] {before_motor or '?'} -> {actual_motor or '?'}")

    ep = _switch_back_to_mit(identity, args.power_cycle_timeout)
    actual_follower = _final_validation(
        ep, motor_model, args.direction, args.min_travel_rad
    )
    _print_result(actual_follower, flash_follower, actual_motor, flashed_motor)
    return 0


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    log.set_level("info")
    try:
        return run(args)
    except KeyboardInterrupt:
        print("\n已中止；重新运行会从设备当前状态重新预检。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\n[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

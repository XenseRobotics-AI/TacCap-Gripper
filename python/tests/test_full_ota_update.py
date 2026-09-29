import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

SPEC = importlib.util.spec_from_file_location(
    "full_ota_update_example",
    EXAMPLES / "full_ota_update.py",
)
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)


def test_image_only_defaults_to_positive_direction():
    args = mod._build_parser().parse_args(["firmware/motor/rs00-0.0.3.32.bin"])
    assert args.direction == "positive"
    assert not args.direction_only


@pytest.mark.parametrize("direction,flags", [("positive", 1), ("negative", 3)])
def test_write_direction_reads_back_and_preserves_travel(monkeypatch, direction, flags):
    g = Mock()
    cfg = SimpleNamespace(flags=flags ^ 2, max_open_rad=1.2)
    g.get_gripper_config.return_value = cfg
    monkeypatch.setattr(mod, "_prepare_motor_admin", Mock())
    monkeypatch.setattr(mod, "_admin_call", lambda g, label, fn: fn())
    mod._write_direction(g, direction)
    assert cfg.flags == flags
    assert cfg.max_open_rad == 1.2
    g.set_gripper_config.assert_called_once_with(cfg)


def test_write_direction_failed_readback_stops(monkeypatch):
    g = Mock()
    g.get_gripper_config.side_effect = [
        SimpleNamespace(flags=1),
        SimpleNamespace(flags=1),
    ]
    monkeypatch.setattr(mod, "_prepare_motor_admin", Mock())
    monkeypatch.setattr(mod, "_admin_call", lambda g, label, fn: fn())
    with pytest.raises(RuntimeError, match="读回不一致"):
        mod._write_direction(g, "negative")


@pytest.mark.parametrize("dry_run", [True, False])
def test_direction_only_never_flashes(model_setup, monkeypatch, dry_run):
    state, events, g, ep = model_setup
    state.name, state.recorded = "RS00", True
    ep.firmware_sn, ep.mcu_serial = "TESTs", "USB"
    g.motor.get_protocol.return_value = mod.MotorProtocol.Mit
    g.get_gripper_config.return_value = SimpleNamespace(flags=1)
    monkeypatch.setattr(mod._target, "resolve_target", lambda _: (ep, None, None))
    write = Mock()
    validate = Mock()
    monkeypatch.setattr(mod, "_write_direction", write)
    monkeypatch.setattr(mod, "_final_validation", validate)
    args = mod._build_parser().parse_args(
        ["unused.bin", "TESTs", "--direction-only", "--direction", "negative", "--yes"]
    )
    args.dry_run = dry_run
    assert mod._direction_only(args, "RS00") == 0
    assert write.call_count == int(not dry_run)
    assert validate.call_count == int(not dry_run)
    assert state.cycles == int(not dry_run)
    g.motor.switch_protocol.assert_not_called()
    g.motor.set_model.assert_not_called()


def test_direction_only_requires_explicit_direction():
    args = mod._build_parser().parse_args(
        ["unused.bin", "--direction-only", "--direction", "keep"]
    )
    with pytest.raises(RuntimeError, match="必须指定"):
        mod._direction_only(args, "RS00")


@pytest.fixture
def model_setup(monkeypatch):
    state = SimpleNamespace(
        name="EL05",
        recorded=False,
        flags=3,
        torque=0.35,
        active=True,
        pending=None,
        cycles=0,
    )
    events = []
    ep = SimpleNamespace(mcu_device="mock-only")
    g = Mock()
    g.__enter__ = Mock(return_value=g)
    g.__exit__ = Mock(return_value=False)
    g.firmware_version = SimpleNamespace(major=1, minor=2, patch=12)
    g.motor.get_model.side_effect = lambda: SimpleNamespace(
        name=state.name, from_flash=state.recorded
    )
    g.get_auto_cal_config.side_effect = lambda: SimpleNamespace(
        flags=state.flags, close_torque=state.torque
    )
    g.home_diag.side_effect = lambda: SimpleNamespace(flags=int(state.active))

    def save_auto(cfg):
        state.flags = cfg.flags
        state.torque = cfg.close_torque
        events.append(("auto", cfg.flags, cfg.close_torque))

    def save_model(model_id):
        assert not state.active
        assert not state.flags & 2
        state.pending = model_id
        events.append(("model", model_id))

    def cycle(*args):
        state.cycles += 1
        events.append(("cycle", state.cycles))
        state.active = False
        if state.pending is not None:
            state.name = "RS00"
            state.recorded = True
            state.pending = None
            state.torque = 1.8  # New model defaults must survive restoration.
        return ep

    g.set_auto_cal_config.side_effect = save_auto
    g.motor.set_model.side_effect = save_model
    monkeypatch.setattr(mod, "FollowerGripper", Mock(return_value=g))
    monkeypatch.setattr(mod, "_wait_ready", Mock(return_value=ep))
    monkeypatch.setattr(mod, "_wait_power_cycle", Mock(side_effect=cycle))
    monkeypatch.setattr("builtins.input", lambda _: "RS00")
    return state, events, g, ep


def test_model_bootstrap_stops_wrong_model_homing_before_write(model_setup):
    state, events, g, ep = model_setup
    assert mod._prepare_recorded_model(None, "RS00", 10) is ep
    assert events == [
        ("auto", 1, 0.35),
        ("cycle", 1),
        ("model", 1),
        ("cycle", 2),
        ("auto", 3, 1.8),
    ]
    g.motor.clear_fault.assert_not_called()
    g.motor.disable.assert_not_called()
    g.motor.switch_protocol.assert_not_called()


def test_correct_model_bootstrap_is_read_only(model_setup, monkeypatch):
    state, events, g, ep = model_setup
    state.name, state.recorded = "RS00", True
    monkeypatch.setattr("builtins.input", Mock(side_effect=AssertionError))
    assert mod._prepare_recorded_model(None, "RS00", 10) is ep
    assert events == []


def test_model_bootstrap_requires_physical_confirmation(model_setup, monkeypatch):
    state, events, g, ep = model_setup
    monkeypatch.setattr("builtins.input", lambda _: "y")
    with pytest.raises(RuntimeError, match="未确认实物型号"):
        mod._prepare_recorded_model(None, "RS00", 10)
    assert events == []


def test_model_bootstrap_refuses_failed_disable_readback(model_setup):
    state, events, g, ep = model_setup
    g.set_auto_cal_config.side_effect = None
    with pytest.raises(RuntimeError, match="禁用自动标定未读回"):
        mod._prepare_recorded_model(None, "RS00", 10)
    g.motor.set_model.assert_not_called()
    mod._wait_power_cycle.assert_not_called()


def test_model_bootstrap_refuses_active_after_restart(model_setup, monkeypatch):
    state, events, g, ep = model_setup
    monkeypatch.setattr(mod, "_wait_power_cycle", lambda *args: ep)
    with pytest.raises(RuntimeError, match="重启后自动标定仍"):
        mod._prepare_recorded_model(None, "RS00", 10)
    g.motor.set_model.assert_not_called()
    assert not state.flags & 2


def test_model_bootstrap_failure_does_not_restore_old_config(model_setup, capsys):
    state, events, g, ep = model_setup
    cycle = mod._wait_power_cycle.side_effect

    def fail_second(*args):
        if state.cycles == 1:
            raise TimeoutError("no reboot")
        return cycle(*args)

    mod._wait_power_cycle.side_effect = fail_second
    with pytest.raises(TimeoutError):
        mod._prepare_recorded_model(None, "RS00", 10)
    assert not state.flags & 2
    assert "自动标定可能仍被禁用" in capsys.readouterr().err


def test_model_bootstrap_resumes_disabled_matching_model(model_setup):
    state, events, g, ep = model_setup
    state.name, state.recorded, state.flags = "RS00", True, 1
    mod._prepare_recorded_model(None, "RS00", 10)
    assert events == [("cycle", 1), ("auto", 3, 0.35)]
    g.motor.set_model.assert_not_called()


class FakeConfig:
    def __init__(self, flags):
        self.flags = flags


class FakeEndpoint:
    def __init__(self, firmware_sn, mcu_serial):
        self.firmware_sn = firmware_sn
        self.mcu_serial = mcu_serial


def test_usb_reconnect_with_continuous_uptime_is_not_a_restart():
    detector = mod._RestartDetector(100_000)
    assert not detector.observe(110_000)
    assert not detector.observe(111_000)


def test_clock_reset_detected_even_without_observed_port_disappearance():
    detector = mod._RestartDetector(100_000)
    assert not detector.observe(1000)
    assert detector.observe(2000)


def test_early_boot_without_observed_rollback_cannot_prove_restart():
    detector = mod._RestartDetector(1000)
    assert not detector.observe(3000)
    assert not detector.observe(4000)


def test_uptime_wrap_is_not_a_restart():
    detector = mod._RestartDetector((1 << 32) - 1000)
    assert not detector.observe(1000)
    assert not detector.observe(2000)


@pytest.mark.parametrize(
    ("baseline", "samples"),
    [
        (84691, (84750, 85000, 85445)),
        (1051, (1100, 1600, 2057)),
        (4717, (4800, 5100, 5438)),
    ],
)
def test_reported_slow_uptime_sequence_does_not_confirm_restart(baseline, samples):
    detector = mod._RestartDetector(baseline)
    # Log baselines/endpoints are real; intermediate samples model slow ticks.
    for uptime in samples:
        assert not detector.observe(uptime)


def test_frozen_clock_does_not_confirm_restart():
    detector = mod._RestartDetector(84691)
    for _ in range(10):
        assert not detector.observe(84691)


def test_single_stale_low_sample_does_not_confirm_restart():
    detector = mod._RestartDetector(84691)
    assert not detector.observe(1000)
    assert not detector.observe(85445)
    assert not detector.observe(86000)


def test_repeated_low_sample_does_not_confirm_restart():
    detector = mod._RestartDetector(84691)
    assert not detector.observe(1000)
    assert not detector.observe(1000)


def test_later_rollback_uses_last_observed_uptime():
    detector = mod._RestartDetector(1000)
    assert not detector.observe(10_000)
    assert not detector.observe(500)
    assert detector.observe(1500)


def test_follower_same_version_skips_unless_reflash_requested():
    current = (1, 2, 11)
    assert not mod._should_flash_follower(
        current, current, reflash=False, allow_downgrade=False
    )
    assert mod._should_flash_follower(
        current, current, reflash=True, allow_downgrade=False
    )


def test_follower_downgrade_is_blocked_by_default():
    with pytest.raises(ValueError, match="refusing follower downgrade"):
        mod._should_flash_follower(
            (1, 2, 12),
            (1, 2, 11),
            reflash=False,
            allow_downgrade=False,
        )

    assert mod._should_flash_follower(
        (1, 2, 12),
        (1, 2, 11),
        reflash=False,
        allow_downgrade=True,
    )


def test_direction_patch_only_changes_reverse_bit():
    cfg = FakeConfig(0x00A3)
    assert mod._apply_direction(cfg, "positive")
    assert cfg.flags == 0x00A1

    assert mod._apply_direction(cfg, "negative")
    assert cfg.flags == 0x00A3

    assert not mod._apply_direction(cfg, "keep")
    assert cfg.flags == 0x00A3


def test_endpoint_identity_requires_both_firmware_and_usb_serial():
    expected = ("TCGU01A28Z0088s", "5C96086009")
    endpoints = [
        FakeEndpoint("TCGU01A28Z0088s", "wrong-usb"),
        FakeEndpoint("wrong-firmware", "5C96086009"),
        FakeEndpoint(*expected),
    ]

    assert mod._find_endpoint(expected, endpoints) is endpoints[-1]


@pytest.mark.parametrize(
    ("answer", "force", "dry_run", "yes", "decision"),
    [
        ("y", False, False, False, "FLASH"),
        ("n", False, False, False, "SKIP"),
        ("", False, False, False, "SKIP"),
        ("n", True, False, False, "FLASH"),
        ("y", False, True, False, None),
        ("n", True, True, False, None),
        ("n", False, False, True, "SKIP"),
    ],
)
def test_follower_reflash_confirmation_and_dry_run(
    monkeypatch, tmp_path, answer, force, dry_run, yes, decision
):
    image = tmp_path / "rs00-0.0.3.32.bin"
    image.write_bytes(b"test image")
    args = mod._build_parser().parse_args([str(image)])
    args.reflash_follower = force
    args.dry_run = dry_run
    args.yes = yes
    ep = FakeEndpoint("TCGU01A28Z0088s", "USB")
    monkeypatch.setattr(mod, "_prepare_recorded_model", lambda *_: ep)
    monkeypatch.setattr(mod._target, "resolve_target", lambda *_: (ep, {}, [ep]))
    monkeypatch.setattr(mod, "_read_initial_state", lambda *_: {"firmware": (1, 2, 12)})
    monkeypatch.setattr(
        mod,
        "_resolve_follower_image",
        lambda *_: ("fake.bin", b"image", {"version": "1.2.12"}),
    )
    monkeypatch.setattr(mod, "_confirm_plan", Mock())
    flash_stage = Mock(side_effect=RuntimeError("FLASH"))
    skip_stage = Mock(side_effect=RuntimeError("SKIP"))
    monkeypatch.setattr(mod, "_ensure_mit_before_follower_ota", flash_stage)
    monkeypatch.setattr(mod, "_wait_ready", skip_stage)
    prompt = Mock(return_value=answer)
    monkeypatch.setattr("builtins.input", prompt)
    if dry_run:
        assert mod.run(args) == 0
        flash_stage.assert_not_called()
        skip_stage.assert_not_called()
    else:
        with pytest.raises(RuntimeError, match=decision):
            mod.run(args)
        assert flash_stage.call_count == int(decision == "FLASH")
        assert skip_stage.call_count == int(decision == "SKIP")
    assert prompt.call_count == int(not force and not dry_run)


@pytest.fixture
def fake_clock(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(mod.time, "monotonic", lambda: now[0])

    def sleep(seconds):
        now[0] += seconds

    monkeypatch.setattr(mod.time, "sleep", sleep)
    return now


def test_protocol_wait_allows_boot_probe_to_settle(fake_clock):
    motor = Mock()
    mit, private = mod.MotorProtocol.Mit, mod.MotorProtocol.Private
    motor.get_protocol.side_effect = [mit, private, mit, private, private]
    assert mod._wait_motor_protocol(motor, private) == private
    assert motor.get_protocol.call_count == 5
    motor.switch_protocol.assert_not_called()


def test_protocol_wait_requires_consecutive_reads_across_errors(fake_clock):
    motor = Mock()
    private = mod.MotorProtocol.Private
    motor.get_protocol.side_effect = [
        private,
        RuntimeError("booting"),
        private,
        private,
    ]
    assert mod._wait_motor_protocol(motor, private) == private
    assert motor.get_protocol.call_count == 4


def test_private_preflight_stops_before_model_checks_if_protocol_never_changes(
    monkeypatch, fake_clock
):
    gripper = Mock()
    gripper.motor.get_protocol.return_value = mod.MotorProtocol.Mit
    context = Mock()
    context.__enter__ = Mock(return_value=gripper)
    context.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(mod, "FollowerGripper", Mock(return_value=context))
    with pytest.raises(RuntimeError, match="稳定超时"):
        mod._verify_private_boot(Mock(mcu_device="fake"), "RS00")
    gripper.motor.get_model.assert_not_called()
    gripper.motor.switch_protocol.assert_not_called()


def test_scanner_errors_do_not_count_as_power_loss(monkeypatch, fake_clock):
    ep = Mock()
    monkeypatch.setattr(mod, "_wait_ready", lambda *_: ep)
    monkeypatch.setattr(
        mod,
        "_read_uptime",
        lambda *_: 100_000 + int(fake_clock[0] * 1000),
    )
    scans = [RuntimeError("busy")] * 10 + [ep] * 20

    def scan(_identity):
        item = scans.pop(0) if scans else ep
        if isinstance(item, Exception):
            raise item
        return item

    monkeypatch.setattr(mod, "_scan_for", scan)
    with pytest.raises(RuntimeError, match="未确认.*MCU 重启"):
        mod._wait_power_cycle(("SN", "USB"), 5.0, "test")


@pytest.mark.parametrize(
    ("reflash", "answer", "expected"),
    [
        (False, "y", True),
        (False, "Y", True),
        (False, "", False),
        (False, "n", False),
        (False, "other", False),
        (False, None, False),
        (True, "n", True),
    ],
)
def test_same_motor_version_requires_confirmation_unless_forced(
    monkeypatch, reflash, answer, expected
):
    prompt = Mock(side_effect=EOFError) if answer is None else Mock(return_value=answer)
    monkeypatch.setattr("builtins.input", prompt)
    gripper = Mock()
    context = Mock()
    context.__enter__ = Mock(return_value=gripper)
    context.__exit__ = Mock(return_value=False)
    session = Mock()
    session.read_uid.return_value = b"12345678"
    monkeypatch.setattr(mod, "FollowerGripper", Mock(return_value=context))
    monkeypatch.setattr(mod, "MotorOtaSession", Mock(return_value=session))
    monkeypatch.setattr(mod.motor_ota_update, "read_version", lambda *_: "0.0.3.32")
    flashed, before = mod._flash_motor(
        Mock(),
        "rs00.bin",
        b"test-image",
        "RS00",
        "0.0.3.32",
        reflash=reflash,
        no_progress=True,
    )
    assert flashed is expected
    assert before == "0.0.3.32"
    session.preflight.assert_called_once_with("RS00")
    assert session.update_from_bytes.call_count == int(expected)
    assert prompt.call_count == int(not reflash)


@pytest.mark.parametrize("flashed", [False, True])
def test_summary_distinguishes_actual_flash_from_skip(capsys, flashed):
    mod._print_result("1.2.11", False, "0.0.3.32", flashed)
    output = capsys.readouterr().out
    assert "从爪 MCU : 1.2.11 — 版本相同，未刷写" in output
    action = "已刷写并校验" if flashed else "版本相同，未刷写"
    assert f"电机固件 : 0.0.3.32 — {action}" in output


def test_power_cycle_wait_accepts_reset_without_disappearance(monkeypatch, fake_clock):
    ep = Mock()
    samples = iter([100_000, 1000, 2000])
    monkeypatch.setattr(mod, "_wait_ready", lambda *_: ep)
    monkeypatch.setattr(mod, "_scan_for", lambda *_: ep)
    monkeypatch.setattr(mod, "_read_uptime", lambda *_: next(samples))
    assert mod._wait_power_cycle(("SN", "USB"), 5.0, "test") is ep


def test_power_cycle_wait_rejects_usb_only_reconnect(monkeypatch, fake_clock):
    ep = Mock()
    monkeypatch.setattr(mod, "_wait_ready", lambda *_: ep)
    monkeypatch.setattr(mod, "_scan_for", lambda *_: None if fake_clock[0] < 3 else ep)
    monkeypatch.setattr(
        mod,
        "_read_uptime",
        lambda *_: 100_000 + int(fake_clock[0] * 1000),
    )
    with pytest.raises(RuntimeError, match="仅拔 USB"):
        mod._wait_power_cycle(("SN", "USB"), 8.0, "test")


def admin_gripper():
    gripper = Mock()
    gripper.firmware_version = SimpleNamespace(major=1, minor=2, patch=11)
    gripper.home_diag.return_value = SimpleNamespace(flags=0x0E)
    return gripper


def test_admin_waits_for_homing_before_any_write(fake_clock):
    gripper = admin_gripper()
    gripper.home_diag.side_effect = [
        SimpleNamespace(flags=1),
        SimpleNamespace(flags=1),
        SimpleNamespace(flags=0x0E),
    ]
    operation = Mock(return_value="ok")
    assert mod._admin_call(gripper, "test", operation) == "ok"
    assert fake_clock[0] == 1.0
    operation.assert_called_once()


def test_active_homing_timeout_prevents_boot_configuration_writes(fake_clock):
    gripper = admin_gripper()
    gripper.motor.get_model.return_value = SimpleNamespace(name="RS00", from_flash=True)
    gripper.home_diag.return_value = SimpleNamespace(flags=1)
    with pytest.raises(RuntimeError, match="清除电机故障.*超时"):
        mod._configure_motor_ota_boot(gripper, "RS00", "positive")
    gripper.motor.clear_fault.assert_not_called()
    gripper.motor.set_model.assert_not_called()
    gripper.set_gripper_config.assert_not_called()


def test_admin_retries_only_explicit_busy(fake_clock):
    gripper = admin_gripper()
    operation = Mock(side_effect=[mod.ProtocolError("NACK: SysBusy"), "ok"])
    assert mod._admin_call(gripper, "test", operation) == "ok"
    assert operation.call_count == 2


@pytest.mark.parametrize(
    "error",
    [
        mod.ProtocolError("NACK: MotorFault"),
        TimeoutError("unknown outcome"),
    ],
)
def test_admin_does_not_retry_other_errors(fake_clock, error):
    operation = Mock(side_effect=error)
    with pytest.raises((RuntimeError, TimeoutError)):
        mod._admin_call(admin_gripper(), "test", operation)
    operation.assert_called_once()


def test_admin_busy_timeout_names_failed_step(fake_clock):
    operation = Mock(side_effect=mod.ProtocolError("NACK: SysBusy"))
    with pytest.raises(RuntimeError, match="切换协议.*超时"):
        mod._admin_call(admin_gripper(), "切换协议", operation, timeout_s=1.0)
    assert operation.call_count == 2


def test_old_follower_uses_busy_nack_without_requiring_home_diag(fake_clock):
    gripper = admin_gripper()
    gripper.firmware_version = SimpleNamespace(major=1, minor=1, patch=5)
    operation = Mock(side_effect=[mod.ProtocolError("NACK: SysBusy"), "ok"])
    assert mod._admin_call(gripper, "test", operation) == "ok"
    gripper.home_diag.assert_not_called()

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
    gripper.home_diag.return_value = SimpleNamespace(flags=1)
    with pytest.raises(RuntimeError, match="清除电机故障.*超时"):
        mod._configure_motor_ota_boot(gripper, "RS00", "positive")
    assert gripper.motor.mock_calls == []
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

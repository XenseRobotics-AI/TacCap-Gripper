import importlib.util
import sys
from pathlib import Path
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


def test_power_cycle_requires_sustained_down_and_stable_up_samples(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(mod.time, "monotonic", lambda: now[0])
    detector = mod._PowerCycleDetector()

    assert not detector.observe(True)
    assert not detector.observe(False)  # one scan miss is not a power cycle
    assert not detector.observe(True)
    assert not detector.saw_down

    assert not detector.observe(False)
    now[0] = 0.5
    assert not detector.observe(False)
    assert not detector.saw_down
    now[0] = 2.0
    assert not detector.observe(False)
    assert detector.saw_down
    assert not detector.observe(True)
    assert detector.observe(True)


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
    scans = [RuntimeError("busy")] * 10 + [ep] * 20

    def scan(_identity):
        item = scans.pop(0) if scans else ep
        if isinstance(item, Exception):
            raise item
        return item

    monkeypatch.setattr(mod, "_scan_for", scan)
    with pytest.raises(RuntimeError, match="断电重启超时"):
        mod._wait_power_cycle(("SN", "USB"), 5.0, "test")


@pytest.mark.parametrize("reflash", [False, True])
def test_same_motor_version_skips_bytes_unless_forced(monkeypatch, reflash):
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
    assert flashed is reflash
    assert before == "0.0.3.32"
    session.preflight.assert_called_once_with("RS00")
    assert session.update_from_bytes.call_count == int(reflash)


@pytest.mark.parametrize("flashed", [False, True])
def test_summary_distinguishes_actual_flash_from_skip(capsys, flashed):
    mod._print_result("1.2.11", False, "0.0.3.32", flashed)
    output = capsys.readouterr().out
    assert "从爪 MCU : 1.2.11 — 版本相同，未刷写" in output
    action = "已刷写并校验" if flashed else "版本相同，未刷写"
    assert f"电机固件 : 0.0.3.32 — {action}" in output

import importlib.util
import sys
from pathlib import Path

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


def test_power_cycle_requires_stable_down_and_up_samples():
    detector = mod._PowerCycleDetector()

    assert not detector.observe(True)
    assert not detector.observe(False)  # one scan miss is not a power cycle
    assert not detector.observe(True)
    assert not detector.saw_down

    assert not detector.observe(False)
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

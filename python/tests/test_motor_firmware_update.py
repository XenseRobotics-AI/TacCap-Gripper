import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
import motor_firmware_update as update
import motor_ota_update as ota


@pytest.mark.parametrize(
    "version,raw",
    [
        ("1.0.5.0.2", (10, 5, 0, 2)),
        ("1.0.5.0.4", (10, 5, 0, 4)),
        ("0.0.3.32", (0, 0, 3, 32)),
    ],
)
def test_version_roundtrip(version, raw):
    assert ota.version_bytes(version) == raw
    assert ota.version_text(raw) == version


def test_el05_image_version_not_truncated():
    assert ota.image_facts("EL05_1.0.5.0.2.bin", b"fw=1.0.5.0.2", None) == (
        "EL05",
        "1.0.5.0.2",
    )


def test_mismatched_image_rejected():
    with pytest.raises(SystemExit):
        ota.image_facts("EL05_1.0.5.0.2.bin", b"fw=1.0.5.0.4", None)


def test_downgrade_requires_permission():
    with pytest.raises(RuntimeError, match="allow-downgrade"):
        update.check_version_policy("1.0.5.0.4", "1.0.5.0.2", False)
    update.check_version_policy("1.0.5.0.4", "1.0.5.0.2", True)


def test_unknown_current_version_never_flashes():
    with pytest.raises(RuntimeError, match="实时"):
        update.check_version_policy(None, "1.0.5.0.2", True)


def test_live_version_formats_el05():
    motor = Mock()
    motor.motor_version.return_value = SimpleNamespace(
        valid=True, from_flash=False, version=(10, 5, 0, 4)
    )
    assert ota.read_version(motor) == "1.0.5.0.4"


def test_dry_run_never_writes(tmp_path, monkeypatch):
    image = tmp_path / "EL05_1.0.5.0.2.bin"
    image.write_bytes(b"1.0.5.0.2")
    ep = SimpleNamespace(firmware_sn="TESTs", mcu_serial="USB", mcu_device="mock")
    monkeypatch.setattr(update._target, "resolve_target", lambda _: (ep, None, None))
    g = Mock()
    g.__enter__ = Mock(return_value=g)
    g.__exit__ = Mock(return_value=False)
    g.firmware_version = SimpleNamespace(major=1, minor=2, patch=16)
    g.motor.get_model.return_value = SimpleNamespace(name="EL05", from_flash=True)
    monkeypatch.setattr(update, "FollowerGripper", lambda **kw: g)
    monkeypatch.setattr(update.flow, "_flash_motor", Mock(side_effect=AssertionError))
    monkeypatch.setattr(
        update.flow, "_prepare_motor_admin", Mock(side_effect=AssertionError)
    )
    args = update.parser().parse_args(
        [str(image), "--target-version", "1.0.5.0.2", "--dry-run"]
    )
    assert update.run(args) == 0
    g.motor.switch_protocol.assert_not_called()
    g.motor.motor_version.assert_not_called()

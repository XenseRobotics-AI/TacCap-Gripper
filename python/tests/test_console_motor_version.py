import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
sys.path.insert(0, str(EXAMPLES))
spec = importlib.util.spec_from_file_location(
    "console_version_test", EXAMPLES / "gripper_console.py"
)
console = importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)


@pytest.mark.parametrize(
    "vendor,stored,label",
    [
        ("1.0.5.0.2", False, "实时读取"),
        ("1.0.5.0.4", True, "Flash存档/非实时"),
        ("0.0.3.32", False, "实时读取"),
    ],
)
def test_version_label_and_source(vendor, stored, label):
    motor = Mock()
    motor.motor_version.return_value = SimpleNamespace(
        valid=True, from_flash=stored, vendor_str=vendor
    )
    assert console._motor_firmware_label(motor) == f"{vendor} [{label}]"
    motor.motor_version.assert_called_once_with(3000)
    assert len(motor.mock_calls) == 1


def test_invalid_version_not_displayed_as_real():
    motor = Mock()
    motor.motor_version.return_value = SimpleNamespace(valid=False)
    assert console._motor_firmware_label(motor).startswith("unknown")
    assert len(motor.mock_calls) == 1


def test_query_error_does_not_break_console():
    motor = Mock()
    motor.motor_version.side_effect = TimeoutError("serial timeout")
    assert "unknown (查询失败: TimeoutError)" == console._motor_firmware_label(motor)
    assert len(motor.mock_calls) == 1

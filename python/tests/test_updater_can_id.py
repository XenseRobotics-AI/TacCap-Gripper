"""CAN ID correction guards; never discover or open actual serial devices."""

import importlib.util
import sys
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import core  # noqa: E402
import version_status  # noqa: E402


@pytest.fixture
def rig():
    side = SimpleNamespace(name="Left")
    parsed = SimpleNamespace(role=SimpleNamespace(name="Follower"), side=side)
    sdk = SimpleNamespace(
        **{
            name: Mock()
            for name in (
                "FollowerGripper",
                "LeaderGripper",
                "MotorOtaSession",
                "MotorProtocol",
                "OtaTargetVersion",
                "scan_grippers",
            )
        },
        parse_serial=Mock(return_value=parsed),
        expected_motor_can_id=Mock(return_value=17),
    )
    flow = SimpleNamespace(_prepare_motor_admin=Mock())
    spec = importlib.util.spec_from_file_location("can_id_driver", APP / "device.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(
        sys.modules,
        {
            "xense.taccap": sdk,
            "full_ota_update": flow,
            "motor_ota_update": Mock(),
        },
    ):
        spec.loader.exec_module(module)
        device = module.Device(
            ("TCGU01A00Z0001s", "test-usb"), Mock(), Mock(return_value=True)
        )
        motor = Mock()
        motor.get_model.return_value = SimpleNamespace(
            name="RS00", id=1, from_flash=True
        )
        motor.get_can_id.side_effect = [127, 127, 17]
        gripper = SimpleNamespace(motor=motor)

        @contextmanager
        def opened():
            yield gripper

        device.opened = opened
        device.power_cycle = Mock()
        yield SimpleNamespace(
            device=device,
            motor=motor,
            flow=flow,
            sdk=sdk,
            parsed=parsed,
            gripper=gripper,
        )


@pytest.mark.parametrize("side,target", [("Left", 17), ("Right", 18)])
def test_rule_write_cycle_and_verify(rig, side, target):
    rig.parsed.side.name = side
    rig.sdk.expected_motor_can_id.return_value = target
    rig.motor.get_can_id.side_effect = [127, 127, target]
    rig.device.ensure_can_id()
    rig.sdk.parse_serial.assert_called_with(rig.device.identity[0])
    rig.sdk.expected_motor_can_id.assert_called_with(1, rig.parsed.side)
    rig.flow._prepare_motor_admin.assert_called_once_with(rig.gripper)
    rig.motor.set_can_id.assert_called_once_with(target)
    rig.device.power_cycle.assert_called_once()
    assert rig.motor.get_can_id.call_count == 3


def test_matching_id_is_read_only(rig):
    rig.motor.get_can_id.side_effect = None
    rig.motor.get_can_id.return_value = 17
    rig.device.ensure_can_id()
    rig.device.ask.assert_not_called()
    rig.motor.set_can_id.assert_not_called()
    rig.device.power_cycle.assert_not_called()
    rig.flow._prepare_motor_admin.assert_not_called()


@pytest.mark.parametrize(
    "case", ["unknown_side", "wrong_role", "unrecorded", "el05", "master"]
)
def test_invalid_identity_or_model_never_writes(rig, case):
    if case == "unknown_side":
        rig.parsed.side = None
    elif case == "wrong_role":
        rig.parsed.role.name = "Leader"
    elif case == "unrecorded":
        rig.motor.get_model.return_value.from_flash = False
    elif case == "el05":
        rig.motor.get_model.return_value.name = "EL05"
    else:
        rig.device.role = "master"
    with pytest.raises(RuntimeError):
        rig.device.ensure_can_id()
    rig.motor.set_can_id.assert_not_called()


def test_cancel_does_not_write(rig):
    rig.device.ask.return_value = False
    with pytest.raises(RuntimeError, match="用户取消"):
        rig.device.ensure_can_id()
    rig.motor.set_can_id.assert_not_called()


def test_state_changed_during_confirmation(rig):
    rig.motor.get_can_id.side_effect = [127, 18]
    with pytest.raises(RuntimeError, match="状态变化"):
        rig.device.ensure_can_id()
    rig.motor.set_can_id.assert_not_called()


@pytest.mark.parametrize("failure", ["busy", "write", "cycle", "readback"])
def test_failure_stops_without_retry(rig, failure):
    if failure == "busy":
        rig.flow._prepare_motor_admin.side_effect = RuntimeError("busy")
    elif failure == "write":
        rig.motor.set_can_id.side_effect = RuntimeError("write failed")
    elif failure == "cycle":
        rig.device.power_cycle.side_effect = RuntimeError("cycle timeout")
    else:
        rig.motor.get_can_id.side_effect = [127, 127, 127]
    with pytest.raises(RuntimeError):
        rig.device.ensure_can_id()
    assert rig.motor.set_can_id.call_count == (0 if failure == "busy" else 1)


def test_display_does_not_write(rig):
    state = rig.device.can_id_state(rig.gripper)
    info = dict(
        sn=rig.device.identity[0],
        model="RS00",
        recorded=True,
        mcu="1.2.16",
        motor="0.0.3.32",
        **state,
    )
    catalog = core.catalog_at(APP / "catalog.json")
    html, text, tone, status = version_status.device_summary(info, "slave", catalog)
    assert "CAN ID 127 → 17" in text
    assert "#B42318" in html and tone == "error"
    assert "确认修正" in status
    rig.motor.set_can_id.assert_not_called()


def test_policy_blocks_ota_on_can_id_failure():
    device = Mock()
    device.inspect.return_value = dict(
        sn="TCGU01A00Z0001s",
        model="RS00",
        recorded=True,
        mcu="1.2.16",
        motor="0.0.3.32",
    )
    device.ensure_can_id.side_effect = RuntimeError("CAN ID rejected")
    image = core.Image("motor", "RS00", "0.0.3.32", "digest", "", b"image")
    with pytest.raises(RuntimeError, match="CAN ID rejected"):
        core.execute(device, None, image, lambda *_: True, Mock())
    device.protocol.assert_not_called()
    device.flash_motor.assert_not_called()
    device.flash_mcu.assert_not_called()


def test_same_version_skip_still_corrects_and_finally_checks_can_id():
    device = Mock()
    device.inspect.return_value = dict(
        sn="TCGU01A00Z0001s",
        model="RS00",
        recorded=True,
        mcu="1.2.16",
        motor="0.0.3.32",
    )
    device.live_motor_version.return_value = "0.0.3.32"
    image = core.Image("motor", "RS00", "0.0.3.32", "digest", "", b"image")
    core.execute(device, None, image, lambda title, _: "重复刷写" not in title, Mock())
    device.ensure_can_id.assert_called_once()
    device.check_can_id.assert_called_once()
    device.flash_motor.assert_not_called()

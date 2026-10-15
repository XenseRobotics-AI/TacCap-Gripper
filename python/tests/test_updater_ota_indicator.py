"""OTA LED lifecycle using fake SDK connections, never real hardware."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest


@pytest.fixture
def rig():
    path = Path(__file__).resolve().parents[2] / "apps/firmware_updater/device.py"
    spec = importlib.util.spec_from_file_location("ota_indicator_driver", path)
    module = importlib.util.module_from_spec(spec)
    leader, follower = MagicMock(), MagicMock()
    for handle in (leader, follower):
        handle.__enter__.return_value = handle
        handle.device.get_sn.return_value = "test-s"
    session = Mock()
    session.read_uid.return_value = b"01234567"
    sdk = SimpleNamespace(
        LeaderGripper=Mock(return_value=leader),
        FollowerGripper=Mock(return_value=follower),
        MotorOtaSession=Mock(return_value=session),
        MotorProtocol=Mock(),
        OtaTargetVersion=Mock(),
        Ws2812EffectType=SimpleNamespace(NormalSolid=1, OtaBlink=3, FaultBlink=4),
        scan_grippers=Mock(),
    )
    flow = Mock()
    with patch.dict(
        sys.modules,
        {"xense.taccap": sdk, "full_ota_update": flow, "motor_ota_update": Mock()},
    ):
        spec.loader.exec_module(module)
    device = module.Device(("test-s", "usb"), Mock())
    device.endpoint = Mock(
        return_value=SimpleNamespace(mcu_device="/dev/test-verified")
    )
    image = SimpleNamespace(
        kind="mcu", model="slave", version="1.2.16", data=b"image", sha256="digest"
    )
    with patch.object(module.time, "sleep") as sleep:
        yield SimpleNamespace(
            device=device,
            module=module,
            sdk=sdk,
            flow=flow,
            leader=leader,
            follower=follower,
            session=session,
            sleep=sleep,
            image=image,
        )


def effects(handle):
    return [call.args[0] for call in handle.led.effect.call_args_list]


def test_mcu_blue_frame_is_requested_before_first_flash_packet(rig):
    order = []
    rig.leader.led.effect.side_effect = lambda *_: order.append("blue")
    rig.sleep.side_effect = lambda seconds: order.append(("monitor_tick", seconds))
    rig.leader.ota.update_from_bytes.side_effect = lambda *_: order.append("flash")
    rig.device.flash_mcu(rig.image)
    assert order[:3] == ["blue", ("monitor_tick", 0.06), "flash"]
    rig.leader.led.effect.assert_called_once_with(3, 0, 0, 0)
    assert rig.device._ota_indicator_active


def test_motor_tunnel_ota_also_requests_blue_after_preflight(rig):
    order = []
    rig.session.preflight.side_effect = lambda *_: order.append("preflight")
    rig.session.read_uid.side_effect = lambda: order.append("uid") or b"01234567"
    rig.follower.led.effect.side_effect = lambda *_: order.append("blue")
    rig.session.update_from_bytes.side_effect = lambda *_: order.append("flash")
    rig.image.model = "EL05"
    rig.device.flash_motor(rig.image)
    assert order == ["preflight", "uid", "blue", "flash"]
    rig.follower.led.effect.assert_called_once_with(3, 0, 0, 0)


def test_skipped_or_cancelled_update_never_changes_leds(rig):
    with rig.device.update_indication():
        with rig.device.opened():
            pass
    with pytest.raises(RuntimeError, match="cancelled"):
        with rig.device.update_indication():
            raise RuntimeError("cancelled")
    rig.follower.led.effect.assert_not_called()
    rig.leader.led.effect.assert_not_called()
    rig.sdk.LeaderGripper.assert_not_called()


def test_preflight_failure_never_claims_an_active_ota(rig):
    rig.session.preflight.side_effect = RuntimeError("model mismatch")
    with pytest.raises(RuntimeError, match="model mismatch"):
        with rig.device.update_indication():
            rig.device.flash_motor(rig.image)
    rig.follower.led.effect.assert_not_called()
    rig.session.update_from_bytes.assert_not_called()


def test_reconnect_reapplies_blue_and_success_restores_white_only_at_end(rig):
    with rig.device.update_indication():
        rig.device.flash_motor(rig.image)
        assert effects(rig.follower) == [3]
        with rig.device.opened():
            pass
        assert effects(rig.follower) == [3, 3]
        rig.sdk.LeaderGripper.assert_not_called()
    assert effects(rig.leader) == [1]
    rig.sdk.LeaderGripper.assert_called_once_with(mcu_device="/dev/test-verified")
    assert rig.device.endpoint.call_count == 2  # cleanup does not rediscover
    assert not rig.device._ota_indicator_active


def test_flash_failure_requests_red_without_masking_original_error_or_reopening(rig):
    rig.session.update_from_bytes.side_effect = RuntimeError("flash failed")
    rig.follower.led.effect.side_effect = [None, TimeoutError("LED no ACK")]
    with pytest.raises(RuntimeError, match="flash failed"):
        with rig.device.update_indication():
            rig.device.flash_motor(rig.image)
    assert effects(rig.follower) == [3, 4]
    rig.sdk.LeaderGripper.assert_not_called()
    assert rig.device.endpoint.call_count == 1
    assert not rig.device._ota_indicator_active


def test_failed_verification_never_restores_normal_white(rig):
    with pytest.raises(RuntimeError, match="wrong version"):
        with rig.device.update_indication():
            rig.device.flash_motor(rig.image)
            raise RuntimeError("wrong version")
    assert effects(rig.follower) == [3]
    rig.sdk.LeaderGripper.assert_not_called()


def test_led_failure_does_not_break_an_otherwise_valid_flash(rig):
    rig.follower.led.effect.side_effect = RuntimeError("unsupported LED")
    with rig.device.update_indication():
        rig.device.flash_motor(rig.image)
        with rig.device.opened():
            pass
    rig.session.update_from_bytes.assert_called_once()
    rig.follower.led.effect.assert_called_once()
    rig.sdk.LeaderGripper.assert_not_called()
    rig.sleep.assert_not_called()
    assert any("警告" in str(call) for call in rig.device.emit.call_args_list)


def test_changed_sn_is_never_written_even_for_led_cleanup(rig):
    rig.leader.device.get_sn.return_value = "other-s"
    with rig.device.update_indication():
        rig.device.flash_motor(rig.image)
    rig.leader.led.effect.assert_not_called()
    assert any("身份变化" in str(call) for call in rig.device.emit.call_args_list)


def test_opened_rechecks_sn_before_reapplying_indicator(rig):
    rig.device._ota_indicator_active = True
    rig.follower.device.get_sn.return_value = "other-s"
    with pytest.raises(RuntimeError, match="身份变化"):
        with rig.device.opened():
            pass
    rig.follower.led.effect.assert_not_called()


def test_master_mcu_keeps_existing_firmware_owned_led_behavior(rig):
    rig.device.identity = ("test-m", "usb")
    rig.device.role = "master"
    rig.leader.device.get_sn.return_value = "test-m"
    rig.image.model = "master"
    with rig.device.update_indication():
        rig.device.flash_mcu(rig.image)
    rig.leader.led.effect.assert_not_called()
    rig.leader.ota.update_from_bytes.assert_called_once()
    rig.sleep.assert_called_once_with(3)  # existing MCU reboot wait only


def test_rejected_image_never_opens_device_or_changes_led(rig):
    rig.image.model = "master"
    with pytest.raises(ValueError, match="角色不匹配"):
        rig.device.flash_mcu(rig.image)
    rig.device.endpoint.assert_not_called()
    rig.leader.led.effect.assert_not_called()

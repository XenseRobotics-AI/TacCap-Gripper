"""Reduce redundant cycles, not live verification or mandatory hard restarts."""

from pathlib import Path
from unittest.mock import Mock

import pytest
import test_updater_ota_indicator as indicator_cases
from test_updater_policy import FakeDevice, image, run

rig = indicator_cases.rig


@pytest.mark.parametrize("protocol", ["Private", "Mit"])
def test_post_flash_uses_actual_protocol_not_cached_version(rig, protocol):
    rig.follower.motor.get_protocol.return_value = getattr(
        rig.sdk.MotorProtocol, protocol
    )
    rig.device.protocol = Mock()
    rig.device.power_cycle = Mock()
    rig.device.live_motor_version = Mock(return_value="1.0.5.0.4")
    assert rig.device.motor_version_after_flash() == "1.0.5.0.4"
    rig.device.live_motor_version.assert_called_once()
    rig.sleep.assert_called_once_with(3)
    if protocol == "Private":
        rig.device.protocol.assert_not_called()
    else:
        rig.device.protocol.assert_called_once_with("Private", allow_recovery=False)
    rig.device.power_cycle.assert_not_called()
    rig.follower.motor.set_motor_fw_version.assert_not_called()


@pytest.mark.parametrize("failure", ["unknown", "disconnected", "switch"])
def test_post_flash_failure_never_falls_back_to_record_or_reflash(rig, failure):
    rig.device.live_motor_version = Mock()
    rig.device.protocol = Mock(side_effect=RuntimeError("switch failed"))
    if failure == "unknown":
        rig.follower.motor.get_protocol.return_value = object()
    elif failure == "disconnected":
        rig.follower.motor.get_protocol.side_effect = RuntimeError("disconnected")
    else:
        rig.follower.motor.get_protocol.return_value = rig.sdk.MotorProtocol.Mit
    with pytest.raises(RuntimeError):
        rig.device.motor_version_after_flash()
    rig.device.live_motor_version.assert_not_called()
    rig.follower.motor.motor_version.assert_not_called()
    rig.follower.motor.set_motor_fw_version.assert_not_called()
    rig.session.update_from_bytes.assert_not_called()


def test_mandatory_final_cycle_is_not_skipped_when_already_mit(rig):
    rig.follower.motor.get_protocol.return_value = rig.sdk.MotorProtocol.Mit
    rig.device.power_cycle = Mock()
    rig.device.protocol("Mit", force_cycle=True)
    rig.device.power_cycle.assert_called_once()
    rig.flow._switch_protocol.assert_not_called()
    rig.flow._wait_motor_protocol.assert_called_once()
    rig.sleep.assert_not_called()  # No second post-flash settling delay.


class CountingDevice(FakeDevice):
    def __init__(self, post_protocol="Private"):
        super().__init__()
        self.current_protocol = "Mit"
        self.post_protocol = post_protocol
        self.cycles = 0

    def power_cycle(self, reason):
        super().power_cycle(reason)
        self.cycles += 1

    def protocol(self, name, force_cycle=False):
        super().protocol(name, force_cycle)
        if self.current_protocol != name or force_cycle:
            self.power_cycle(name)
        self.current_protocol = name

    def flash_motor(self, img):
        super().flash_motor(img)
        self.current_protocol = self.post_protocol

    def motor_version_after_flash(self):
        if self.current_protocol != "Private":
            self.protocol("Private")
        return self.live_motor_version()


@pytest.mark.parametrize(
    "combined,post_protocol,cycles",
    [(False, "Private", 2), (True, "Private", 3), (False, "Mit", 3), (True, "Mit", 4)],
)
def test_cycle_budget_for_existing_configured_device(combined, post_protocol, cycles):
    device = CountingDevice(post_protocol)
    run(
        device,
        image("mcu", device.mcu) if combined else None,
        image("motor", device.motor),
    )
    assert device.cycles == cycles
    assert device.current_protocol == "Mit"
    assert device.events[-1] == ("homing",)


def test_failed_live_verification_stops_without_recording_or_success():
    device = CountingDevice()
    device.wrong_version = True
    device.motor = "1.0.5.0.2"
    with pytest.raises(RuntimeError, match="实时版本"):
        run(device, motor=image("motor", "1.0.5.0.4"))
    assert not any(event[0] == "record" for event in device.events)
    assert sum(event[0] == "flash_motor" for event in device.events) == 1


def test_new_post_flash_prompts_have_complete_english_translations(monkeypatch):
    monkeypatch.syspath_prepend(
        str(Path(__file__).resolve().parents[2] / "apps/firmware_updater")
    )
    import i18n

    previous = i18n.language()
    try:
        i18n.set_language("en")
        for message in (
            "电机已提交，等待自动重启后读取实际协议和版本",
            "电机刷后仍为 Private：直接核验实时版本，硬重启与恢复 MIT 合并",
            "电机刷后已回到 MIT：实时版本查询需要 Private，必须额外切换并断电",
            "电机刷后协议未知，停止；不能用镜像或存档代替实时版本",
        ):
            translated = i18n.tr(message)
            assert not i18n.CJK.search(translated) and "\\u" not in translated
    finally:
        i18n.set_language(previous)

"""Both directions must share approval/readback guards, not bypass them."""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import core  # noqa: E402
import test_updater_mcu_downgrade as downgrade_cases  # noqa: E402
import test_updater_ota_indicator as indicator_cases  # noqa: E402
from test_updater_policy import FakeDevice, FakeMaster, image, run  # noqa: E402

external_mcu_file = downgrade_cases.external_mcu_file
# Reuse fixtures without importing their names into the test parameter scope.
ui = downgrade_cases.ui
rig = indicator_cases.rig


@pytest.mark.parametrize("role", ["master", "slave"])
@pytest.mark.parametrize("direction", ["upgrade", "downgrade", "same"])
def test_mcu_direction_matrix_with_real_readback(role, direction):
    device = FakeMaster() if role == "master" else FakeDevice()
    low, high = ("1.2.5", "1.2.6") if role == "master" else ("1.2.14", "1.2.16")
    device.mcu = low if direction == "upgrade" else high
    target = low if direction == "downgrade" else high
    prompts = []
    events = run(
        device,
        mcu=image("mcu", target, role),
        ask=lambda title, body: prompts.append(title) or True,
    )
    action = {"upgrade": "升级", "downgrade": "回滚", "same": "重复刷写"}[direction]
    assert any("MCU" in p and action in p for p in prompts)
    assert device.mcu == target
    assert events[-1][0] == "success"
    assert not any(e[0] == "flash_motor" for e in device.events)


@pytest.mark.parametrize(
    "role,current,target",
    [
        ("master", "1.2.5", "1.2.6"),
        ("master", "1.2.6", "1.2.5"),
        ("slave", "1.2.14", "1.2.16"),
        ("slave", "1.2.16", "1.2.14"),
    ],
)
def test_cancel_either_direction_writes_nothing(role, current, target):
    device = FakeMaster() if role == "master" else FakeDevice()
    device.mcu = current
    with pytest.raises(RuntimeError, match="用户取消"):
        run(
            device,
            mcu=image("mcu", target, role),
            ask=lambda title, _: "MCU" not in title,
        )
    assert not device.events


@pytest.mark.parametrize(
    "role,current,target,changed",
    [
        ("master", "1.2.5", "1.2.6", "1.2.6"),
        ("master", "1.2.6", "1.2.5", "1.2.7"),
        ("slave", "1.2.14", "1.2.16", "1.2.15"),
        ("slave", "1.2.16", "1.2.14", "1.2.15"),
    ],
)
def test_version_change_during_confirmation_stops_before_write(
    role, current, target, changed
):
    device = FakeMaster() if role == "master" else FakeDevice()
    device.mcu = current

    def ask(title, _):
        if "MCU" in title:
            device.mcu = changed
        return True

    with pytest.raises(RuntimeError, match="版本.*变化"):
        run(device, mcu=image("mcu", target, role), ask=ask)
    assert not any(e[0] == "flash_mcu" for e in device.events)


@pytest.mark.parametrize("current,target", [("1.2.14", "1.2.16"), ("1.2.16", "1.2.14")])
def test_changed_rs00_mcu_stops_before_can_id_write(current, target):
    device = FakeDevice()
    inspect = device.inspect
    device.inspect = lambda: {**inspect(), "model": "RS00"}
    device.mcu = current
    device.ensure_can_id = lambda: device.events.append(("can_id_write",))

    def ask(title, _):
        if "MCU" in title:
            device.mcu = "1.2.15"
        return True

    with pytest.raises(RuntimeError, match="版本.*变化"):
        run(device, mcu=image("mcu", target, "slave"), ask=ask)
    assert not device.events


@pytest.mark.parametrize("role", ["master", "slave"])
def test_zero_build_suffix_is_not_a_different_version(role):
    device = FakeMaster() if role == "master" else FakeDevice()
    version = "1.2.6" if role == "master" else "1.2.16"
    device.mcu = version
    device.mcu_version = lambda: device.mcu + ".0"
    prompts = []
    events = run(
        device,
        mcu=image("mcu", version, role),
        ask=lambda title, body: prompts.append(title) or True,
    )
    assert any("重复刷写" in title for title in prompts)
    assert not any("MCU回滚" in title for title in prompts)
    assert events[-1][0] == "success"


def test_version_keys_compare_wire_motor_format_and_mcu_suffix_consistently():
    assert core.version_key("1.2.16") == core.version_key("1.2.16.0")
    assert core.version_key("1.0.5.0.4") == core.version_key("10.5.0.4")


@pytest.mark.parametrize("direction", ["upgrade", "downgrade"])
def test_combined_el05_direction_keeps_safe_order(direction):
    device = FakeDevice()
    device.mcu = "1.2.14" if direction == "upgrade" else "1.2.16"
    device.motor = "1.0.5.0.2" if direction == "upgrade" else "1.0.5.0.4"
    target_mcu = "1.2.16" if direction == "upgrade" else "1.2.14"
    target_motor = "1.0.5.0.4" if direction == "upgrade" else "1.0.5.0.2"
    run(device, image("mcu", target_mcu), image("motor", target_motor))
    names = [e[0] for e in device.events]
    assert (names.index("flash_mcu") < names.index("flash_motor")) == (
        direction == "upgrade"
    )
    assert names[-1] == "homing"


def test_slave_1_2_15_is_an_approved_external_target_not_a_new_default():
    catalog = core.catalog_at(APP / "catalog.json")
    entries = [
        e for e in catalog["images"] if e["kind"] == "mcu" and e["version"] == "1.2.15"
    ]
    assert len(entries) == 1 and entries[0]["model"] == "slave"
    assert catalog["history"][entries[0]["file"]]["commit"]


def test_build_does_not_require_any_historical_bin_or_git_history():
    # The maintained build includes exactly the approved defaults, not all
    # optional catalog files. Validate the selected payload set itself.
    import bundled

    assert set(bundled.packaged_files()) == set(bundled.FILES.values())
    assert len(bundled.packaged_files()) == 4
    assert all(
        "1.2.5.bin" not in path and "1.2.14.bin" not in path
        for path in bundled.packaged_files()
    )


@pytest.mark.parametrize(
    "role,target",
    [
        ("master", "1.2.5"),
        ("master", "1.2.6"),
        ("slave", "1.2.14"),
        ("slave", "1.2.15"),
        ("slave", "1.2.16"),
    ],
)
def test_native_adapter_sends_exact_target_and_bytes_in_both_directions(
    rig, role, target
):
    sn = "test-m" if role == "master" else "test-s"
    rig.device.identity = (sn, "usb")
    rig.device.role = role
    rig.leader.device.get_sn.return_value = sn
    rig.image.model = role
    rig.image.version = target
    rig.device.flash_mcu(rig.image)
    rig.sdk.OtaTargetVersion.assert_called_once_with(*map(int, target.split(".")), 0)
    rig.leader.ota.update_from_bytes.assert_called_once_with(
        rig.image.data, rig.sdk.OtaTargetVersion.return_value, rig.device.progress
    )
    rig.flow._parse_version_text.assert_called_once_with(target)
    rig.flow._wait_follower_version.assert_called_once()
    assert rig.flow._wait_follower_version.call_args.args == (
        rig.device.identity,
        rig.flow._parse_version_text.return_value,
        30,
    )


@pytest.mark.parametrize("target", ["1.2.5", "1.2.6"])
def test_native_transfer_failure_does_not_enter_readback_as_if_success(rig, target):
    rig.device.identity = ("test-m", "usb")
    rig.device.role = "master"
    rig.leader.device.get_sn.return_value = "test-m"
    rig.image.model = "master"
    rig.image.version = target
    rig.leader.ota.update_from_bytes.side_effect = RuntimeError(
        "native transfer failed"
    )
    with pytest.raises(RuntimeError, match="native transfer failed"):
        rig.device.flash_mcu(rig.image)
    rig.flow._wait_follower_version.assert_not_called()


@pytest.mark.parametrize("choice", ["cancel", "tampered", "wrong-version"])
def test_external_choice_failure_keeps_previous_selection(ui, tmp_path, choice):
    window, _ = ui
    expected = next(
        e
        for e in window.catalog["images"]
        if e["kind"] == "mcu" and e["version"] == "1.2.15"
    )
    if choice == "wrong-version":
        path = external_mcu_file(
            window.catalog, "slave", "1.2.14", tmp_path / "wrong.bin"
        )
    else:
        path = tmp_path / "tampered.bin"
        path.write_bytes(b"tampered")
    before, overrides = window.paths.copy(), window.overrides.copy()
    with (
        patch(
            "qt_ui.QFileDialog.getOpenFileName",
            return_value=("" if choice == "cancel" else str(path), ""),
        ),
        patch("qt_ui.notice") as notice,
    ):
        window.select_mcu_version(expected)
    assert window.paths == before and window.overrides == overrides
    assert notice.call_count == (0 if choice == "cancel" else 1)


@pytest.mark.parametrize("current,target", [("1.2.14", "1.2.15"), ("1.2.16", "1.2.15")])
def test_external_follower_1_2_15_works_as_upgrade_and_downgrade(
    ui, tmp_path, current, target
):
    window, _ = ui
    window.selected_identity = ("TESTs", "usb")
    window.current_info = dict(recorded=True, model="EL05", mcu=current)
    entry = next(
        e
        for e in window.catalog["images"]
        if e["kind"] == "mcu" and e["version"] == target
    )
    path = external_mcu_file(window.catalog, "slave", target, tmp_path / "firmware.bin")
    with patch("qt_ui.QFileDialog.getOpenFileName", return_value=(str(path), "")):
        window.select_mcu_version(entry)
    assert window.mcu_target.version == target
    assert window.mcu_hint.isVisible() == (current == "1.2.16")
    device = FakeDevice()
    device.mcu = current
    assert run(device, mcu=window.mcu_target)[-1][0] == "success"
    assert device.mcu == target

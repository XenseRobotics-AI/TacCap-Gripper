"""Safety/lifecycle coverage without scanning or moving physical hardware."""

import math
import sys
import threading
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/firmware_updater"))
import control_test as control  # noqa: E402


@pytest.fixture
def rig():
    calls = []
    motor = Mock()
    motor.get_model.return_value = NS(name="EL05", id=0, from_flash=True)
    motor.get_protocol.return_value = "MIT"
    motor.read_status.return_value = NS(status=0)
    motor.get_spec.return_value = NS(t_max_nm=6)
    motor.disable.side_effect = lambda: calls.append("disable")
    motor.enable.side_effect = lambda: calls.append("enable")
    gripper = Mock(motor=motor)
    gripper.firmware_version = NS(major=1, minor=2, patch=16)
    gripper.home_diag.return_value = NS(flags=14, fail_reason=0)
    gripper.get_gripper_config.return_value = NS(
        flags=1, max_open_rad=1.2, min_open_rad=0
    )
    gripper.audit_envelope.return_value = NS(
        effective=NS(flags=0x2003, cont_torque_nm=1, peak_torque_nm=2)
    )
    config = NS(grasp_torque_nm=1.1)
    sample = NS(
        running=True,
        state=NS(name="TRACKING"),
        fault_reason="",
        observation=NS(
            valid=True,
            age_ms=1,
            position=0.63,
            torque=0.02,
            velocity=0,
            motor_temp_c=30,
            status=1,
        ),
    )
    ctl = Mock()
    ctl.snapshot.return_value = sample
    ctl.start.side_effect = lambda: calls.append("start")
    ctl.stop.side_effect = lambda: calls.append("stop")
    sdk = NS(
        MotorProtocol=NS(Mit="MIT"),
        ForcePositionConfig=NS(for_spec=Mock(return_value=config)),
        ForcePositionController=Mock(return_value=ctl),
        parse_serial=Mock(return_value=NS(valid=True, side="Left")),
        expected_open_reverse=Mock(return_value=True),
        expected_motor_can_id=Mock(return_value=17),
    )

    @contextmanager
    def opened():
        calls.append("open")
        try:
            yield gripper
        finally:
            calls.append("close")

    events = []
    session = control.ControlSession(
        ("TCGU01A25Z0017s", "fake"),
        lambda k, v: events.append((k, v)),
        opener=opened,
        sdk=sdk,
    )
    return NS(
        g=gripper,
        sdk=sdk,
        ctl=ctl,
        config=config,
        sample=sample,
        calls=calls,
        events=events,
        session=session,
    )


def test_constructing_session_has_no_hardware_side_effects(rig):
    assert rig.calls == []
    rig.sdk.ForcePositionController.assert_not_called()
    rig.session.stop()
    rig.session.run()
    assert rig.calls == []
    assert rig.events[-1] == ("finished", {"error": None, "disabled": False})


@pytest.mark.parametrize(
    "failure",
    [
        "model",
        "private",
        "active",
        "failed",
        "invalid",
        "nan",
        "envelope",
        "unlimited",
        "too_high",
        "fault",
        "legacy",
        "rs00_direction",
        "rs00_can",
        "rs00_identity",
    ],
)
def test_preflight_rejects_unsafe_state_without_enable_or_writes(rig, failure):
    if failure == "model":
        rig.g.motor.get_model.return_value.from_flash = False
    elif failure == "private":
        rig.g.motor.get_protocol.return_value = "Private"
    elif failure == "active":
        rig.g.home_diag.return_value.flags |= 1
    elif failure == "failed":
        rig.g.home_diag.return_value.fail_reason = 4
    elif failure == "invalid":
        rig.g.get_gripper_config.return_value.flags = 0
    elif failure == "nan":
        rig.g.get_gripper_config.return_value.max_open_rad = math.nan
    elif failure == "envelope":
        rig.g.audit_envelope.return_value.effective = None
    elif failure == "unlimited":
        rig.g.audit_envelope.return_value.effective.peak_torque_nm = 0
    elif failure == "too_high":
        rig.g.audit_envelope.return_value.effective.peak_torque_nm = 9
    elif failure == "fault":
        rig.g.motor.read_status.return_value.status = 0x100
    elif failure == "legacy":
        rig.g.firmware_version.patch = 13
    else:
        rig.g.motor.get_model.return_value = NS(name="RS00", id=1, from_flash=True)
        rig.g.get_gripper_config.return_value.flags = 3
        rig.g.motor.get_can_id.return_value = 17
        if failure == "rs00_direction":
            rig.g.get_gripper_config.return_value.flags = 1
        elif failure == "rs00_can":
            rig.g.motor.get_can_id.return_value = 127
        else:
            rig.sdk.parse_serial.return_value.valid = False
    rig.session.run()
    assert rig.events[-1][1]["error"]
    assert rig.calls == ["open", "close"]
    rig.g.motor.enable.assert_not_called()
    rig.g.motor.clear_fault.assert_not_called()
    rig.g.set_gripper_config.assert_not_called()
    rig.g.ensure_envelope.assert_not_called()


def test_lifecycle_current_position_latest_target_and_stop_priority(rig):
    def emit(kind, value):
        rig.events.append((kind, value))
        if kind == "ready":
            assert value["position"] == 0.63
            for target in (0.2, 0.4, 0.8):
                rig.session.set_target(target)
        elif kind == "sample":
            rig.session.set_target(0)
            rig.session.stop()

    rig.session.emit = emit
    rig.session.run()
    assert rig.calls == [
        "open",
        "disable",
        "start",
        "enable",
        "stop",
        "disable",
        "close",
    ]
    rig.ctl.set_target.assert_called_once_with(0.8)
    assert rig.config.grasp_torque_nm == 0.5
    assert rig.config.close_speed_radps == 0.25
    assert rig.config.close_preload_nm == 0
    assert rig.events[-1] == ("finished", {"error": None, "disabled": True})


@pytest.mark.parametrize(
    "bad", ["stale", "invalid", "fault", "state", "nan", "not_running"]
)
def test_invalid_stream_stops_controller_before_disabling(rig, bad):
    def emit(kind, value):
        if kind == "ready":
            if bad == "stale":
                rig.sample.observation.age_ms = 351
            elif bad == "invalid":
                rig.sample.observation.valid = False
            elif bad == "fault":
                rig.sample.fault_reason = "link lost"
            elif bad == "state":
                rig.sample.state.name = "FAULT"
            elif bad == "nan":
                rig.sample.observation.torque = math.nan
            else:
                rig.sample.running = False
        rig.events.append((kind, value))

    rig.session.emit = emit
    rig.session.run()
    assert rig.calls[-3:] == ["stop", "disable", "close"]
    assert rig.events[-1][1]["error"]
    rig.ctl.set_target.assert_not_called()


def test_stop_during_start_never_enables_motor(rig):
    rig.ctl.start.side_effect = rig.session.stop
    rig.session.run()
    rig.g.motor.enable.assert_not_called()
    rig.ctl.stop.assert_called_once()
    assert rig.calls[-2:] == ["disable", "close"]


def test_disable_failure_is_not_reported_as_success(rig):
    rig.g.motor.disable.side_effect = [None, OSError("disconnected")]
    rig.session.emit = lambda k, v: (
        rig.events.append((k, v)),
        rig.session.stop() if k == "ready" else None,
    )
    rig.session.run()
    result = rig.events[-1][1]
    assert not result["disabled"]
    assert "24 V" in result["error"]


@pytest.mark.parametrize("value", [math.nan, math.inf, -0.1, 1.01])
def test_invalid_targets_are_rejected(rig, value):
    with pytest.raises(ValueError):
        rig.session.set_target(value)


def test_session_time_limit(rig):
    with patch.object(control, "SESSION_SECONDS", -1):
        rig.session.run()
    rig.ctl.set_target.assert_not_called()
    assert rig.events[-1][1]["disabled"]


def test_controller_start_failure_still_stops_and_disables(rig):
    rig.ctl.start.side_effect = RuntimeError("start failed")
    rig.session.run()
    rig.g.motor.enable.assert_not_called()
    rig.ctl.stop.assert_called_once()
    assert "start failed" in rig.events[-1][1]["error"]


def test_rs00_uses_spec_and_keeps_tighter_envelope_budget(rig):
    rig.g.motor.get_model.return_value = NS(name="RS00", id=1, from_flash=True)
    rig.g.motor.get_can_id.return_value = 17
    rig.g.get_gripper_config.return_value.flags = 3
    rig.g.audit_envelope.return_value.effective.cont_torque_nm = 0.3
    cfg = control.preflight(rig.g, rig.session.identity[0], rig.sdk)
    assert cfg.grasp_torque_nm == 0.3
    rig.sdk.ForcePositionConfig.for_spec.assert_called_once_with(
        rig.g.motor.get_spec.return_value
    )


@pytest.fixture
def ui():
    pytest.importorskip("PySide6")
    import i18n
    import qt_ui
    from PySide6.QtCore import QCoreApplication, QEvent
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    qt_ui.configure(app)
    previous = i18n.language()
    window = qt_ui.App(auto_connect=False)
    window.show()
    app.processEvents()
    yield window, app
    window.busy = window.updating = False
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    app.processEvents()
    i18n.set_language(previous)


def test_main_test_action_follower_only_and_ota_mutual_exclusion(ui):
    window, app = ui
    assert not window.test_button.isEnabled()
    window.selected_identity = ("TCGU01A25Z0017s", "fake")
    window.current_info = {"recorded": True}
    window.update_actions()
    assert window.test_button.isEnabled()

    def modal():
        assert window.busy and window.updating
        assert not window.start.isEnabled() and not window.refresh.isEnabled()
        before = window.selected_identity
        window.scan()
        window.run()
        assert window.selected_identity == before
        return 0

    with patch("qt_ui.ControlTestDialog.exec", side_effect=modal):
        window.open_control_test()
    assert not window.busy and window.test_button.isEnabled()
    window.selected_identity = ("TCGU01A25Z0017m", "fake")
    window.update_actions()
    assert window.test_button.isHidden()


def test_dialog_no_auto_connection_confirmation_and_safe_close(ui):
    from control_test_ui import ControlTestDialog

    window, app = ui
    dialog = ControlTestDialog(("TCGU01A25Z0017s", "fake"), window)
    dialog.show()
    app.processEvents()
    assert dialog.worker is None
    assert not dialog.slider.isEnabled()
    with patch("control_test_ui.confirm", return_value=False):
        dialog.enable.click()
    assert dialog.worker is None
    released = threading.Event()
    thread = threading.Thread(target=lambda: released.wait(2))
    dialog.session = Mock()
    dialog.session.stopping = threading.Event()
    dialog.worker = thread
    thread.start()
    dialog.slider.setEnabled(True)
    dialog.reject()
    assert dialog.isVisible() and not dialog.slider.isEnabled()
    dialog.session.stop.assert_called_once()
    dialog.emit("finished", {"error": None, "disabled": True})
    dialog.drain()
    assert dialog.isVisible()  # finished event is not enough while worker lives
    released.set()
    thread.join(2)
    dialog.drain()
    assert not dialog.isVisible()
    dialog.deleteLater()


def test_dialog_worker_round_trip_and_restart(ui, rig):
    import time

    from control_test_ui import ControlTestDialog

    window, app = ui
    dialog = ControlTestDialog(rig.session.identity, window)
    dialog.show()

    def wait_for(predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.005)
        assert predicate()

    def factory(identity, emit):
        return control.ControlSession(
            identity, emit, opener=rig.session.opener, sdk=rig.sdk
        )

    with (
        patch("control_test_ui.confirm", return_value=True),
        patch("control_test_ui.ControlSession", side_effect=factory),
    ):
        for _ in range(2):
            dialog.enable.click()
            wait_for(lambda: dialog.slider.isEnabled())
            assert dialog.slider.value() == 63
            dialog.slider.setValue(80)
            wait_for(lambda: rig.ctl.set_target.called)
            dialog.stop_button.click()
            assert not dialog.slider.isEnabled()
            wait_for(lambda: dialog.worker is None)
            assert dialog.enable.isEnabled()
            assert "失能已确认" in dialog.status.text()
            rig.ctl.set_target.reset_mock()
    assert rig.g.motor.enable.call_count == 2
    dialog.close()
    dialog.deleteLater()


def test_unconfirmed_disable_remains_visible_on_close(ui):
    from control_test_ui import ControlTestDialog

    window, app = ui
    dialog = ControlTestDialog(("TCGU01A25Z0017s", "fake"), window)
    dialog.show()
    dialog.worker = Mock()
    dialog.worker.is_alive.return_value = False
    dialog.session = Mock()
    dialog.closing = True
    dialog.emit("finished", {"error": "24 V: unconfirmed disable", "disabled": False})
    dialog.drain()
    assert dialog.isVisible() and not dialog.enable.isEnabled()
    assert "24 V" in dialog.status.text()
    dialog.close()
    dialog.deleteLater()


@pytest.mark.parametrize("lang", ["zh", "en"])
def test_dialog_translation_and_no_overlap(ui, lang):
    import i18n
    from control_test_ui import ControlTestDialog

    window, app = ui
    i18n.set_language(lang)
    dialog = ControlTestDialog(("TCGU01A25Z0017s", "fake"), window)
    dialog.show()
    app.processEvents()
    assert dialog.slider.geometry().bottom() < dialog.feedback.geometry().top()
    assert dialog.status.geometry().bottom() < dialog.enable.geometry().top()
    assert dialog.width() <= 720
    if lang == "en":
        assert "Follower function test" == dialog.windowTitle()
        assert not i18n.CJK.search(dialog.warning.text())
    dialog.close()
    dialog.deleteLater()

"""Customer workflows with fake native devices; no hardware or calibration writes."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import xense.taccap as sdk
from xense.taccap import gripper as api


def endpoint(side=None, role=None, sn="TC-test", path="/dev/fake"):
    return NS(side=side or api._native.Side.Right,
              role=role or api._native.Role.Follower, firmware_sn=sn, mcu_device=path)


@pytest.fixture
def native(monkeypatch):
    sample = NS(actual_pos=0.5, actual_vel=0.0, actual_torque=0.0,
                motor_temp_c=25.0, status=0)
    execution = NS(owner=0, last_error=0, flags=0, torque_cap_nm=1.0,
                   target_seq=0, applied_seq=0)
    device = Mock()
    device.firmware_version = NS(major=1, minor=1, patch=7)
    device.position_map.return_value = NS(max_open_rad=1.3, min_open_rad=0., reverse=False)
    device.motor.read_status.return_value = sample
    device.motor.execution_status.return_value = execution
    device.rad_to_pos.side_effect = lambda p: p
    snap = NS(state=api._native.ForcePositionState.HOLDING_POSITION,
              control_mode=api._native.ForcePositionMode.POSITION, blocked=False,
              running=True, command_sequence=0, submitted_sequence=0,
              submission_observation_sequence=0, active_torque_limit_nm=.35,
              execution_telemetry=False, execution=execution, target_error_rad=None, feedback_frames=50,
              command_frames=49, elapsed_s=.5, max_feedback_gap_ms=11.,
              max_command_gap_ms=12., last_submit_latency_ms=3., max_submit_latency_ms=9.,
              target_position=0.5, fault_reason="", observation=NS(
                  position=0.5, velocity=0.0, torque=0.0, motor_temp_c=25.0,
                  valid=True, age_ms=5.0, seq=1, status=0))
    controller = Mock()
    def snapshot():
        snap.observation.seq += 1
        return snap
    controller.snapshot.side_effect = snapshot

    def command(position, mode, phase):
        snap.target_position = position
        snap.control_mode = mode
        snap.state = phase
        snap.blocked = False
        snap.command_sequence += 1
        snap.submitted_sequence = snap.command_sequence
        snap.submission_observation_sequence = snap.observation.seq
        execution.target_seq += 1
        execution.applied_seq = execution.target_seq
    controller.set_position_target.side_effect = lambda p, t, v: command(
        p, api._native.ForcePositionMode.POSITION, api._native.ForcePositionState.MOVING_POSITION)
    controller.set_grasp_target.side_effect = lambda p, t, v: command(
        p, api._native.ForcePositionMode.GRASP, api._native.ForcePositionState.CLOSING)
    controller.set_impedance_target.side_effect = lambda p, *args: command(
        p, api._native.ForcePositionMode.IMPEDANCE, api._native.ForcePositionState.IMPEDANCE)
    controller.hold_position.side_effect = lambda: command(
        snap.observation.position, api._native.ForcePositionMode.POSITION,
        api._native.ForcePositionState.HOLDING_POSITION)
    def start():
        snap.running = True
        execution.owner = 1
    def stop():
        snap.running = False
        execution.owner = 0
    controller.start.side_effect = start
    controller.stop.side_effect = stop
    factory = Mock(return_value=device)
    controls = Mock(return_value=controller)
    scan = Mock(return_value=[endpoint()])
    monkeypatch.setattr(api._native, "FollowerGripper", factory)
    monkeypatch.setattr(api._native, "ForcePositionController", controls)
    monkeypatch.setattr(api._native, "scan_grippers", scan)
    return NS(device=device, controller=controller, snap=snap, sample=sample,
              factory=factory, controls=controls, scan=scan, execution=execution)


def test_connection_is_passive_and_first_operation_starts_once(native):
    with sdk.Gripper() as g:
        native.controls.assert_not_called()
        native.device.motor.enable.assert_not_called()
        assert g.state.position == 0.5
        g.move(0.8)
        g.grasp(torque_nm=0.4)
        g.release()
        native.controller.start.assert_called_once()
        assert native.controller.set_position_target.call_args_list[-1].args == (1.0, 0.35, 0.5)
    native.controller.stop.assert_called_once()
    native.device.transport.stop.assert_called_once()


def test_active_state_reads_cache_not_serial(native):
    with sdk.Gripper("right") as g:
        g.move(0.7)
        state = g.state
        assert state.target == 0.7 and state.mode == "moving_position"
        assert state.age_ms == 5
        native.device.motor.read_status.assert_not_called()
        with pytest.raises(AttributeError):
            state.position = 0.2


@pytest.mark.parametrize("position", [float("nan"), float("inf"), -0.1, 1.1])
def test_invalid_position_never_starts_motion(native, position):
    with sdk.Gripper() as g:
        with pytest.raises(ValueError):
            g.move(position)
        native.controls.assert_not_called()


@pytest.mark.parametrize("torque", [float("nan"), float("inf"), -1, 0, 1.21])
def test_invalid_torque_never_starts_motion(native, torque):
    with sdk.Gripper() as g:
        with pytest.raises(ValueError):
            g.grasp(torque_nm=torque)
        native.controls.assert_not_called()


def test_stop_disables_without_active_controller_and_allows_restart(native):
    with sdk.Gripper() as g:
        g.stop()
        native.device.motor.disable.assert_called_once()
        g.move(0.9)
        g.stop()
        native.controller.stop.assert_called_once()
        g.grasp()
        assert native.controls.call_count == 2


def test_clear_fault_stops_before_clear_and_does_not_restart(native):
    with sdk.Gripper() as g:
        g.grasp()
        calls = Mock()
        calls.attach_mock(native.controller.stop, "stop")
        calls.attach_mock(native.device.motor.clear_fault, "clear")
        g.clear_fault()
        assert [c[0] for c in calls.mock_calls] == ["stop", "clear"]
        assert native.controller.start.call_count == 1


def test_close_on_exception_and_idempotence(native):
    g = sdk.Gripper()
    with pytest.raises(ValueError):
        with g:
            g.grasp()
            raise ValueError("application failed")
    g.close()
    native.controller.stop.assert_called_once()
    native.device.transport.stop.assert_called_once()
    with pytest.raises(RuntimeError, match="closed"):
        g.move(0.3)


def test_failed_start_cleans_up_controller(native):
    native.controller.start.side_effect = RuntimeError("not ready")
    with sdk.Gripper() as g:
        with pytest.raises(RuntimeError, match="not ready"):
            g.move(0.5)
        native.controller.stop.assert_called_once()
        native.controller.set_position_target.assert_not_called()


@pytest.mark.parametrize("owner", [1, 2, 3, 4])
def test_busy_or_latched_firmware_never_starts(native, owner):
    native.device.motor.execution_status.return_value.owner = owner
    with sdk.Gripper() as g:
        with pytest.raises(RuntimeError, match="Cannot start gripper"):
            g.grasp()
        native.controls.assert_not_called()


def test_missing_calibration_closes_passively(native):
    native.device.position_map.side_effect = RuntimeError("uncalibrated")
    with pytest.raises(RuntimeError, match="uncalibrated"):
        sdk.Gripper()
    native.device.transport.stop.assert_called_once()
    native.device.motor.disable.assert_not_called()


def test_old_firmware_closes_without_motion(native):
    native.device.firmware_version.patch = 6
    with pytest.raises(RuntimeError, match="1.1.7"):
        sdk.Gripper()
    native.device.transport.stop.assert_called_once()
    native.controls.assert_not_called()


def test_discovery_filters_role_and_requires_unique_match(native):
    native.scan.return_value = [endpoint(role=api._native.Role.Leader), endpoint()]
    sdk.Gripper().close()
    native.factory.assert_called_once_with(mcu_device="/dev/fake")
    native.scan.return_value = [endpoint(sn="A"), endpoint(sn="B")]
    with pytest.raises(RuntimeError, match="found 2"):
        sdk.Gripper("right")
    sdk.Gripper("B").close()
    native.scan.return_value = []
    with pytest.raises(RuntimeError, match="found 0"):
        sdk.Gripper()


def test_explicit_path_skips_discovery(native):
    sdk.Gripper("/dev/serial/by-id/test").close()
    native.scan.assert_not_called()
    native.factory.assert_called_once_with(mcu_device="/dev/serial/by-id/test")


def test_wait_distinguishes_contact_and_reached(native):
    with sdk.Gripper() as g:
        g.grasp()
        native.snap.state = api._native.ForcePositionState.HOLDING_FORCE
        s = g.wait()
        assert s.contact and not s.reached  # obstruction at 0.5, target 0.0
        g.release()
        native.snap.observation.position = 1.0
        s = g.wait()
        assert s.reached and not s.contact


def test_wait_timeout_preserves_operation(native):
    with sdk.Gripper() as g:
        g.move(1.0)
        with pytest.raises(sdk.TimeoutError):
            g.wait(timeout=0.001)
        native.controller.stop.assert_not_called()


def test_wait_rejects_stale_feedback_and_fault(native):
    with sdk.Gripper() as g:
        g.grasp()
        native.snap.observation.age_ms = 400
        with pytest.raises(RuntimeError, match="stale"):
            g.wait()
        native.snap.observation.age_ms = 5
        native.snap.fault_reason = "motor fault"
        with pytest.raises(RuntimeError, match="motor fault"):
            g.release()


def test_hold_uses_current_controller_target(native):
    with sdk.Gripper() as g:
        g.hold()
        native.controller.hold_position.assert_called_once()
        assert g.state.target == 0.5


def test_legacy_named_and_wildcard_exports_still_resolve():
    assert sdk.ControlLoop is sdk.advanced.ControlLoop
    assert sdk.Frame is sdk.advanced.Frame
    imported = {}
    exec("from xense.taccap import *", imported)
    for name in sdk.advanced.__all__:
        assert imported[name] is getattr(sdk.advanced, name)
    with pytest.raises(AttributeError):
        getattr(sdk, "does_not_exist")


@pytest.mark.parametrize("operation,position,contact,expected", [
    ("move", .5, True, "blocked"),
    ("move", .2, False, "reached"),
    ("grasp", .5, True, "contact"),
    ("grasp", 0, False, "no_object"),
    ("grasp", 0, True, "no_object"),  # end stop is not a verified grasp
    ("release", .5, True, "blocked"),
    ("release", 1, False, "reached"),
])
def test_operation_specific_results(native, operation, position, contact, expected):
    with sdk.Gripper() as g:
        if operation == "move":
            g.move(.2)
        else:
            getattr(g, operation)()
        native.snap.observation.position = position
        if contact:
            native.snap.state = api._native.ForcePositionState.HOLDING_FORCE
            native.snap.blocked = operation != "grasp"
        result = g.wait_result()
        assert isinstance(result, sdk.GripperResult)
        assert (result.operation, result.status) == (operation, expected)
        assert result.state.position == position
        native.controller.stop.assert_not_called()


def test_blocking_move_returns_result(native):
    with sdk.Gripper() as g:
        result = g.move(.5, wait=True)
        assert result.operation == "move" and result.status == "reached"


def test_blocking_grasp_returns_empty_result(native):
    native.snap.observation.position = 0
    with sdk.Gripper() as g:
        result = g.grasp(wait=True)
        assert result.status == "no_object"


def test_partial_release_and_disable(native):
    with sdk.Gripper() as g:
        result = g.release(.5, wait=True)
        assert result.operation == "release" and result.status == "reached"
        g.disable()
        native.controller.stop.assert_called_once()
        assert g.state.target is None


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_blocking_bad_timeout_never_starts(native, timeout):
    with sdk.Gripper() as g:
        with pytest.raises(ValueError):
            g.move(.2, wait=True, timeout=timeout)
        native.controls.assert_not_called()


def test_invalid_wait_never_starts(native):
    with sdk.Gripper() as g:
        with pytest.raises(TypeError):
            g.move(.2, wait="false")
        native.controls.assert_not_called()


def test_blocking_timeout_keeps_control_active(native):
    with sdk.Gripper() as g:
        with pytest.raises(sdk.TimeoutError):
            g.move(1, wait=True, timeout=.001)
        native.controller.stop.assert_not_called()


@pytest.mark.parametrize("replacement", ["release", "stop", "hold", "close"])
def test_wait_does_not_complete_a_replacement_operation(native, monkeypatch, replacement):
    with sdk.Gripper() as g:
        g.move(1)
        monkeypatch.setattr(api.time, "sleep", lambda _: getattr(g, replacement)())
        with pytest.raises(RuntimeError, match="stopped or replaced"):
            g.wait_result()


def test_result_fault_is_not_a_success(native):
    with sdk.Gripper() as g:
        g.grasp()
        native.snap.fault_reason = "motor fault"
        with pytest.raises(RuntimeError, match="motor fault"):
            g.wait_result()


def test_modes_share_one_runtime_and_use_distinct_native_targets(native):
    with sdk.Gripper() as g:
        g.move(.7, torque_limit_nm=.8, speed_rad_s=.4)
        native.controller.set_position_target.assert_called_with(.7, .8, .4)
        assert g.state.control_mode == "position"
        g.grasp(torque_nm=.4, speed_rad_s=.2)
        native.controller.set_grasp_target.assert_called_with(0., .4, .2)
        assert g.state.control_mode == "grasp"
        g.impedance(.6, kp=8, kd=.4, feedforward_torque_nm=-.1, velocity_rad_s=.2)
        native.controller.set_impedance_target.assert_called_with(.6, 8, .4, -.1, .2, 1.)
        assert g.state.control_mode == "impedance"
        assert not g.state.reached
        with pytest.raises(sdk.GripperError, match="sustained"):
            g.wait_result()
        g.hold(wait=True)
        assert g.state.control_mode == "position"
        native.controller.start.assert_called_once()
        native.controller.stop.assert_not_called()
        native.device.motor.disable.assert_not_called()


@pytest.mark.parametrize("kwargs", [
    {"kp": -1}, {"kp": float("nan")}, {"kp": 501}, {"kd": 6},
    {"velocity_rad_s": 51}, {"feedforward_torque_nm": 1.1},
    {"torque_limit_nm": 0}, {"torque_limit_nm": 1.3},
])
def test_bad_impedance_rejected_without_interrupting_current_operation(native, kwargs):
    params = {"kp": 8, "kd": .5}
    params.update(kwargs)
    with sdk.Gripper() as g:
        g.grasp()
        generation = g._generation
        with pytest.raises(ValueError):
            g.impedance(.5, **params)
        native.controller.set_impedance_target.assert_not_called()
        native.controller.stop.assert_not_called()
        assert g._generation == generation
        assert g.state.control_mode == "grasp"


@pytest.mark.parametrize("method,kwargs", [
    ("move", {"speed_rad_s": 0}), ("move", {"torque_limit_nm": 1.21}),
    ("move", {"torque_nm": .4, "torque_limit_nm": .5}),
    ("release", {"speed_rad_s": float("inf")}), ("grasp", {"speed_rad_s": 51}),
    ("hold", {"timeout": float("nan")}),
])
def test_bad_parameters_before_start(native, method, kwargs):
    with sdk.Gripper() as g:
        with pytest.raises(ValueError):
            getattr(g, method)(*([.7] if method == "move" else []), **kwargs)
        native.controls.assert_not_called()


@pytest.mark.parametrize("submitted,seq", [(0, 100), (1, 1)])
def test_old_or_unsent_feedback_never_completes(native, submitted, seq):
    with sdk.Gripper(settle_time=0) as g:
        g.move(.5)
        native.snap.submitted_sequence = submitted
        native.snap.submission_observation_sequence = 1
        native.snap.observation.seq = seq
        native.controller.snapshot.side_effect = None
        native.controller.snapshot.return_value = native.snap
        with pytest.raises(sdk.TimeoutError):
            g.wait_result(timeout=.01)


def test_settle_requires_more_than_one_fresh_sample(native):
    with sdk.Gripper(settle_time=.01) as g:
        g.move(.5)
        native.snap.observation.seq = native.snap.submission_observation_sequence + 1
        native.controller.snapshot.side_effect = None
        native.controller.snapshot.return_value = native.snap
        with pytest.raises(sdk.TimeoutError):
            g.wait_result(timeout=.02)


@pytest.mark.parametrize("position,expected", [(.0167, "no_object"), (.04, "contact")])
def test_empty_close_margin_is_not_reported_as_a_grasp(native, position, expected):
    with sdk.Gripper() as g:
        g.grasp()
        native.snap.observation.position = position
        native.snap.state = api._native.ForcePositionState.HOLDING_FORCE
        assert g.wait_result().status == expected


def test_idle_state_sees_firmware_latch(native):
    native.execution.owner = 4
    native.execution.last_error = -22
    with sdk.Gripper() as g:
        assert "-22" in g.state.fault
        with pytest.raises(sdk.FaultError, match="-22"):
            g.move(.5)
        native.controls.assert_not_called()


def test_terminal_result_checks_firmware_rejection(native):
    with sdk.Gripper() as g:
        g.move(.5)
        native.execution.owner = 4
        native.execution.last_error = -22
        with pytest.raises(sdk.FaultError, match="-22"):
            g.wait_result()
        with pytest.raises(sdk.FaultError):
            g.impedance(.5, kp=1, kd=1)


def test_motor_fault_bit_rejected_before_policy_next_tick(native):
    with sdk.Gripper() as g:
        g.grasp()
        native.snap.observation.status = 2
        with pytest.raises(sdk.FaultError):
            g.move(.5)
        native.controller.set_position_target.assert_not_called()


def test_failed_clear_stays_faulted(native):
    with sdk.Gripper() as g:
        native.execution.owner = 4
        native.execution.last_error = -22
        with pytest.raises(sdk.FaultError, match="did not recover"):
            g.clear_fault()
        with pytest.raises(sdk.FaultError):
            g.grasp()


def test_discovery_info_and_limits(native):
    devices = sdk.Gripper.discover()
    assert devices == (sdk.GripperDevice("/dev/fake", "TC-test", "right"),)
    native.scan.reset_mock()
    with sdk.Gripper(devices[0]) as g:
        native.scan.assert_not_called()
        assert g.info.firmware_version == (1, 1, 7)
        assert g.info.motor_travel_rad == 1.3
        assert g.info.control_modes == ("position", "grasp", "impedance")
        assert g.limits.max_torque_nm == 1.2
        assert g.diagnostics().effective_torque_limit_nm == 1.0
        with pytest.raises(AttributeError):
            g.info.device = "bad"


def test_threaded_wait_is_interrupted_by_replacement(native):
    import threading
    entered = threading.Event()
    completed = threading.Event()
    errors = []
    with sdk.Gripper() as g:
        g.move(.9)
        def waiter():
            entered.set()
            try:
                g.wait_result()
            except Exception as exc:
                errors.append(exc)
            finally:
                completed.set()
        thread = threading.Thread(target=waiter)
        thread.start()
        assert entered.wait(1)
        # Ensure the waiter has evaluated its initial target before replacement.
        import time
        time.sleep(.03)
        g.release()
        assert completed.wait(1)
        thread.join()
        assert len(errors) == 1 and isinstance(errors[0], sdk.OperationInterrupted)


def test_disable_failure_is_visible_and_requires_recovery(native):
    g = sdk.Gripper()
    g.move(.9)
    native.controller.stop.side_effect = RuntimeError("disable NACK")
    with pytest.raises(RuntimeError, match="NACK"):
        g.disable()
    assert "state unknown" in g.state.fault
    with pytest.raises(sdk.FaultError):
        g.move(.1)
    native.execution.owner = 0
    g.clear_fault()
    g.grasp()
    native.controller.stop.side_effect = None
    g.close()


@pytest.mark.parametrize("owner", [0, 1, 2, 3, 4])
def test_passive_and_busy_contexts_do_not_disable_other_owners(native, owner):
    native.execution.owner = owner
    with sdk.Gripper() as g:
        if owner:
            with pytest.raises(sdk.GripperError):
                g.grasp()
        else:
            assert g.info.device == "/dev/fake"
    native.controller.stop.assert_not_called()
    native.device.motor.disable.assert_not_called()
    native.device.transport.stop.assert_called_once()


def test_startup_cleanup_failure_is_retried_on_close(native):
    native.controller.start.side_effect = RuntimeError("start failed")
    native.controller.stop.side_effect = RuntimeError("disable failed")
    g = sdk.Gripper()
    with pytest.raises(RuntimeError, match="disable failed"):
        g.move(.9)
    with pytest.raises(sdk.FaultError, match="state unknown"):
        g.grasp()
    g.close()
    native.device.motor.disable.assert_called_once()


def test_all_legacy_public_attributes_are_available_at_package_root():
    # Named imports historically worked even for names missing from __all__.
    # Checking only the export list against itself misses these regressions.
    for name, value in vars(sdk.advanced).items():
        if not name.startswith("_"):
            assert getattr(sdk, name) is value, name
            assert name in sdk.__all__, name


def test_ota_cli_help_imports_without_opening_hardware(monkeypatch, capsys):
    import runpy
    import sys
    from pathlib import Path

    examples = Path(__file__).resolve().parents[1] / "examples"
    monkeypatch.syspath_prepend(str(examples))
    monkeypatch.setattr(sys, "argv", ["ota_update.py", "--help"])
    monkeypatch.setattr(sdk.advanced._taccap_native, "scan_grippers",
                        Mock(side_effect=AssertionError("help must not scan hardware")))
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(examples / "ota_update.py"), run_name="__main__")
    assert exc.value.code == 0
    assert "--target-version" in capsys.readouterr().out


@pytest.mark.parametrize("status", ["contact", "no_object"])
def test_grasp_demo_holds_until_user_release(monkeypatch, status):
    import runpy
    import sys
    from pathlib import Path
    from unittest.mock import MagicMock

    events = []
    device = MagicMock()
    device.__enter__.return_value = device
    device.__exit__.side_effect = lambda *args: events.append("close") or False
    device.release.side_effect = lambda **kwargs: events.append("release") or NS(status="reached")
    device.grasp.side_effect = lambda **kwargs: events.append("grasp") or NS(
        operation="grasp", status=status, state="feedback")
    monkeypatch.setattr(sdk, "Gripper", Mock(return_value=device))
    monkeypatch.setattr("builtins.input", lambda prompt: events.append("input") or "")
    monkeypatch.setattr(sys, "argv", ["gripper.py", "left", "--grasp", "--hold"])
    script = Path(__file__).resolve().parents[1] / "examples" / "gripper.py"
    runpy.run_path(str(script), run_name="__main__")
    assert events == ["release", "input", "grasp"] + (
        ["input"] if status == "contact" else []) + ["release", "close"]


def test_timeout_reports_observation_without_disabling(native):
    with sdk.Gripper() as g:
        g.release()
        with pytest.raises(sdk.TimeoutError, match="position=0.5000.*target=1.0") as exc:
            g.wait_result(timeout=.001)
        assert exc.value.state.position == .5
        assert exc.value.state.target == 1.
        native.controller.stop.assert_not_called()


def test_position_defaults_and_cached_timing(native):
    with sdk.Gripper(torque_limit_nm=.6, speed_rad_s=.8, update_hz=50) as g:
        assert g.timing is None
        g.start()
        g.set_position(.3)
        g.set_position(.7)
        native.controller.start.assert_called_once()
        native.controller.set_position_target.assert_called_with(.7, .6, .8)
        cfg = native.controls.call_args.args[1]
        assert cfg.motor_stream_hz == 50
        assert cfg.grasp_torque_nm == pytest.approx(.6)
        queries = native.device.motor.execution_status.call_count
        assert g.timing.feedback_hz == 100
        assert g.timing.command_hz == 98
        assert native.device.motor.execution_status.call_count == queries


def test_wait_accepts_compensated_target_outside_nominal_tolerance(native):
    native.device.firmware_version.patch = 8
    with sdk.Gripper(close_compensation_rad=.03, settle_time=0) as g:
        g.move(.5)
        # At the compensated midpoint: 0.015 rad past the nominal target.
        native.snap.observation.position = .5 - .015 / 1.3
        native.snap.target_error_rad = 0.
        assert abs(g.state.position_error) > g.state.position_tolerance
        assert g.wait_result(timeout=.01).status == "reached"


def test_wait_rejects_nominal_zero_before_compensated_target(native):
    native.device.firmware_version.patch = 8
    with sdk.Gripper(close_compensation_rad=.03, settle_time=0) as g:
        g.move(0)
        native.snap.observation.position = 0.
        native.snap.target_error_rad = -.03
        assert not g.state.reached
        with pytest.raises(sdk.TimeoutError):
            g.wait_result(timeout=.001)


@pytest.mark.parametrize("kwargs", [{"update_hz": hz} for hz in (0, 10, 30, 200, 50., True)] +
                         [{"close_compensation_rad": v} for v in (-.01, .06, float("nan"))])
def test_bad_stream_configuration_does_not_connect(native, kwargs):
    with pytest.raises(ValueError):
        sdk.Gripper(**kwargs)
    native.factory.assert_not_called()


def test_position_example_sequence_and_compensation(native, monkeypatch, capsys):
    import runpy
    import sys
    from pathlib import Path
    native.device.firmware_version.patch = 8
    example = Path(__file__).parents[1] / "examples" / "position_control.py"
    monkeypatch.setattr(sys, "argv", [str(example), "/dev/fake", "--sequence", "1,0,1",
                                     "--dwell", ".001", "--close-compensation-rad", ".03"])
    monkeypatch.setattr("builtins.input", lambda _: "")
    runpy.run_path(str(example), run_name="__main__")
    assert [c.args[0] for c in native.controller.set_position_target.call_args_list] == [1, 0, 1]
    assert native.controls.call_args.args[1].close_compensation_rad == pytest.approx(.03)
    assert "last_submit_latency_ms" in capsys.readouterr().out
    native.controller.stop.assert_called_once()

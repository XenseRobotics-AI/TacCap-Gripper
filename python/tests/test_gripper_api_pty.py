"""Run the customer API through pybind11, native control and a simulated UART."""
import os
import pty
import select
import struct
import threading
import time

import pytest
from xense.taccap import Gripper, advanced as api


@pytest.fixture(params=[7, 8], ids=["fw117", "fw118"])
def follower_uart(request):
    master, slave = pty.openpty()
    state = {"submits": 0, "disables": 0, "stream": False, "frozen": False, "error": None,
             "owner": 0, "motor_pos": .6, "torque": 0., "motor_error": 0, "last_error": 0,
             "commands": [], "stream_starts": 0, "patch": request.param, "mask": 0,
             "sample_age_ms": 0, "omit_execution": False, "execution_queries": 0,
             "drop_active_query": False, "period": .01}
    stop = threading.Event()

    def send(frame, payload):
        os.write(master, api.pack_frame(api.Address.MCU, frame.seq, api.FrameType.ACK,
                                        frame.cmd, payload))

    def execution():
        return struct.pack("<4BHHIIiI11f", state["owner"], 0, 0, 4, 500, 0,
                           state["submits"], state["submits"], state["last_error"], 0, *([0]*10), 1.0)

    def status(extended=False):
        packet = struct.pack("<4fH3fB", max(0, state["motor_pos"]), 0, state["torque"], 25,
                           1 | state["motor_error"], .6, 0, 0, 4)
        if extended and state["mask"] & 0x20 and not state["omit_execution"]:
            packet += execution() + struct.pack("<If", state["sample_age_ms"], state["motor_pos"])
        return packet

    def run():
        parser = api.FrameParser()
        next_status = time.monotonic()
        try:
            while not stop.is_set():
                if select.select([master], [], [], 0.005)[0]:
                    parser.feed(os.read(master, 4096))
                    while (frame := parser.pop()) is not None:
                        cmd = frame.cmd
                        if cmd == api.Cmd.GetVersion:
                            send(frame, bytes([1, 1, state["patch"], 0]))
                        elif cmd == api.Cmd.GetSn:
                            send(frame, b"TCGU01A28Z0001s")
                        elif cmd == api.Cmd.GetGripperConfig:
                            send(frame, struct.pack("<IHHff16s", 0x47435047, 1, 1, 1.3, 0, bytes(16)))
                        elif cmd == api.Cmd.GetMotorExecutionStatus:
                            state["execution_queries"] += 1
                            if not (state["stream"] and state["drop_active_query"]):
                                send(frame, execution())
                        elif cmd == api.Cmd.MotorGetStartupLimitTorque:
                            send(frame, struct.pack("<f", 5.5))
                        elif int(cmd) == 0x69:
                            send(frame, struct.pack("<IHH4f3H2B", 0x4743414C, 1, 1, .35, .35, .25, .35, 30, 0, 0, 1, 1))
                        elif cmd == api.Cmd.GetMotorStatus:
                            send(frame, status())
                        elif cmd == api.Cmd.StartStream:
                            state["mask"] = struct.unpack_from("<H", frame.payload)[0]
                            state["period"] = 1 / struct.unpack_from("<H", frame.payload, 9)[0]
                            state["stream"] = True
                            state["stream_starts"] += 1
                            send(frame, b"\0")
                        elif cmd == api.Cmd.StopStream:
                            state["stream"] = False
                            send(frame, b"\0")
                        elif cmd == api.Cmd.MotorDisable:
                            state["disables"] += 1
                            state["owner"] = 0
                            send(frame, b"\0")
                        elif cmd == api.Cmd.MotorImpedanceCtrl:
                            state["submits"] += 1
                            state["owner"] = 1
                            state["commands"].append(struct.unpack("<5f", frame.payload))
                        else:
                            raise AssertionError(f"Unexpected command: {cmd}")
                now = time.monotonic()
                if state["stream"] and not state["frozen"] and now >= next_status:
                    os.write(master, api.pack_frame(api.Address.MCU, 0, api.FrameType.DATA,
                                                    api.Cmd.GetMotorStatus, status(True)))
                    next_status = now + state["period"]
        except BaseException as exc:
            state["error"] = exc

    thread = threading.Thread(target=run)
    thread.start()
    try:
        yield os.ttyname(slave), state
    finally:
        stop.set()
        thread.join(timeout=2)
        os.close(master)
        os.close(slave)
        assert not thread.is_alive()
        assert state["error"] is None, state["error"]


def command_torque(command, raw):
    p, kp, kd, ff, v = command
    return kp * (p - raw) + kd * v + ff  # fixture reports zero velocity


def wait_for(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.01)
    return predicate()


def test_customer_context_moves_reads_and_disables(follower_uart):
    path, uart = follower_uart
    with Gripper(path) as g:
        assert uart["submits"] == 0
        g.move(.8)
        assert wait_for(lambda: uart["submits"] >= 3)
        state = g.state
        assert state.mode == "moving_position"
        assert state.position == pytest.approx(.6 / 1.3)
        assert state.target == pytest.approx(.8)
        assert state.fault is None
        g.hold()
        assert g.wait().reached
    assert uart["disables"] >= 1
    assert not uart["stream"]
    before = uart["submits"]
    time.sleep(.05)
    assert uart["submits"] == before


def test_customer_sees_frozen_feedback_fault(follower_uart):
    path, uart = follower_uart
    with Gripper(path) as g:
        g.grasp()
        assert wait_for(lambda: uart["submits"] >= 3)
        uart["frozen"] = True
        assert wait_for(lambda: g.state.fault is not None)
        with pytest.raises(RuntimeError, match="fault"):
            g.wait()
        assert wait_for(lambda: uart["disables"] >= 1)


def test_blocking_result_through_native_controller(follower_uart):
    path, uart = follower_uart
    with Gripper(path) as g:
        result = g.move(.6 / 1.3, wait=True, timeout=2)
        assert result.operation == "move"
        assert result.status == "reached"
        assert result.state.position == pytest.approx(.6 / 1.3)
        assert wait_for(lambda: uart["submits"] >= 3)
        g.disable()
        assert uart["disables"] >= 1
        assert not uart["stream"]


def test_mode_switches_do_not_disable_or_restart_stream(follower_uart):
    path, uart = follower_uart
    with Gripper(path) as g:
        g.move(.8, torque_limit_nm=.6, speed_rad_s=.4)
        assert wait_for(lambda: g.state.ready)
        assert g.state.control_mode == "position"
        first_command = g.state.command_id
        g.grasp(torque_nm=.3)
        assert wait_for(lambda: g.state.ready and g.state.command_id > first_command)
        assert g.state.control_mode == "grasp"
        assert uart["commands"][-1][1] == 0  # velocity-damped MIT approach, no position gain
        g.impedance(.6, kp=4, kd=.4, feedforward_torque_nm=.1, torque_limit_nm=.7)
        assert wait_for(lambda: g.state.ready and g.state.control_mode == "impedance")
        assert uart["commands"][-1][1] > 0
        assert uart["disables"] == 0
        assert uart["stream_starts"] == 1
        result = g.hold(wait=True, timeout=2)
        assert result.status == "reached"
        assert result.state.control_mode == "position"
        assert uart["disables"] == 0


@pytest.mark.parametrize("target", [.1, .9])
def test_position_obstruction_both_directions(follower_uart, target):
    path, uart = follower_uart
    uart["torque"] = .2  # stationary feedback at .6 rad: physical obstruction
    with Gripper(path) as g:
        with pytest.raises(api.TimeoutError):
            g.move(target, wait=True, timeout=.5)
        assert not g.state.blocked and not g.state.reached
        direction = 1 if target > .6 / 1.3 else -1
        assert command_torque(uart["commands"][-1], .6) * direction > 0
        # Repeated identical targets keep gripping torque; reversal releases.
        for _ in range(20):
            g.set_position(target)
        assert wait_for(lambda: g.state.ready)
        assert command_torque(uart["commands"][-1], .6) * direction > 0
        g.set_position(1 - target)
        assert wait_for(lambda: command_torque(uart["commands"][-1], .6) * direction < 0)
        assert uart["disables"] == 0


def test_grasp_contact_is_not_reported_as_move_success(follower_uart):
    path, uart = follower_uart
    uart["torque"] = .2
    with Gripper(path) as g:
        result = g.grasp(torque_nm=.3, wait=True, timeout=2)
        assert result.status == "contact"
        assert result.state.control_mode == "grasp"
        assert uart["commands"][-1][1:3] == (0, 0)  # pure torque hold


def test_firmware_fault_with_fresh_motor_stream_stops_impedance(follower_uart):
    path, uart = follower_uart
    with Gripper(path) as g:
        g.impedance(.5, kp=4, kd=.5)
        assert wait_for(lambda: g.state.ready)
        uart["last_error"] = -23
        assert wait_for(lambda: g.state.fault is not None)
        assert wait_for(lambda: uart["disables"] >= 1)
        assert not uart["frozen"]
        with pytest.raises(RuntimeError, match="fault"):
            g.move(.6)


def test_stream_latest_target_and_timing_without_execution_queries(follower_uart):
    path, uart = follower_uart
    if uart["patch"] < 8:
        pytest.skip("execution stream added in 1.1.8")
    with Gripper(path, update_hz=50, torque_limit_nm=.4) as g:
        g.start()
        assert wait_for(lambda: g.timing.feedback_frames >= 3)
        queries = uart["execution_queries"]
        uart["drop_active_query"] = True
        before = g.timing
        for i in range(1000):
            g.set_position(.1 + .8 * i / 999)
        assert wait_for(lambda: g.state.ready)
        time.sleep(.3)  # legacy worker would issue 0x56 here
        stats = g.timing
        assert uart["execution_queries"] == queries
        assert stats.command_frames - before.command_frames <= stats.feedback_frames - before.feedback_frames + 1
        assert stats.command_frames - before.command_frames < 100
        assert g.state.target == pytest.approx(.9)
        assert stats.execution_telemetry
        assert stats.feedback_hz > 0 and stats.command_hz > 0
        assert stats.last_submit_latency_ms >= 0
        assert g.state.effective_torque_limit_nm == pytest.approx(1.)
        assert g.state.fault is None


@pytest.mark.parametrize("failure", ["stale_can", "missing_execution"])
def test_extended_stream_rejects_invalid_feedback(follower_uart, failure):
    path, uart = follower_uart
    if uart["patch"] < 8:
        pytest.skip("execution stream added in 1.1.8")
    with Gripper(path) as g:
        g.set_position(.5)
        assert wait_for(lambda: g.state.ready)
        if failure == "stale_can":
            uart["sample_age_ms"] = 200
        else:
            uart["omit_execution"] = True
        assert wait_for(lambda: g.state.fault is not None)
        assert wait_for(lambda: uart["disables"] > 0)


def test_compensation_uses_raw_below_zero_feedback(follower_uart):
    path, uart = follower_uart
    if uart["patch"] < 8:
        with pytest.raises(RuntimeError, match="1.1.8"):
            Gripper(path, close_compensation_rad=.03)
        return
    uart["motor_pos"] = -.01
    with Gripper(path, close_compensation_rad=.03) as g:
        g.set_position(0)
        assert wait_for(lambda: g.state.ready)
        assert g.state.position == 0  # public opening remains [0,1]
        assert g.state.target_error_rad == pytest.approx(-.02)
        assert not g.state.reached
        assert wait_for(lambda: command_torque(uart["commands"][-1], -.01) < 0)
        g.set_position(1)
        assert wait_for(lambda: command_torque(uart["commands"][-1], -.01) > 0)
        assert g.hold(wait=True, timeout=2).state.reached


def test_start_holds_raw_position_when_already_below_zero(follower_uart):
    path, uart = follower_uart
    if uart["patch"] < 8:
        pytest.skip("raw position stream added in 1.1.8")
    uart["motor_pos"] = -.02
    with Gripper(path, close_compensation_rad=.03) as g:
        g.start()
        assert wait_for(lambda: uart["submits"] >= 2)
        assert all(abs(command_torque(c, -.02)) < .001 for c in uart["commands"])

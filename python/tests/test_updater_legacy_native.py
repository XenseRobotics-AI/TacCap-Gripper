"""Real SDK on an emulated 1.1.6 serial port; no connected hardware."""

import os
import pty
import select
import threading

import pytest
import xense.taccap as sdk


@pytest.fixture
def old_mcu():
    master, slave = pty.openpty()
    stopped = threading.Event()
    commands = []
    errors = []

    def serve():
        parser = sdk.FrameParser()
        try:
            while not stopped.is_set():
                if not select.select([master], [], [], 0.05)[0]:
                    continue
                parser.feed(os.read(master, 4096))
                while (frame := parser.pop()) is not None:
                    commands.append(frame.cmd)
                    command = frame.cmd
                    if command == sdk.Cmd.GetVersion:
                        data = bytes([1, 1, 6, 0])
                    elif command == sdk.Cmd.GetSn:
                        data = b"TCGU01A25Z0017s\0"
                    elif command == sdk.Cmd.MotorGetProtocol:
                        data = bytes([int(sdk.MotorProtocol.Mit)])
                    elif command in (
                        sdk.Cmd.StopStream,
                        sdk.Cmd.MotorClearFault,
                        sdk.Cmd.MotorDisable,
                    ):
                        data = b"\0"
                    else:
                        # Old firmware really lacks GetMotorModel and all
                        # newer records/diagnostics. Do not ACK them falsely.
                        command = sdk.Cmd(0)
                        data = bytes([int(sdk.ErrorCode.InvalidCmd)])
                    os.write(
                        master,
                        sdk.pack_frame(
                            sdk.Address.MCU, frame.seq, sdk.FrameType.ACK, command, data
                        ),
                    )
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield os.ttyname(slave), commands
    finally:
        stopped.set()
        thread.join(timeout=2)
        os.close(master)
        os.close(slave)
        assert not thread.is_alive() and not errors


def test_real_sdk_identifies_legacy_without_control_constructor(old_mcu):
    path, commands = old_mcu
    with sdk.LeaderGripper(
        mcu_device=path, baudrate=9600, ack_timeout_ms=100, max_retries=1
    ) as gripper:
        assert (
            gripper.firmware_version.major == 1 and gripper.firmware_version.minor == 1
        )
        assert gripper.firmware_version.patch == 6
        assert gripper.device.get_sn() == "TCGU01A25Z0017s"
    assert sdk.Cmd.GetMotorModel not in commands


def test_real_sdk_legacy_override_survives_optional_model_nacks(old_mcu):
    path, commands = old_mcu
    with sdk.FollowerGripper(
        mcu_device=path,
        baudrate=9600,
        ack_timeout_ms=100,
        max_retries=1,
        allow_outdated_firmware=True,
    ) as gripper:
        gripper.motor.clear_fault()
        gripper.motor.disable()
        assert gripper.motor.get_protocol() == sdk.MotorProtocol.Mit
    assert (
        sdk.Cmd.GetMotorModel in commands
    )  # Native constructor's optional probes NACK safely.
    assert sdk.Cmd.MotorClearFault in commands and sdk.Cmd.MotorDisable in commands
    assert set(commands) <= {
        sdk.Cmd.StopStream,
        sdk.Cmd.GetVersion,
        sdk.Cmd.GetSn,
        sdk.Cmd.GetMotorModel,
        sdk.Cmd.MotorClearFault,
        sdk.Cmd.MotorDisable,
        sdk.Cmd.MotorGetProtocol,
    }

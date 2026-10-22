"""Native V2 timestamp width and boot identity survive the Python boundary."""
import os
import pty
import select
import struct
import threading

import xense.taccap as t


def test_native_64bit_time_and_boot_session_over_pty():
    master, slave = pty.openpty()
    stopped = threading.Event()
    errors = []
    capture = (5 << 32) + 1234

    def device():
        parser = t.FrameParser()
        try:
            while not stopped.is_set():
                if not select.select([master], [], [], .02)[0]:
                    continue
                parser.feed(os.read(master, 4096))
                while (frame := parser.pop()) is not None:
                    payload = b"\x00"
                    if frame.cmd == t.Cmd.GetVersion:
                        payload = bytes([1, 2, 6, 0])
                    elif frame.cmd == t.Cmd.GetSn:
                        payload = b"TCGU01A25Z0016m\0"
                    elif frame.cmd == t.Cmd.TimeSession:
                        assert len(frame.payload) == 8
                        payload = struct.pack("<QQ", capture, 123)
                    elif frame.cmd == t.Cmd.GetImuV2:
                        raw = struct.pack("<IH10hH", capture & 0xffffffff, 15,
                                          0, 0, 1000, 0, 0, 0, 0, 0, 0, 2500, 7)
                        payload = raw + struct.pack("<QQ", capture, 123)
                    os.write(master, t.pack_frame(t.Address.MCU, frame.seq,
                                                 t.FrameType.ACK, frame.cmd, payload))
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=device)
    thread.start()
    gripper = None
    try:
        gripper = t.LeaderGripper(mcu_device=os.ttyname(slave), open_cameras=False)
        sample = gripper.imu.read_once(timeout_ms=500)
        assert sample.mcu_timestamp_us == capture
        assert sample.boot_session_id == 123
        assert sample.timestamp_is_64bit and sample.timestamp_valid
        assert not sample.timestamp_discontinuity
        assert abs(sample.accel_mps2[2] - 9.80665) < .0001
    finally:
        if gripper is not None:
            gripper.close()
        stopped.set()
        thread.join(timeout=2)
        os.close(master)
        os.close(slave)
    assert not thread.is_alive()
    assert not errors

# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""The fisheye calibration resolution, as the Python surface exposes it.

640x480 and 1920x1080 are different sensor crops, so the 8 fisheye params only
hold at the resolution they were measured at. Firmware V2.10 (leader 1.2.8 /
follower 1.2.18) records that resolution; older firmware does not, and the SDK
infers it with a rule shared verbatim with the PC calibration tools. These
tests pin the names and shapes those tools code against: tuples for sizes,
None for "no answer", and resolve_fisheye() still unpacking as the 3-tuple it
used to be.

Run with:  pytest python/tests
"""

from __future__ import annotations

import os
import pty
import select
import struct
import threading
from contextlib import contextmanager

import pytest
import xense.taccap as t

VGA = t.CameraFisheyeCal(
    213.0303, 212.7928, 321.4, 239.95, -0.0172, 0.0091, -0.0146, 0.0051
)
HD = t.CameraFisheyeCal(477.2, 476.9, 968.3, 563.1, -0.021, 0.0061, -0.0042, 0.0011)

OP_READ, OP_WRITE, OP_READ_SIZED, OP_WRITE_SIZED = 0x00, 0x01, 0x02, 0x03
ERR_INVALID_PARAM = int(t.ErrorCode.InvalidParam.value)
ERR_CAL_NOT_SET = int(t.ErrorCode.CalNotSet.value)


def _params(cal) -> bytes:
    return struct.pack(
        "<8f", cal.fx, cal.fy, cal.cx, cal.cy, cal.k1, cal.k2, cal.k3, cal.k4
    )


# ---- Hardware-free -----------------------------------------------------------


def test_infer_recognises_the_vga_reference():
    assert t.infer_fisheye_cal_size(t.FISHEYE_FALLBACK_CAL) == (640, 480)


def test_infer_recognises_1080p():
    assert t.infer_fisheye_cal_size(HD) == (1920, 1080)


def test_infer_returns_none_when_nothing_matches():
    assert t.infer_fisheye_cal_size(t.CameraFisheyeCal(477, 477, 600, 400)) is None
    assert t.infer_fisheye_cal_size(t.CameraFisheyeCal()) is None


def test_plausibility_is_the_firmware_rule():
    assert t.fisheye_cal_size_plausible(HD, 1920, 1080)
    assert not t.fisheye_cal_size_plausible(HD, 640, 480)
    assert not t.fisheye_cal_size_plausible(VGA, 640, 4097)


def test_record_defaults_to_an_unknown_size():
    r = t.FisheyeCalRecord()
    assert (r.width, r.height) == (0, 0)
    assert not r.size_known()
    r = t.FisheyeCalRecord(HD, 1920, 1080)
    assert r.size_known()
    assert r.calibration.cx == pytest.approx(968.3)


def test_undistorter_calibrated_size_keyword():
    u = t.FisheyeUndistorter(HD, 1920, 1080, calibrated_size=(1920, 1080))
    assert (u.width, u.height) == (1920, 1080)
    with pytest.raises(RuntimeError, match="must not be rescaled"):
        t.FisheyeUndistorter(HD, 640, 480, calibrated_size=(1920, 1080))
    # Omitted, the params are taken to be 640x480, as before.
    assert t.FisheyeUndistorter(VGA).width == 640
    with pytest.raises(RuntimeError):
        t.FisheyeUndistorter(HD, 1920, 1080)


# ---- Against a fake leader over a pty ----------------------------------------


class FakeFisheyeStore:
    """The firmware's 0x2B handler, V2.10 or (sized=False) older."""

    def __init__(self, sized: bool, cal=None, width: int = 0, height: int = 0):
        self.sized = sized
        self.params = _params(cal) if cal is not None else None
        self.size = (width, height)
        self.ops: list[int] = []

    def handle(self, payload: bytes) -> bytes:
        op = payload[0]
        self.ops.append(op)
        if op in (OP_READ_SIZED, OP_WRITE_SIZED) and not self.sized:
            return bytes([ERR_INVALID_PARAM])
        if op in (OP_READ, OP_READ_SIZED):
            if self.params is None:
                return bytes([ERR_CAL_NOT_SET])
            tail = struct.pack("<HH", *self.size) if op == OP_READ_SIZED else b""
            return bytes([op]) + self.params + tail
        if op == OP_WRITE:
            self.params, self.size = payload[1:33], (0, 0)
            return b"\x00"
        if op == OP_WRITE_SIZED:
            self.params, self.size = payload[1:33], struct.unpack("<HH", payload[33:37])
            return b"\x00"
        return bytes([ERR_INVALID_PARAM])


@contextmanager
def fake_leader(store: FakeFisheyeStore):
    master, slave = pty.openpty()
    stopped = threading.Event()
    errors = []

    def device():
        parser = t.FrameParser()
        try:
            while not stopped.is_set():
                if not select.select([master], [], [], 0.02)[0]:
                    continue
                parser.feed(os.read(master, 4096))
                while (frame := parser.pop()) is not None:
                    payload = b"\x00"
                    if frame.cmd == t.Cmd.GetVersion:
                        payload = bytes([1, 2, 8, 0])
                    elif frame.cmd == t.Cmd.GetSn:
                        payload = b"TCGU01A25Z0016m\0"
                    elif frame.cmd == t.Cmd.CameraFisheyeCal:
                        payload = store.handle(bytes(frame.payload))
                    os.write(
                        master,
                        t.pack_frame(
                            t.Address.MCU,
                            frame.seq,
                            t.FrameType.ACK,
                            frame.cmd,
                            payload,
                        ),
                    )
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=device)
    thread.start()
    gripper = None
    try:
        gripper = t.LeaderGripper(mcu_device=os.ttyname(slave), open_cameras=False)
        yield gripper
    finally:
        if gripper is not None:
            gripper.close()
        stopped.set()
        thread.join(timeout=2)
        os.close(master)
        os.close(slave)
    assert not errors


def test_new_firmware_records_and_reads_back_the_size():
    store = FakeFisheyeStore(sized=True)
    with fake_leader(store) as g:
        cal = g.calibration
        assert cal.supports_fisheye_size() is True
        assert cal.read_fisheye_record() is None
        cal.write_fisheye_sized(HD, 1920, 1080)
        rec = cal.read_fisheye_record()
        assert isinstance(rec, t.FisheyeCalRecord)
        assert (rec.width, rec.height) == (1920, 1080)
        resolved = cal.resolve_fisheye()
        assert isinstance(resolved, t.ResolvedFisheyeCal)
        assert resolved.calibrated_size == (1920, 1080)
        assert resolved.size_inferred is False
        assert resolved.is_reference is False
        # Still unpacks as the 3-tuple resolve_fisheye() used to return.
        c, is_reference, reason = resolved
        assert not is_reference and reason == ""
        assert c.cx == pytest.approx(968.3, abs=1e-3)
    assert store.size == (1920, 1080)


def test_old_firmware_infers_the_size_and_refuses_a_1080p_write():
    store = FakeFisheyeStore(sized=False, cal=VGA)
    with fake_leader(store) as g:
        cal = g.calibration
        assert cal.supports_fisheye_size() is False
        rec = cal.read_fisheye_record()
        assert (rec.width, rec.height) == (0, 0)
        assert not rec.size_known()
        resolved = cal.resolve_fisheye()
        assert resolved.calibrated_size == (640, 480)
        assert resolved.size_inferred is True
        with pytest.raises(RuntimeError, match="1.2.8"):
            cal.write_fisheye_sized(HD, 1920, 1080)
        cal.write_fisheye_sized(VGA, 640, 480)  # legacy fallback
    assert OP_WRITE in store.ops and OP_WRITE_SIZED not in store.ops


def test_implausible_write_is_refused_before_it_is_sent():
    store = FakeFisheyeStore(sized=True)
    with fake_leader(store) as g:
        with pytest.raises(RuntimeError, match="not plausible"):
            g.calibration.write_fisheye_sized(HD, 640, 480)
    assert store.ops == []

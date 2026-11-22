# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""The fisheye calibration resolution, as the Python surface exposes it.

640x480 and 1920x1080 are different sensor crops, so the 8 fisheye params only
hold at the resolution they were measured at. Firmware V2.10 (1.3.0-rc1 /
1.3.1) records that resolution; older firmware does not, and the SDK
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
OP_READ_SLOT, OP_CLEAR_SLOT = 0x04, 0x05  # V2.11 (dual slots)
VGA_SIZE, HD_SIZE = (640, 480), (1920, 1080)
ERR_INVALID_PARAM = int(t.ErrorCode.InvalidParam.value)
ERR_CAL_NOT_SET = int(t.ErrorCode.CalNotSet.value)
ERR_LENGTH_MISMATCH = int(t.ErrorCode.LengthMismatch.value)


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


class FakeFisheyeError(Exception):
    """A handler error. Real firmware sends it on the cmd=0 pure-ACK path
    (protocol_handler.c), not echoed with the cmd: answering the echoed way
    hid a probe that threw "NACK: InvalidParam" on a 1.2.17 follower."""

    def __init__(self, code: int):
        super().__init__(code)
        self.code = code


class FakeFisheyeStore:
    """The firmware's 0x2B handler: V2.10, (sized=False) older, or
    (slots=True) V2.11 with a VGA and an HD slot.

    In the V2.11 persona `params`/`size` are the VGA slot and `hd` the HD slot.
    """

    def __init__(
        self,
        sized: bool,
        cal=None,
        width: int = 0,
        height: int = 0,
        slots: bool = False,
        hd=None,
    ):
        self.sized = sized or slots
        self.slots = slots
        self.params = _params(cal) if cal is not None else None
        self.size = (width, height)
        self.hd = _params(hd) if hd is not None else None
        self.ops: list[int] = []

    def handle(self, payload: bytes) -> bytes:
        op = payload[0]
        self.ops.append(op)
        if self.slots:
            return self._handle_v2_11(op, payload)
        if op == OP_READ_SIZED and not self.sized:
            raise FakeFisheyeError(ERR_INVALID_PARAM)
        if op == OP_WRITE_SIZED and not self.sized:
            # Old firmware's command table caps 0x2B at 33 bytes.
            raise FakeFisheyeError(ERR_LENGTH_MISMATCH)
        if op in (OP_READ, OP_READ_SIZED):
            if self.params is None:
                raise FakeFisheyeError(ERR_CAL_NOT_SET)
            tail = struct.pack("<HH", *self.size) if op == OP_READ_SIZED else b""
            return bytes([op]) + self.params + tail
        if op == OP_WRITE:
            self.params, self.size = payload[1:33], (0, 0)
            return b"\x00"
        if op == OP_WRITE_SIZED:
            self.params, self.size = payload[1:33], struct.unpack("<HH", payload[33:37])
            return b"\x00"
        raise FakeFisheyeError(ERR_INVALID_PARAM)

    def _handle_v2_11(self, op: int, payload: bytes) -> bytes:
        # The VGA slot reports its size only when it is exactly 640x480.
        vga_size = self.size if tuple(self.size) == VGA_SIZE else (0, 0)
        if op in (OP_READ, OP_READ_SIZED):
            if len(payload) != 1:
                raise FakeFisheyeError(ERR_LENGTH_MISMATCH)
            if self.params is None:
                raise FakeFisheyeError(ERR_CAL_NOT_SET)
            tail = struct.pack("<HH", *vga_size) if op == OP_READ_SIZED else b""
            return bytes([op]) + self.params + tail
        if op in (OP_READ_SLOT, OP_CLEAR_SLOT):
            if len(payload) != 5:
                raise FakeFisheyeError(ERR_LENGTH_MISMATCH)
            size = struct.unpack("<HH", payload[1:5])
            if size not in (VGA_SIZE, HD_SIZE):
                raise FakeFisheyeError(ERR_INVALID_PARAM)
            if op == OP_CLEAR_SLOT:
                if size == VGA_SIZE:
                    self.params, self.size = None, (0, 0)
                else:
                    self.hd = None
                return b"\x00"
            if size == VGA_SIZE:
                if self.params is None:
                    raise FakeFisheyeError(ERR_CAL_NOT_SET)
                return bytes([op]) + self.params + struct.pack("<HH", *vga_size)
            if self.hd is None:
                raise FakeFisheyeError(ERR_CAL_NOT_SET)
            return bytes([op]) + self.hd + struct.pack("<HH", *HD_SIZE)
        if op == OP_WRITE:
            if len(payload) != 33:
                raise FakeFisheyeError(ERR_LENGTH_MISMATCH)
            self.params, self.size = payload[1:33], (0, 0)
            return b"\x00"
        if op == OP_WRITE_SIZED:
            if len(payload) != 37:
                raise FakeFisheyeError(ERR_LENGTH_MISMATCH)
            fx, fy, cx, cy, *_ = struct.unpack("<8f", payload[1:33])
            w, h = struct.unpack("<HH", payload[33:37])
            ok = (
                (w, h) in (VGA_SIZE, HD_SIZE)
                and fx > 0
                and fy > 0
                and abs(cx - w / 2) <= 0.15 * w
                and abs(cy - h / 2) <= 0.15 * h
            )
            if not ok:
                raise FakeFisheyeError(ERR_INVALID_PARAM)
            if (w, h) == VGA_SIZE:
                self.params, self.size = payload[1:33], (w, h)
            else:
                self.hd = payload[1:33]
            return b"\x00"
        raise FakeFisheyeError(ERR_INVALID_PARAM)


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
                    cmd = frame.cmd
                    if frame.cmd == t.Cmd.GetVersion:
                        payload = bytes([1, 2, 8, 0])
                    elif frame.cmd == t.Cmd.GetSn:
                        payload = b"TCGU01A25Z0016m\0"
                    elif frame.cmd == t.Cmd.CameraFisheyeCal:
                        try:
                            payload = store.handle(bytes(frame.payload))
                        except FakeFisheyeError as err:
                            cmd, payload = t.Cmd(0), bytes([err.code])
                    os.write(
                        master,
                        t.pack_frame(
                            t.Address.MCU,
                            frame.seq,
                            t.FrameType.ACK,
                            cmd,
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
        with pytest.raises(RuntimeError, match="1.3.1"):
            cal.write_fisheye_sized(HD, 1920, 1080)
        cal.write_fisheye_sized(VGA, 640, 480)  # legacy fallback
    assert OP_WRITE in store.ops and OP_WRITE_SIZED not in store.ops


def test_implausible_write_is_refused_before_it_is_sent():
    store = FakeFisheyeStore(sized=True)
    with fake_leader(store) as g:
        with pytest.raises(RuntimeError, match="not plausible"):
            g.calibration.write_fisheye_sized(HD, 640, 480)
    assert store.ops == []


# ---- Dual calibration slots (V2.11) ------------------------------------------


def test_slot_plausibility_is_the_v2_11_rule():
    assert t.is_fisheye_cal_slot_size(640, 480)
    assert t.is_fisheye_cal_slot_size(1920, 1080)
    assert not t.is_fisheye_cal_slot_size(1280, 720)
    assert t.fisheye_cal_slot_plausible(VGA, 640, 480)
    assert t.fisheye_cal_slot_plausible(HD, 1920, 1080)
    # V2.10's rule accepts a VGA set labelled 1080p; V2.11's does not.
    assert t.fisheye_cal_size_plausible(VGA, 1920, 1080)
    assert not t.fisheye_cal_slot_plausible(VGA, 1920, 1080)
    # Inclusive window: cx = 320 - 0.15 * 640.
    assert t.fisheye_cal_slot_plausible(
        t.CameraFisheyeCal(213, 213, 224, 240), 640, 480
    )
    assert not t.fisheye_cal_slot_plausible(
        t.CameraFisheyeCal(213, 213, 223.5, 240), 640, 480
    )


def test_v2_11_keeps_one_calibration_per_resolution():
    store = FakeFisheyeStore(sized=True, slots=True, cal=VGA, width=640, height=480)
    with fake_leader(store) as g:
        cal = g.calibration
        assert cal.supports_fisheye_slots() is True
        assert cal.read_fisheye_slot(1920, 1080) is None
        cal.write_fisheye_sized(HD, 1920, 1080)
        hd = cal.read_fisheye_slot(1920, 1080)
        assert (hd.width, hd.height) == (1920, 1080)
        assert hd.calibration.cx == pytest.approx(968.3, abs=1e-3)
        vga = cal.read_fisheye_slot(640, 480)
        assert (vga.width, vga.height) == (640, 480)
        assert vga.calibration.cx == pytest.approx(321.4, abs=1e-3)
        # 0.4.7's read still answers with the VGA slot.
        assert (cal.read_fisheye_record().width, cal.read_fisheye_record().height) == (
            640,
            480,
        )
        r = cal.resolve_fisheye_for(1920, 1080)
        assert isinstance(r, t.ResolvedFisheyeCal)
        assert r.is_reference is False
        assert r.calibrated_size == (1920, 1080)
        assert cal.resolve_fisheye_for(640, 480).calibrated_size == (640, 480)
        with pytest.raises(RuntimeError, match="only 640x480 and 1920x1080"):
            cal.read_fisheye_slot(1280, 720)
    assert store.size == (640, 480)


def test_v2_11_clear_slot():
    store = FakeFisheyeStore(
        sized=True, slots=True, cal=VGA, width=640, height=480, hd=HD
    )
    with fake_leader(store) as g:
        cal = g.calibration
        cal.clear_fisheye_slot(640, 480)
        assert cal.read_fisheye_slot(640, 480) is None
        assert cal.read_fisheye() is None
        assert cal.read_fisheye_slot(1920, 1080) is not None
        cal.clear_fisheye_slot(640, 480)  # already empty: still Ok
        r = cal.resolve_fisheye_for(640, 480)
        assert r.is_reference is True and "640x480" in r.reason
    assert store.params is None and store.size == (0, 0) and store.hd is not None


def test_v2_11_host_refuses_a_vga_set_labelled_1080p():
    store = FakeFisheyeStore(sized=True, slots=True)
    with fake_leader(store) as g:
        with pytest.raises(RuntimeError, match="within 15% of the frame centre"):
            g.calibration.write_fisheye_sized(VGA, 1920, 1080)
        with pytest.raises(RuntimeError, match="640x480 and for 1920x1080 only"):
            g.calibration.write_fisheye_sized(
                t.CameraFisheyeCal(320, 320, 641, 359), 1280, 720
            )
    assert OP_WRITE_SIZED not in store.ops


@pytest.mark.parametrize("sized", [True, False])
def test_single_record_firmware_has_no_slots(sized):
    store = FakeFisheyeStore(sized=sized, cal=HD, width=1920, height=1080)
    if not sized:
        store.size = (0, 0)
    with fake_leader(store) as g:
        cal = g.calibration
        assert cal.supports_fisheye_slots() is False
        with pytest.raises(RuntimeError, match="cannot clear one"):
            cal.clear_fisheye_slot(1920, 1080)
        if sized:
            assert cal.read_fisheye_slot(1920, 1080) is not None
            assert cal.read_fisheye_slot(640, 480) is None
            assert cal.resolve_fisheye_for(1920, 1080).is_reference is False
        r = cal.resolve_fisheye_for(640, 480)
        assert r.is_reference is True and "not 640x480" in r.reason
    assert OP_CLEAR_SLOT not in store.ops


def test_v2_11_migrated_1080p_record_stays_in_the_vga_slot():
    # A 1920x1080 record written by V2.10 rc1..rc3 stays in the VGA slot after
    # the upgrade and reads with size (0, 0).
    store = FakeFisheyeStore(sized=True, slots=True, cal=HD, width=1920, height=1080)
    with fake_leader(store) as g:
        cal = g.calibration
        rec = cal.read_fisheye_slot(640, 480)
        assert (rec.width, rec.height) == (0, 0)
        assert cal.read_fisheye_slot(1920, 1080) is None
        r = cal.resolve_fisheye_for(640, 480)
        assert r.is_reference is True
        assert r.mismatched_size == (1920, 1080)
        assert cal.resolve_fisheye_for(1920, 1080).mismatched_size is None


def test_reference_values_are_recognised():
    """Leader TCGU01A28Z0096m stored FISHEYE_FALLBACK_CAL bit-for-bit
    (2026-11-20): the factory flashed the reference, not its calibration."""
    assert t.matches_fisheye_fallback_cal(t.FISHEYE_FALLBACK_CAL)
    own = t.CameraFisheyeCal()
    for k, v in dict(
        fx=213.207,
        fy=213.594,
        cx=319.828,
        cy=245.048,
        k1=-0.017411,
        k2=-0.016309,
        k3=0.020412,
        k4=-0.008667,
    ).items():
        setattr(own, k, v)
    assert not t.matches_fisheye_fallback_cal(own)


def test_resolve_flags_a_unit_that_stores_the_reference():
    store = FakeFisheyeStore(
        sized=True, cal=t.FISHEYE_FALLBACK_CAL, width=640, height=480
    )
    with fake_leader(store) as g:
        resolved = g.calibration.resolve_fisheye()
    assert resolved.is_reference is False
    assert resolved.matches_reference is True
    assert "matches_reference=True" in repr(resolved)


def test_hd_reference_is_exported_and_matched_by_size():
    hd = t.FISHEYE_FALLBACK_CAL_HD
    assert t.FISHEYE_FALLBACK_CAL_HD_SOURCE == "avg:TCGU01A45A0016s+0017s"
    assert abs(hd.fx - 475.625034) < 1e-4 and abs(hd.cy - 552.115953) < 1e-4
    assert t.matches_fisheye_fallback_cal(hd, 1920, 1080)
    assert not t.matches_fisheye_fallback_cal(hd)  # the 1-arg form is the 640x480 one
    assert t.fisheye_cal_slot_plausible(hd, 1920, 1080)
    assert t.fisheye_fallback_cal_for(1920, 1080).fx == hd.fx
    assert t.fisheye_fallback_cal_for(640, 480).fx == t.FISHEYE_FALLBACK_CAL.fx

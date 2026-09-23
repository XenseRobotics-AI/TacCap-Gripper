# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Guards the reader/dispatcher split at the layer where it actually matters.

A Python subscriber callback must take the GIL. When callbacks ran inline on
the transport's reader thread, that GIL acquire sat directly in the read()
loop: CPython's default 5 ms switch interval is already ~3x the per-frame
budget of a 200 Hz three-source stream (~1.67 ms), so a busy main thread would
stall the reader, overflow the kernel tty buffer (4 KB on n_tty), and cost
*bytes* — the stream silently dropping to a fraction of its configured rate,
which is what customers reported as "encoder state 降频".

The C++ tests cover the queue mechanics. This one exists because only Python
reproduces the GIL half: a callback that holds the GIL must not stop the
reader from draining the port.

Hardware-free — the "firmware" is a pty the test writes frames into.

Run with:  pytest python/tests
"""

from __future__ import annotations

import os
import pty
import threading
import time

import pytest
from xense.taccap import Address, Cmd, FrameType, Transport, pack_frame


def _wait_until(pred, timeout_s: float = 5.0) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.001)
    return pred()


@pytest.fixture
def fake_mcu():
    """A pty pair: the test writes DATA frames, the Transport reads them."""
    master, slave = pty.openpty()
    dev = os.ttyname(slave)
    t = Transport(device=dev, baudrate=9600, ack_timeout_ms=50, max_retries=1)
    try:
        yield t, master
    finally:
        t.stop()
        os.close(master)
        os.close(slave)


def _encoder_frame(seq: int) -> bytes:
    return pack_frame(Address.MCU, seq, FrameType.DATA, Cmd.GetEncoder,
                      bytes(16))


LOSS_FIELDS = ("crc_errors", "resync_bytes", "parser_overflow_bytes",
               "queue_dropped", "queue_high_water", "callback_max_us")


def test_stats_expose_loss_accounting(fake_mcu):
    """The counters that make a rate drop attributable must reach Python.

    Without these, "the firmware sent fewer frames" and "we lost bytes on the
    host" are indistinguishable from user code — which is precisely the
    question this SDK could not answer when the drop was first reported.
    """
    transport, _ = fake_mcu
    stats = transport.stats
    for field in LOSS_FIELDS:
        assert getattr(stats, field) == 0, f"{field} should start at zero"
    assert "queue_dropped" in repr(stats), "repr must be usable for a log line"


def test_stop_with_a_python_subscriber_does_not_crash(fake_mcu):
    """Regression: Transport.subscribe used a bare make_shared.

    stop() runs with the GIL released, so dropping the subscriptions decref'd
    a Python object from a GIL-less thread and segfaulted — on every shutdown
    with a live raw subscription. The callback must be owned by the GIL-safe
    holder instead.
    """
    transport, master = fake_mcu
    transport.subscribe(Cmd.GetEncoder, lambda frame: None)
    os.write(master, _encoder_frame(1))
    assert _wait_until(lambda: transport.stats.frames_received >= 1)
    transport.stop()          # must not segfault; fixture stops again


def test_slow_python_callback_does_not_stall_the_reader(fake_mcu):
    """The regression that caused the customer-visible rate drop.

    The callback parks while still holding the GIL. The reader thread must go
    on reading and parsing the rest of the burst regardless — pre-dispatcher
    it stopped dead at the first frame.
    """
    transport, master = fake_mcu

    entered = threading.Event()
    release = threading.Event()
    seen: list[int] = []

    def on_encoder(frame):
        if not entered.is_set():
            entered.set()
            release.wait(5.0)      # parked *with the GIL held*
        seen.append(frame.seq)

    transport.subscribe(Cmd.GetEncoder, on_encoder)

    n_frames = 40
    os.write(master, _encoder_frame(1))
    assert entered.wait(5.0), "callback never fired"
    for i in range(2, n_frames + 1):
        os.write(master, _encoder_frame(i))

    # Callback is still parked here.
    assert _wait_until(lambda: transport.stats.frames_received >= n_frames), (
        "reader stalled behind a GIL-holding callback: frames_received="
        f"{transport.stats.frames_received} of {n_frames}"
    )
    stats = transport.stats
    assert stats.crc_errors == 0, "bytes were lost while the callback was parked"
    assert stats.queue_dropped == 0, "40 frames must fit the 256-deep queue"

    release.set()
    assert _wait_until(lambda: len(seen) == n_frames)
    assert seen == list(range(1, n_frames + 1)), "frames were reordered"
    assert transport.stats.callback_max_us > 0, (
        "callback_max_us must attribute the stall to the callback"
    )


def test_queue_depth_is_configurable_and_drops_oldest(fake_mcu):
    """A persistently slow consumer loses history, not currency.

    These are state samples: the newest one is the useful one, so the queue
    evicts from the front and says so via queue_dropped.
    """
    transport, _ = fake_mcu
    transport.stop()   # replace with a shallow-queue instance

    master, slave = pty.openpty()
    t = Transport(device=os.ttyname(slave), baudrate=9600,
                  dispatch_queue_frames=4)
    try:
        entered = threading.Event()
        release = threading.Event()
        seen: list[int] = []

        def on_encoder(frame):
            if not entered.is_set():
                entered.set()
                release.wait(5.0)
            seen.append(frame.seq)

        t.subscribe(Cmd.GetEncoder, on_encoder)
        os.write(master, _encoder_frame(1))
        assert entered.wait(5.0)
        for i in range(2, 21):
            os.write(master, _encoder_frame(i))
        assert _wait_until(lambda: t.stats.frames_received >= 20)

        release.set()
        assert _wait_until(lambda: len(seen) == 5)
        assert seen == [1, 17, 18, 19, 20], f"expected the freshest four, got {seen}"
        assert t.stats.queue_dropped == 15
        assert t.stats.queue_high_water == 4
    finally:
        t.stop()
        os.close(master)
        os.close(slave)

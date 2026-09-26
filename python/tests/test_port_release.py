# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""A port is released by close(), not by stop(), and not by losing interest.

Ports are opened exclusively (SDK 0.3.3). In Python the object's destructor
runs only when the last reference goes away, which a GUI or a worker thread can
postpone indefinitely -- tc-gu-01-pc hit exactly this: after "disconnect" a
background thread still held the gripper, so the next scan reported the port
in_use and reconnecting failed. close() is the explicit release; stop() only
joins the threads.

pty-backed; no hardware.
"""

from __future__ import annotations

import os
import pty

import pytest
from xense.taccap import IoError, Transport


@pytest.fixture
def tty():
    master, slave = pty.openpty()
    yield os.ttyname(slave)
    os.close(master)
    os.close(slave)


def _open(dev: str) -> Transport:
    return Transport(device=dev, baudrate=9600, ack_timeout_ms=50, max_retries=1)


def test_a_second_handle_on_an_open_port_is_refused(tty):
    t = _open(tty)
    with pytest.raises(IoError):
        _open(tty)
    t.close()


def test_close_releases_the_port_while_the_object_is_still_referenced(tty):
    t = _open(tty)
    assert t.is_open
    t.close()
    assert not t.is_open
    again = _open(tty)  # `t` is still alive; the port must be free anyway
    again.close()


def test_stop_alone_keeps_the_port(tty):
    t = _open(tty)
    t.stop()
    assert t.is_open
    with pytest.raises(IoError):
        _open(tty)
    t.close()


def test_close_is_idempotent(tty):
    t = _open(tty)
    t.close()
    t.close()
    assert not t.is_open

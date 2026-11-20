# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Guards against zero-stride numpy views escaping the pybind11 bindings.

`py::array_t<T> a(n)` — a single integer, not a shape container — picks a
different overload on pybind11 2.9 and yields a shape-(n,) array with stride
0: a broadcast view of element [0]. Writes to p[1], p[2], ... all land on the
same address, so the array reads back as [x, x, x, ...] with no error
anywhere. It is silent, and it corrupts data rather than crashing.

This bit `ImuSample.accel_mps2` / `.gyro_radps` / `.mag_uT` (every Python IMU
read returned the x component three times) and `CameraFisheyeCal.D`. The fix
is to always spell the shape as a container.

Run with:  pytest python/tests
"""

from __future__ import annotations

import os

import numpy as np
import pytest
from xense.taccap import CameraFisheyeCal


def assert_not_broadcast(arr: np.ndarray, what: str) -> None:
    """A real buffer has non-zero strides on every non-degenerate axis."""
    assert arr.ndim >= 1, f"{what}: expected an array"
    for axis, (dim, stride) in enumerate(zip(arr.shape, arr.strides)):
        if dim > 1:
            assert stride != 0, (
                f"{what}: axis {axis} has stride 0 — this is a broadcast view "
                f"of a single element, not a real buffer. Check that the "
                f"binding spells the shape as a container, not array_t<T>(n)."
            )


# ---- Hardware-free: CameraFisheyeCal owns its own K / D buffers -------------


def test_fisheye_d_is_a_real_buffer():
    cal = CameraFisheyeCal(fx=1, fy=2, cx=3, cy=4, k1=10, k2=20, k3=30, k4=40)
    assert_not_broadcast(cal.D, "CameraFisheyeCal.D")
    assert cal.D.tolist() == [10.0, 20.0, 30.0, 40.0]


def test_fisheye_k_is_a_real_buffer():
    cal = CameraFisheyeCal(fx=1, fy=2, cx=3, cy=4)
    assert_not_broadcast(cal.K, "CameraFisheyeCal.K")
    assert cal.K.tolist() == [[1.0, 0.0, 3.0], [0.0, 2.0, 4.0], [0.0, 0.0, 1.0]]


# ---- Needs a gripper: the IMU vectors go through make_vec3 ------------------


@pytest.fixture(scope="module")
def any_gripper():
    """A *leader*, specifically -- the IMU is leader-only -- and only the one
    named by TACCAP_TEST_LEADER (a /dev/serial/by-id/...-if02 path).

    This used to call scan_grippers() whenever a gripper was attached. That
    opens EVERY gripper port and sends StopStream to each, so a plain
    `pytest python/tests` on a shared bench interrupted another session's
    leader acceptance mid-run (2026-11-20). Hardware is now opt-in and opens
    exactly one port:

        TACCAP_TEST_LEADER=/dev/serial/by-id/usb-1a86_..._-if02 pytest python/tests
    """
    from xense.taccap import LeaderGripper

    dev = os.environ.get("TACCAP_TEST_LEADER")
    if not dev:
        pytest.skip("hardware test: set TACCAP_TEST_LEADER to a leader's by-id path")
    if not dev.startswith("/dev/serial/by-id/"):
        pytest.fail("TACCAP_TEST_LEADER must be a /dev/serial/by-id/ path, not ttyACMn")

    g = LeaderGripper(mcu_device=dev, open_cameras=False)
    yield g
    g.transport.stop()


def test_imu_vectors_are_real_buffers(any_gripper):
    s = any_gripper.imu.read_once()
    for name in ("accel_mps2", "gyro_radps", "mag_uT"):
        assert_not_broadcast(getattr(s, name), f"ImuSample.{name}")


def test_accel_magnitude_is_about_one_g(any_gripper):
    """A stride-0 accel reads as [x, x, x], whose magnitude is |x|*sqrt(3) —
    for a gripper at rest that lands far from g, so this catches the bug even
    if someone 'fixes' the stride check away."""
    s = any_gripper.imu.read_once()
    mag = float(np.linalg.norm(s.accel_mps2))
    assert 8.0 < mag < 11.5, (
        f"|accel| = {mag:.3f} m/s^2 is not ~9.81 — the gripper is either "
        f"being moved, or the vector is not x/y/z."
    )

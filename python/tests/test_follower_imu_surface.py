"""The Python opt-in stays optional; the leader's existing signature is intact."""

import re

import xense.taccap as t


def test_follower_keeps_motor_default_and_opts_in_to_imu():
    doc = t.FollowerGripper.start_streaming.__doc__
    assert re.search(r"motor_hz: [^,]* = 100", doc)
    assert re.search(r"imu_hz: [^,)]* = 0", doc)


def test_leader_signature_is_unchanged():
    doc = t.LeaderGripper.start_streaming.__doc__
    assert re.search(r"imu_hz: [^,]* = 100", doc)
    assert re.search(r"encoder_hz: [^,)]* = 100", doc)


def test_calibration_has_explicit_long_timeout():
    doc = t.IMU.calibrate_accel_gyro.__doc__
    assert "16000" in doc
    assert "disable" in doc

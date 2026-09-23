# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Pins the Python-facing config surface of the two controllers.

This exists because the surface itself is the contract. `ForcePositionConfig`
went from 16 fields to 6, and the sister repo `taccap_gripper_ros2` plus every
customer script talks to the controllers through exactly these attribute names
and constructor kwargs. A field silently reappearing, disappearing, or
defaulting to a different number is an API break that no C++ test can see: the
C++ side compiles fine either way, and pybind11 restates every default by hand
on the far side of the boundary. That hand-restating is not hypothetical --
`rated_torque_nm` moved 2.0 -> 1.8 Nm in C++ and the binding kept handing
Python 2.0 until it was caught here.

Hardware-free: nothing below constructs a controller, only its config.

Run with:  pytest python/tests
"""

from __future__ import annotations

import pytest

import xense.taccap as t


# ---- ForcePositionConfig -------------------------------------------------

# The whole tunable surface. Two values are the grasp itself, two are the
# motor's nameplate ratings, two describe the transport, one is the closed-end
# preload.
FORCE_POSITION_FIELDS = {
    "grasp_torque_nm": 1.1,
    "close_speed_radps": 0.5,
    "hold_torque_limit_nm": 1.8,
    "motion_torque_limit_nm": 6.0,
    "status_timeout_ms": 350,
    "motor_stream_hz": 100,
    # Seats the jaw against the closed mechanical stop. Measured: 0.15 Nm
    # seats it fully (raw 0.00000) and 0.50 Nm moves it no further, so 0.25
    # is the seating torque with headroom, not a force anyone should raise
    # hoping for a tighter close. See the header for the sweep.
    "close_preload_nm": 0.25,
}

# Removed in the 16 -> 6 trim. The first eight are firmware-constant mirrors or
# gains measured on this hardware and now live in detail::ForcePositionTuning,
# which Python cannot reach on purpose; contact_samples is derived from
# stall_hold_ms and motor_stream_hz; close_position was never read by any
# control path.
FORCE_POSITION_REMOVED = [
    "contact_torque_nm",
    "contact_vel_radps",
    "contact_vel_ratio",
    "contact_moved_rad",
    "position_kp",
    "position_kd",
    "brake_distance_rad",
    "startup_guard_ms",
    "contact_samples",
    "close_position",
    # 接触状态机随 2026-09 重构一并删除,见 docs/CONTROL_REFACTOR.md
    "arrival_band_rad",
    "brake_distance_rad",
    "startup_guard_ms",
    "stall_hold_ms",
]


def _public_fields(obj) -> set:
    return {a for a in dir(obj) if not a.startswith("_")}


def test_force_position_config_has_exactly_the_declared_fields():
    cfg = t.ForcePositionConfig()
    assert _public_fields(cfg) == set(FORCE_POSITION_FIELDS)


@pytest.mark.parametrize("name,expected", sorted(FORCE_POSITION_FIELDS.items()))
def test_force_position_config_defaults(name, expected):
    assert getattr(t.ForcePositionConfig(), name) == pytest.approx(expected)


@pytest.mark.parametrize("name", FORCE_POSITION_REMOVED)
def test_removed_force_position_fields_stay_removed(name):
    cfg = t.ForcePositionConfig()
    assert not hasattr(cfg, name)
    # pybind11 classes have no __dict__, so a typo'd assignment must raise
    # rather than quietly attaching an attribute the C++ side never reads --
    # which would look exactly like the setting having taken effect.
    with pytest.raises(AttributeError):
        setattr(cfg, name, 1.0)


def test_force_position_fields_are_writable():
    cfg = t.ForcePositionConfig()
    cfg.grasp_torque_nm = 1.2
    cfg.close_speed_radps = 0.4
    cfg.status_timeout_ms = 200
    assert cfg.grasp_torque_nm == pytest.approx(1.2)
    assert cfg.close_speed_radps == pytest.approx(0.4)
    assert cfg.status_timeout_ms == 200


def test_torque_ceilings_are_the_motor_ratings():
    # Not arbitrary safety margins: 1.8 Nm is the EduLite05's rated torque and
    # 6.0 Nm its peak, which is also the firmware's own 0x700B default and max.
    assert t.FORCE_POSITION_MAX_HOLD_TORQUE_NM == pytest.approx(1.8)
    assert t.FORCE_POSITION_MAX_MOTION_TORQUE_NM == pytest.approx(6.0)
    cfg = t.ForcePositionConfig()
    assert cfg.hold_torque_limit_nm == pytest.approx(t.FORCE_POSITION_MAX_HOLD_TORQUE_NM)
    assert cfg.motion_torque_limit_nm == pytest.approx(t.FORCE_POSITION_MAX_MOTION_TORQUE_NM)


def test_force_position_states_cover_the_machine():
    names = {v.name for v in t.ForcePositionState.__members__.values()}
    assert names == {
        "IDLE", "HOLDING_POSITION", "CLOSING", "HOLDING_FORCE", "OPENING", "FAULT",
    }


# ---- ImpedanceController -------------------------------------------------

# The supervised sibling of ForcePositionController. Same split: the gains and
# the error clamp are what a task tunes, rated_torque_nm is the motor's own
# rating, two describe the transport. The measured ceiling constants live
# in detail::ImpedanceTuning and are deliberately unreachable from Python.
IMPEDANCE_FIELDS = {
    "kp": 20.0,
    "kd": 1.0,
    "feedforward_torque": 0.0,
    # 1.1 = EL05 连续堵转额定,和 ForcePositionConfig.grasp_torque_nm 同一个数。
    # 被挡住的爪子会稳定坐在这个预算上、没有任何东西给它计时,所以它必须是能
    # 无限期保持的力矩。曾是 1.5 —— 那是还指望失速守卫来终止保持的时候。
    "max_position_torque_nm": 1.1,
    "rated_torque_nm": 1.8,
    "status_timeout_ms": 350,
    "motor_stream_hz": 100,
}

IMPEDANCE_INTERNAL = [
    "rated_hold_ms",
    "rated_release_rad",
    # 失速守卫已删除,这几个字段连内部都不存在了 —— 列在这里是为了钉住"它们不该
    # 再出现":误差钳位就是保护,再加一层跳闸会把它撤销掉。
    "stall_torque_nm",
    "stall_vel_radps",
    "stall_hold_ms",
    "stall_torque_floor_nm",
    # No submit phase and no `hz`: the controller is always stream-locked,
    # which is the measured-correct choice.
    "phase",
    "hz",
    "stall_action",
]


def test_impedance_config_has_exactly_the_seven_fields():
    assert _public_fields(t.ImpedanceConfig()) == set(IMPEDANCE_FIELDS)


@pytest.mark.parametrize("name,expected", sorted(IMPEDANCE_FIELDS.items()))
def test_impedance_config_defaults(name, expected):
    assert getattr(t.ImpedanceConfig(), name) == pytest.approx(expected)


@pytest.mark.parametrize("name", IMPEDANCE_INTERNAL)
def test_impedance_internals_are_not_exposed(name):
    cfg = t.ImpedanceConfig()
    assert not hasattr(cfg, name)
    with pytest.raises(AttributeError):
        setattr(cfg, name, 1.0)


def test_impedance_ceiling_is_the_rated_torque_not_the_peak():
    # The hold TORQUE_CAPPED produces is indefinite and nothing times it out,
    # so the peak rating would be the wrong bound.
    assert t.ImpedanceConfig().rated_torque_nm == pytest.approx(t.MOTOR_RATED_TORQUE_NM)
    assert t.MOTOR_RATED_TORQUE_NM == pytest.approx(1.8)
    assert t.MOTOR_PEAK_TORQUE_NM == pytest.approx(6.0)


def test_impedance_states_are_ordered_by_precedence():
    names = [v.name for v in t.ImpedanceState.__members__.values()]
    # 没有 STALLED:失速守卫已删除,被挡住的爪子让误差钳位饱和在预算上并保持,
    # 那是 TRACKING 的稳态而不是另一个状态。
    assert names == ["IDLE", "TRACKING", "TORQUE_CAPPED", "FAULT"]


def test_impedance_controller_exposes_a_snapshot():
    # One call, one consistent view -- not a property per guard, each read
    # under its own lock.
    assert hasattr(t.ImpedanceController, "snapshot")
    for field in ("state", "observation", "target_position", "effective_position",
                  "commanded_torque_nm", "torque_capped",
                  "torque_caps", "fault_reason"):
        assert hasattr(t.ImpedanceSnapshot, field), field


@pytest.mark.parametrize("name", ["ControlLoop", "SubmitPhase", "StallAction"])
def test_control_loop_is_gone(name):
    # ControlLoop carried a second copy of the impedance law that had drifted
    # from ImpedanceController's (a different budget, and the stall guard that
    # collapsed the grip). It was removed rather than kept in sync; this pins
    # that it does not come back through a stale binding.
    assert not hasattr(t, name)
    assert not hasattr(t._taccap_native, name)


def test_rated_torque_default_matches_the_force_position_hold_ceiling():
    # Both controllers hold indefinitely in the same way, so they cap it in the
    # same place. Nothing bounds how LONG either holds.
    assert t.ImpedanceConfig().rated_torque_nm == pytest.approx(
        t.FORCE_POSITION_MAX_HOLD_TORQUE_NM
    )

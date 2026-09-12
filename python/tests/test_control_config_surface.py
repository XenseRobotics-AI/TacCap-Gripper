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

import re

import pytest

import xense.taccap as t


# ---- ForcePositionConfig -------------------------------------------------

# The whole tunable surface. Two values are the grasp itself, two are the
# motor's nameplate ratings, two describe the transport.
FORCE_POSITION_FIELDS = {
    "grasp_torque_nm": 1.1,
    "close_speed_radps": 0.5,
    "hold_torque_limit_nm": 1.8,
    "motion_torque_limit_nm": 6.0,
    "status_timeout_ms": 350,
    "motor_stream_hz": 100,
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


def test_force_position_config_has_exactly_the_six_fields():
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
# rating, two describe the transport. The measured stall/ceiling constants live
# in detail::ImpedanceTuning and are deliberately unreachable from Python.
IMPEDANCE_FIELDS = {
    "kp": 20.0,
    "kd": 1.0,
    "feedforward_torque": 0.0,
    "max_position_torque_nm": 1.5,
    "rated_torque_nm": 1.8,
    "status_timeout_ms": 350,
    "motor_stream_hz": 100,
}

IMPEDANCE_INTERNAL = [
    "rated_hold_ms",
    "rated_release_rad",
    "stall_torque_nm",
    "stall_vel_radps",
    "stall_hold_ms",
    # No submit phase and no `hz`: the controller is always stream-locked,
    # which is the measured-correct choice. ControlLoop keeps free-running.
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
    assert names == ["IDLE", "TRACKING", "STALLED", "TORQUE_CAPPED", "FAULT"]


def test_impedance_controller_exposes_a_snapshot():
    # The reason this class exists rather than ControlLoop's six independent
    # properties: one call, one consistent view.
    assert hasattr(t.ImpedanceController, "snapshot")
    for field in ("state", "observation", "target_position", "effective_position",
                  "commanded_torque_nm", "stalled", "torque_capped",
                  "stall_trips", "torque_caps", "fault_reason"):
        assert hasattr(t.ImpedanceSnapshot, field), field


def test_control_loop_is_untouched_by_the_new_controller():
    # ControlLoop stays as the low-level primitive; nothing about it moved.
    assert hasattr(t.ControlLoop, "observation")
    assert hasattr(t.ControlLoop, "stalled")
    assert hasattr(t.ControlLoop, "torque_capped")
    assert hasattr(t, "SubmitPhase") and hasattr(t, "StallAction")


# ---- ControlLoop ---------------------------------------------------------

# ControlLoop takes its config as constructor kwargs rather than a struct, so
# the defaults live in the binding and are what Python actually gets.
CONTROL_LOOP_KWARGS = {
    "hz": 100,
    "kp": 20.0,
    "kd": 1.0,
    "feedforward_torque": 0.0,
    "motor_stream_hz": 100,
    "max_position_torque_nm": 1.5,
    "rated_torque_nm": 1.8,
    "rated_hold_ms": 20,
    "rated_release_rad": 0.05,
    "stall_torque_nm": 1.2,
    "stall_vel_radps": 0.15,
    "stall_hold_ms": 60,
    "status_timeout_ms": 350,
}


def _control_loop_signature() -> str:
    doc = t.ControlLoop.__init__.__doc__ or ""
    assert doc, "pybind11 signature docstring missing"
    return doc


def _control_loop_defaults() -> dict:
    """Pull `name = <default>` out of the pybind11 signature.

    The defaults are only observable here without a gripper. Matched by name
    and `= value` rather than by the rendered type, because pybind11 spells
    those differently across versions (`int` vs
    `typing.SupportsInt | typing.SupportsIndex`) and a test that pins the type
    rendering fails on an upgrade while saying nothing about the defaults.
    """
    out = {}
    for name, raw in re.findall(r"(\w+): [^,()]*? = ([^,)]+)", _control_loop_signature()):
        try:
            out[name] = float(raw)
        except ValueError:
            out[name] = raw.strip()
    return out


@pytest.mark.parametrize("name,expected", sorted(CONTROL_LOOP_KWARGS.items()))
def test_control_loop_default_reaches_python(name, expected):
    defaults = _control_loop_defaults()
    assert name in defaults, (
        f"{name} is not a ControlLoop kwarg:\n{_control_loop_signature()}"
    )
    assert defaults[name] == pytest.approx(expected)


def test_control_loop_exposes_the_stream_liveness_timeout():
    # Losing the status stream freezes every guard in the loop rather than
    # degrading them, so a caller must be able to see and set the timeout.
    assert "status_timeout_ms" in _control_loop_defaults()


def test_control_loop_reports_both_guards():
    for prop in ("stalled", "stall_trips", "torque_capped", "torque_caps"):
        assert isinstance(getattr(t.ControlLoop, prop), property), prop


def test_rated_torque_default_matches_the_force_position_hold_ceiling():
    # Both controllers hold indefinitely in the same way, so they cap it in the
    # same place. Nothing bounds how LONG either holds.
    assert _control_loop_defaults()["rated_torque_nm"] == pytest.approx(
        t.FORCE_POSITION_MAX_HOLD_TORQUE_NM
    )

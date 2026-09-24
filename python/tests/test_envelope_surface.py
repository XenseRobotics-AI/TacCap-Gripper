# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Pins the Python-facing surface of the motion safety envelope audit.

The audit exists because a follower can store one envelope and enforce another:
the firmware clamps `cont_torque_nm` down to the installed motor's continuous
STALL rating and reports that only on a UART that is not wired to USB. A bench
unit was found storing 1.800 while running at 1.100. `effective` is the field
that makes the difference visible, and `None` there means the firmware enforces
*nothing* -- not "an empty envelope", because in this record 0 means unlimited.

Getting either of those backwards would reintroduce the exact bug the audit was
added to kill, and no C++ test can see the Python side of it: pybind11 restates
every field and flag by hand on the far side of the boundary.

Hardware-free: nothing below opens a device.

Run with:  pytest python/tests
"""

from __future__ import annotations

import pytest
import xense.taccap as t

AUDIT_FIELDS = {
    "stored",
    "recommended",
    "effective",
    "issues",
    "peak_from_device",
    "cont_from_device",
    "motor_model",
    "detail",
    "ok",
    "needs_write",
}

WRITE_FIELDS = {"before", "wrote", "written"}

ISSUE_NAMES = [
    "GRIPPER_ENVELOPE_ISSUE_NOT_WRITTEN",
    "GRIPPER_ENVELOPE_ISSUE_NOT_ENFORCED",
    "GRIPPER_ENVELOPE_ISSUE_LAYOUT_MISMATCH",
    "GRIPPER_ENVELOPE_ISSUE_PEAK_UNLIMITED",
    "GRIPPER_ENVELOPE_ISSUE_CONT_UNLIMITED",
    "GRIPPER_ENVELOPE_ISSUE_CONT_ABOVE_STALL_RATING",
    "GRIPPER_ENVELOPE_ISSUE_PEAK_NOT_ABOVE_CONT",
]

# Repairable == "the firmware ignores this record or it misreports what is
# enforced". PEAK_NOT_ABOVE_CONT is deliberately excluded: it means the stored
# envelope is TIGHTER than recommended, which is a choice, not a hole.
REPAIRABLE = ISSUE_NAMES[:-1]


def _public_fields(obj) -> set:
    return {a for a in dir(obj) if not a.startswith("_")}


def test_audit_exposes_exactly_the_declared_fields():
    assert _public_fields(t.EnvelopeAudit) - {"__entries"} == AUDIT_FIELDS


def test_write_exposes_exactly_the_declared_fields():
    assert _public_fields(t.EnvelopeWrite) - {"__entries"} == WRITE_FIELDS


@pytest.mark.parametrize("name", ISSUE_NAMES)
def test_issue_bits_are_distinct_powers_of_two(name):
    v = getattr(t, name)
    assert v > 0 and v & (v - 1) == 0, f"{name} = {v} is not a single bit"


def test_issue_bits_do_not_collide():
    values = [getattr(t, n) for n in ISSUE_NAMES]
    assert len(set(values)) == len(values)


def test_repair_mask_is_exactly_the_repairable_bits():
    expected = 0
    for name in REPAIRABLE:
        expected |= getattr(t, name)
    assert t.GRIPPER_ENVELOPE_ISSUE_REPAIR_MASK == expected
    # The advisory bit must stay out of it, or ensure_envelope() would start
    # widening records someone deliberately tightened.
    assert not (
        t.GRIPPER_ENVELOPE_ISSUE_REPAIR_MASK
        & t.GRIPPER_ENVELOPE_ISSUE_PEAK_NOT_ABOVE_CONT
    )


def test_follower_exposes_the_audit_and_the_repair():
    for name in ("audit_envelope", "ensure_envelope", "get_envelope", "set_envelope"):
        assert hasattr(t.FollowerGripper, name), name


def test_stall_rating_is_exported_and_is_what_both_budgets_default_to():
    # This used to be a comment in test_control_config_surface.py: that 1.1 is
    # the EL05 continuous stall rating and that both controllers default to the
    # same number for the same reason. A constant makes it an assertion.
    assert t.MOTOR_STALL_CONT_TORQUE_NM == pytest.approx(1.1)
    assert t.ForcePositionConfig().grasp_torque_nm == pytest.approx(
        t.MOTOR_STALL_CONT_TORQUE_NM
    )
    assert t.ImpedanceConfig().max_position_torque_nm == pytest.approx(
        t.MOTOR_STALL_CONT_TORQUE_NM
    )


def test_the_three_ratings_are_three_different_numbers():
    # The whole class of bug this change fixes is one rating being used for
    # another: 1.8 is the ROTATING rating, 1.1 what a BLOCKED jaw holds
    # indefinitely, 6.0 the absolute ceiling. --cont defaulted to 1.6 for
    # months because the first two were conflated.
    assert (
        t.MOTOR_STALL_CONT_TORQUE_NM < t.MOTOR_RATED_TORQUE_NM < t.MOTOR_PEAK_TORQUE_NM
    )


def test_envelope_flags_still_mean_what_the_audit_assumes():
    assert t.GRIPPER_ENVELOPE_VALID == 0x0001
    assert t.GRIPPER_ENVELOPE_ENFORCE == 0x0002
    assert t.GRIPPER_ENVELOPE_LAYOUT_VERSION >= 2

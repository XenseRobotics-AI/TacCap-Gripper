# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Examples name controller states with `.name`, never by parsing `str()`.

The bindings give ForcePositionState / ImpedanceState a custom `__str__` that
returns the C++ `to_string()` spelling ("holding_position"). Whether pybind11
actually uses it depends on the pybind11 version the extension was built with:
one build prints "ForcePositionState.HOLDING_POSITION", another prints
"holding_position". An example that did `str(s.state).split(".")[-1]` and
compared against "HOLDING_POSITION" therefore worked on one machine and, on the
other, never matched -- every step of force_position_control.py waited out its
full 8 s timeout and was reported as a failure. `.name` is the member name on
every build.
"""

from __future__ import annotations

import pathlib
import re

import xense.taccap as t

EXAMPLES = pathlib.Path(__file__).resolve().parents[1] / "examples"


def test_no_example_parses_a_state_out_of_str():
    pattern = re.compile(r"str\([^)]*\.state\)")
    offenders = [
        f"{p.name}:{i}"
        for p in sorted(EXAMPLES.glob("*.py"))
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1)
        if pattern.search(line)
    ]
    assert not offenders, f"use state.name instead of str(state): {offenders}"


def test_force_position_terminal_states_are_real_member_names():
    src = (EXAMPLES / "force_position_control.py").read_text(encoding="utf-8")
    m = re.search(r"^TERMINAL = \(([^)]*)\)", src, re.MULTILINE)
    assert m, "force_position_control.py no longer defines TERMINAL"
    names = re.findall(r'"([A-Z_]+)"', m.group(1))
    assert names
    for n in names:
        assert n in t.ForcePositionState.__members__, n

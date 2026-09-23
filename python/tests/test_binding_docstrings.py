# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Keeps the documented Python surface documented.

`help()` is the only reference a caller reaching for `xense.taccap` has in the
interpreter, and pybind11 fills `__doc__` with a signature whether or not anyone
wrote a docstring — so an undocumented method does not look undocumented, it
looks like a method whose docstring is "the signature". That is why this is a
test and not a review item: the failure mode is silent.

Scope is deliberately the surface a caller drives hardware through, not every
bound symbol, and it checks classes and methods only. Fields and properties are
exempt on purpose: they carry a docstring where the name does not already say it
— a unit, an encoding, a sentinel — and padding the self-evident ones would make
`help()` longer without making it more useful.

Run with:  pytest python/tests
"""

from __future__ import annotations

import re

import pytest

import xense.taccap as t

# The classes a caller actually holds. Adding one here is a commitment to
# document it; that is the point.
DOCUMENTED_CLASSES = [
    "ImpedanceController",
    "ImpedanceConfig",
    "ImpedanceSnapshot",
    "ImpedanceState",
    "ForcePositionController",
    "ForcePositionConfig",
    "ForcePositionSnapshot",
    "ForcePositionState",
    "GripperObservation",
    "FollowerGripper",
    "LeaderGripper",
    "Motor",
]

# pybind11 emits `name(...) -> ret` as the first line(s) of __doc__, and for an
# overload set a numbered list of them under "Overloaded function.". None of
# that is a docstring; strip it before asking whether one exists.
_SIGNATURE = re.compile(r"^\s*(?:\d+\.\s*)?[A-Za-z_][A-Za-z0-9_]*\(.*\)(?:\s*->.*)?$")


def _prose(doc: str | None) -> str:
    if not doc:
        return ""
    kept = [
        line
        for line in doc.splitlines()
        if line.strip()
        and not _SIGNATURE.match(line)
        and line.strip() != "Overloaded function."
    ]
    return "\n".join(kept).strip()


def _public_members(cls) -> list[str]:
    return [
        name
        for name in dir(cls)
        if not name.startswith("_") and callable(getattr(cls, name, None))
    ]


@pytest.mark.parametrize("name", DOCUMENTED_CLASSES)
def test_class_has_a_docstring(name):
    cls = getattr(t, name)
    assert _prose(cls.__doc__), f"{name} has no class docstring"


@pytest.mark.parametrize("name", DOCUMENTED_CLASSES)
def test_every_public_method_has_a_docstring(name):
    cls = getattr(t, name)
    undocumented = [m for m in _public_members(cls) if not _prose(getattr(cls, m).__doc__)]
    assert not undocumented, (
        f"{name}: {sorted(undocumented)} have only a pybind-generated signature. "
        "Pass a docstring as the last argument to .def(...) in python/bindings/."
    )


def test_the_package_documents_its_own_surface():
    # The module docstring is the first thing `help(xense.taccap)` prints, and it
    # had gone stale once already: it listed the framing layer as the whole API
    # long after the controllers and components landed.
    doc = t.__doc__ or ""
    for expected in ("ImpedanceController", "ForcePositionController", "FollowerGripper"):
        assert expected in doc, f"the package docstring does not mention {expected}"


def test_exports_are_unique_and_resolvable():
    # `find_leader` and `find_follower` were each assigned and listed twice.
    assert len(t.__all__) == len(set(t.__all__)), "duplicate names in __all__"
    missing = [n for n in t.__all__ if not hasattr(t, n)]
    assert not missing, f"__all__ names nothing exports: {missing}"

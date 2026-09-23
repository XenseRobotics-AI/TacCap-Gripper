"""The project version has exactly one hand-written home: pyproject.toml.

Everything else derives from it — CMake reads `[project].version` into
PROJECT_VERSION, `version.hpp.in` bakes it into the extension as
TACCAP_VERSION_STRING, and the package takes `__version__` from the extension.

This file exists because that chain was not always a chain. Until 0.2.1 the
number was hand-written in three unconnected places (pyproject.toml,
`project(taccap_gripper VERSION ...)` in CMakeLists.txt, and a checked-in
`python/xense/taccap/_version.py`) and it had already drifted: the package
reported 0.2.0 while the extension, `hello()` and the wheel metadata all said
0.2.1. A public API reporting the wrong version is worse than reporting none,
because callers read it as authoritative and gate on it.

Same shape as the firmware bug fixed in 1.2.5 — one number, two copies, nothing
tying them together — so it gets the same treatment and the same kind of guard.
"""

from __future__ import annotations

import pathlib
import re

import pytest
import xense.taccap as t

_REPO = pathlib.Path(__file__).resolve().parents[2]
_PYPROJECT = _REPO / "pyproject.toml"


def _declared_version() -> str:
    """The one hand-written version, read the way CMake reads it."""
    text = _PYPROJECT.read_text(encoding="utf-8")
    hits = re.findall(r"^version[ \t]*=[ \t]*\"([^\"]+)\"", text, re.MULTILINE)
    assert len(hits) == 1, (
        f"expected exactly one line-anchored 'version =' in {_PYPROJECT}, "
        f"found {len(hits)}: {hits}. CMake's file(STRINGS ... REGEX) reads the "
        "same way and hard-fails too, so this is a build break, not just a test."
    )
    return hits[0]


def test_package_version_matches_pyproject():
    assert t.__version__ == _declared_version()


def test_extension_version_matches_pyproject():
    # The extension's copy is compiled in, so a mismatch here means the build
    # ran against a different pyproject than the one on disk -- i.e. the tree
    # was edited without rebuilding.
    assert t._taccap_native.__version__ == _declared_version()


def test_package_and_extension_agree():
    assert t.__version__ == t._taccap_native.__version__


def test_hello_reports_the_same_version():
    assert _declared_version() in t.hello()


def test_no_hand_written_version_module_comes_back():
    stale = _REPO / "python" / "xense" / "taccap" / "_version.py"
    assert not stale.exists(), (
        f"{stale} is back. The version is derived from pyproject.toml via the "
        "extension; a second hand-written copy is what drifted before."
    )


def test_cmake_does_not_hard_code_the_version():
    text = (_REPO / "CMakeLists.txt").read_text(encoding="utf-8")
    m = re.search(r"^project\(taccap_gripper VERSION ([^\s)]+)", text, re.MULTILINE)
    assert m, "could not find the project() call in CMakeLists.txt"
    assert m.group(1) == "${TACCAP_VERSION}", (
        f"project() hard-codes {m.group(1)!r}. It must take the version from "
        "pyproject.toml so there is only one place to edit."
    )


@pytest.mark.parametrize("attr", ["__version__"])
def test_version_is_a_three_part_string(attr):
    value = getattr(t, attr)
    assert re.fullmatch(r"\d+\.\d+\.\d+", value), value

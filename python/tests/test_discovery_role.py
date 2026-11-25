"""find_left / find_right take a role filter (hardware-free).

Calling them would scan every gripper port, which conftest forbids; the
signature is checked on the native functions instead. The selection itself is
covered by the C++ SelectSide tests.
"""

import xense.taccap as t


def test_find_side_accepts_role_keyword():
    native = t._taccap_native
    for fn in (native.find_left, native.find_right):
        sig = fn.__doc__.splitlines()[0]
        assert "role:" in sig and "= None" in sig, sig


def test_role_enum_values_usable_as_filter():
    assert t.Role.Follower != t.Role.Leader


def test_conftest_blocks_side_lookups():
    import pytest

    for fn in (t.find_left, t.find_right, t.find_one):
        with pytest.raises(RuntimeError, match="must not discover hardware"):
            fn()

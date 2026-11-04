"""Make `import xense.taccap` resolve to THIS checkout.

Without this file the suite silently tests someone else's code.

Both the `taccap` and `lerobot-xense` conda envs carry an editable install of
`taccap_gripper`, and that install registers a `ScikitBuildRedirectingFinder`
on `sys.meta_path`. A meta_path finder runs *before* `sys.path`, so it wins over
both `PYTHONPATH` and a `sys.path.insert` — `import xense.taccap` lands in
whatever checkout that install points at, which is pinned to whatever commit its
submodule happens to sit on.

The failure mode is not an import error. It is 38 assertion failures claiming
`ForcePositionConfig` still has `brake_distance_rad` and `contact_moved_rad` —
fields deleted by the control refactor — which reads exactly like a real
regression in this repo. It is not; it is a different repo answering.

So: drop the finder, put this checkout first, and assert we got it.
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import os
import pathlib
import sys

_REPO_PYTHON = str(pathlib.Path(__file__).resolve().parents[1])

sys.meta_path[:] = [
    f for f in sys.meta_path if "taccap" not in type(f).__module__.lower()
]
if _REPO_PYTHON not in sys.path:
    sys.path.insert(0, _REPO_PYTHON)

# Explicit isolated native build for validation without replacing the checkout's
# live extension. Default remains strict in-tree loading. Never just relabel
# __file__: load the actual module and verify its resolved location below.
_native_dir_value = os.environ.get("TACCAP_TEST_NATIVE_DIR")
_expected_native_dir = None
if _native_dir_value:
    _expected_native_dir = pathlib.Path(_native_dir_value).resolve(strict=True)
    if not _expected_native_dir.is_dir():
        raise RuntimeError("TACCAP_TEST_NATIVE_DIR must name a directory")
    _candidates = [
        _expected_native_dir / ("_taccap_native" + suffix)
        for suffix in importlib.machinery.EXTENSION_SUFFIXES
        if (_expected_native_dir / ("_taccap_native" + suffix)).is_file()
    ]
    if len(_candidates) != 1:
        raise RuntimeError(
            "TACCAP_TEST_NATIVE_DIR must contain exactly one native extension"
        )
    _name = "xense.taccap._taccap_native"
    _spec = importlib.util.spec_from_file_location(_name, _candidates[0])
    _module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_module)
    sys.modules[_name] = _module

import xense.taccap as _t  # noqa: E402  (must follow the path surgery above)

assert _REPO_PYTHON in _t.__file__, (
    f"xense.taccap resolved to {_t.__file__}, not this checkout. "
    "An editable install's meta_path finder is still winning."
)

# `xense.taccap` is a namespace package whose search path spans both this
# checkout and site-packages, so __file__ pointing here does NOT prove the
# compiled extension came from here. Check the extension separately.
_native = sys.modules.get("xense.taccap._taccap_native")
assert _native is not None and (
    pathlib.Path(_native.__file__).resolve().parent == _expected_native_dir
    if _expected_native_dir
    else _REPO_PYTHON in _native.__file__
), (
    f"_taccap_native came from {getattr(_native, '__file__', None)}, not this "
    "checkout — rebuild with `uv pip install -e . --no-build-isolation`."
)

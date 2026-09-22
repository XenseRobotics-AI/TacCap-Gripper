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

import pathlib
import sys

_REPO_PYTHON = str(pathlib.Path(__file__).resolve().parents[1])

sys.meta_path[:] = [
    f for f in sys.meta_path if "taccap" not in type(f).__module__.lower()
]
if _REPO_PYTHON not in sys.path:
    sys.path.insert(0, _REPO_PYTHON)

import xense.taccap as _t  # noqa: E402  (must follow the path surgery above)

assert _REPO_PYTHON in _t.__file__, (
    f"xense.taccap resolved to {_t.__file__}, not this checkout. "
    "An editable install's meta_path finder is still winning."
)

# `xense.taccap` is a namespace package whose search path spans both this
# checkout and site-packages, so __file__ pointing here does NOT prove the
# compiled extension came from here. Check the extension separately.
_native = sys.modules.get("xense.taccap._taccap_native")
assert _native is not None and _REPO_PYTHON in _native.__file__, (
    f"_taccap_native came from {getattr(_native, '__file__', None)}, not this "
    "checkout — rebuild with `pip install -e . --no-build-isolation`."
)

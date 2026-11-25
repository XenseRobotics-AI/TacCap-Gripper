"""The tactile examples must show the same orientation the recorders store.

The SDK's portrait (700, 400) image is rotated 90° clockwise to (400, 700), like
the backpack collector (taccap-core RECTIFY_ROTATE_DEG = 270) and the LeRobot
driver. It used to be a transpose — that rotation plus a left-right flip — so
every example viewer showed a mirrored gel.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


@pytest.fixture(scope="module")
def tactile():
    spec = importlib.util.spec_from_file_location("_tactile", EXAMPLES / "_tactile.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("_tactile", module)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("channels", [None, 3])
def test_to_display_rotates_clockwise_not_mirrors(tactile, channels):
    shape = (700, 400) if channels is None else (700, 400, channels)
    image = np.zeros(shape, dtype=np.uint8)
    image[0, 0] = 255  # unique top-left pixel

    shown = tactile.to_display(image)

    assert shown.shape[:2] == (400, 700)
    assert shown[0, 699].max() == 255  # clockwise: top-left -> top-right
    assert shown[0, 0].max() == 0  # the old transpose put it here
    assert np.array_equal(shown, np.rot90(image, -1))

"""Release identity and legacy command entry points, without opening devices."""
import re
import subprocess
import sys
from pathlib import Path

import pytest
import xense.taccap as sdk
from xense.taccap import _taccap_native

ROOT = Path(__file__).resolve().parents[2]


def test_python_and_native_release_identity():
    version_file = (ROOT / "python/xense/taccap/_version.py").read_text()
    source_version = re.search(r'__version__ = "([^"]+)"', version_file).group(1)
    assert sdk.__version__ == _taccap_native.__version__ == source_version
    assert source_version in sdk.hello()


def test_public_exports_are_unique_and_legacy_types_are_identical():
    assert len(sdk.__all__) == len(set(sdk.__all__))
    for name in sdk.advanced.__all__:
        assert getattr(sdk, name) is getattr(sdk.advanced, name)


@pytest.mark.parametrize("old,new", [
    ("gripper.py", "gripper_control.py"),
    ("frequency_test.py", "frequency_benchmark.py"),
])
def test_legacy_cli_help_matches_canonical_command(old, new):
    results = []
    for name in (old, new):
        result = subprocess.run([sys.executable, str(ROOT / "python/examples" / name), "--help"],
                                capture_output=True, text=True, check=True)
        results.append(result.stdout.replace(old, new))
    assert results[0] == results[1]

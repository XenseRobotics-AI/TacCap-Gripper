"""Check the single-file wrapper without launching Qt or touching hardware."""

import importlib.util
import os
import subprocess
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[2] / "packaging/firmware_updater/portable.py"
spec = importlib.util.spec_from_file_location("updater_portable", SOURCE)
portable = importlib.util.module_from_spec(spec)
spec.loader.exec_module(portable)


@pytest.mark.parametrize("code", [0, 7])
def test_portable_arguments_exit_code_and_cleanup(tmp_path, code):
    frozen = tmp_path / "fake bundle"
    frozen.mkdir()
    program = frozen / "taccap-firmware-updater"
    program.write_text(
        '#!/bin/sh\n[ -z "${PYTHONPATH:-}" ] || exit 99\n'
        'printf "%s\\n" "$@"\n' + f"exit {code}\n"
    )
    program.chmod(0o755)
    output = tmp_path / "portable tool.run"
    portable.build_portable(frozen, output)
    result = subprocess.run(
        [str(output), "argument with spaces"],
        capture_output=True,
        text=True,
        env={**os.environ, "TMPDIR": str(tmp_path), "PYTHONPATH": "/not/used"},
    )
    if os.geteuid() == 0:
        assert result.returncode == 1
        assert "not root" in result.stderr
    else:
        assert result.returncode == code
        assert result.stdout == "argument with spaces\n"
    assert not list(tmp_path.glob("xense-taccap.*"))
    assert not list(tmp_path.glob("*.payload.tmp"))

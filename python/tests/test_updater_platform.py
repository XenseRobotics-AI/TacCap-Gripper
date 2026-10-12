"""Architecture and distro guards do not need a Jetson or attached gripper."""

import sys
from pathlib import Path

import pytest

BUILD = Path(__file__).resolve().parents[2] / "packaging/firmware_updater"
sys.path.insert(0, str(BUILD))
from platform_config import TARGETS, check_elf, native_target  # noqa: E402


@pytest.mark.parametrize(
    "machine,version,arch",
    [("aarch64", "22.04", "arm64"), ("x86_64", "24.04", "amd64")],
)
def test_native_target(machine, version, arch):
    target = native_target(machine, f'ID=ubuntu\nVERSION_ID="{version}"\n')
    assert target["deb_arch"] == arch
    assert target["machine"] == machine


@pytest.mark.parametrize(
    "machine,release",
    [
        ("aarch64", 'ID=ubuntu\nVERSION_ID="24.04"'),
        ("x86_64", 'ID=ubuntu\nVERSION_ID="22.04"'),
        ("aarch64", 'ID=debian\nVERSION_ID="22.04"'),
        ("armv7l", 'ID=ubuntu\nVERSION_ID="22.04"'),
    ],
)
def test_reject_relabelled_platform(machine, release):
    with pytest.raises(ValueError):
        native_target(machine, release)


def test_elf_machine_validation(tmp_path):
    elf = tmp_path / "sdk.so"
    header = bytearray(20)
    header[:6] = b"\x7fELF\x02\x01"
    header[18:20] = (183).to_bytes(2, "little")
    elf.write_bytes(header)
    check_elf(elf, dict(TARGETS["aarch64"], machine="aarch64"))
    with pytest.raises(ValueError):
        check_elf(elf, dict(TARGETS["x86_64"], machine="x86_64"))
    elf.write_bytes(b"not an ELF")
    with pytest.raises(ValueError):
        check_elf(elf, dict(TARGETS["aarch64"], machine="aarch64"))


def test_arm_tools_have_separate_digests():
    for key in ("tool_sha", "runtime_sha"):
        assert len(TARGETS["aarch64"][key]) == 64
        assert TARGETS["aarch64"][key] != TARGETS["x86_64"][key]

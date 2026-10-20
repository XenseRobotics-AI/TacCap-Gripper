"""Native build targets; changing a filename is never cross compilation."""

import platform
from pathlib import Path

TARGETS = {
    "x86_64": {
        "deb_arch": "amd64",
        "ubuntu": "24.04",
        "glibc": "2.39",
        "elf_machine": 62,
        "tool_sha": "95cbe7cce9717fce90c484e34052ee7c7f1d7635b33c12525b4776826a7d29b6",
        "runtime_sha": "156f4bdbde9c52d01814600013e0a273f0118dc2de98975f3c8c63427ec79074",
    },
    "aarch64": {
        "deb_arch": "arm64",
        "ubuntu": "22.04",
        "glibc": "2.35",
        "elf_machine": 183,
        # Official GitHub release asset digests inspected on 2026-10-11.
        "tool_sha": "a595ea34cd6136c7f595e9dcbb16f3e9725d7610efb9e43b38c3c6e86cafc270",
        "runtime_sha": "b4ff0030242d0c3bb12ce40541828303cf167493f4793456f0436edd6255c39d",
    },
}


def native_target(machine=None, os_release=None):
    machine = machine or platform.machine()
    if machine not in TARGETS:
        raise ValueError(f"Unsupported build architecture: {machine}")
    data = os_release if os_release is not None else Path("/etc/os-release").read_text()
    release = dict(line.split("=", 1) for line in data.splitlines() if "=" in line)
    target = dict(TARGETS[machine], machine=machine)
    if (
        release.get("ID", "").strip('"') != "ubuntu"
        or release.get("VERSION_ID", "").strip('"') != target["ubuntu"]
    ):
        raise ValueError(
            f"Build {machine} on Ubuntu {target['ubuntu']}; do not relabel another platform's binaries"
        )
    return target


def check_elf(path, target):
    """Reject wrong-architecture SDKs, including accidental stale in-tree builds."""
    with Path(path).open("rb") as stream:
        header = stream.read(20)
    if (
        len(header) < 20
        or header[:6] != b"\x7fELF\x02\x01"
        or int.from_bytes(header[18:20], "little") != target["elf_machine"]
    ):
        raise ValueError(f"Expected 64-bit {target['machine']} ELF: {path}")

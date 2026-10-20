"""Hardware-free checks for AppImage extraction in emulated builders."""

import struct
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "packaging/firmware_updater")
)
from build_appimage import extract_appimage


def test_extract_offset(tmp_path):
    image = tmp_path / "tool.AppImage"
    header = bytearray(64)
    header[:7] = b"\x7fELF\x02\x01\x01"
    header[8:11] = b"AI\x02"
    struct.pack_into("<Q", header, 40, 64)
    struct.pack_into("<HH", header, 58, 64, 1)
    image.write_bytes(header + bytes(64) + b"hsqs")
    dest = tmp_path / "extracted"
    with patch("build_appimage.subprocess.run") as run:
        assert extract_appimage(image, dest) == dest
        run.assert_called_once_with(
            ["unsquashfs", "-quiet", "-o", "128", "-d", str(dest), str(image)],
            check=True,
        )


@pytest.mark.parametrize("data", [b"", bytes(64), b"\x7fELF\x02\x01\x01" + bytes(57)])
def test_reject_invalid_image(tmp_path, data):
    image = tmp_path / "bad.AppImage"
    image.write_bytes(data)
    with patch("build_appimage.subprocess.run") as run, pytest.raises(ValueError):
        extract_appimage(image, tmp_path / "out")
    run.assert_not_called()

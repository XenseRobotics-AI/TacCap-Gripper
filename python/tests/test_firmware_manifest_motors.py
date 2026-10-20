# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""firmware/manifest.json `motors` entries must match the files on disk.

motor_ota_update.py picks the image by the model recorded on the gripper and
refuses one whose CRC32 disagrees, so a stale entry breaks the default path
customers are told to use. Reads JSON and bytes only; no native module.
"""

import json
import re
import zlib
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2] / "firmware"


def _motors() -> dict:
    return json.loads((FIRMWARE / "manifest.json").read_text())["motors"]


def test_motor_images_match_manifest():
    motors = _motors()
    assert motors, "manifest has no motors section"
    for model, entry in motors.items():
        data = (FIRMWARE / entry["file"]).read_bytes()
        assert len(data) == entry["size_bytes"], model
        assert f"0x{zlib.crc32(data):08x}" == entry["crc32"], model


def test_motor_filename_names_model_and_version():
    # The script reads model and version from the file name (image_facts).
    for model, entry in _motors().items():
        m = re.match(r"(?i)(rs00|el05)[-_](\d+(?:\.\d+){3,4})\.bin$", entry["file"])
        assert m, entry["file"]
        assert m.group(1).upper() == model
        assert m.group(2) == entry["version"]

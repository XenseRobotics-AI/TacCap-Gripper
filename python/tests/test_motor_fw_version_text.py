# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Motor firmware version surface (no hardware).

The wire carries 4 raw bytes; RobStride writes the EL05's {10,5,0,4} as
"1.0.5.0.4". The vendor text <-> bytes conversion itself is pinned in C++
(cpp/tests/test_codec_motor_model.cpp, MotorFwVersionText.*); set_motor_fw_version
parses with that same function. This pins that Python exposes the pieces.
"""

import xense.taccap as t


def test_motor_version_exposes_vendor_form_and_provenance():
    for name in ("vendor_str", "from_flash", "source", "version", "valid"):
        assert hasattr(t.MotorVersion, name), name


def test_set_motor_fw_version_is_bound_and_documents_the_vendor_form():
    doc = t.Motor.set_motor_fw_version.__doc__ or ""
    assert "1.0.5.0.4" in doc and "0.0.3.32" in doc
    assert "MotorFwVersionRecord" in t.__all__

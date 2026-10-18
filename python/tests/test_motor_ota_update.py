"""motor_ota_update.py without hardware: image identification and the two-run
flow (switch to private, flash, repair the envelope, back to MIT).

The script's module-level imports are satisfied by stubs that are torn down
again right after loading -- see the note in test_ota_update_all.py on why a
stub left in sys.modules masks failures in other test modules.
"""

import importlib.util
import sys
import types
from pathlib import Path

import pytest

xense = types.ModuleType("xense")
taccap = types.ModuleType("xense.taccap")


class MotorProtocol:
    Mit = "mit"
    Private = "private"


class _StubGripper:
    pass


class _StubSession:
    pass


taccap.MotorProtocol = MotorProtocol
taccap.FollowerGripper = _StubGripper
taccap.MotorOtaSession = _StubSession

_target = types.ModuleType("_target")
_target.add_target_argument = lambda ap, required=False: ap.add_argument(
    "target", nargs=None if required else "?"
)
_target.resolve_target = lambda target: (None, False, [])

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "motor_ota_update_example", ROOT / "examples" / "motor_ota_update.py"
)
_STUBS = {"xense": xense, "xense.taccap": taccap, "_target": _target}
_SAVED = {name: sys.modules.get(name) for name in _STUBS}
sys.modules.update(_STUBS)
try:
    mod = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(mod)
finally:
    for name, previous in _SAVED.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


# ---------------------------------------------------------------------------
# Image identification
# ---------------------------------------------------------------------------


def test_el05_five_part_version_is_read_from_name_and_image():
    """EL05 versions have five parts; the first cut kept four ("1.0.5.0")."""
    data = b"\x00junk EL05 app 1.0.5.0.4 build\x00"
    assert mod.image_facts("EL05_1.0.5.0.4.bin", data, None) == ("EL05", "1.0.5.0.4")


def test_rs00_four_part_version_still_works():
    data = b"\x00RS00 0.0.3.32\x00"
    assert mod.image_facts("rs00-0.0.3.32.bin", data, None) == ("RS00", "0.0.3.32")


def test_renamed_image_takes_its_version_from_the_content():
    data = b"\x00 1.0.5.0.4 \x00"
    assert mod.image_facts("motor.bin", data, "EL05") == ("EL05", "1.0.5.0.4")


def test_name_and_content_disagreeing_is_refused():
    with pytest.raises(SystemExit):
        mod.image_facts("EL05_1.0.5.0.4.bin", b"\x00 1.0.5.0.2 \x00", None)


def test_unknown_model_is_refused():
    with pytest.raises(SystemExit):
        mod.image_facts("motor.bin", b"\x00 1.0.5.0.4 \x00", None)


# ---------------------------------------------------------------------------
# The two-run flow
# ---------------------------------------------------------------------------


class FakeVersion:
    def __init__(self, vendor, from_flash=False):
        self.valid = True
        self.vendor_str = vendor
        self.from_flash = from_flash
        self.version = tuple(int(x) for x in vendor.split("."))


class FakeEnvelope:
    def __init__(self, cont, peak):
        self.cont_torque_nm = cont
        self.peak_torque_nm = peak


class FakeAudit:
    def __init__(self, needs_write):
        self.needs_write = needs_write
        self.stored = FakeEnvelope(1.0, 2.0)
        self.recommended = FakeEnvelope(1.1, 1.8)


class FakeMotor:
    def __init__(self, protocol, version="1.0.5.0.2"):
        self.protocol = protocol
        self.version = version
        self.switches = []
        self.recorded = None

    def get_protocol(self):
        return self.protocol

    def switch_protocol(self, p):
        self.switches.append(p)

    def get_can_id(self):
        return 1

    def motor_version(self, timeout_ms=0):
        return FakeVersion(self.version)

    def set_motor_fw_version(self, v):
        self.recorded = v


class FakeGripper:
    def __init__(self, motor, needs_envelope):
        self.motor = motor
        self.firmware_version = types.SimpleNamespace(major=1, minor=2, patch=16)
        self._needs = needs_envelope
        self.envelope_writes = 0

    def audit_envelope(self):
        return FakeAudit(self._needs)

    def ensure_envelope(self):
        self.envelope_writes += 1
        return types.SimpleNamespace(wrote=True, written=FakeEnvelope(1.1, 1.8))


class FakeSession:
    def __init__(self, motor, can_id):
        self.motor = motor

    def preflight(self, model):
        assert model == "EL05"

    def read_uid(self):
        return bytes(8)

    def update_from_bytes(self, data, progress):
        self.motor.version = "1.0.5.0.4"  # what the motor answers after flashing
        progress(1, 1)


@pytest.fixture
def run(monkeypatch, tmp_path):
    image = tmp_path / "EL05_1.0.5.0.4.bin"
    image.write_bytes(b"\x00 1.0.5.0.4 \x00")
    endpoint = types.SimpleNamespace(
        firmware_sn="TCGU01A25Z0017s", mcu_device="/dev/fake"
    )
    monkeypatch.setattr(mod, "resolve_target", lambda target: (endpoint, False, []))
    monkeypatch.setattr(mod, "MotorOtaSession", FakeSession)
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)

    def _run(motor, *argv, needs_envelope=False):
        gripper = FakeGripper(motor, needs_envelope)
        monkeypatch.setattr(mod, "FollowerGripper", lambda mcu_device: gripper)
        args = [a if a != "IMAGE" else str(image) for a in argv]
        monkeypatch.setattr(sys, "argv", ["motor_ota_update.py", *args])
        return mod.main(), gripper

    return _run


def test_first_run_on_mit_switches_to_private_and_flashes_nothing(run):
    motor = FakeMotor(MotorProtocol.Mit)
    rc, g = run(motor, "IMAGE", "left", "--yes")
    assert rc == 0
    assert motor.switches == [MotorProtocol.Private]
    assert motor.recorded is None and g.envelope_writes == 0


def test_second_run_flashes_records_repairs_envelope_and_returns_to_mit(run):
    motor = FakeMotor(MotorProtocol.Private)
    rc, g = run(motor, "IMAGE", "left", "--yes", needs_envelope=True)
    assert rc == 0
    assert motor.recorded == "1.0.5.0.4"
    assert g.envelope_writes == 1
    assert motor.switches == [MotorProtocol.Mit]


def test_envelope_at_rating_is_not_written(run):
    motor = FakeMotor(MotorProtocol.Private)
    rc, g = run(motor, "IMAGE", "left", "--yes", needs_envelope=False)
    assert rc == 0 and g.envelope_writes == 0


def test_keep_envelope_skips_the_repair(run):
    motor = FakeMotor(MotorProtocol.Private)
    rc, g = run(motor, "IMAGE", "left", "--yes", "--keep-envelope", needs_envelope=True)
    assert rc == 0 and g.envelope_writes == 0


def test_version_mismatch_after_flash_stays_on_private(run, monkeypatch):
    motor = FakeMotor(MotorProtocol.Private)
    monkeypatch.setattr(FakeSession, "update_from_bytes", lambda self, d, p: None)
    rc, _ = run(motor, "IMAGE", "left", "--yes")
    assert rc == 2
    assert motor.switches == []  # left on private so the user can retry


def test_to_mit_only_switches_back(run):
    motor = FakeMotor(MotorProtocol.Private)
    rc, g = run(motor, "--to-mit", "left")
    assert rc == 0
    assert motor.switches == [MotorProtocol.Mit] and g.envelope_writes == 0

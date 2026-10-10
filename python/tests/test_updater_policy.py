import hashlib
import importlib.util
import sys
from pathlib import Path

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
spec = importlib.util.spec_from_file_location("updater_policy", APP / "core.py")
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


def image(kind, version, model=None):
    return core.Image(
        kind,
        model or ("slave" if kind == "mcu" else "EL05"),
        version,
        "digest",
        "risk warning",
        b"image",
    )


class FakeDevice:
    def __init__(self):
        self.events = []
        self.mcu = "1.2.16"
        self.motor = "1.0.5.0.4"
        self.recorded = True
        self.fail = None
        self.wrong_version = False

    def inspect(self):
        return dict(
            sn="TESTs",
            model="EL05",
            recorded=self.recorded,
            mcu=self.mcu,
            motor=self.motor,
        )

    def mcu_version(self):
        return self.mcu

    def protocol(self, name, force_cycle=False):
        self.events.append(("protocol", name, force_cycle))

    def flash_mcu(self, img):
        self.events.append(("flash_mcu", img.version))
        if self.fail == "mcu":
            raise RuntimeError("MCU failure")
        self.mcu = img.version

    def power_cycle(self, reason):
        self.events.append(("cycle", reason))
        if self.fail == "cycle":
            raise RuntimeError("power timeout")

    def flash_motor(self, img):
        self.events.append(("flash_motor", img.version))
        if self.fail == "motor":
            raise RuntimeError("motor failure")
        if not self.wrong_version:
            self.motor = img.version

    def live_motor_version(self):
        return self.motor

    def record_motor(self, value):
        self.events.append(("record", value))

    def check_homing(self):
        self.events.append(("homing",))
        if self.fail == "homing":
            raise RuntimeError("homing failure")


def run(device, mcu=None, motor=None, ask=lambda *_: True):
    events = []
    core.execute(device, mcu, motor, ask, lambda *e: events.append(e))
    return events


def test_rollback_motor_before_mcu():
    d = FakeDevice()
    events = run(d, image("mcu", "1.2.14"), image("motor", "1.0.5.0.2"))
    names = [e[0] for e in d.events]
    assert names.index("flash_motor") < names.index("record") < names.index("flash_mcu")
    assert events[-1][0] == "success"


def test_upgrade_mcu_before_motor():
    d = FakeDevice()
    d.mcu = "1.2.14"
    run(d, image("mcu", "1.2.16"), image("motor", "1.0.5.0.4"))
    names = [e[0] for e in d.events]
    assert names.index("flash_mcu") < names.index("flash_motor")


@pytest.mark.parametrize("kind", ["mcu", "motor"])
def test_single_image_does_not_flash_other(kind):
    d = FakeDevice()
    run(d, **{kind: image(kind, "1.2.14" if kind == "mcu" else "1.0.5.0.2")})
    assert not any(
        e[0] == ("flash_motor" if kind == "mcu" else "flash_mcu") for e in d.events
    )


@pytest.mark.parametrize("failure", ["mcu", "motor", "cycle", "homing"])
def test_failure_never_reports_success(failure):
    d = FakeDevice()
    d.fail = failure
    out = []
    with pytest.raises(RuntimeError):
        core.execute(
            d,
            image("mcu", "1.2.14"),
            image("motor", "1.0.5.0.2"),
            lambda *_: True,
            lambda *e: out.append(e),
        )
    assert not any(e[0] == "success" for e in out)
    assert sum(e[0] == "flash_motor" for e in d.events) <= 1
    assert sum(e[0] == "flash_mcu" for e in d.events) <= 1


def test_mismatch_does_not_record_or_flash_mcu():
    d = FakeDevice()
    d.wrong_version = True
    with pytest.raises(RuntimeError, match="实时版本核验失败"):
        run(d, image("mcu", "1.2.14"), image("motor", "1.0.5.0.2"))
    assert not any(e[0] in ("record", "flash_mcu") for e in d.events)


@pytest.mark.parametrize("recorded,model", [(False, "EL05"), (True, "RS00")])
def test_model_mismatch_before_any_write(recorded, model):
    d = FakeDevice()
    d.recorded = recorded
    with pytest.raises(RuntimeError):
        run(d, motor=image("motor", "0.0.3.32", model))
    assert d.events == []


def test_cancel_plan_no_writes():
    d = FakeDevice()
    with pytest.raises(RuntimeError, match="用户取消"):
        run(d, motor=image("motor", "1.0.5.0.2"), ask=lambda *_: False)
    assert d.events == []


def test_rollback_confirmation_is_separate():
    d = FakeDevice()
    prompts = []

    def ask(title, text):
        prompts.append(title)
        return "电机回滚" not in title

    with pytest.raises(RuntimeError, match="用户取消"):
        run(d, motor=image("motor", "1.0.5.0.2"), ask=ask)
    assert prompts == ["开始升级/回滚", "确认电机回滚"]
    assert not any(e[0] == "flash_motor" for e in d.events)


def test_unknown_current_version_fails_closed():
    d = FakeDevice()
    d.motor = None
    with pytest.raises(RuntimeError, match="当前版本"):
        run(d, motor=image("motor", "1.0.5.0.2"))
    assert not any(e[0] == "flash_motor" for e in d.events)


def test_file_hash_not_name_identifies_image(tmp_path):
    data = b"approved"
    entry = dict(
        kind="motor",
        model="EL05",
        version="1.0.5.0.4",
        sha256=hashlib.sha256(data).hexdigest(),
        warning="",
    )
    path = tmp_path / "RS00-fake.bin"
    path.write_bytes(data)
    assert core.load_image(path, "motor", {"images": [entry]}).model == "EL05"
    path.write_bytes(b"tampered")
    with pytest.raises(ValueError):
        core.load_image(path, "motor", {"images": [entry]})


def test_wrong_image_kind_rejected(tmp_path):
    path = tmp_path / "test.bin"
    path.write_bytes(b"approved")
    entry = dict(
        kind="motor",
        model="EL05",
        version="1.0.5.0.4",
        sha256=hashlib.sha256(b"approved").hexdigest(),
        warning="",
    )
    with pytest.raises(ValueError):
        core.load_image(path, "mcu", {"images": [entry]})


def test_catalog_has_unique_hashes():
    entries = core.catalog_at(APP / "catalog.json")["images"]
    assert len({r["sha256"] for r in entries}) == len(entries)
    assert any(r["version"] == "1.0.5.0.2" and r["warning"] for r in entries)


def test_skip_same_mcu_never_flashes_or_cycles():
    d = FakeDevice()
    run(d, mcu=image("mcu", d.mcu), ask=lambda title, _: "重复刷写" not in title)
    assert not any(e[0] in ("flash_mcu", "cycle") for e in d.events)


def test_skip_same_motor_uses_live_version_and_restores_mit():
    d = FakeDevice()
    run(d, motor=image("motor", d.motor), ask=lambda title, _: "重复刷写" not in title)
    assert not any(e[0] == "flash_motor" for e in d.events)
    assert ("protocol", "Mit", False) in d.events
    assert ("protocol", "Private", True) not in d.events


class FakeMaster(FakeDevice):
    def inspect(self):
        return dict(
            sn="TESTm",
            role="master",
            model="主爪",
            recorded=True,
            mcu=self.mcu,
            motor="不适用",
        )


def test_master_only_mcu_and_usb_cycle():
    d = FakeMaster()
    d.mcu = "1.2.2"
    events = run(d, mcu=image("mcu", "1.2.6", "master"))
    assert [e[0] for e in d.events] == ["flash_mcu", "cycle"]
    assert "USB" in d.events[1][1]
    assert events[-1][0] == "success"


@pytest.mark.parametrize(
    "mcu,motor",
    [
        (image("mcu", "1.2.16"), None),
        (image("mcu", "1.2.6", "master"), image("motor", "1.0.5.0.4")),
    ],
)
def test_master_rejects_wrong_role_and_motor_before_writes(mcu, motor):
    d = FakeMaster()
    with pytest.raises(ValueError):
        run(d, mcu=mcu, motor=motor)
    assert not d.events


def test_follower_rejects_master_image():
    d = FakeDevice()
    with pytest.raises(RuntimeError):
        run(d, mcu=image("mcu", "1.2.6", "master"))
    assert not d.events

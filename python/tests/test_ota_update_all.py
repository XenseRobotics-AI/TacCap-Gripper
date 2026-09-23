import importlib.util
import sys
import types
import zlib
from pathlib import Path

xense = types.ModuleType("xense")
taccap = types.ModuleType("xense.taccap")


class _DummySide:
    Left = "left"
    Right = "right"


class _DummyOtaTargetVersion:
    def __init__(self, major, minor, patch, build=0):
        self.major = major
        self.minor = minor
        self.patch = patch
        self.build = build


def _format_version(major, minor, patch):
    return f"{major}.{minor}.{patch}"


taccap.Side = _DummySide
taccap.LeaderGripper = object
taccap.OtaSession = object
taccap.OtaTargetVersion = _DummyOtaTargetVersion
# A real CRC-32, not a constant: the manifest lookup that identifies an image
# (and therefore the version we send the firmware) is keyed on this value, so a
# stub returning 0 would make every identification test pass or fail for the
# wrong reason. zlib.crc32 IS CRC-32/ISO-HDLC -- the same polynomial, init and
# final XOR -- which is why the manifest's crc32 fields verify against it.
taccap.crc32_iso_hdlc = lambda data: zlib.crc32(data) & 0xFFFFFFFF

taccap.log = types.SimpleNamespace(set_level=lambda *args, **kwargs: None)

# ota_update.py takes both of these from _target (the examples' shared device
# selector). They lived in _calib_flow until that module was split; the stub has
# to name whichever module the script actually imports, or the import in
# ota_update.py reaches the real one and walks the USB bus during collection.
_target = types.ModuleType("_target")
_target.format_version = _format_version


def _resolve_target(target):
    return None, False, []


_target.resolve_target = _resolve_target

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ota_update_example",
    ROOT / "examples" / "ota_update.py",
)

# The stubs exist only so ota_update.py's module-level imports resolve without
# hardware. They MUST be torn down again: pytest imports every test module
# during collection, so a stub left in sys.modules is still there when the
# other test modules run, and they silently get the fake `xense.taccap`
# instead of the real one. That turned a genuine failure in
# test_numpy_views.py into "TypeError: object() takes no arguments" -- an
# error that points at a constructor signature and has nothing to do with the
# actual bug. Anything left installed here would mask future failures too.
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


class FakeEndpoint:
    def __init__(self, firmware_sn, side="left", mcu_device="/dev/fake"):
        self.firmware_sn = firmware_sn
        self.side = side
        self.mcu_device = mcu_device
        self.mcu_serial = "fake-serial"


def _manifest_file(role: str) -> str:
    """The release filename for a role, per firmware/manifest.json.

    Asserted against rather than hard-coded: shipped names carry the version
    (tc-gu-01-slave-1.2.5.bin), so a literal here would have to be edited on
    every firmware bump and would fail as a stale test rather than a real one.
    """
    return mod._load_manifest()["images"][role]["file"]


def _shipped(role: str) -> bytes:
    return (Path(mod._firmware_dir()) / _manifest_file(role)).read_bytes()


def test_image_version_is_identified_by_content_not_filename():
    """A renamed .bin must still report its real version.

    The filename carries the version, which is what makes a file in a download
    folder identifiable — but a name is just a name: copy it, rename it, and it
    starts lying. CRC32 against the manifest cannot: it identifies the bytes.
    """
    role, meta = mod._identify_image(_shipped("slave"))
    assert role == "slave"
    assert meta["version"] == mod._load_manifest()["images"]["slave"]["version"]


def test_tampered_image_is_not_identified():
    """One flipped byte makes it "not one of ours", which is the honest answer.

    Failing to identify is not an error — a locally built or third-party image
    is simply unknown, and the caller falls back to 0.0.0 rather than claiming
    a version it cannot support.
    """
    raw = bytearray(_shipped("master"))
    raw[100] ^= 0xFF
    assert mod._identify_image(bytes(raw)) == (None, None)


def test_target_version_defaults_to_the_identified_image_version():
    """What we send the firmware should say what we are actually flashing.

    The firmware writes this into its bank metadata and post-install log. It
    used to default to 0.0.0 — "we do not know what this is" — while the CRC
    lookup right next to it had already worked out the answer.
    """
    declared = mod._load_manifest()["images"]["slave"]["version"]
    _role, meta = mod._identify_image(_shipped("slave"))
    v = mod._parse_version(meta["version"])
    assert f"{v.major}.{v.minor}.{v.patch}" == declared
    assert v.build == 0

    # An unidentified image still falls back to 0.0.0 rather than guessing.
    zero = mod._parse_version(None)
    assert (zero.major, zero.minor, zero.patch, zero.build) == (0, 0, 0, 0)


def test_manifest_names_match_the_files_on_disk():
    """The manifest's `file` is how role selectors and --all find an image.

    This is the failure mode that versioned filenames introduce: bump the
    version in the manifest but forget to rename the .bin (or the reverse) and
    every role-selector invocation dies with "firmware file not found", while
    an explicitly-named flash still works — so it survives casual testing and
    breaks exactly the path customers are told to use.
    """
    firmware_dir = Path(mod._firmware_dir())
    for role in ("master", "slave"):
        name = _manifest_file(role)
        assert (firmware_dir / name).is_file(), (
            f"manifest names {name} for {role}, but it is not in {firmware_dir}. "
            f"Present: {sorted(p.name for p in firmware_dir.glob('*.bin'))}"
        )


def test_manifest_filename_carries_its_declared_version():
    """Filename and manifest version must agree, or the name lies.

    The whole point of putting the version in the filename is that a bare .bin
    is identifiable on sight; a name that disagrees with the manifest is worse
    than no version at all.
    """
    for role in ("master", "slave"):
        entry = mod._load_manifest()["images"][role]
        assert entry["version"] in entry["file"], (
            f"{role}: file {entry['file']!r} does not carry version "
            f"{entry['version']!r}"
        )


def test_role_upgrade_plan_includes_both_master_and_slave():
    eps_list = [
        FakeEndpoint("TCGU01A28Z0001m", side="left"),
        FakeEndpoint("TCGU01A28Z0002s", side="right"),
    ]

    jobs = mod._build_upgrade_jobs(
        target=None,
        firmware=None,
        all_grippers=True,
        scan_grippers=lambda: eps_list,
    )

    roles = [job["role"] for job in jobs]
    assert roles == ["master", "slave"]
    assert [Path(job["firmware"]).name for job in jobs] == [
        _manifest_file("master"),
        _manifest_file("slave"),
    ]


def test_single_slave_role_target_is_supported():
    eps_list = [
        FakeEndpoint("TCGU01A28Z0001m", side="left"),
        FakeEndpoint("TCGU01A28Z0002s", side="right"),
    ]

    jobs = mod._build_upgrade_jobs(
        target="slave",
        firmware=None,
        all_grippers=False,
        scan_grippers=lambda: eps_list,
    )

    assert len(jobs) == 1
    assert jobs[0]["role"] == "slave"
    assert Path(jobs[0]["firmware"]).name == _manifest_file("slave")


def test_cli_treats_role_name_as_target_not_firmware_path():
    args = types.SimpleNamespace(
        firmware="slave", target=None, get_status=False, all=False
    )
    normalized = mod._normalize_cli_target(args)
    assert normalized.firmware is None
    assert normalized.target == "slave"

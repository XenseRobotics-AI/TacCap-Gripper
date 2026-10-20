#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
Motor firmware update: reflash the RobStride motor's OWN firmware over the
follower gripper's USB-C cable.

This is not the gripper firmware update -- that is ota_update.py. Every CAN
frame here is relayed by the follower MCU through Motor.can_ext_xfer (0x5B), so
follower firmware >= 1.2.8 is required. Full procedure: docs/MOTOR_OTA.md.

The motor answers OTA frames only on its PRIVATE protocol (measured: not on
MIT), while the SDK normally runs it on MIT. So one update runs the same
command twice, with a 24 V power cycle after each (USB may stay in):
  1. First run: the motor is on MIT -> the script switches it to the private
     protocol and exits. Power-cycle.
  2. Second run: the motor is on the private protocol -> flash, verify, record
     the new version in the follower's flash, and switch back to MIT.
     Power-cycle; once auto-calibration finishes the gripper is ready to use.
     (An RS00 comes back on MIT by itself after flashing; an EL05 stays on the
     private protocol and the script switches it back.)
If a run stops with the motor still on the private protocol, --to-mit switches
it back.

Checks before flashing; any failure refuses:
  - The gripper's recorded motor model matches the image. RobStride's OTA
    protocol does NOT check the model -- an RS00 image is accepted by an EL05
    motor -- so the model comes from the file name prefix (rs00- / el05-, any
    case, - or _) or from --model.
  - The version in the file name matches the version string inside the image.
    Versions are written the vendor's way: four parts for RS00
    (rs00-0.0.3.32.bin), five for EL05 (EL05_1.0.5.0.4.bin -- the motor itself
    answers the four bytes 10.5.0.4, and the vendor splits the 10 into "1.0").
  - Flash the gripper firmware first: a gripper OTA switches the motor to MIT.

After a successful flash the script also sets the motion envelope to the
motor's rating (EL05 cont 1.1 / peak 1.8 N*m, FollowerGripper.ensure_envelope(),
which writes flash only if the stored record differs). Early example scripts
wrote other values (for example cont 1.0 / peak 2.0) and the device never
corrects them itself. --keep-envelope skips this.

The current images ship in firmware/ and are listed under "motors" in
firmware/manifest.json. Without an image argument the script takes the one for
the motor model recorded on the gripper; a shipped image is also checked
against the manifest's CRC32.

Usage:
    python python/examples/motor_ota_update.py left                      # run twice
    python python/examples/motor_ota_update.py EL05_1.0.5.0.4.bin left   # explicit image
    python python/examples/motor_ota_update.py rs00-0.0.3.32.bin TCGU01A28Z0086s
    python python/examples/motor_ota_update.py --to-mit left             # back to MIT only
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _target import add_target_argument, resolve_target  # noqa: E402
from xense.taccap import FollowerGripper, MotorOtaSession, MotorProtocol  # noqa: E402

KNOWN_MODELS = ("RS00", "EL05")
MIN_FW = (1, 2, 8)
POWER_CYCLE = (
    "unplug the gripper's 24 V power cable for ~2 s and plug it back (USB may stay in)"
)
# A RobStride version as the vendor writes it: four parts (RS00 0.0.3.32) or
# five (EL05 1.0.5.0.4, the vendor's split of a leading byte 10). Five first, so
# an EL05 string is never cut down to its first four parts -- that is how the
# first version of this script read EL05_1.0.5.0.4.bin as "1.0.5.0".
_VERSION_RE = r"\d+(?:\.\d+){3,4}"
FIRMWARE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "firmware")
)


def _shipped_motors() -> dict:
    """The "motors" section of firmware/manifest.json, {} if absent."""
    try:
        with open(os.path.join(FIRMWARE_DIR, "manifest.json"), "rb") as f:
            return json.load(f).get("motors", {})
    except (OSError, ValueError):
        return {}


def shipped_image(motor) -> str:
    """The shipped image for the motor model recorded on the gripper."""
    rec = motor.get_model()
    if not rec.from_flash:
        # A compile-time default is a guess, and RobStride's OTA accepts any
        # model's image -- picking by it could flash the wrong motor line.
        sys.exit(
            "error: the gripper has no recorded motor model (compile-time default "
            f"{rec.name}), so the image cannot be picked for it. Pass the image "
            "explicitly, or set the model first (firmware/README.md, Follower motor setup)"
        )
    entry = _shipped_motors().get(rec.name)
    if not entry:
        sys.exit(
            f"error: no shipped image for motor {rec.name} in firmware/manifest.json; "
            "pass the image explicitly"
        )
    return os.path.join(FIRMWARE_DIR, entry["file"])


def resolve_image(path: str) -> str:
    """Resolve explicit paths and legacy basenames in the classified layout."""
    if os.path.isfile(path):
        return path
    name = os.path.basename(path)
    candidates = [
        os.path.join(FIRMWARE_DIR, path),
        os.path.join(FIRMWARE_DIR, name),
        os.path.join(FIRMWARE_DIR, "motor", name),
    ]
    candidates.extend(
        os.path.join(FIRMWARE_DIR, entry["file"])
        for entry in _shipped_motors().values()
        if os.path.basename(entry["file"]) == name
    )
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    sys.exit(f"error: image not found: {path} (shipped images live in firmware/)")


def check_shipped_crc(path: str, data: bytes) -> None:
    """The shipped image in firmware/ must match its manifest CRC32. Images
    from elsewhere are only checked against their own name (image_facts)."""
    name = os.path.basename(path)
    for entry in _shipped_motors().values():
        shipped = os.path.join(FIRMWARE_DIR, entry["file"])
        if os.path.realpath(path) == os.path.realpath(shipped):
            crc = f"0x{zlib.crc32(data):08x}"
            if crc != entry.get("crc32"):
                sys.exit(
                    f"error: {name} has CRC32 {crc}, firmware/manifest.json says "
                    f"{entry.get('crc32')}; the file is damaged or not the released image"
                )
            return


def version_bytes(text):
    parts = tuple(int(p) for p in text.split("."))
    if len(parts) == 5:
        if parts[0] != 1 or not 0 <= parts[1] <= 9:
            raise ValueError(f"invalid vendor version: {text}")
        parts = (10 + parts[1], *parts[2:])
    if len(parts) != 4 or any(p < 0 or p > 255 for p in parts):
        raise ValueError(f"invalid motor version: {text}")
    return parts


def version_text(parts):
    parts = tuple(parts)
    if 10 <= parts[0] <= 19:
        return ".".join(map(str, (1, parts[0] - 10, *parts[1:])))
    return ".".join(map(str, parts))


def image_facts(path: str, data: bytes, model_arg: str | None):
    """(model, version) from the filename, cross-checked against the image."""
    name = os.path.basename(path)
    m = re.match(r"(?i)(rs00|el05)[-_](" + _VERSION_RE + r")\.bin$", name)
    model = (model_arg or (m.group(1) if m else "")).upper()
    if model not in KNOWN_MODELS:
        sys.exit(
            f"error: cannot tell the motor model from {name!r}; pass --model "
            f"({'/'.join(KNOWN_MODELS)})"
        )
    version = m.group(2) if m else None
    embedded = sorted(
        {
            version_text(version_bytes(v.decode()))
            for v in re.findall(rb"(?<![\d.])\d+(?:\.\d+){3,4}(?![\d.])", data)
        }
    )
    if version is not None:
        version = version_text(version_bytes(version))
    if version is not None and embedded and version not in embedded:
        sys.exit(f"error: filename says {version}, the image contains {embedded}")
    return model, version or (embedded[0] if len(embedded) == 1 else None)


def read_version(motor, timeout_s: float = 20.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            v = motor.motor_version(3000)
            # Live reads only: follower >= 1.2.14 answers from its flash record
            # when the motor is silent, and that would be the OLD version.
            if v.valid and not getattr(v, "from_flash", False):
                # Keep the SDK vendor form and compatibility with raw tuples.
                return getattr(v, "vendor_str", None) or version_text(v.version)
        except Exception:  # the motor is restarting; keep asking
            pass
        time.sleep(1.0)
    return None


def record_version(motor, version) -> None:
    """Record the flashed version in the follower's flash (0x5C, follower >=
    1.2.14), so motor_version() reports it on MIT, where the motor itself
    cannot answer."""
    if not version:
        return
    try:
        motor.set_motor_fw_version(version)
        print(f"  recorded     : motor fw {version} in follower flash")
    except Exception as exc:  # older follower firmware has no 0x5C
        print(f"  not recorded : {exc}")


def repair_envelope(g) -> None:
    """Set the motion envelope to the motor's rating. Never fails the update."""
    try:
        w = g.ensure_envelope()
        if w.wrote:
            e = w.written
            print(
                f"  envelope     : set to cont {e.cont_torque_nm:.3f} / peak "
                f"{e.peak_torque_nm:.3f} N*m (takes effect after the power cycle)"
            )
    except Exception as exc:  # the motor update itself already succeeded
        print(
            f"  envelope     : NOT repaired ({exc}); run impedance_control.py --set-envelope"
        )


def confirm(question: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    return input(f"{question} type 'yes': ").strip() == "yes"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "image",
        nargs="?",
        help="RobStride motor firmware .bin; default: the shipped image in firmware/ "
        "for the motor model recorded on the gripper",
    )
    add_target_argument(ap, required=True)
    ap.add_argument("--model", help="override the model taken from the filename")
    ap.add_argument(
        "--yes", action="store_true", help="do not ask before switching or flashing"
    )
    ap.add_argument(
        "--keep-envelope",
        action="store_true",
        help="do not repair the motion envelope after flashing",
    )
    ap.add_argument(
        "--to-mit",
        action="store_true",
        help="only switch the motor back to the MIT protocol (no flashing)",
    )
    args = ap.parse_args()

    data = model = version = None
    if not args.to_mit and args.image:
        args.image = resolve_image(args.image)
        with open(args.image, "rb") as f:
            data = f.read()
        check_shipped_crc(args.image, data)
        model, version = image_facts(args.image, data, args.model)

    ep, _, _ = resolve_target(args.target)
    if not ep.firmware_sn.endswith("s"):
        sys.exit(
            f"error: {ep.firmware_sn} is not a follower; only followers have a motor"
        )
    g = FollowerGripper(mcu_device=ep.mcu_device)
    time.sleep(0.5)
    fw = g.firmware_version
    if (fw.major, fw.minor, fw.patch) < MIN_FW:
        sys.exit(
            f"error: follower firmware {fw} lacks the 0x5B relay; need "
            f">= {'.'.join(map(str, MIN_FW))} (flash it with ota_update.py first)"
        )

    motor = g.motor
    if not args.to_mit and not args.image:
        # Picked before anything is switched, so a missing image fails here.
        args.image = shipped_image(motor)
        with open(args.image, "rb") as f:
            data = f.read()
        check_shipped_crc(args.image, data)
        model, version = image_facts(args.image, data, args.model)
    proto = motor.get_protocol()

    if args.to_mit:
        if proto == MotorProtocol.Mit:
            print("the motor is already on the MIT protocol; nothing to do")
            return 0
        motor.switch_protocol(MotorProtocol.Mit)
        print(f"switched the motor to MIT. Now {POWER_CYCLE};")
        print(
            "the gripper auto-calibrates (~10 s, keep the jaws clear) and is ready to use."
        )
        return 0

    # Step 1 of 2: the motor answers OTA frames only on its private protocol.
    if proto != MotorProtocol.Private:
        print("=== motor OTA, step 1 of 2 ===")
        print(f"  gripper      : {ep.firmware_sn}  firmware {fw}")
        print(
            "  the motor is on the MIT protocol and does not answer OTA frames there,"
        )
        print("  so it has to be switched to its private protocol first.")
        if not confirm("Switch the motor to the private protocol?", args.yes):
            print("aborted, nothing changed")
            return 1
        motor.switch_protocol(MotorProtocol.Private)
        print(f"\ndone. Now {POWER_CYCLE},")
        print("wait ~10 s, then run the same command again to flash.")
        return 0

    session = MotorOtaSession(motor, motor.get_can_id())
    session.preflight(model)  # private protocol + recorded model, raises otherwise
    before = read_version(motor, 5.0)
    uid = session.read_uid()

    print("=== motor OTA, step 2 of 2 ===")
    print(f"  gripper      : {ep.firmware_sn}  firmware {fw}")
    print(f"  motor        : {model}  can_id {motor.get_can_id()}  uid {uid.hex()}")
    print(f"  motor fw now : {before or 'unknown'}")
    print(f"  image        : {args.image}")
    print(
        f"                 {len(data):,} B, {(len(data) + 7) // 8:,} packs, "
        f"version {version or 'unknown'}"
    )
    fix_envelope = False
    if not args.keep_envelope:
        audit = g.audit_envelope()
        fix_envelope = audit.needs_write
        if fix_envelope:
            rec = audit.recommended
            print(
                f"  envelope     : stored cont {audit.stored.cont_torque_nm:.3f} / peak "
                f"{audit.stored.peak_torque_nm:.3f} N*m, will be set to cont "
                f"{rec.cont_torque_nm:.3f} / peak {rec.peak_torque_nm:.3f} after flashing"
            )
        else:
            print("  envelope     : at the motor's rating, unchanged")
    if before and version and before == version:
        print("  note         : same version as installed -- this is a reflash")
    if not confirm("Flash the MOTOR with this image?", args.yes):
        print("aborted, nothing sent. The motor is still on the private protocol;")
        print("run with --to-mit to switch it back.")
        return 1

    t0 = time.monotonic()

    def progress(done, total):
        pct = 100.0 * done / total
        bar = "#" * int(pct / 2.5)
        print(f"\r  [{bar:<40}] {pct:5.1f}%  {done:,}/{total:,}", end="", flush=True)

    session.update_from_bytes(data, progress)
    print(f"\n=== done in {time.monotonic() - t0:.1f}s; waiting for the motor ===")

    if fix_envelope:
        repair_envelope(g)

    # Measured on the first real flash: the motor restarts on MIT, and the MCU
    # follows it there. The version frame needs private, so under MIT "no
    # answer" says nothing about whether the update took.
    time.sleep(3.0)
    if motor.get_protocol() != MotorProtocol.Private:
        # RS00 (0086s): the update restarts the motor on MIT and the MCU
        # follows, so the version frame cannot be read here.
        print("  the motor restarted on the MIT protocol by itself.")
        # Nothing to verify against here, so the image's version is recorded.
        record_version(motor, version)
        print(
            f"\nNow {POWER_CYCLE}; the gripper auto-calibrates (~10 s, keep the jaws clear)."
        )
        print("follower_status.py then shows the recorded motor firmware version.")
        return 0
    after = read_version(motor)
    print(f"  motor fw     : {before or '?'} -> {after or 'no answer'}")
    if version and after != version:
        print(
            f"!! expected {version}. The motor stays on the private protocol so you can"
        )
        print(
            f"   {POWER_CYCLE} and run the same command again. Use --to-mit to give up."
        )
        return 2
    # Recorded only once the motor confirmed it: a record that outlives a failed
    # flash would make every later MIT read report the wrong version.
    record_version(motor, version)
    # EL05 (0017s): the motor stays on the private protocol after the update.
    motor.switch_protocol(MotorProtocol.Mit)
    print("  switched the motor back to the MIT protocol.")
    print(f"\nDone. Now {POWER_CYCLE};")
    print(
        "the gripper auto-calibrates (~10 s, keep the jaws clear) and is ready to use."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

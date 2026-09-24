#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
TacCap-Gripper firmware over-the-air (OTA) update demo.

Pushes a firmware .bin to the MCU's inactive Flash bank, verifies its
CRC32, and triggers the bank-swap reboot. No SWD probe needed.

POWER-CYCLE THE GRIPPER AFTERWARDS -- unplug the USB cable AND the power cable
at the same time, then plug both back in. They feed different domains, so
pulling only one leaves the other half of the board energised.
The bank-swap reboot is a soft reset: it
restarts the MCU but never powers down the USB-serial bridge, and the device
comes back in a degraded state that is indistinguishable from a healthy one --
right version string, stream running, counters clean. The only symptom is that
it quietly drops status frames. Measured on hardware, same unit, same firmware,
same cable, 60-second runs: 35-39 frames lost per run after OTA alone, zero
after unplugging and replugging, three runs each way. Treat the replug as part
of the update, not as troubleshooting.

The released images ship in this repo under `firmware/`, and their filenames
carry the version (tc-gu-01-slave-1.2.6.bin). Prefer the role selectors below:
they read the current filename from firmware/manifest.json, so they keep
working across releases, while a literal filename goes stale the next time the
firmware is bumped. A name you do pass resolves against firmware/ from any
working directory, including a parent repo that vendors this one.

Usage:

    # Upgrade every attached gripper with its matching role image. The usual one.
    python python/examples/ota_update.py --all

    # Role selector: update whichever attached gripper matches the role.
    # The last character of the firmware SN decides the role: 'm' == master,
    # 's' == slave.
    python python/examples/ota_update.py master
    python python/examples/ota_update.py slave

    # Side selector: update the left or right half of the rig.
    python python/examples/ota_update.py master left
    python python/examples/ota_update.py slave right

    # An explicit image, when you mean a specific build rather than the
    # current release.
    python python/examples/ota_update.py tc-gu-01-master-1.2.4.bin

    # The version sent to the firmware (it writes bank metadata and the
    # post-install verification log with it) is taken from manifest.json by
    # matching the image's CRC32 -- by CONTENT, not by filename, which anyone
    # can rename. Pass --target-version only to override that, or to tag an
    # image the manifest does not know.
    python python/examples/ota_update.py master --target-version 1.2.5

    # Just probe — don't flash anything
    python python/examples/ota_update.py --get-status right

Notes:

  - After the final OtaApply ACK the firmware reboots; the SDK
    Transport's next command on the same /dev/ttyACM* will time out,
    which is expected. Wait ~3 s for USB re-enumeration, then
    re-open the gripper.
  - This script opens LeaderGripper to print the pre-update firmware
    version + SN. That itself sends Cmd::GetVersion + GetSn, which
    incidentally also drains any leftover DATA backlog before the OTA
    starts (same trick as discovery::scan_all).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Optional

import _target
from xense.taccap import (
    LeaderGripper,
    OtaTargetVersion,
    Side,
    crc32_iso_hdlc,
    log,
)


def _open_gripper(target: str | None) -> tuple[LeaderGripper, object]:
    """Resolve the repo-wide selector (`left` / `right` / SN / None) → gripper."""
    eps, _by_side, _all = _target.resolve_target(target)
    # OTA only needs the MCU control link; cameras stay off (the default).
    g = LeaderGripper(mcu_device=eps.mcu_device)
    return g, eps


def _parse_version(spec: Optional[str]) -> OtaTargetVersion:
    """Parse MAJOR.MINOR.PATCH, or the legacy four-part form.

    Versions are presented as three parts everywhere now, so that is what
    people will type. The wire still carries a fourth "build" byte; it
    defaults to 0, which is what firmware has always set it to. The 4-part
    form keeps working so older scripts and docs do not break.
    """
    if spec is None:
        return OtaTargetVersion(0, 0, 0, 0)
    parts = spec.split(".")
    if len(parts) not in (3, 4):
        raise SystemExit(f"--target-version must be MAJOR.MINOR.PATCH, got {spec!r}")
    try:
        nums = [int(p) for p in parts]
    except ValueError:
        raise SystemExit(f"--target-version components must be integers: {spec!r}")
    if any(n < 0 or n > 255 for n in nums):
        raise SystemExit(
            f"--target-version components must each fit in uint8: {spec!r}"
        )
    if len(nums) == 3:
        nums.append(0)
    return OtaTargetVersion(*nums)


def _format_size(n: int) -> str:
    return f"{n:,} B ({n / 1024.0:.1f} KiB)"


def _make_progress_callback(total_bytes: int, quiet: bool):
    """Return an `(written, total) -> None` printer with a 5 Hz throttle."""
    t_start = time.monotonic()
    last_print = [t_start]

    def cb(written: int, total: int) -> None:
        now = time.monotonic()
        # Always print the final 100% line; throttle others to 5 Hz so the
        # terminal doesn't flicker on fast hosts (OTA write throughput on
        # USART3 @ 3M baud routinely hits hundreds of KB/s).
        if quiet:
            return
        if written < total and (now - last_print[0]) < 0.20:
            return
        last_print[0] = now
        pct = 100.0 * written / total
        elapsed = max(now - t_start, 1e-6)
        kbps = (written / 1024.0) / elapsed
        bar_len = 40
        filled = int(bar_len * written / total)
        bar = "#" * filled + "-" * (bar_len - filled)
        sys.stdout.write(
            f"\r  [{bar}] {pct:5.1f}%  {written:>7,}/{total:,} B  {kbps:7.1f} KB/s"
        )
        sys.stdout.flush()
        if written >= total:
            sys.stdout.write("\n")

    return cb


def _gripper_role(eps) -> str:
    sn = (getattr(eps, "firmware_sn", "") or "").strip()
    if sn.endswith("m"):
        return "master"
    if sn.endswith("s"):
        return "slave"
    return "unknown"


def _default_firmware_for_role(role: str) -> str:
    """The shipped image for a role, named by firmware/manifest.json.

    The filename carries the version (tc-gu-01-slave-1.2.6.bin), so it cannot be
    derived from the role alone — the manifest is the single place the current
    release's filename is written, and `file` there is what this reads. That
    also means the manifest is load-bearing for every role-selector and --all
    invocation, so a missing or stale entry fails loudly here rather than
    resolving to some older image that happens to still be on disk.
    """
    role = role.lower()
    if role not in {"master", "slave"}:
        raise SystemExit(f"unknown role {role!r}: expected 'master' or 'slave'")
    entry = _load_manifest().get("images", {}).get(role, {})
    name = entry.get("file")
    if not name:
        raise SystemExit(
            f"cannot resolve the {role} image: firmware/manifest.json is "
            f"missing or has no images.{role}.file entry.\n"
            f"Shipped images live in {_firmware_dir()}; pass one by name "
            f"instead of using a role selector."
        )
    return os.path.join(_firmware_dir(), name)


def _role_selector(target: str | None) -> str | None:
    if target is None:
        return None
    t = target.strip().lower()
    if t in {"master", "m"}:
        return "master"
    if t in {"slave", "s"}:
        return "slave"
    return None


def _normalize_cli_target(args: argparse.Namespace) -> argparse.Namespace:
    """Interpret bare role names as a target selector, not a firmware path."""
    if args.target is not None and args.target.strip().lower() in {"all", "both"}:
        args.all = True
        args.target = None

    if args.firmware is not None and args.target is None:
        role = _role_selector(args.firmware)
        if role is not None:
            args.target = args.firmware
            args.firmware = None

    if args.get_status and args.firmware and not args.target:
        args.firmware, args.target = None, args.firmware
    return args


def _build_upgrade_jobs(
    target: str | None, firmware: str | None, all_grippers: bool, scan_grippers
):
    """Return a list of {eps, role, firmware_path} jobs.

    When all_grippers is true, each attached gripper upgrades with its matching
    role image automatically. This keeps the master and slave builds together in
    one run without forcing the user to browse the side selector or remember
    which build belongs to which SN suffix.
    """
    role_target = _role_selector(target)
    if all_grippers:
        epses = scan_grippers()
        if not epses:
            raise SystemExit("error: no gripper is plugged in.")
        jobs = []
        for eps in epses:
            role = _gripper_role(eps)
            if role == "unknown":
                raise SystemExit(
                    f"error: gripper firmware SN {eps.firmware_sn!r} does not end with "
                    "'m' or 's'; cannot infer a master/slave image."
                )
            image = _resolve_or_report(firmware or _default_firmware_for_role(role))
            if image is None:
                raise SystemExit(f"missing firmware image for {role} role")
            jobs.append({"eps": eps, "role": role, "firmware": image})
        return jobs

    if role_target is not None:
        epses = scan_grippers()
        matches = [e for e in epses if _gripper_role(e) == role_target]
        if not matches:
            raise SystemExit(
                f"error: no gripper with role={role_target!r} is plugged in. "
                f"currently visible: {len(epses)} gripper(s)"
            )
        if len(matches) > 1:
            raise SystemExit(
                f"error: {len(matches)} grippers report role={role_target!r}; "
                "pass a specific SN or side instead."
            )
        eps = matches[0]
        resolved = _resolve_or_report(
            firmware or _default_firmware_for_role(role_target)
        )
        if resolved is None:
            raise SystemExit(f"firmware image not found for role {role_target!r}")
        return [{"eps": eps, "role": role_target, "firmware": resolved}]

    eps, _by_side, _all = _target.resolve_target(target)
    if firmware is None:
        role = _gripper_role(eps)
        if role == "unknown":
            raise SystemExit(
                f"error: gripper firmware SN {eps.firmware_sn!r} does not end with "
                "'m' or 's'; please pass an explicit firmware image."
            )
        firmware = _default_firmware_for_role(role)
    resolved = _resolve_or_report(firmware)
    if resolved is None:
        raise SystemExit(f"firmware image not found for {target!r}")
    return [{"eps": eps, "role": _gripper_role(eps), "firmware": resolved}]


def _cmd_get_status(g: LeaderGripper) -> int:
    st = g.ota.get_status()
    state_names = {
        0: "Idle",
        1: "Started",
        2: "Receiving",
        3: "Verified",
        4: "Applying",
        5: "Error",
    }
    print(f"  state         = {state_names.get(st.state, 'Unknown')} ({st.state})")
    print(f"  error_code    = 0x{st.error_code:02X}")
    print(f"  bytes_written = {st.bytes_written}")
    print(f"  progress_ppt  = {st.progress_ppt}  ({st.progress_ppt / 10:.1f}%)")
    return 0


def _dim(t: str) -> str:
    return f"\033[2m{t}\033[0m" if sys.stdout.isatty() else t


def _sdk_root() -> str:
    """This repo's root — two levels up from python/examples/."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", ".."))


def _firmware_dir() -> str:
    return os.path.join(_sdk_root(), "firmware")


def _load_manifest() -> dict:
    """Read firmware/manifest.json if it is there. Absent is fine."""
    try:
        with open(os.path.join(_firmware_dir(), "manifest.json"), "rb") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _resolve_firmware(path: str) -> Optional[str]:
    """Find the image whether `path` is relative to the cwd or to this repo.

    The images ship inside this repo, but the repo is usually vendored as a
    submodule of something else — so `firmware/tc-gu-01-master-1.2.4.bin`, the path
    our docs print because it works from the SDK root, is not the path that
    works from the parent repo's root. Rather than making every downstream
    README carry its own prefix, accept both: the literal path first, then the
    same path and the bare filename under our own firmware/.

    Note the shipped names carry a version, so a hard-coded one from an older
    README will simply not be found; the caller reports what is actually in
    firmware/ so the fix is obvious.
    """
    if os.path.isfile(path):
        return path
    for cand in (
        os.path.join(_sdk_root(), path),
        os.path.join(_firmware_dir(), os.path.basename(path)),
    ):
        if os.path.isfile(cand):
            return cand
    return None


def _resolve_or_report(path: str) -> Optional[str]:
    resolved = _resolve_firmware(path)
    if resolved is None:
        shipped = (
            sorted(n for n in os.listdir(_firmware_dir()) if n.endswith(".bin"))
            if os.path.isdir(_firmware_dir())
            else []
        )
        hint = (
            "        shipped images: " + ", ".join(shipped)
            if shipped
            else "        no .bin found there"
        )
        print(
            f"[ERROR] firmware file not found: {path}\n"
            f"        shipped images live in {_firmware_dir()}\n"
            f"{hint}\n"
            f"        release filenames carry the version, so a name from an "
            f"older README will not resolve — prefer the role selectors "
            f"(master / slave / --all), which read the current name from "
            f"manifest.json.",
            file=sys.stderr,
        )
    return resolved


def _identify_image(fw_bytes: bytes):
    """CRC32 反查 firmware/manifest.json,返回 (role, meta) 或 (None, None)。

    **按内容认,不按文件名认。** 发布镜像的文件名带版本
    (tc-gu-01-slave-1.2.6.bin),但文件名是可以被改的 —— 拷走、重命名、从别处
    下载,名字就开始说谎。CRC32 不会:它认的是这一份字节。

    认不出来不是错误,只是"不是我们发布的镜像"(自己编的、第三方的),调用方按
    未知处理即可。
    """
    images = _load_manifest().get("images", {})
    if not images:
        return None, None

    # 按整数比,不要按字符串:"0x...".upper() 会把前缀里的 "x" 也大写,
    # 于是字符串比较永远不匹配 —— 这个坑踩过一次。
    def _crc_of(meta) -> int:
        try:
            return int(str(meta.get("crc32", "")), 16)
        except ValueError:
            return -1

    crc = crc32_iso_hdlc(fw_bytes)
    for role, meta in images.items():
        if _crc_of(meta) == crc:
            return role, meta
    return None, None


def _check_role(fw_bytes: bytes, firmware_sn: str, force: bool) -> int:
    """Refuse to flash an image built for the other role.

    A gripper's role is the last character of its firmware SN, NOT which hand
    it is on — two grippers on opposite sides of a rig are often both masters.
    Flashing the wrong role's image bricks the MCU and needs an SWD probe to
    recover, so this is worth blocking rather than warning about.

    Identification is by CRC32 against firmware/manifest.json, so it only
    fires for our released images; a hand-built or third-party .bin is
    unidentifiable and passes through with a note.
    """
    if not firmware_sn:
        return 0
    matched, meta = _identify_image(fw_bytes)
    if matched is None:
        print(f"  role check  : {_dim('image not in manifest, cannot verify')}")
        return 0

    # matched 之后还要 images,是为了反查「那这台其实该刷哪个」—— 给出替代文件名
    # 比只说「刷错了」有用得多。
    images = _load_manifest().get("images", {})
    want = meta.get("sn_suffix", "")
    have = firmware_sn[-1:]
    if have == want:
        print(f"  role check  : OK — {matched} image, SN ends {have!r}")
        return 0

    other = next(
        (r for r, m in images.items() if m.get("sn_suffix") == have), "unknown"
    )
    msg = (
        f"[ROLE MISMATCH] this is the {matched.upper()} image "
        f"(expects SN ending {want!r}), but {firmware_sn} ends {have!r} "
        f"— it is a {other.upper()}."
    )
    if not force:
        alt = images.get(other, {}).get("file", f"tc-gu-01-{other}.bin")
        print(
            f"\n{msg}\n"
            f"Flashing the wrong role bricks the MCU and needs an SWD probe "
            f"to recover.\nUse {alt} instead (shipped in {_firmware_dir()}), "
            f"or --force if you really mean it.",
            file=sys.stderr,
        )
        return 1
    print(f"\n{msg}\n--force given, proceeding anyway.", file=sys.stderr)
    return 0


def _cmd_update(args: argparse.Namespace, g: LeaderGripper, eps, fw_path: str) -> int:
    if fw_path is None:
        return 1
    fw_size = os.path.getsize(fw_path)
    with open(fw_path, "rb") as f:
        fw_bytes = f.read()
    if len(fw_bytes) != fw_size:
        print(
            f"[ERROR] short read: expected {fw_size}, got {len(fw_bytes)}",
            file=sys.stderr,
        )
        return 1

    crc = crc32_iso_hdlc(fw_bytes)
    role, meta = _identify_image(fw_bytes)
    image_ver = meta.get("version") if meta else None

    # 没显式指定就用镜像自报的版本。这个字段是发给固件的,它拿去写 bank 元数据和
    # 安装后的校验日志;缺省的 0.0.0 等于告诉固件"不知道刷的是什么",而我们其实
    # 知道 —— CRC 刚刚认出来了。认不出来(自编/第三方镜像)才退回 0.0.0。
    target = _parse_version(args.target_version or image_ver)

    print("=== OTA update ===")
    print(f"  firmware     : {fw_path}")
    print(f"  size         : {_format_size(fw_size)}")
    print(f"  CRC32        : 0x{crc:08X}")
    if image_ver:
        src = (
            "manifest, matched by CRC32"
            if not args.target_version
            else "manifest, matched by CRC32; --target-version overrides what is sent"
        )
        print(f"  image ver    : {image_ver} ({role})  [{src}]")
    else:
        print(f"  image ver    : {_dim('未知 —— CRC32 不在 manifest 里,不是发布镜像')}")
    print(
        f"  target ver   : {_target.format_version(target.major, target.minor, target.patch)}"
        + (
            ""
            if args.target_version or image_ver
            else _dim("  (没有版本可报,固件会记下 0.0.0)")
        )
    )
    if _check_role(fw_bytes, getattr(eps, "firmware_sn", "") or "", args.force):
        return 1
    print()

    if not args.yes:
        try:
            resp = input("Proceed? Firmware will be flashed + MCU will reboot. [y/N] ")
        except EOFError:
            resp = ""
        if resp.strip().lower() not in ("y", "yes"):
            print("Aborted.")
            return 0

    progress = _make_progress_callback(fw_size, quiet=args.no_progress)
    t0 = time.monotonic()
    try:
        # Use update_from_bytes to avoid re-reading the file inside the
        # SDK (we already have the bytes for the CRC32 pre-print above).
        g.ota.update_from_bytes(fw_bytes, target, progress)
    except Exception as e:
        elapsed = time.monotonic() - t0
        print(
            f"\n[ERROR] OTA aborted after {elapsed:.1f}s: {type(e).__name__}: {e}",
            file=sys.stderr,
        )
        try:
            st = g.ota.get_status(timeout_ms=500)
            print(
                f"        firmware OTA state = {st.state}, err = 0x{st.error_code:02X}",
                file=sys.stderr,
            )
        except Exception:
            pass
        return 1

    elapsed = time.monotonic() - t0
    print(
        f"\n=== OTA complete in {elapsed:.1f}s "
        f"({fw_size / elapsed / 1024:.1f} KB/s avg) ==="
    )
    print("Firmware is rebooting now.")
    print("Wait ~3 s for USB re-enumeration, then re-open the gripper")
    print("to confirm GetVersion returns the new version.")
    print()
    print("!! POWER-CYCLE THE GRIPPER BEFORE YOU TRUST ANY MEASUREMENT.")
    print("   Unplug the USB cable AND the power cable at the same time, then")
    print("   plug both back in. Pulling only one leaves the other half of the")
    print("   board energised, which does not reset it.")
    print("   The reboot above is a SOFT reset: it restarts the MCU but does")
    print("   not power the USB-serial bridge down, and the device comes back")
    print("   in a degraded state that looks completely healthy. Measured on")
    print("   hardware, same unit, same firmware, same cable, 60s runs:")
    print("     after OTA alone   35-39 status frames lost per run")
    print("     after power cycle 0 lost, three runs in a row")
    print("   Nothing in the version string, the stream, or the counters")
    print("   distinguishes the two states -- the only symptom is that your")
    print("   numbers are quietly wrong. Unplug and replug it.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "firmware",
        nargs="?",
        help="Path to firmware .bin (omit with --get-status or --all)",
    )
    p.add_argument(
        "target",
        nargs="?",
        metavar="left|right|master|slave|SN|all",
        help="Which gripper or role to flash: 'left' / 'right' (side comes "
        "from the firmware SN), 'master' / 'slave', an explicit SN, or "
        "'all' to upgrade every attached gripper with its matching "
        "master/slave image. Omit when exactly one gripper is plugged in.",
    )
    p.add_argument(
        "--target-version",
        default=None,
        help="Target version MAJOR.MINOR.PATCH (informational; default 0.0.0)",
    )
    p.add_argument(
        "--no-progress", action="store_true", help="Suppress per-block progress bar"
    )
    p.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="Skip the interactive confirmation prompt",
    )
    p.add_argument(
        "--force",
        action="store_true",
        help="flash even if the image's role does not match the "
        "gripper's firmware SN (bricks the MCU if wrong)",
    )
    p.add_argument(
        "--all",
        action="store_true",
        help="Upgrade every attached gripper using the matching "
        "master/slave image automatically",
    )
    p.add_argument(
        "--get-status",
        action="store_true",
        help="Print current firmware-side OTA state machine + exit",
    )
    args = p.parse_args(argv)

    args = _normalize_cli_target(args)

    # `--get-status` takes no firmware, so a lone positional there is the
    # gripper selector, not a path. Without this, `--get-status right` reports
    # "firmware file not found: right", which is a confusing way to say
    # "positional order differs when you are not flashing".
    if args.get_status and args.firmware:
        p.error(
            "--get-status takes no firmware image "
            f"(got {args.firmware!r}); pass only the gripper selector"
        )
    if args.all and args.target:
        p.error("--all and a gripper selector are mutually exclusive")
    if (
        not args.get_status
        and not args.all
        and args.firmware is None
        and args.target is None
    ):
        p.error("firmware argument required unless --get-status or --all is given")

    # Resolve before touching hardware: a typo'd path should not cost a
    # discovery + open round-trip to find out about.
    if args.firmware:
        args.firmware = _resolve_or_report(args.firmware)
        if args.firmware is None:
            return 1

    log.set_level("info")

    if args.all:
        jobs = _build_upgrade_jobs(None, None, True, _target.scan_grippers)
        rc = 0
        for job in jobs:
            eps = job["eps"]
            g = LeaderGripper(mcu_device=eps.mcu_device)
            side = "left" if eps.side == Side.Left else "right"
            print(
                f"[discovery] {side}  ch343={eps.mcu_serial}  fw_sn={eps.firmware_sn!r}"
            )
            print(f"            {eps.mcu_device}")
            print()
            status = _cmd_update(args, g, eps, job["firmware"])
            if status != 0:
                rc = status
                break
        return rc

    if args.get_status:
        g, eps = _open_gripper(args.target)
        side = "left" if eps.side == Side.Left else "right"
        print(f"[discovery] {side}  ch343={eps.mcu_serial}  fw_sn={eps.firmware_sn!r}")
        print(f"            {eps.mcu_device}")
        print()
        print("=== current OTA state ===")
        try:
            return _cmd_get_status(g)
        except Exception as e:
            print(f"[ERROR] get_status: {type(e).__name__}: {e}", file=sys.stderr)
            return 1
        finally:
            del g

    jobs = _build_upgrade_jobs(args.target, args.firmware, False, _target.scan_grippers)
    if len(jobs) != 1:
        raise SystemExit(f"error: expected exactly one target gripper, got {len(jobs)}")
    job = jobs[0]
    eps = job["eps"]
    g = LeaderGripper(mcu_device=eps.mcu_device)
    side = "left" if eps.side == Side.Left else "right"
    print(f"[discovery] {side}  ch343={eps.mcu_serial}  fw_sn={eps.firmware_sn!r}")
    print(f"            {eps.mcu_device}")
    print()
    rc = _cmd_update(args, g, eps, job["firmware"])
    # Don't try to stop_streaming / clean shutdown — after OtaApply the
    # firmware is rebooting and any wire command will time out, which
    # would dirty the output. Let Python tear the gripper down on exit.
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

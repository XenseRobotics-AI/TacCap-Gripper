#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Fail when the SDK's protocol mirror has drifted from the firmware headers.

The SDK hand-mirrors the firmware's wire protocol in
``cpp/include/taccap/protocol/{commands.hpp,payloads.hpp}``. The existing
``static_assert(sizeof(...) == N)`` lines catch *mirroring it wrong*; nothing
catches *forgetting to mirror it at all*. That is the failure this script
exists for — command set V2.2 landed in the firmware on 2026-07-31 and the SDK
did not notice until someone went looking two weeks later.

Two independent checks:

1. **Command / error tables** — parsed out of ``protocol_cmd.h`` and
   ``commands.hpp`` and compared *by wire value*. Zero maintenance: a new
   ``#define CMD_*`` fails the build until the enum gains it.

2. **Payload sizes** — a generated translation unit includes the firmware's C
   header *and* the SDK's C++ header and compares ``sizeof`` directly, so the
   numbers come from the compiler rather than from a regex. This needs the
   small ``STRUCT_MAP`` below, because the names genuinely diverge: the
   firmware's ``motor_status_t`` is the SDK's ``MotorStatusExt``, while the
   SDK's ``MotorStatus`` is ``motor_status_legacy_t``.

Usage::

    scripts/check_protocol_drift.py                 # skip if no firmware clone
    scripts/check_protocol_drift.py --require       # absent clone is an error
    scripts/check_protocol_drift.py --firmware DIR

Exit codes: 0 = match (or skipped), 1 = drift, 2 = cannot run.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_FIRMWARE = REPO / "third_party" / "firmware" / "tc-gu-01"

# Firmware macro name (prefix stripped) -> SDK enumerator, where a plain
# Title-case conversion does not already produce the right name. Keep this as
# short as possible: every entry is a place the two sides disagree on wording.
NAME_ALIASES = {
    "LenMismatch": "LengthMismatch",
}

# Firmware struct -> SDK struct. Only listed types are size-checked; a firmware
# type that is deliberately not mirrored (host-irrelevant, or represented
# differently) simply stays out of this table.
STRUCT_MAP = {
    "firmware_version_t": "FirmwareVersion",
    "sn_info_t": "SnInfo",
    "imu_data_packet_t": "ImuData",
    "imu_config_t": "ImuConfig",
    "imu_mag_cal_t": "ImuMagCal",
    "encoder_data_packet_t": "EncoderData",
    "encoder_config_t": "EncoderConfig",
    "eskin_data_header_t": "EskinHeader",
    "eskin_config_t": "EskinConfig",
    "combined_sensor_header_t": "CombinedSensorHeader",
    "motor_pos_ctrl_t": "MotorPosCtrl",
    "motor_vel_ctrl_t": "MotorVelCtrl",
    "motor_torque_ctrl_t": "MotorTorqueCtrl",
    "motor_impedance_ctrl_t": "MotorImpedanceCtrl",
    # NOTE the crossover: 0x50 and the DATA stream carry the *legacy* struct.
    "motor_status_legacy_t": "MotorStatus",
    "motor_status_t": "MotorStatusExt",
    "motor_fault_report_t": "MotorFaultReport",
    "motor_control_stats_packet_t": "MotorControlStats",
    "gripper_config_t": "GripperConfig",
    "gripper_auto_cal_config_t": "GripperAutoCalConfig",
    "gripper_auto_cal_stall_param_t": "GripperAutoCalStallParam",
    "gripper_auto_cal_stall_param_ex_t": "GripperAutoCalStallParamEx",
    "stream_config_t": "StreamConfig",
    "key_status_payload_t": "KeyStatusPayload",
    "ack_payload_t": "AckPayload",
    "cal_set_payload_t": "CalSetPayload",
    "cal_set_all_payload_t": "CalSetAllPayload",
    "cal_get_response_t": "CalGetResponse",
    "sensor_error_report_t": "SensorErrorReport",
    "ota_start_t": "OtaStart",
    "ota_status_t": "OtaStatus",
}

# Firmware structs whose WIRE size is not sizeof(): the firmware declares a
# smaller C struct but memcpys a padded length onto the wire. The SDK models
# the wire form, so compare against the firmware's own size macro instead.
# Only add an entry when the firmware has such a macro and actually uses it as
# the transfer length — otherwise you are just muting a real mismatch.
WIRE_SIZE_MACRO = {
    # 10-byte struct, but ESKIN_HEADER_SIZE == 12 and task_data_stream sends
    # 12; the SDK's EskinHeader carries an explicit `_reserved2` for the gap.
    "eskin_data_header_t": "ESKIN_HEADER_SIZE",
}


def pascal(macro_tail: str) -> str:
    """GET_MOTOR_STATUS_EXT -> GetMotorStatusExt (then apply aliases)."""
    name = "".join(part.title() for part in macro_tail.split("_") if part)
    return NAME_ALIASES.get(name, name)


def parse_firmware_macros(path: Path, prefix: str) -> dict[str, int]:
    """Collect ``#define <prefix>NAME <int>`` from a firmware header."""
    # The trailing [uUlL]* matters: the command/error tables are written without
    # an integer suffix, but MOTOR_STOP_REASON_* uses `0x00U`, and `\b` after
    # the digits never matches when a suffix letter follows.
    pattern = re.compile(
        r"^\s*#\s*define\s+" + prefix + r"(\w+)\s+(0[xX][0-9A-Fa-f]+|\d+)[uUlL]*\b"
    )
    out: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = pattern.match(line)
        if m:
            # Later #define wins, matching the preprocessor's own behaviour for
            # the #ifndef-guarded block at the bottom of protocol_cmd.h.
            out[m.group(1)] = int(m.group(2), 0)
    return out


def parse_sdk_enum(path: Path, enum_name: str) -> dict[str, int]:
    """Collect ``Name = 0xNN,`` from one ``enum class`` body in a header."""
    text = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(
        r"enum\s+class\s+" + re.escape(enum_name) + r"\s*:\s*\w+\s*\{(.*?)\}\s*;",
        text,
        re.DOTALL,
    )
    if not m:
        die(f"could not find `enum class {enum_name}` in {path}")
    body = m.group(1)
    # Strip comments so a hex value mentioned in prose is never parsed.
    body = re.sub(r"//[^\n]*", "", body)
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
    return {
        name: int(value, 0)
        for name, value in re.findall(
            r"(\w+)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\s*,", body
        )
    }


def compare_tables(what: str, fw: dict[str, int], sdk: dict[str, int]) -> list[str]:
    """Compare by wire value; names are a secondary, warn-only signal."""
    errors: list[str] = []
    fw_by_value = {v: k for k, v in fw.items()}
    sdk_by_value = {v: k for k, v in sdk.items()}

    for value in sorted(set(fw_by_value) - set(sdk_by_value)):
        errors.append(
            f"{what} 0x{value:02X} ({fw_by_value[value]}) is in the firmware "
            f"but MISSING from the SDK — expected enumerator "
            f"`{pascal(fw_by_value[value])}`"
        )
    for value in sorted(set(sdk_by_value) - set(fw_by_value)):
        errors.append(
            f"{what} 0x{value:02X} ({sdk_by_value[value]}) is in the SDK but "
            f"NOT in the firmware — stale, or the firmware dropped it"
        )
    for value in sorted(set(fw_by_value) & set(sdk_by_value)):
        want, have = pascal(fw_by_value[value]), sdk_by_value[value]
        if want != have:
            print(
                f"  note: 0x{value:02X} named `{have}` in the SDK, "
                f"`{fw_by_value[value]}` in the firmware "
                f"(add an alias if intentional)"
            )
    return errors


def _emit_sizes(
    tmp: Path, tag: str, source: str, compiler: str, flags: list[str]
) -> tuple[dict[str, int], str | None]:
    """Compile and run a size-dumping program; return {name: size} or an error.

    The two headers CANNOT share a translation unit: the SDK deliberately
    reuses the firmware's macro names as ``constexpr`` (``ESKIN_MAX_ROWS``,
    ``MOTOR_STATUS_LEGACY_SIZE``, ``OTA_BLOCK_SIZE``, …), so including the C
    header first makes the preprocessor eat the C++ declarations. Compiling
    them separately sidesteps that entirely — and lets the firmware header be
    compiled as C11, where its ``_Static_assert`` is valid.
    """
    ext = ".c" if tag == "firmware" else ".cpp"
    src, exe = tmp / f"sizes_{tag}{ext}", tmp / f"sizes_{tag}"
    src.write_text(source, encoding="utf-8")
    build = subprocess.run(
        [compiler, *flags, "-o", str(exe), str(src)],
        capture_output=True, text=True,
    )
    if build.returncode != 0:
        return {}, (
            f"{tag} size probe failed to compile — a struct in STRUCT_MAP is "
            f"missing or renamed on that side:\n" + indent(build.stderr.strip())
        )
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    if run.returncode != 0:
        return {}, f"{tag} size probe failed to run: {run.stderr.strip()}"
    sizes = {}
    for row in run.stdout.split():
        name, _, value = row.partition("=")
        sizes[name] = int(value)
    return sizes, None


def check_sizes(firmware: Path, cc: str, cxx: str) -> list[str]:
    """Compare sizeof() for every STRUCT_MAP pair, as the compiler sees it."""
    c_names = sorted(STRUCT_MAP)

    def fw_expr(name: str) -> str:
        macro = WIRE_SIZE_MACRO.get(name)
        return f"(size_t)({macro})" if macro else f"sizeof({name})"

    c_src = "\n".join(
        ['#include <stdio.h>', '#include "protocol_data.h"', "int main(void) {"]
        + [f'  printf("{n}=%zu\\n", {fw_expr(n)});' for n in c_names]
        + ["  return 0;", "}", ""]
    )
    cxx_src = "\n".join(
        ["#include <cstdio>", "#include <taccap/protocol/payloads.hpp>",
         "namespace tp = xense::taccap::protocol;", "int main() {"]
        + [f'  std::printf("{STRUCT_MAP[n]}=%zu\\n", sizeof(tp::{STRUCT_MAP[n]}));'
           for n in c_names]
        + ["  return 0;", "}", ""]
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        fw_sizes, err = _emit_sizes(
            tmp, "firmware", c_src, cc,
            ["-std=c11", f"-I{firmware / 'App' / 'protocol'}",
             f"-I{firmware / 'App' / 'drivers'}"],
        )
        if err:
            return [err]
        sdk_sizes, err = _emit_sizes(
            tmp, "sdk", cxx_src, cxx,
            ["-std=c++17", f"-I{REPO / 'cpp' / 'include'}"],
        )
        if err:
            return [err]

    errors = []
    for n in c_names:
        if fw_sizes[n] == sdk_sizes[STRUCT_MAP[n]]:
            continue
        via = f" ({WIRE_SIZE_MACRO[n]})" if n in WIRE_SIZE_MACRO else ""
        errors.append(
            f"wire size mismatch: firmware {n}{via} = {fw_sizes[n]} B, "
            f"SDK {STRUCT_MAP[n]} = {sdk_sizes[STRUCT_MAP[n]]} B"
        )
    return errors


def indent(text: str, pad: str = "    ") -> str:
    return "\n".join(pad + line for line in text.splitlines())


def die(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    raise SystemExit(2)


def firmware_revision(firmware: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(firmware), "log", "-1", "--format=%h %s"],
            capture_output=True, text=True, timeout=10,
        )
        return out.stdout.strip() or "(not a git clone)"
    except (OSError, subprocess.SubprocessError):
        return "(unavailable)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--firmware", type=Path,
        default=Path(os.environ.get("TACCAP_FIRMWARE_DIR", DEFAULT_FIRMWARE)),
        help="firmware clone (default: third_party/firmware/tc-gu-01, "
             "or $TACCAP_FIRMWARE_DIR)",
    )
    ap.add_argument(
        "--require", action="store_true",
        help="treat a missing firmware clone as an error instead of skipping",
    )
    ap.add_argument("--cc", default=os.environ.get("CC", "gcc"),
                    help="C compiler for the firmware-side size probe")
    ap.add_argument("--cxx", default=os.environ.get("CXX", "g++"),
                    help="C++ compiler for the SDK-side size probe")
    args = ap.parse_args()

    fw_dir = args.firmware
    cmd_h = fw_dir / "App" / "protocol" / "protocol_cmd.h"
    data_h = fw_dir / "App" / "protocol" / "protocol_data.h"

    if not cmd_h.is_file() or not data_h.is_file():
        msg = (f"no firmware headers under {fw_dir} — clone tc-gu-01 there "
               f"(see README 'Firmware / PC GUI reference repos')")
        if args.require:
            die(msg)
        print(f"SKIP: {msg}")
        return 0

    for tool, flag in ((args.cc, "--cc/$CC"), (args.cxx, "--cxx/$CXX")):
        if shutil.which(tool) is None:
            die(f"{tool} not found; set {flag}")

    print(f"firmware: {fw_dir}")
    print(f"revision: {firmware_revision(fw_dir)}")
    print()

    commands_hpp = REPO / "cpp" / "include" / "taccap" / "protocol" / "commands.hpp"
    errors: list[str] = []

    fw_cmds = parse_firmware_macros(cmd_h, "CMD_")
    sdk_cmds = parse_sdk_enum(commands_hpp, "Cmd")
    print(f"commands:    firmware {len(fw_cmds):>3}   SDK {len(sdk_cmds):>3}")
    errors += compare_tables("command", fw_cmds, sdk_cmds)

    fw_errs = parse_firmware_macros(cmd_h, "ERR_")
    sdk_errs = parse_sdk_enum(commands_hpp, "ErrorCode")
    print(f"error codes: firmware {len(fw_errs):>3}   SDK {len(sdk_errs):>3}")
    errors += compare_tables("error code", fw_errs, sdk_errs)

    # Stop reasons. This table was NOT covered until a firmware change added
    # MOTOR_STOP_REASON_HOST_TIMEOUT (0x06) and the script still reported "OK":
    # exactly the "the firmware added something and we never noticed" case this
    # script exists to catch, just on a table nobody had wired up.
    payloads_hpp = REPO / "cpp" / "include" / "taccap" / "protocol" / "payloads.hpp"
    fw_stops = parse_firmware_macros(data_h, "MOTOR_STOP_REASON_")
    sdk_stops = parse_sdk_enum(payloads_hpp, "MotorStopReason")
    print(f"stop reasons: firmware {len(fw_stops):>2}   SDK {len(sdk_stops):>3}")
    errors += compare_tables("stop reason", fw_stops, sdk_stops)

    print(f"payloads:    {len(STRUCT_MAP)} structs size-checked "
          f"({args.cc} / {args.cxx})")
    errors += check_sizes(fw_dir, args.cc, args.cxx)

    print()
    if errors:
        print(f"DRIFT: {len(errors)} problem(s)\n")
        for e in errors:
            print(f"  - {e}")
        print(
            "\nThe firmware headers are canonical. Update"
            "\n  cpp/include/taccap/protocol/{commands.hpp,payloads.hpp}"
            "\n  cpp/src/protocol/commands.cpp   (to_string)"
            "\nand add a test alongside cpp/tests/test_codec_v22.cpp."
        )
        return 1

    print("OK: SDK protocol mirror matches the firmware headers.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

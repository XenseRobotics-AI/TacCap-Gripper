#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""
电机固件升级:刷 RobStride 电机模组**自己**的程序,经从爪 USB-C 串口。

不是夹爪固件升级 —— 那是 ota_update.py。这里每一帧 CAN 都由从爪 MCU 经
Motor.can_ext_xfer(0x5B)转发,所以需要从爪固件 >= 1.2.8。

前提,缺一条就拒绝:
  - 电机在**私有协议**下。MIT 下电机不回 OTA 帧(实测)。切过去:
        motor.switch_protocol(MotorProtocol.Private),然后断电重插
    (USB 线和 24V 电源线同时拔)。夹爪 OTA 会把电机切回 MIT,所以先刷夹爪、
    再刷电机。
  - 本机记录的电机型号与镜像一致。**RobStride 的 OTA 协议不校验型号**,
    RS00 的包刷进 EL05 电机也会被接受。型号取自文件名前缀(rs00-/el05-),
    可以用 --model 显式指定。
  - 文件名里的版本号(如 rs00-0.0.3.32.bin)与镜像内嵌的版本字符串一致。

刷完之后(0086s 首次实刷实测):电机重启回到 **MIT**,从爪 MCU 跟着切过去,
所以刷完当场读不到版本(版本帧要私有协议)。断电重插一次,自动标定能跑完即说明
电机在正常工作;要核对版本号,再切私有、断电、读。

用法:
    python python/examples/motor_ota_update.py ~/Downloads/rs00-0.0.3.32.bin TCGU01A28Z0086s
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _target import add_target_argument, resolve_target  # noqa: E402
from xense.taccap import FollowerGripper, MotorOtaSession, MotorProtocol  # noqa: E402

KNOWN_MODELS = ("RS00", "EL05")
MIN_FW = (1, 2, 8)


def image_facts(path: str, data: bytes, model_arg: str | None):
    """(model, version) from the filename, cross-checked against the image."""
    name = os.path.basename(path)
    m = re.match(r"(?i)(rs00|el05)[-_](\d+\.\d+\.\d+\.\d+)\.bin$", name)
    model = (model_arg or (m.group(1) if m else "")).upper()
    if model not in KNOWN_MODELS:
        sys.exit(
            f"error: cannot tell the motor model from {name!r}; pass --model "
            f"({'/'.join(KNOWN_MODELS)})"
        )
    version = m.group(2) if m else None
    embedded = sorted({v.decode() for v in re.findall(rb"\d+\.\d+\.\d+\.\d+", data)})
    if version is not None and embedded and version not in embedded:
        sys.exit(f"error: filename says {version}, the image contains {embedded}")
    return model, version or (embedded[0] if len(embedded) == 1 else None)


def read_version(motor, timeout_s: float = 20.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            v = motor.motor_version(3000)
            if v.valid:
                return ".".join(str(x) for x in v.version)
        except Exception:  # the motor is restarting; keep asking
            pass
        time.sleep(1.0)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("image", help="RobStride motor firmware .bin")
    add_target_argument(ap, required=True)
    ap.add_argument("--model", help="override the model taken from the filename")
    ap.add_argument("--yes", action="store_true", help="do not ask before flashing")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        data = f.read()
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
    session = MotorOtaSession(motor, motor.get_can_id())
    session.preflight(model)  # private protocol + recorded model, raises otherwise
    before = read_version(motor, 5.0)
    uid = session.read_uid()

    print("=== motor OTA ===")
    print(f"  gripper      : {ep.firmware_sn}  firmware {fw}")
    print(f"  motor        : {model}  can_id {motor.get_can_id()}  uid {uid.hex()}")
    print(f"  motor fw now : {before or 'unknown'}")
    print(f"  image        : {args.image}")
    print(
        f"                 {len(data):,} B, {(len(data) + 7) // 8:,} packs, "
        f"version {version or 'unknown'}"
    )
    if before and version and before == version:
        print("  note         : same version as installed -- this is a reflash")
    if not args.yes:
        if input("Flash the MOTOR with this image? type 'yes': ").strip() != "yes":
            print("aborted, nothing sent")
            return 1

    t0 = time.monotonic()

    def progress(done, total):
        pct = 100.0 * done / total
        bar = "#" * int(pct / 2.5)
        print(f"\r  [{bar:<40}] {pct:5.1f}%  {done:,}/{total:,}", end="", flush=True)

    session.update_from_bytes(data, progress)
    print(f"\n=== done in {time.monotonic() - t0:.1f}s; waiting for the motor ===")

    # Measured on the first real flash: the motor restarts on MIT, and the MCU
    # follows it there. The version frame needs private, so under MIT "no
    # answer" says nothing about whether the update took.
    time.sleep(3.0)
    if motor.get_protocol() != MotorProtocol.Private:
        print(
            "  the motor restarted on the MIT protocol (seen before: the update "
            "resets it),\n  so its version cannot be read here. Power-cycle "
            "(USB and 24 V together);\n  if auto-calibration completes the motor is "
            "running. To confirm the version,\n  switch_protocol(Private), "
            f"power-cycle, and expect {version or 'the new version'}."
        )
        return 0
    after = read_version(motor)
    print(f"  motor fw     : {before or '?'} -> {after or 'no answer'}")
    if version and after != version:
        print(
            f"!! expected {version}. Power-cycle (USB and 24 V together) and read again "
            "before drawing conclusions."
        )
        return 2
    print(
        "\nThe motor is on the PRIVATE protocol. To use the gripper, switch it "
        "back:\n    motor.switch_protocol(MotorProtocol.Mit)\nthen power-cycle "
        "(USB and 24 V together)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

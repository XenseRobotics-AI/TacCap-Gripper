#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""读取腕相机内参,输出 JSON。只读 —— 不写 flash,不动电机。

走的是 `Calibration.resolve_fisheye()`,不是 `read_fisheye()`。区别很重要:
**没标定过的机器对读请求回的是一条全零记录,而不是 NACK**,把它交给
FisheyeUndistorter 会把每一帧都重映射成全黑。resolve_fisheye() 把"读 → 判可用
→ 回退到 SDK 参考值"这条策略做在一处,和 SDK 内部用的是同一个决定。

输出里的 `source` 字段就是这个决定的结果:

    "device"     内参来自这台设备烧录的标定记录
    "reference"  设备没标定(或记录不可用),用的是 SDK 的共享参考值。
                 参考值是近似的 —— 镜头装配存在个体差异,拿它做精确测量之前
                 先想清楚。用 --require-device 可以让这种情况直接失败。

JSON 走 stdout,诊断信息走日志(stderr),所以可以直接管道:

    python python/examples/read_intrinsics.py left > left_cal.json
    python python/examples/read_intrinsics.py left | jq .K

用法
    python python/examples/read_intrinsics.py                # 只插一台时可省略
    python python/examples/read_intrinsics.py left
    python python/examples/read_intrinsics.py right --out right_cal.json
    python python/examples/read_intrinsics.py left --require-device
"""

from __future__ import annotations

import argparse
import json
import sys

import _target
from xense.taccap import FollowerGripper, LeaderGripper, log

# 标定记录里**不存图像尺寸**。SDK 一律按标定时的 640x480 使用这组内参,按别的
# 分辨率缩放就是猜 —— 所以这里把假设显式写进输出,而不是假装知道。
ASSUMED_IMAGE_SIZE = [640, 480]


def _enum_name(v) -> str:
    return str(v).split(".")[-1].lower()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--follower",
        action="store_true",
        help="以 FollowerGripper 打开(默认 LeaderGripper)。"
        "内参两个类都能读,默认走 leader 是为了绕开从爪的固件版本门",
    )
    ap.add_argument("--out", metavar="FILE", help="写入文件,不给就打到 stdout")
    ap.add_argument(
        "--require-device",
        action="store_true",
        help="内参若回退到 SDK 参考值则报错退出,而不是静默使用",
    )
    ap.add_argument("--indent", type=int, default=2, help="JSON 缩进,0 = 单行")
    args = ap.parse_args()

    ep, _by_side, _all = _target.resolve_target(args.target)
    cls = FollowerGripper if args.follower else LeaderGripper
    g = cls(mcu_device=ep.mcu_device)

    try:
        cal, is_reference, reason = g.calibration.resolve_fisheye()

        if is_reference:
            log.warning(
                f"内参回退到 SDK 参考值({reason})—— 这台设备没有可用的"
                "标定记录。参考值是近似的,镜头装配有个体差异。"
            )
            if args.require_device:
                sys.exit(
                    "error: --require-device 要求设备自带标定,但当前回退到了"
                    f"参考值:{reason}"
                )

        doc = {
            "sn": ep.firmware_sn,
            "side": _enum_name(ep.side),
            "role": _enum_name(ep.role),
            "mcu_device": ep.mcu_device,
            "firmware": _target.firmware_version(g),
            "source": "reference" if is_reference else "device",
            "reason": reason,
            "model": "fisheye_equidistant",
            "assumed_image_size": ASSUMED_IMAGE_SIZE,
            "fx": cal.fx,
            "fy": cal.fy,
            "cx": cal.cx,
            "cy": cal.cy,
            "k1": cal.k1,
            "k2": cal.k2,
            "k3": cal.k3,
            "k4": cal.k4,
            "K": [[float(v) for v in row] for row in cal.K],
            "D": [float(v) for v in cal.D],
        }
    finally:
        g.transport.stop()

    text = json.dumps(doc, indent=args.indent or None, ensure_ascii=False)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        log.info(f"内参已写入 {args.out} (source={doc['source']})")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

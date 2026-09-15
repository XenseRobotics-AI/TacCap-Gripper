#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ControlLoop 的用法:阻抗控制。

后台线程按状态流相位提交 MIT 帧,策略侧只碰两个非阻塞调用:

    loop.set_target(p)        # p in [0,1],0=闭合 1=张开
    obs = loop.observation()  # 最新观测,不阻塞

运动安全包络
------------
固件 1.1.7 在包络未配置时使用 cont=1.0 / peak=1.5 Nm 默认值。
--set-envelope 用于有意修改设备配置；日常使用见 gripper.py。

用法
    python python/examples/impedance_control.py --show-envelope
    python python/examples/impedance_control.py --set-envelope --peak 2.0 --cont 1.0
    python python/examples/impedance_control.py right

安全:真实运动,退出路径必定下发零力矩并 disable。
"""
from __future__ import annotations

import argparse
import time

import _calib_flow

from xense.taccap.advanced import (
    ControlLoop, FollowerGripper, StallAction,
    GRIPPER_ENVELOPE_VALID, GRIPPER_ENVELOPE_ENFORCE, log,
)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    _calib_flow.add_target_argument(ap)
    ap.add_argument("--kp", type=float, default=20.0, help="阻抗刚度 Nm/rad")
    ap.add_argument("--kd", type=float, default=1.0, help="阻抗阻尼 Nm·s/rad")
    ap.add_argument("--show-envelope", action="store_true", help="打印包络后退出")
    ap.add_argument("--set-envelope", action="store_true", help="写入包络后继续")
    ap.add_argument("--peak", type=float, default=2.0, help="运动瞬态力矩上限 Nm")
    ap.add_argument("--cont", type=float, default=1.0, help="可持续力矩上限 Nm")
    ap.add_argument("--temp-derate-start", type=int, default=0, help="降额起点 °C,0=固件默认")
    ap.add_argument("--temp-wall", type=int, default=0, help="温度墙 °C,0=固件默认")
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _calib_flow.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    if args.set_envelope:
        e = g.get_envelope()
        e.cont_torque_nm, e.peak_torque_nm = args.cont, args.peak
        e.temp_derate_start_c, e.temp_wall_c = args.temp_derate_start, args.temp_wall
        e.flags = GRIPPER_ENVELOPE_VALID | GRIPPER_ENVELOPE_ENFORCE
        g.set_envelope(e)
    env = g.get_envelope()
    print(f"[envelope] {env}")
    if not (env.flags & GRIPPER_ENVELOPE_ENFORCE):
        print("[envelope] 未显式配置：1.1.7 使用默认包络，1.1.6 则不执行包络。")
    if args.show_envelope:
        return 0

    loop = ControlLoop(g, kp=args.kp, kd=args.kd,
                       stall_action=StallAction.HOLD_POSITION)
    try:
        g.motor.clear_fault()
        loop.start()            # 以当前位置为初始目标,不会跳变
        g.motor.enable()

        for target in (1.0, 0.6, 0.3, 0.6, 1.0):
            loop.set_target(target)
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 2.0:
                time.sleep(0.02)
            o = loop.observation()
            # 全部来自状态流,控制期间不碰总线 —— read_status() 那类需要 ACK 的
            # 调用会和控制帧对撞,而 observation() 不会。
            print(f"  target={target:.2f} -> pos={o.position:.4f} "
                  f"err={target - o.position:+.4f} tq={o.torque:+.3f} Nm "
                  f"temp={o.motor_temp_c:.0f}°C age={o.age_ms:.1f}ms")

        print(f"\n[loop] submit_hz={loop.submit_hz:.1f} submits={loop.submit_count} "
              f"堵转保护触发 {loop.stall_trips} 次")
    finally:
        try:
            loop.stop()          # stop() 先下发零力矩
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

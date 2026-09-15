#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ForcePositionController 的用法:力位混合控制。

和纯阻抗的区别在于被挡住之后的行为:

    kp=0 的速度阻尼闭合  ->  接触判定  ->  kp=kd=0 的纯 tau_ff 保持

进入 HoldingForce 后 kp 与 kd 都是 0,位置误差在结构上无法再对输出力矩做贡献。
接触判定是**力矩下限 且 运动停止 且 连续确认**,不是裸力矩阈值 —— 做分离的是
速度门,不是力矩数字。

接口和 ControlLoop 一样只有两个非阻塞调用:

    c.set_target(p)      # 低于当前开度走接触感知路径,高于则有界张开
    s = c.snapshot()     # 状态 + 观测 + 命令力矩

注意 snapshot().grasp_torque_nm 是**设定值**;固件的 I2t 与温度墙会把实际输出
降下来,所以握持力不是常数。

用法
    python python/examples/force_position_control.py right
    python python/examples/force_position_control.py --grasp-torque 0.35

安全:真实运动 + 夹持力,退出路径必定下发零力矩并 disable。
"""
from __future__ import annotations

import argparse
import time

import _calib_flow

from xense.taccap.advanced import (
    FollowerGripper, ForcePositionConfig, ForcePositionController,
    GRIPPER_ENVELOPE_ENFORCE, log,
)

TERMINAL = ("HOLDING_FORCE", "HOLDING_POSITION", "FAULT")


def settle(c: ForcePositionController, budget: float = 8.0):
    """等控制器收敛到终态,返回 (状态名, 快照, 用时)。"""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        s = c.snapshot()
        name = str(s.state).split(".")[-1]
        if name in TERMINAL:
            return name, s, time.perf_counter() - t0
        time.sleep(0.01)
    s = c.snapshot()
    return str(s.state).split(".")[-1], s, time.perf_counter() - t0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    _calib_flow.add_target_argument(ap)
    ap.add_argument("--grasp-torque", type=float, default=0.35,
                    help="接触后的纯前馈保持力矩 Nm")
    ap.add_argument("--close-speed", type=float, default=0.5, help="闭合速度 rad/s")
    ap.add_argument("--contact-torque", type=float, default=0.080,
                    help="接触力矩下限 Nm,固件同名常数的默认值")
    args = ap.parse_args()

    log.set_level("warn")
    g, _ep = _calib_flow.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")
    env = g.get_envelope()
    print(f"[envelope] {env}")
    if not (env.flags & GRIPPER_ENVELOPE_ENFORCE):
        print("[warn] 包络未启用 —— 固件侧的 I2t 与温度墙不生效,"
              "长时间保持没有热保护。见 impedance_control.py --set-envelope")

    cfg = ForcePositionConfig()
    cfg.grasp_torque_nm = args.grasp_torque
    cfg.close_speed_radps = args.close_speed
    cfg.contact_torque_nm = args.contact_torque

    c = ForcePositionController(g, cfg)
    try:
        g.motor.clear_fault()
        c.start()               # 校验设备持久化的 0x700B 启动上限,并播种位置保持
        g.motor.enable()

        for target in (1.0, 0.0, 1.0):
            c.set_target(target)
            name, s, dt = settle(c)
            # 位置/力矩/温度全部来自状态流的 snapshot,控制期间不碰总线。
            print(f"  target={target:.2f} -> {name:16s} pos={s.observation.position:.4f} "
                  f"cmd={s.commanded_torque_nm:.3f} 实测={s.observation.torque:+.3f} Nm "
                  f"temp={s.observation.motor_temp_c:.0f}°C 用时 {dt:.2f}s")
            if name == "FAULT":
                print(f"  fault: {s.fault_reason}")
                break
    finally:
        try:
            c.stop()             # stop() 先下发零力矩
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

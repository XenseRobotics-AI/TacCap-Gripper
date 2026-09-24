#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ImpedanceController 的用法:受监督的位置阻抗控制。

后台线程按状态流相位提交 MIT 帧,策略侧只碰两个非阻塞调用:

    c.set_target(p)     # p in [0,1],0=闭合 1=张开
    s = c.snapshot()    # 状态 + 观测 + 两个保护的标志,一把锁一致视图

这个脚本同时是**上机验收**:跑一遍目标序列,并检查那条把「到位」和「被挡住」
分开的不变式 ——

    被挡在目标外时,正确行为是误差钳位饱和在 max_position_torque_nm 上并
    保持 —— 那就是夹持力,不是故障。没有失速状态可查:接触不需要检测,
    饱和本身就是接触。

在命令位置上还报保护,说明保护把正常的跟随稳态误判成了堵转。任何一步失败 →
退出码非零。

运动安全包络
------------
被挡住时 kp x 位置误差没有上界,固件侧的包络负责钳它。包络默认**不启用**,
用 --set-envelope 写一次(掉电保持,每台设备配一次即可)。

用法
    python python/examples/impedance_control.py --show-envelope
    python python/examples/impedance_control.py --set-envelope
    python python/examples/impedance_control.py right
    python python/examples/impedance_control.py --targets 1.0,0.5,0.0

安全:真实运动,退出路径必定下发零力矩并 disable。
"""

from __future__ import annotations

import argparse
import time

import _target
from xense.taccap import (
    ImpedanceConfig,
    ImpedanceController,
    ImpedanceState,
    log,
)

# 认为「到位」的归一化误差。阻抗跟随会停在 kp x 误差 抵住摩擦的地方,不是数学上
# 的零点,所以这里比控制器内部的判据宽一些,免得把通过的步骤报成失败。
ARRIVED = 0.03


def settle(c: ImpedanceController, budget: float = 3.0):
    """等控制器停下来,返回 (状态名, 快照, 用时)。

    等的是「速度落下来」而不是固定睡 2 秒:固定等待既可能截断一次慢的运动,
    又会在快的运动上白等。
    """
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        s = c.snapshot()
        if s.state == ImpedanceState.FAULT:
            break
        if s.observation.valid and abs(s.observation.velocity) < 0.02:
            # 连着两帧都停了才算,避免过零点的一瞬被当成停住。
            time.sleep(0.05)
            s2 = c.snapshot()
            if s2.observation.valid and abs(s2.observation.velocity) < 0.02:
                return str(s2.state).split(".")[-1], s2, time.perf_counter() - t0
        time.sleep(0.01)
    s = c.snapshot()
    return str(s.state).split(".")[-1], s, time.perf_counter() - t0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument("--kp", type=float, default=20.0, help="阻抗刚度 Nm/rad")
    ap.add_argument("--kd", type=float, default=1.0, help="阻抗阻尼 Nm·s/rad")
    ap.add_argument(
        "--show-envelope", action="store_true", help="打印包络(存的/生效的)后退出"
    )
    ap.add_argument(
        "--set-envelope",
        action="store_true",
        help="按设备自报的电机额定修好包络后继续(写 MCU flash,已正确则不写)",
    )
    ap.add_argument(
        "--targets",
        default="1.0,0.6,0.3,0.6,1.0",
        help="逗号分隔的归一化目标序列",
    )
    args = ap.parse_args()

    try:
        targets = [float(x) for x in args.targets.split(",") if x.strip()]
    except ValueError:
        ap.error(f"--targets 解析失败: {args.targets!r}")
    if not targets or any(not 0.0 <= t <= 1.0 for t in targets):
        ap.error("--targets 必须是 [0,1] 内的归一化位置")

    log.set_level("warn")
    g, _ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # 包络填什么不是调用方的问题:设备自己知道它装的电机额定多少,而固件无论写
    # 进去什么都按那个额定钳。数由 SDK 从设备读(0x56)。
    #
    # stored 与 effective 会不一样:固件把 cont 钳下去只记在一条没接到 USB 的
    # UART 上,而读回的是 flash 里的记录。
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective "
        f"{a.effective if a.effective else '*** 固件什么都不执行 ***'}"
    )
    if not a.ok:
        print(f"[warn] {a.detail}")
    if args.set_envelope:
        w = g.ensure_envelope()
        print(
            f"[envelope] 已写入 {w.written}" if w.wrote else "[envelope] 已正确,未写入"
        )
    if args.show_envelope:
        return 0

    cfg = ImpedanceConfig()
    cfg.kp = args.kp  # 刚度 Nm/rad
    cfg.kd = args.kd  # 阻尼 Nm·s/rad,同时定接近速度
    # 其余走默认:误差钳位 1.5 Nm(命令目标限在实测位置 ±1.5/kp rad),力矩天花板
    # 1.8 Nm(作用在实测力矩上),流超时 350 ms,状态流 100 Hz。

    c = ImpedanceController(g, cfg)
    failures = 0
    try:
        g.motor.clear_fault()
        c.start()  # 以当前位置为初始目标,不会跳变
        g.motor.enable()

        for target in targets:
            c.set_target(target)
            name, s, dt = settle(c)
            o = s.observation
            err = target - o.position
            # 全部来自状态流,控制期间不碰总线 —— read_status() 那类需要 ACK 的
            # 调用会和控制帧对撞,而 snapshot() 不会。
            print(
                f"  target={target:.2f} -> {name:14s} pos={o.position:.4f} "
                f"err={err:+.4f} tq={o.torque:+.3f} Nm cmd={s.commanded_torque_nm:.3f} "
                f"temp={o.motor_temp_c:.0f}°C 用时 {dt:.2f}s"
            )

            if name == "FAULT":
                print(f"    FAIL  fault: {s.fault_reason}")
                failures += 1
                break

            budget = cfg.max_position_torque_nm
            if abs(err) <= ARRIVED:
                if s.torque_capped:
                    # 到位了还顶到天花板,说明天花板把正常跟随稳态当成了堵转。
                    print("    FAIL  到位却顶到力矩天花板")
                    failures += 1
                else:
                    print("    ok    到位")
            elif s.commanded_torque_nm >= budget - 0.05:
                # 停在目标外而命令力矩饱和在预算上 —— 这正是被挡住该有的样子,
                # 而且它会一直保持,这就是夹持力。
                print(
                    f"    ok    被挡在目标外 {abs(err):.4f},命令饱和在预算 "
                    f"{s.commanded_torque_nm:.3f}/{budget:.3f} Nm 并保持"
                )
            else:
                # 没到位、也没饱和:要么增益太软推不动,要么预算根本没用上。
                print(
                    f"    FAIL  停在目标外 {abs(err):.4f},但命令只有 "
                    f"{s.commanded_torque_nm:.3f} Nm,没有饱和在预算 {budget:.3f}"
                )
                failures += 1

            if o.torque > cfg.rated_torque_nm + 0.15:
                print(
                    f"    FAIL  实测力矩 {o.torque:.3f} Nm 越过了天花板 "
                    f"{cfg.rated_torque_nm:.3f} Nm"
                )
                failures += 1

        final = c.snapshot()
        print(
            f"\n[guards] 力矩天花板触发 {final.torque_caps} 次"
            f"(误差钳位不是状态,它每帧都在生效,没有计数)"
        )
    finally:
        try:
            c.stop()  # stop() 先下发零力矩
        except Exception as exc:
            print("stop:", exc)
        try:
            g.motor.disable()
        except Exception as exc:
            print("disable:", exc)

    if failures:
        print(f"\n{failures} 步失败")
        return 1
    print(f"\n{len(targets)} 步全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

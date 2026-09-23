#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""ForcePositionController 的用法:力位混合控制。

**全程只有一条控制律**,没有接触检测:

    command(target, kp, kd, torque_budget = grasp_torque_nm)

PD 请求对预算做误差钳位,自由行程只付摩擦,被挡住就钳在恰好的预算上并保持。
不检测接触 —— **饱和本身就是接触**。SDK 0.2.0 删掉了主机侧的接触状态机:MCU
已经在 500Hz 跑同一个堵转判据并且是权威,主机再跑一份,比只在任一侧跑都糟,
因为同一个物理事件的两份副本会漂开。

(这段文档一度描述的是被删掉的那套「kp=0 闭合 → 接触判定 → kp=kd=0 纯 tau_ff
保持」。脚本一直能跑,但教的是一套不存在的模型 —— 现在跑起来状态是
HOLDING_POSITION,不会进 HoldingForce,除非真的被挡住。)

行进用**时间基准的斜坡**调速,而不是「当前位置 + 误差上限」:后者会让位置误差
永久饱和,那是恒力推不是跟踪速度,实测会跑到目标速度的两倍。

闭合端点额外带一个**有界前馈预压**(close_preload_nm,默认 0.25 Nm),把爪子坐死
在机械止点上。没有它,kp*(target-actual) 在目标处恰好归零,爪子到了就不再施力,
齿隙留在那里 —— 实测闭合位保持力矩只有 0.001 Nm。预压是从预算里**预留**而非
叠加,所以总请求不超过 grasp_torque_nm 这条不变量仍然成立。

接口和 ImpedanceController 一样只有两个非阻塞调用:

    c.set_target(p)      # p in [0,1],0=闭合 1=张开
    s = c.snapshot()     # 状态 + 观测 + 命令力矩,一把锁一致视图

注意 snapshot().grasp_torque_nm 是**设定值**;固件的 I2t 与温度墙会把实际输出
降下来,所以握持力不是常数。

这个脚本同时是**上机验收**:它跑一遍带中间位置的目标序列,并检查那条不变式 ——

    报告 holding 时,爪子必须明显**没到**命令位置。

holding 是观测量:命令用满了力矩预算而爪子仍然没在走。在命令位置上报 holding
意味着预算被一次本该到位的移动用满了,那是标定或映射出了问题。

任何一步失败 → 退出码非零。

用法
    python python/examples/force_position_control.py right
    python python/examples/force_position_control.py --grasp-torque 1.1
    python python/examples/force_position_control.py --targets 1.0,0.7,0.35,0.0

安全:真实运动 + 夹持力,退出路径必定下发零力矩并 disable。
"""

from __future__ import annotations
import argparse
import time

import _target

from xense.taccap import (
    FollowerGripper,
    ForcePositionConfig,
    ForcePositionController,
    GRIPPER_ENVELOPE_ENFORCE,
    log,
)

TERMINAL = ("HOLDING_FORCE", "HOLDING_POSITION", "FAULT")

# detail::ForcePositionTuning::arrival_eps_rad —— 控制器判「到位」的半径。
# 这里放宽一点再用来判定,免得测量噪声把通过的步骤报成失败。
ARRIVAL_BAND_RAD = 0.010
ARRIVAL_SLACK = 2.5


def await_command(c: ForcePositionController, target: float,
                  budget: float = 1.0) -> bool:
    """等控制器真正接下这个目标。

    set_target() 是**排队**的:命令在下一帧状态流的相位上才应用(这样每次写都落
    在 MCU 空闲的窗口里)。刚下发完就去读 snapshot(),读到的是**上一个**目标的
    终态 —— 于是「还没开始动」会被当成「已经收敛」,整轮验收在爪子一动不动的
    情况下全部通过。别删这一步。
    """
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < budget:
        if abs(c.snapshot().target_position - target) <= 1e-4:
            return True
        time.sleep(0.005)
    return False


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
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--grasp-torque", type=float, default=1.1,
        help="力矩预算 Nm —— 自由行程用不到,被挡住时就停在这个值"
    )
    ap.add_argument("--close-speed", type=float, default=0.5, help="闭合速度 rad/s")
    ap.add_argument(
        "--targets",
        default="1.0,0.5,0.0,0.5,1.0",
        help="逗号分隔的归一化目标序列。默认带中间位置 —— 只跑端点看不出"
             "「到位/接触」判错",
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
    env = g.get_envelope()
    print(f"[envelope] {env}")
    if not (env.flags & GRIPPER_ENVELOPE_ENFORCE):
        print(
            "[warn] 包络未启用 —— 固件侧的 I2t 与温度墙不生效,"
            "长时间保持没有热保护。见 impedance_control.py --set-envelope"
        )

    cfg = ForcePositionConfig()
    cfg.grasp_torque_nm = args.grasp_torque    # 接触后的纯前馈保持力矩 = 夹持力
    # 闭合/张开速度。与上一项不独立:阻尼增益是 grasp/close_speed(上限 5),低于
    # grasp/5 会被 validate_config() 拒掉,否则夹持力会悄悄低于设定值。
    cfg.close_speed_radps = args.close_speed
    # 其余四项走默认:hold=1.8(额定,无限期保持上限)、motion=6.0(峰值,瞬态)、
    # status_timeout_ms=350(流断 → FAULT + 零力矩)、motor_stream_hz=100。

    c = ForcePositionController(g, cfg)
    failures = 0
    try:
        g.motor.clear_fault()
        c.start()  # 校验设备持久化的 0x700B 启动上限,并播种位置保持
        g.motor.enable()

        # 到位带换算到归一化:控制器在 rad 空间判定,这里的位置是归一化的。
        travel = g.position_map().max_open_rad
        band = ARRIVAL_BAND_RAD * ARRIVAL_SLACK / travel
        print(f"[band] 行程 {travel:.4f} rad,到位 |误差| <= {band:.4f}(归一化)")

        for target in targets:
            before = str(c.snapshot().state).split(".")[-1]
            c.set_target(target)
            if not await_command(c, target):
                print(f"  target={target:.2f} -> FAIL  控制器没有接下这个目标")
                failures += 1
                continue
            name, s, dt = settle(c)
            pos = s.observation.position
            err = abs(pos - target)
            # 位置/力矩/温度全部来自状态流的 snapshot,控制期间不碰总线。
            print(
                f"  target={target:.2f} -> {name:16s} pos={pos:.4f} err={err:.4f} "
                f"cmd={s.commanded_torque_nm:.3f} 实测={s.observation.torque:+.3f} Nm "
                f"temp={s.observation.motor_temp_c:.0f}°C 用时 {dt:.2f}s"
            )

            if name == "FAULT":
                print(f"    FAIL  fault: {s.fault_reason}")
                failures += 1
                break

            # 已经夹稳时,一个更深的闭合目标是被**故意**吞掉的:爪子已经在这个
            # 力上推不动了,重新起步只会把好好的夹持抖掉。不算通过也不算失败 ——
            # 这一步根本没有发生运动,不该拿它去判「保护是否正确」。
            if before == "HOLDING_FORCE" and name == "HOLDING_FORCE" and dt < 0.05:
                print("    skip  已在力保持中,更深的闭合目标被忽略(设计如此);"
                      "要离开只能下发张开目标")
                continue
            if name not in ("HOLDING_POSITION", "HOLDING_FORCE"):
                print("    FAIL  未在预算内收敛到终态")
                failures += 1
                continue

            # 这条是核心:holding 只有在爪子被挡在命令位置**之外**时才成立。
            # 到位了还报 holding,说明一次本该到位的移动把力矩预算用满了 ——
            # 标定或归一化映射出了问题。
            if s.holding and err <= band:
                print("    FAIL  在命令位置上报 holding —— 一次本该到位的移动"
                      "用满了力矩预算")
                failures += 1
            elif s.holding:
                print(f"    ok    被挡在目标外 {err:.4f}(归一化),holding 正确")
            elif s.arrived:
                print("    ok    到位")
            else:
                # 既没到位也没在 holding,却已经稳定下来 —— 没有任何东西解释
                # 它为什么停在这里。
                print(f"    FAIL  停在目标外 {err:.4f} 却既未到位也未 holding"
                      f" —— 运动没有真正发生?")
                failures += 1
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
    print(f"\n{len(targets)} 步执行完毕,无失败")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

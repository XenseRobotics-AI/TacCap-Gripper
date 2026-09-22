#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""单个 follower 夹爪的键盘交互控制台 —— 走 SDK 控制器,不裸发 MIT 帧。

原版控制台直接调 `motor.submit_impedance()`。那是**电机原语**:拼一帧 MIT 丢上
总线,没有误差钳位、没有力矩天花板、没有堵转保护 —— 主机侧的保护一层都不在它
的路径上(见 cpp/src/components/motor.cpp)。kp=20 顶住硬物体再把目标压到全闭,
kp x 位置误差一路涨到电机自己的 0x700B 上限(6 Nm),24 V 母线被电流拉垮:松手
掉件,状态位打出 EN|FAULT|UNDER_VOLT,此后无力矩,必须重新上电。

这一版把控制交给 SDK 的两个控制器,并把固件侧的运动安全包络一起配好。

    --mode impedance       ImpedanceController —— 误差钳位 + 力矩天花板 + 堵转保持
    --mode force-position  ForcePositionController —— 接触判定后转纯力矩保持

三层的分工见 docs/CONTROL_LAYERING.md。要记住的一条:**固件包络是唯一在 MIT
路径上、谁都绕不过的一层**,而它出厂默认不启用(GripperConfig.reserved 全 0)。
不开就没有,主机侧这两个控制器也替代不了它 —— 100 Hz 相位锁的链路,主机反应
下限几十毫秒,8 rad/s 下就是 0.24 rad。用 --set-envelope 写一次,掉电保持。

用法
    python python/examples/gripper_console.py --show-envelope
    python python/examples/gripper_console.py --set-envelope --peak 2.0 --cont 1.6
    python python/examples/gripper_console.py --mode force-position --grasp-torque 1.2

按键
    j / k   — 目标开度 + / -(一个 --step;归一化 0..1,0=闭合 1=张开)
    o       — 全开(1.0)          c — 全合(0.0)
    h       — 保持当前位置(仅力位混合)
    e / d   — 使能 / 失能          f — 清故障(力位混合同时 reset 出 Fault)
    q / ESC — 退出

安全:真实运动。退出路径必定先 stop()(下发零力矩)再 disable()。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import sys
import termios
import threading
import time
import tty
from typing import Optional

import _calib_flow

from xense.taccap import (
    FollowerGripper,
    ForcePositionConfig,
    ImpedanceConfig,
    ImpedanceController,
    ForcePositionController,
    GRIPPER_ENVELOPE_VALID,
    GRIPPER_ENVELOPE_ENFORCE,
    log,
)


# ── 状态位(protocol::MotorStatusBit) ────────────────────────────────────────

_STATUS_BITS = [
    (1, "EN"),
    (2, "FAULT"),
    (4, "STALL"),
    (8, "OVER_TEMP"),
    (16, "OVER_CURR"),
    (32, "OVER_VOLT"),
    (64, "UNDER_VOLT"),
    (128, "ENC_ERR"),
]


def _status_str(mask: Optional[int]) -> str:
    if mask is None:
        return "-"
    if mask == 0:
        return "DISABLED"
    bits = [name for bit, name in _STATUS_BITS if mask & bit]
    return "|".join(bits) if bits else f"0x{mask:02X}"


# ── 键盘原始模式 ──────────────────────────────────────────────────────────────


class RawKeyboard:
    """将 stdin 切换到 non-blocking 原始模式,逐字节 poll。"""

    def __enter__(self):
        self._saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._saved)
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    @staticmethod
    def poll() -> Optional[str]:
        try:
            b = os.read(sys.stdin.fileno(), 1)
            return b.decode("utf-8", errors="replace") if b else None
        except (BlockingIOError, InterruptedError):
            return None


# ── 两种后端 ──────────────────────────────────────────────────────────────────
#
# 控制器自己拥有状态流和控制路径:不要再调 g.start_streaming(),也不要注册
# motor.on_status() —— 观测一律从 observation()/snapshot() 取,控制期间不碰总线。


class ImpedanceBackend:
    """ImpedanceController:误差钳位 + 力矩天花板 + 堵转保持,带状态机。

    走 ImpedanceController 而不是 ControlLoop:后者把 stalled / torque_capped
    报成两个各自加锁的独立布尔,轮询两次可能读到从未同时存在过的组合,而且
    submit 失败时循环直接退出、不留 fault 原因。这里所有字段来自同一个
    snapshot(),同一把锁。ControlLoop 仍在,做底层用。
    """

    label = "IMPEDANCE"

    def __init__(self, g: FollowerGripper, args):
        cfg = ImpedanceConfig()
        cfg.kp = args.kp                     # 刚度 Nm/rad
        cfg.kd = args.kd                     # 阻尼 Nm·s/rad,同时定接近速度
        # 误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内。主保护 —— 它同时
        # 限住了「接近」和「堵转」,而只防堵转是不够的:实测不钳位的大步进会以
        # 5.5 rad/s 掠过物体把它撞飞,全程力矩不到 0.03 Nm,没有持续接触可检测。
        cfg.max_position_torque_nm = args.max_position_torque
        # 力矩天花板:作用在实测力矩上(不是命令值)。顶住后改发 kp=kd=0 的纯前馈
        # 帧,位置误差再也加不进输出。上限是额定 1.8 而非峰值 6.0 —— 这个保持无限期。
        cfg.rated_torque_nm = args.rated_torque
        # status_timeout_ms / motor_stream_hz 用默认值(350 ms / 100 Hz)。
        self.ctl = ImpedanceController(g, cfg)
        self._snap = None

    def start(self):
        self.ctl.start()

    def stop(self):
        self.ctl.stop()

    def set_target(self, p):
        self.ctl.set_target(p)

    def observation(self):
        self._snap = self.ctl.snapshot()
        return self._snap.observation

    def open_full(self):
        self.ctl.set_target(1.0)

    def after_fault_clear(self):
        self.ctl.reset()

    def hold(self) -> Optional[float]:
        """停在当前位置。返回新的目标,None 表示还没有观测可用。"""
        s = self._snap or self.ctl.snapshot()
        if not s.observation.valid:
            return None
        self.ctl.set_target(s.observation.position)
        return s.observation.position

    def detail(self) -> str:
        s = self._snap
        if s is None:
            return ""
        name = str(s.state).split(".")[-1]
        stall = "HOLD" if s.stalled else "-"
        cap = "ON" if s.torque_capped else "-"
        out = (
            f"state={name:16s} cmd={s.commanded_torque_nm:5.3f}Nm  "
            f"eff={s.effective_position:.3f}  "
            f"stall={stall:>4s}({s.stall_trips})  "
            f"cap={cap:>3s}({s.torque_caps})"
        )
        if s.fault_reason:
            out += f"  fault: {s.fault_reason}"
        return out


class ForcePositionBackend:
    """ForcePositionController:接触判定后 kp=kd=0 的纯 tau_ff 保持。"""

    label = "FORCE-POSITION"

    def __init__(self, g: FollowerGripper, args):
        cfg = ForcePositionConfig()
        cfg.grasp_torque_nm = args.grasp_torque    # 接触后的保持力矩 = 夹持力
        # 闭合/张开速度。与上一项不独立:阻尼增益是 grasp/close_speed(上限 5),
        # 所以 close_speed < grasp/5 时增益饱和,爪子会堵转在低于设定的力上 ——
        # validate_config() 直接拒掉,而不是给一个悄悄变软的夹持。
        cfg.close_speed_radps = args.close_speed
        # hold/motion 上限与传输参数用默认值(1.8 / 6.0 Nm,350 ms / 100 Hz)。
        # 接触判定常数与位置增益不再是可配项:它们是固件常量的镜像和实测值,
        # 对这台夹爪只有一个正确答案。--kp/--kd 仍然喂 ImpedanceBackend。
        self.ctl = ForcePositionController(g, cfg)
        self._snap = None

    def start(self):
        self.ctl.start()

    def stop(self):
        self.ctl.stop()

    def set_target(self, p):
        self.ctl.set_target(p)

    def observation(self):
        self._snap = self.ctl.snapshot()
        return self._snap.observation

    def detail(self) -> str:
        s = self._snap
        if s is None:
            return ""
        name = str(s.state).split(".")[-1]
        out = (
            f"state={name:16s} hold={'Y' if s.holding else 'N'} "
            f"arr={'Y' if s.arrived else 'N'}  "
            f"cmd={s.commanded_torque_nm:5.3f}Nm  "
            f"grasp={s.grasp_torque_nm:5.3f}Nm  "
            f"limit hold={s.hold_torque_limit_nm:.2f}/"
            f"motion={s.motion_torque_limit_nm:.2f}/"
            f"dev={s.device_limit_nm:.2f}Nm"
        )
        if s.fault_reason:
            out += f"  fault: {s.fault_reason}"
        return out

    def open_full(self):
        # 有界速度阻尼张开,而不是位置阶跃。
        self.ctl.release()

    def hold(self) -> Optional[float]:
        self.ctl.hold_position()
        s = self.ctl.snapshot()
        return s.hold_position

    def after_fault_clear(self):
        # 顺序是硬性的:先 motor.clear_fault(),再 reset() 退出 Fault 态。
        self.ctl.reset()


# ── UI ────────────────────────────────────────────────────────────────────────


def _redraw(
    backend, head: str, envline: str, target: float, obs, norm_ok: bool, last_key: str
) -> None:
    if obs is not None and obs.valid:
        act_n = f"{obs.position:.3f}" if norm_ok else "  N/A"
        row = (
            f"{target:>10.3f}{act_n:>10s}{obs.raw_pos:>11.3f}"
            f"{obs.velocity:>10.3f}{obs.torque:>10.3f}"
            f"{obs.motor_temp_c:>9.0f}{_status_str(obs.status):>17s}"
        )
        age = f"age={obs.age_ms:.1f}ms  seq={obs.seq}"
    else:
        row = (
            f"{target:>10.3f}"
            + "-".rjust(10)
            + "-".rjust(11)
            + "-".rjust(10)
            + "-".rjust(10)
            + "-".rjust(9)
            + "-".rjust(17)
        )
        age = "age=  —  (还没收到状态帧)"
    lines = [
        head,
        envline,
        "  j/k=open-/+  o=open  c=close  h=hold  e/d=en/dis  f=fault_clear  q=quit",
        f"{'Tgt[0-1]':>10}{'Act[0-1]':>10}{'Act(rad)':>11}"
        f"{'Vel(r/s)':>10}{'Torq(Nm)':>10}{'Temp(C)':>9}{'State':>17}",
        "-" * 77,
        row,
        "  " + backend.detail(),
        f"  {age}   last key: {last_key}",
    ]
    out = "\033[H" + "\r\n".join(line + "\033[K" for line in lines) + "\033[J"
    sys.stdout.write(out)
    sys.stdout.flush()


# ── main ──────────────────────────────────────────────────────────────────────


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _calib_flow.add_target_argument(ap)
    ap.add_argument(
        "--mode",
        default="impedance",
        choices=("impedance", "force-position"),
        help="阻抗 (ImpedanceController) 或力位混合 (ForcePositionController)",
    )
    ap.add_argument("--kp", type=float, default=20.0, help="刚度 Nm/rad")
    ap.add_argument("--kd", type=float, default=1.0, help="阻尼 Nm·s/rad")
    ap.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="UI 刷新率。控制提交率不由它决定 —— 两个控制器都锁在状态流的"
        "相位上,一帧一提交",
    )
    ap.add_argument(
        "--step", type=float, default=0.05, help="j/k 步进量(归一化 0..1,默认 0.05)"
    )
    # ---- 阻抗模式的两层主机侧保护 ----
    ap.add_argument(
        "--max-position-torque",
        type=float,
        default=1.5,
        dest="max_position_torque",
        help="误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内",
    )
    ap.add_argument(
        "--rated-torque",
        type=float,
        default=2.0,
        dest="rated_torque",
        help="力矩天花板:实测力矩到顶后转纯前馈保持",
    )
    # ---- ForcePositionController ----
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=1.1,
        dest="grasp_torque",
        help="接触后的纯前馈保持力矩 Nm",
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=0.5,
        dest="close_speed",
        help="闭合速度 rad/s",
    )
    # ---- 固件运动安全包络 ----
    ap.add_argument("--show-envelope", action="store_true", help="打印包络后退出")
    ap.add_argument("--set-envelope", action="store_true", help="写入包络后继续")
    ap.add_argument("--peak", type=float, default=2.0, help="运动瞬态力矩上限 Nm")
    ap.add_argument("--cont", type=float, default=1.6, help="可持续力矩上限 Nm")
    ap.add_argument(
        "--temp-derate-start",
        type=int,
        default=0,
        dest="temp_derate_start",
        help="降额起点 °C,0=固件默认 90",
    )
    ap.add_argument(
        "--temp-wall",
        type=int,
        default=0,
        dest="temp_wall",
        help="温度墙 °C,0=固件默认 100",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, ep = _calib_flow.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # ---- 运动安全包络:固件侧、MIT 路径上唯一绕不过的一层 ----
    if args.set_envelope:
        e = g.get_envelope()
        e.cont_torque_nm, e.peak_torque_nm = args.cont, args.peak
        e.temp_derate_start_c = args.temp_derate_start
        e.temp_wall_c = args.temp_wall
        e.flags = GRIPPER_ENVELOPE_VALID | GRIPPER_ENVELOPE_ENFORCE
        g.set_envelope(e)
    env = g.get_envelope()
    print(f"[envelope] {env}")
    enforced = bool(env.flags & GRIPPER_ENVELOPE_ENFORCE)
    if not enforced:
        print(
            "[warn] 包络未启用 —— 被挡住时固件不钳 kp x 误差,I2t 与温度墙也不"
            "生效。\n"
            "       用 --set-envelope --peak 2.0 --cont 1.6 写一次(掉电保持)。"
        )
    if args.show_envelope:
        return 0

    cfg = g.get_gripper_config()
    norm_ok = bool(cfg.flags & 0x0001)
    if not norm_ok:
        log.warning("gripper not calibrated —— Act[0-1] 显示 N/A")

    Backend = {"impedance": ImpedanceBackend, "force-position": ForcePositionBackend}[
        args.mode
    ]
    backend = Backend(g, args)

    envline = (
        f"  envelope: cont={env.cont_torque_nm:.3f} "
        f"peak={env.peak_torque_nm:.3f} Nm  "
        f"temp={env.temp_derate_start_c or 90}/{env.temp_wall_c or 100}C  "
        + ("ENFORCED" if enforced else "*** INACTIVE ***")
    )
    # 只在阻抗模式显示增益:力位模式的位置增益已经不是可配项(在
    # detail::ForcePositionTuning 里),再把 --kp/--kd 印在标题上会让人以为
    # 它们生效了。
    gains = (f"  kp={args.kp:.2f} kd={args.kd:.2f}"
             if args.mode == "impedance"
             else f"  grasp={args.grasp_torque:.2f}Nm")
    head = (
        f"=== Gripper Console [{backend.label}]  {ep.firmware_sn}  "
        f"fw {g.firmware_version}{gains} ==="
    )

    last_key = "-"
    sys.stdout.write("\033[2J")
    sys.stdout.flush()

    try:
        g.motor.clear_fault()
        backend.start()  # 控制器自己起状态流,并以当前位置播种,不会跳变
        g.motor.enable()

        # 等首帧,用实际开度初始化本地目标。控制器 start() 已经播过种了,这里
        # 只是让 UI 和按键从同一个数出发 —— 不主动下发,避免多余的一次命令。
        target = 0.0
        deadline_init = time.monotonic() + 3.0
        while time.monotonic() < deadline_init:
            obs = backend.observation()
            if obs.valid:
                target = obs.position
                break
            time.sleep(0.01)

        period = 1.0 / max(args.hz, 1.0)
        with RawKeyboard():
            while True:
                ch = RawKeyboard.poll()
                if ch is not None:
                    last_key = repr(ch)

                if ch == "j":
                    target = min(1.0, target + args.step)
                    backend.set_target(target)
                elif ch == "k":
                    target = max(0.0, target - args.step)
                    backend.set_target(target)
                elif ch == "o":
                    target = 1.0
                    backend.open_full()
                elif ch == "c":
                    target = 0.0
                    backend.set_target(target)
                elif ch == "h":
                    held = backend.hold()
                    if held is not None:
                        target = held
                elif ch == "e":
                    g.motor.enable()
                elif ch == "d":
                    g.motor.disable()
                elif ch == "f":
                    g.motor.clear_fault()
                    backend.after_fault_clear()
                elif ch in ("q", "\x1b", "\x03"):
                    break

                _redraw(
                    backend,
                    head,
                    envline,
                    target,
                    backend.observation(),
                    norm_ok,
                    last_key,
                )
                time.sleep(period)
    finally:
        try:
            backend.stop()  # stop() 先下发零力矩
        except Exception as exc:
            print(f"\r\nstop: {exc}")
        try:
            g.motor.disable()
        except Exception as exc:
            print(f"\r\ndisable: {exc}")
        sys.stdout.write("\r\n[exit] controller stopped, motor disabled\r\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

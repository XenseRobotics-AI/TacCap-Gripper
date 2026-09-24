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
    python python/examples/gripper_console.py --set-envelope --peak 2.0 --cont 1.1
    python python/examples/gripper_console.py --mode force-position --grasp-torque 1.2

按键
    j / k   — 目标开度 + / -(一个 --step;归一化 0..1,0=闭合 1=张开)
    o       — 全开(1.0)          c — 全合(0.0)
    h       — 保持当前位置(仅力位混合)
    e / d   — 恢复控制/失能         f — 清故障(力位混合同时 reset 出 Fault)
    q / ESC — 退出

安全:真实运动。退出路径必定先 stop()(下发零力矩)再 disable()。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import sys
import termios
import time
import tty
from typing import Optional

import _target
from xense.taccap import (
    GRIPPER_ENVELOPE_ENFORCE,
    GRIPPER_ENVELOPE_VALID,
    MOTOR_RATED_TORQUE_NM,
    FollowerGripper,
    ForcePositionConfig,
    ForcePositionController,
    ImpedanceConfig,
    ImpedanceController,
    log,
)

# argparse 的默认值一律从库默认派生,不写字面量 —— 这里的 --rated-torque 曾经
# 硬编码 2.0,而 SDK 侧把上界从峰值收到额定后没人改它,阻抗模式默认参数崩了十天。
_IMP = ImpedanceConfig()
_FP = ForcePositionConfig()

# --cont 的回退值。这一个是字面量,因为 SDK 没有导出「连续堵转额定」这个常量 ——
# MOTOR_RATED_TORQUE_NM 是 1.8,那是**旋转**额定,拿来当 cont 用正好是这段代码要
# 防的错。真值源是电机自己:motor.get_spec().stall_cont_torque_nm,只有在设备报 0
# (手册未给该型号的堵转额定)时才落到这里。
_FALLBACK_CONT_NM = 1.1


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
    """ImpedanceController:误差钳位 + 力矩天花板,带状态机。

    被挡住时误差钳位饱和在 max_position_torque_nm 上并保持,那就是夹持力;
    天花板只兜底实测力矩。所有字段来自同一个 snapshot(),同一把锁。
    """

    label = "IMPEDANCE"

    def __init__(self, g: FollowerGripper, args):
        cfg = ImpedanceConfig()
        cfg.kp = args.kp  # 刚度 Nm/rad
        cfg.kd = args.kd  # 阻尼 Nm·s/rad,同时定接近速度
        # 误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内。主保护 —— 它同时
        # 限住了「接近」和「堵转」,而只防堵转是不够的:实测不钳位的大步进会以
        # 5.5 rad/s 掠过物体把它撞飞,全程力矩不到 0.03 Nm,没有持续接触可检测。
        cfg.max_position_torque_nm = args.max_position_torque
        # 力矩天花板:作用在实测力矩上(不是命令值)。顶住后改发 kp=kd=0 的纯前馈
        # 帧,位置误差再也加不进输出。上限是额定 1.8 而非峰值 6.0 —— 这个保持无限期。
        cfg.rated_torque_nm = args.rated_torque
        # 前馈保持 0。它一度被加进来是为了在失速跳闸后还剩点力,而那个守卫已经
        # 删了 —— 现在误差钳位自己就保持在预算上。再给前馈只会叠加在总命令上,
        # 把持续夹持力抬到预算之上,正好绕过"预算必须能无限期保持"这个选择。
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
        cap = "ON" if s.torque_capped else "-"
        out = (
            f"state={name:16s} cmd={s.commanded_torque_nm:5.3f}Nm  "
            f"eff={s.effective_position:.3f}  "
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
        cfg.grasp_torque_nm = args.grasp_torque  # 接触后的保持力矩 = 夹持力
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
    _target.add_target_argument(ap)
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
        default=_IMP.max_position_torque_nm,
        dest="max_position_torque",
        help=f"力矩预算/误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内。"
        f"被挡住时命令饱和在这里并一直保持,那就是夹持力 "
        f"(默认 {_IMP.max_position_torque_nm:.2f},须低于 --rated-torque)",
    )
    # 默认值从 MOTOR_RATED_TORQUE_NM 派生,不要再写字面量:这里原本硬编码 2.0,
    # 而 aa3d0a9 把 ImpedanceConfig 的上界从峰值 6.0 收到额定 1.8(那个保持是
    # 无限期的,所以按额定封顶),两边就此错开,阻抗模式默认参数直接抛 ValueError。
    ap.add_argument(
        "--rated-torque",
        type=float,
        default=_IMP.rated_torque_nm,
        dest="rated_torque",
        help=f"力矩天花板:实测力矩到顶后转纯前馈保持(上限=额定 {MOTOR_RATED_TORQUE_NM:.2f} Nm)",
    )
    # ---- ForcePositionController ----
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=_FP.grasp_torque_nm,
        dest="grasp_torque",
        help=f"接触后的纯前馈保持力矩 Nm(默认 {_FP.grasp_torque_nm:.2f} = EL05 连续"
        f"堵转额定 —— 被挡住的爪子会无限期坐在这个力矩上,没有任何东西给它计时)",
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=_FP.close_speed_radps,
        dest="close_speed",
        help=f"闭合速度 rad/s(默认 {_FP.close_speed_radps:.2f})。与 --grasp-torque "
        f"不独立:阻尼增益是 grasp/close_speed 且上限 5,所以低于 grasp/5 时增益"
        f"饱和,validate_config() 会直接拒掉而不是给一个悄悄变软的夹持",
    )
    # ---- 固件运动安全包络 ----
    ap.add_argument("--show-envelope", action="store_true", help="打印包络后退出")
    ap.add_argument("--set-envelope", action="store_true", help="写入包络后继续")
    ap.add_argument(
        "--peak",
        type=float,
        default=2.0,
        help="运动瞬态力矩上限 Nm(默认 2.0)。固件把命令位置钳在实测位置 "
        "±peak/kp 以内,所以它是 kp x 误差 的天花板;它同时定接近速度,约 "
        "peak/kd(实测 peak 1.5 / kd 1.0 -> 1.70 rad/s),想合得慢就调小它",
    )
    ap.add_argument(
        "--cont",
        type=float,
        default=None,
        help="可无限期维持的力矩上限 Nm。默认取**电机自报的连续堵转额定**"
        f"({_FALLBACK_CONT_NM:.2f} on EL05),不写字面量 —— 超过这个数固件会"
        "静默钳位,而状态行读的是 flash 里存的值,看不出来",
    )
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
    g, ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # ---- 运动安全包络:固件侧、MIT 路径上唯一绕不过的一层 ----
    #
    # cont 的上界是电机的**堵转**额定,不是手册首页那个「额定负载」。EL05 的
    # 1.8 N·m 是旋转额定(@100rpm);过载曲线给的是 6->1s / 4->6s / 1.8->175s /
    # 1.1->无限,而夹爪的主工况就是堵住不放,所以管我们的是 1.1。
    #
    # 固件按型号把 cont 钳到这个数,并且是**钳位而不是拒绝**(拒绝会让 flash 里
    # 带旧配置的设备直接失去保护),只打一条限频日志 —— 而日志走物理 UART7,没引
    # 到 USB,主机侧看不到。更糟的是 get_envelope() 读回的是 flash 里存的值,不是
    # 实际执行的值:写 1.6 进去,状态行会一直显示 1.600 而固件在按 1.1 执行。
    # 这个例子的默认值曾经就是 1.6。
    stall_cont = 0.0
    try:
        stall_cont = float(g.motor.get_spec().stall_cont_torque_nm)
    except Exception as exc:
        log.warning(f"读不到电机规格,--cont 回退到 {_FALLBACK_CONT_NM:.2f} Nm: {exc}")
    if stall_cont <= 0.0:
        # 该型号手册没给堵转额定 —— 固件也就不会钳,保护完全落在这个数上。
        stall_cont = _FALLBACK_CONT_NM
    cont = args.cont if args.cont is not None else stall_cont
    if cont > stall_cont:
        print(
            f"[warn] --cont {cont:.3f} 超过电机连续堵转额定 {stall_cont:.3f} Nm,"
            f"固件会静默钳到 {stall_cont:.3f}。\n"
            f"       写进去的值仍是 {cont:.3f},状态行显示的也是它 —— "
            f"读回的是 flash,不是实际执行的值。"
        )

    if args.set_envelope:
        e = g.get_envelope()
        e.cont_torque_nm, e.peak_torque_nm = cont, args.peak
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
            f"       用 --set-envelope --peak {args.peak:.1f} --cont {cont:.1f} "
            f"写一次(掉电保持)。"
        )
    if env.cont_torque_nm > stall_cont:
        # 实测在 0015s 上撞到过:flash 里存着 cont=1.800(那是**旋转**额定),
        # 固件一直按 1.100 执行,而控制台此前只显示 1.800。
        print(
            f"[warn] 存的 cont={env.cont_torque_nm:.3f} 超过连续堵转额定 "
            f"{stall_cont:.3f} Nm —— 固件实际按 {stall_cont:.3f} 执行。\n"
            f"       读回的是 flash 里的记录,不是生效值;固件那条钳位日志走"
            f"物理 UART7,主机侧看不到。\n"
            f"       要让两者一致:--set-envelope --cont {stall_cont:.1f}"
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

    # 存的值超出堵转额定时,把固件实际执行的数一起显示 —— 只显示 flash 里的值
    # 会让人以为设置生效了。
    cont_shown = f"{env.cont_torque_nm:.3f}"
    if env.cont_torque_nm > stall_cont:
        cont_shown += f"->{stall_cont:.3f}(固件钳位)"
    envline = (
        f"  envelope: cont={cont_shown} "
        f"peak={env.peak_torque_nm:.3f} Nm  "
        f"temp={env.temp_derate_start_c or 90}/{env.temp_wall_c or 100}C  "
        + ("ENFORCED" if enforced else "*** INACTIVE ***")
    )
    # 只在阻抗模式显示增益:力位模式的位置增益已经不是可配项(在
    # detail::ForcePositionTuning 里),再把 --kp/--kd 印在标题上会让人以为
    # 它们生效了。
    gains = (
        f"  kp={args.kp:.2f} kd={args.kd:.2f}"
        if args.mode == "impedance"
        else f"  grasp={args.grasp_torque:.2f}Nm"
    )
    head = (
        f"=== Gripper Console [{backend.label}]  {ep.firmware_sn}  "
        f"fw {g.firmware_version}{gains} ==="
    )

    last_key = "-"
    # The controller owns the periodic MIT stream.  A bare Motor.disable() is
    # not enough while that stream is alive: the next status frame would send
    # another control command and the firmware can re-enter its run mode.
    # Keep this bit as the lifecycle guard for the e/d keys.  `d` stops the
    # controller (which sends zero torque and disables), and `e` starts it
    # again before enabling the motor, matching the SDK lifecycle contract.
    controller_active = False
    sys.stdout.write("\033[2J")
    sys.stdout.flush()

    try:
        g.motor.clear_fault()
        backend.start()  # 控制器自己起状态流,并以当前位置播种,不会跳变
        g.motor.enable()
        controller_active = True

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

                if ch == "j" and controller_active:
                    target = min(1.0, target + args.step)
                    backend.set_target(target)
                elif ch == "k" and controller_active:
                    target = max(0.0, target - args.step)
                    backend.set_target(target)
                elif ch == "o" and controller_active:
                    target = 1.0
                    backend.open_full()
                elif ch == "c" and controller_active:
                    target = 0.0
                    backend.set_target(target)
                elif ch == "h" and controller_active:
                    held = backend.hold()
                    if held is not None:
                        target = held
                elif ch == "e" and not controller_active:
                    # start() must precede enable(): it seeds the controller
                    # from the disabled motor's current position and starts
                    # the status stream before any control frame is accepted.
                    backend.start()
                    g.motor.enable()
                    controller_active = True
                    # ...and that seeding is why the local target has to be
                    # re-read. The motor was DISABLED while paused, so the jaw
                    # is back-drivable and may have been moved by hand; start()
                    # takes wherever it is now as the target, while `target`
                    # still holds the pre-pause number. Leave them apart and the
                    # header lies about what is commanded, then the next j/k
                    # steps from the stale value -- a jump, not a step. start()
                    # reads one status synchronously, so this is valid at once.
                    resumed = backend.observation()
                    if resumed.valid:
                        target = resumed.position
                elif ch == "d" and controller_active:
                    # Stop owns the safe ordering: zero torque first, then
                    # disable, and finally tear down the status subscription.
                    # Calling motor.disable() alone races the controller's
                    # next MIT frame and is why `d` used to appear ineffective.
                    backend.stop()
                    controller_active = False
                elif ch == "f":
                    g.motor.clear_fault()
                    if controller_active:
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

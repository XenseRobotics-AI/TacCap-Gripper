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

包络填什么不是使用者要回答的问题,所以这里没有数值旋钮:设备自己知道它装的电机
额定多少,而固件无论写进去什么都按那个额定钳。--show-envelope 会把**存的**和
**实际生效的**分开打印 —— 那两个数会不一样。

用法
    python python/examples/gripper_console.py --show-envelope
    python python/examples/gripper_console.py --set-envelope
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
    FollowerGripper,
    ForcePositionConfig,
    ForcePositionController,
    ImpedanceConfig,
    ImpedanceController,
    log,
)

# argparse 的默认值全部是 None,真正的默认在设备打开之后由它自报的电机规格推出
# (ImpedanceConfig.for_spec / ForcePositionConfig.for_spec)。
#
# 不能在这里写死:EL05 的持续夹持是 1.1 Nm,RS00 是 3.6 —— 差 3.3 倍,而那正是
# 换电机的理由。更隐蔽的是 kd:接近速度是 预算/kd,写死 kd 会让"夹得更紧"顺带
# 变成"撞得更快"。这两件事必须一起由电机决定。
#
# (历史:--rated-torque 曾经硬编码 2.0,SDK 把上界从峰值收到额定后没人改它,
#  阻抗模式默认参数崩了十天。写死默认值的代价这里付过一次了。)


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
        # 默认值来自设备自报的电机规格,不是编译期常量 —— EL05 和 RS00 的持续
        # 夹持差 3.3 倍,而 kd 又跟着预算走好让接近速度不变。
        cfg = ImpedanceConfig.for_spec(g.motor.get_spec())
        if args.kp is not None:
            cfg.kp = args.kp  # 刚度 Nm/rad
        if args.kd is not None:
            cfg.kd = args.kd  # 阻尼 Nm·s/rad,同时定接近速度
        # 误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内。主保护 —— 它同时
        # 限住了「接近」和「堵转」,而只防堵转是不够的:实测不钳位的大步进会以
        # 5.5 rad/s 掠过物体把它撞飞,全程力矩不到 0.03 Nm,没有持续接触可检测。
        if args.max_position_torque is not None:
            cfg.max_position_torque_nm = args.max_position_torque
        # 力矩天花板:作用在实测力矩上(不是命令值)。顶住后改发 kp=kd=0 的纯前馈
        # 帧,位置误差再也加不进输出。上限是额定 1.8 而非峰值 6.0 —— 这个保持无限期。
        if args.rated_torque is not None:
            cfg.rated_torque_nm = args.rated_torque
        # 前馈保持 0。它一度被加进来是为了在失速跳闸后还剩点力,而那个守卫已经
        # 删了 —— 现在误差钳位自己就保持在预算上。再给前馈只会叠加在总命令上,
        # 把持续夹持力抬到预算之上,正好绕过"预算必须能无限期保持"这个选择。
        # status_timeout_ms / motor_stream_hz 用默认值(350 ms / 100 Hz)。
        self.ctl = ImpedanceController(g, cfg)
        self.cfg = cfg  # 标题行显示**生效值**,不是命令行原值
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
        # 同上:grasp / hold / motion / close_speed 全部按设备的电机推出来。
        cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
        if args.grasp_torque is not None:
            cfg.grasp_torque_nm = args.grasp_torque  # 保持力矩 = 夹持力
        # 闭合速度与 grasp **是独立的**。曾经有过 grasp/close_speed 的耦合规则
        # (行程阻尼增益当年就是这个比值),现在斜坡自己调速、预算拆分定增益,
        # 所以慢就只是慢 —— 见 force_position_controller.cpp 的 validate_config。
        if args.close_speed is not None:
            cfg.close_speed_radps = args.close_speed
        # hold/motion 上限也来自电机(EL05 1.8/6.0,RS00 5.0/14.0)。
        # 接触判定常数与位置增益不再是可配项:它们是固件常量的镜像和实测值,
        # 对这台夹爪只有一个正确答案。--kp/--kd 仍然喂 ImpedanceBackend。
        self.ctl = ForcePositionController(g, cfg)
        self.cfg = cfg
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
        # Vel/Torq 是夹爪坐标系:**正 = 闭合**。电机装配方向已被消掉,所以两台
        # 装反的爪子上同一个动作符号相同(0.3.1 之前不是这样)。
        f"{'Vel(+闭合)':>10}{'Torq(+闭合)':>10}{'Temp(C)':>9}{'State':>17}",
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
    ap.add_argument("--kp", type=float, default=None, help="刚度 Nm/rad(默认 20)")
    ap.add_argument(
        "--kd",
        type=float,
        default=None,
        help="阻尼 Nm·s/rad。**同时决定接近速度**(≈ 预算/kd)。默认由电机推出,"
        "让接近速度落在 2 rad/s",
    )
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
        default=None,
        dest="max_position_torque",
        help="力矩预算/误差钳位:命令目标限制在实测位置 ±(该值/kp) rad 内。"
        "被挡住时命令饱和在这里并一直保持,那就是夹持力。默认取电机自报的"
        "**连续堵转额定**(EL05 1.10 / RS00 3.60),须低于 --rated-torque",
    )
    # 默认值从 MOTOR_RATED_TORQUE_NM 派生,不要再写字面量:这里原本硬编码 2.0,
    # 而 aa3d0a9 把 ImpedanceConfig 的上界从峰值 6.0 收到额定 1.8(那个保持是
    # 无限期的,所以按额定封顶),两边就此错开,阻抗模式默认参数直接抛 ValueError。
    ap.add_argument(
        "--rated-torque",
        type=float,
        default=None,
        dest="rated_torque",
        help="力矩天花板:实测力矩到顶后转纯前馈保持。默认取电机自报的**旋转额定**"
        "(EL05 1.80 / RS00 5.00);start() 会拿设备实际额定卡它",
    )
    # ---- ForcePositionController ----
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=None,
        dest="grasp_torque",
        help="接触后的纯前馈保持力矩 Nm = 夹持力。默认取电机自报的**连续堵转额定**"
        "(EL05 1.10 / RS00 3.60)—— 被挡住的爪子会无限期坐在这个力矩上,"
        "没有任何东西给它计时",
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=None,
        dest="close_speed",
        help="闭合速度 rad/s,默认 2.0(MOTOR_APPROACH_SPEED_RADPS)。它与 "
        "--grasp-torque **是独立的** —— 曾经有过 grasp/close_speed 的耦合规则,"
        "因为行程阻尼增益当年就是这个比值;现在斜坡自己调速、预算拆分定增益,"
        "所以慢就只是慢",
    )
    # ---- 固件运动安全包络 ----
    ap.add_argument(
        "--show-envelope",
        action="store_true",
        help="打印包络(存的 / 实际生效的)后退出,只读",
    )
    ap.add_argument(
        "--set-envelope",
        action="store_true",
        help="按设备自报的电机额定修好包络后继续。**写 MCU flash**,掉电保持;"
        "已经正确就什么都不写。绝不放宽 —— 特意收紧过的设备保持原值",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")

    # ---- 运动安全包络:固件侧、MIT 路径上唯一绕不过的一层 ----
    #
    # 这里没有数值旋钮,这是有意的。包络该填什么不是调用方的问题:设备自己知道它
    # 装的电机额定多少,而固件无论写进去什么都按那个额定钳。数由 SDK 从设备读。
    #
    # 要看清的是 stored 与 effective 的区别 —— 固件把 cont 钳下去这件事只记在一条
    # 没接到 USB 的 UART 上,而读回的是 flash 里的记录。实测 0015s 存着 1.800、
    # 一直按 1.100 跑,而这个控制台以前只会显示 1.800。
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective {a.effective if a.effective else '*** 固件什么都不执行 ***'}"
    )
    if not a.ok:
        print(f"[warn] {a.detail}")
        if a.needs_write:
            print("       用 --set-envelope 修一次(写 MCU flash,掉电保持)。")

    if args.set_envelope:
        w = g.ensure_envelope()
        print(
            f"[envelope] 已写入 {w.written}" if w.wrote else "[envelope] 已正确,未写入"
        )
        a = g.audit_envelope()

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
    eff = a.effective
    envline = (
        f"  envelope: cont={eff.cont_torque_nm:.3f} "
        f"peak={eff.peak_torque_nm:.3f} Nm  "
        f"temp={eff.temp_derate_start_c or 90}/{eff.temp_wall_c or 100}C  ENFORCED"
        if eff
        else "  envelope: *** 未生效 —— 固件不钳 kp x 误差,I2t 与温度墙也不生效 ***"
    )
    # 显示的是**生效值**而不是命令行原值 —— 现在这些默认值由设备的电机规格推出来
    # (for_spec),命令行不给就是 None,直接格式化会抛 TypeError。
    #
    # 只在阻抗模式显示增益:力位模式的位置增益不是可配项(在
    # detail::ForcePositionTuning 里),把它印在标题上会让人以为它生效了。
    c = backend.cfg
    gains = (
        f"  预算={c.max_position_torque_nm:.2f}Nm kp={c.kp:.1f} kd={c.kd:.2f}"
        f" ({c.max_position_torque_nm / c.kd:.1f}rad/s)"
        if args.mode == "impedance"
        else f"  grasp={c.grasp_torque_nm:.2f}Nm close={c.close_speed_radps:.2f}rad/s"
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

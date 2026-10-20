#!/usr/bin/env python3
# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Keyboard console for a single follower gripper — goes through the SDK
controllers, never sends bare MIT frames.

The original console called `motor.submit_impedance()` directly. That is a
**motor primitive**: it packs one MIT frame and drops it on the bus, with no
error clamp, no torque ceiling and no stall protection — not one layer of
host-side protection is on its path (see cpp/src/components/motor.cpp). With
kp=20 against a hard object and the target pushed to fully closed, kp x position
error kept rising up to the motor's own 0x700B limit (that model's t_max: EL05
6 Nm / RS00 14 Nm), and the current dragged the 24 V bus down: the jaw let go
and dropped the part, the status bits read EN|FAULT|UNDER_VOLT, and from then on
there was no torque until a power cycle.

This version hands control to the SDK's two controllers and configures the
firmware-side motion safety envelope along with them.

    --mode impedance       ImpedanceController — error clamp + torque ceiling
                           + stall hold
    --mode force-position  ForcePositionController — one bounded-torque control
                           law throughout, saturation is contact

The one thing to remember: **the firmware envelope is the only layer on the MIT
path that nobody can bypass**, and it ships disabled (GripperConfig.reserved all
0). If it is not turned on, it is not there, and the two host-side controllers
cannot replace it — over a 100 Hz phase-locked link the host reacts in tens of
milliseconds at best, which at 8 rad/s is 0.24 rad. Write it once with
--set-envelope; it persists across power loss.

What to put in the envelope is not a question for the user, so there are no
numeric knobs here: the device knows the rating of the motor it carries, and the
firmware clamps to that rating whatever is written. --show-envelope prints the
**stored** and the **actually effective** values separately — those two numbers
will differ.

Usage
    python python/examples/gripper_console.py --show-envelope
    python python/examples/gripper_console.py --set-envelope
    python python/examples/gripper_console.py --mode force-position --grasp-torque 0.8

Keys
    j / k   — target opening + / - (one --step; normalized 0..1, 0=closed 1=open)
    o       — fully open (1.0)       c — fully closed (0.0)
    h       — hold current position (force-position only)
    e / d   — resume control/disable
    f       — clear fault (force-position also resets out of Fault)
    q / ESC — quit

Safety: real motion. The exit path always calls stop() (sends zero torque) first,
then disable().
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

# Every argparse default is None; the real defaults are derived, after the device
# is opened, from the motor spec it reports (ImpedanceConfig.for_spec /
# ForcePositionConfig.for_spec).
#
# They cannot be hardcoded here: EL05 sustained grip is 1.1 Nm, RS00 is 3.6 — a
# 3.3x difference, and that is exactly the reason for changing motors. The subtler
# one is kd: approach speed is budget/kd, so a hardcoded kd turns "grip harder"
# into "crash faster" as a side effect. Both must be decided by the motor together.
#
# (History: --rated-torque was once hardcoded to 2.0; nobody changed it after the
#  SDK lowered the upper bound from peak to rated, and impedance mode's default
#  arguments were broken for ten days. This file has already paid the price of a
#  hardcoded default once.)


# ── Status bits (protocol::MotorStatusBit) ───────────────────────────────────

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


# ── Keyboard raw mode ─────────────────────────────────────────────────────────


class RawKeyboard:
    """Switch stdin to non-blocking raw mode and poll it byte by byte."""

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


# ── Two backends ──────────────────────────────────────────────────────────────
#
# The controller owns the status stream and the control path: do not also call
# g.start_streaming(), and do not register motor.on_status() — all observations
# come from observation()/snapshot(), and nothing touches the bus while
# controlling.


class ImpedanceBackend:
    """ImpedanceController: error clamp + torque ceiling, with a state machine.

    When blocked, the error clamp saturates at max_position_torque_nm and holds
    there; that is the grip force. The ceiling is only a backstop on measured
    torque. All fields come from the same snapshot(), under the same lock.
    """

    label = "IMPEDANCE"

    def __init__(self, g: FollowerGripper, args):
        # Defaults come from the motor spec the device reports, not compile-time
        # constants — EL05 and RS00 sustained grip differ by 3.3x, and kd follows
        # the budget so that approach speed stays the same.
        cfg = ImpedanceConfig.for_spec(g.motor.get_spec())
        if args.kp is not None:
            cfg.kp = args.kp  # stiffness Nm/rad
        if args.kd is not None:
            cfg.kd = args.kd  # damping Nm·s/rad, also sets approach speed
        # Error clamp: the commanded target is limited to within ±(this/kp) rad of
        # the measured position. The main protection — it bounds both "approach"
        # and "stall", and guarding only against stall is not enough: measured, an
        # unclamped large step sweeps past the object at 5.5 rad/s and knocks it
        # away, with under 0.03 Nm of torque the whole time and no sustained
        # contact to detect.
        if args.max_position_torque is not None:
            cfg.max_position_torque_nm = args.max_position_torque
        # Torque ceiling: acts on measured torque (not the commanded value). Once
        # hit, it switches to pure-feedforward frames with kp=kd=0, so position
        # error can no longer add to the output. The cap is rated 1.8, not peak
        # 6.0 — this hold is indefinite.
        if args.rated_torque is not None:
            cfg.rated_torque_nm = args.rated_torque
        # Feedforward stays 0. It was once added to leave some force after a stall
        # trip, but that guard has been removed — the error clamp now holds at the
        # budget by itself. Adding feedforward would only stack on the total
        # command and lift the sustained grip above the budget, bypassing exactly
        # the choice that "the budget must be holdable indefinitely".
        # status_timeout_ms / motor_stream_hz use the defaults (350 ms / 100 Hz).
        self.ctl = ImpedanceController(g, cfg)
        self.cfg = cfg  # header shows the **effective values**, not the raw CLI values
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
        """Stop at the current position.

        Returns the new target, None if no observation is available yet.
        """
        s = self._snap or self.ctl.snapshot()
        if not s.observation.valid:
            return None
        self.ctl.set_target(s.observation.position)
        return s.observation.position

    def detail(self) -> str:
        s = self._snap
        if s is None:
            return ""
        name = s.state.name
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
    """ForcePositionController: one bounded-torque control law throughout.

    The error is clamped at grasp_torque_nm; saturation is contact, there is no
    separate contact detection.
    """

    label = "FORCE-POSITION"

    def __init__(self, g: FollowerGripper, args):
        # Same as above: grasp / hold / motion / close_speed are all derived from
        # the device's motor.
        cfg = ForcePositionConfig.for_spec(g.motor.get_spec())
        if args.grasp_torque is not None:
            cfg.grasp_torque_nm = args.grasp_torque  # torque budget = grip force
        # Close speed **is independent** of grasp. There used to be a coupling rule
        # between grasp/close_speed (the travel damping gain was that ratio back
        # then); now the ramp sets the speed itself and the budget split sets the
        # gains, so slow is just slow — see validate_config in
        # force_position_controller.cpp.
        if args.close_speed is not None:
            cfg.close_speed_radps = args.close_speed
        # The hold/motion limits also come from the motor (EL05 1.8/6.0, RS00
        # 5.0/14.0). Position gains are not configurable (the contact-detection
        # constants were removed together with the state machine): they are
        # measured values, and for this gripper there is only one right answer.
        # --kp/--kd still feed ImpedanceBackend.
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
        name = s.state.name
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
        # Open with bounded velocity damping, not a position step.
        self.ctl.release()

    def hold(self) -> Optional[float]:
        self.ctl.hold_position()
        s = self.ctl.snapshot()
        return s.hold_position

    def after_fault_clear(self):
        # The order is mandatory: motor.clear_fault() first, then reset() to leave
        # the Fault state.
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
        age = "age=  —  (no status frame received yet)"
    lines = [
        head,
        envline,
        "  j/k=open-/+  o=open  c=close  h=hold  e/d=en/dis  f=fault_clear  q=quit",
        f"{'Tgt[0-1]':>10}{'Act[0-1]':>10}{'Act(rad)':>11}"
        # Vel/Torq are in the gripper frame: **positive = closing** ("cls"). The
        # motor mounting direction has been cancelled out, so the same motion
        # has the same sign on two mirror-mounted jaws (not so before 0.3.1).
        f"{'Vel(+cls)':>10}{'Torq(+cls)':>10}{'Temp(C)':>9}{'State':>17}",
        "-" * 77,
        row,
        "  " + backend.detail(),
        f"  {age}   last key: {last_key}",
    ]
    out = "\033[H" + "\r\n".join(line + "\033[K" for line in lines) + "\033[J"
    sys.stdout.write(out)
    sys.stdout.flush()


# ── main ──────────────────────────────────────────────────────────────────────


def _motor_firmware_label(motor) -> str:
    """Read once before control starts, never inside the control loop.

    The query may stop older motors; MIT may return only a flash record.
    Do not switch protocol or write records merely to populate the display.
    """
    try:
        version = motor.motor_version(3000)
        if not version.valid:
            return "unknown (无有效版本记录)"
        source = "Flash存档/非实时" if version.from_flash else "实时读取"
        return f"{version.vendor_str} [{source}]"
    except Exception as exc:
        return f"unknown (查询失败: {type(exc).__name__})"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    _target.add_target_argument(ap)
    ap.add_argument(
        "--mode",
        default="impedance",
        choices=("impedance", "force-position"),
        help="impedance (ImpedanceController) or force-position hybrid "
        "(ForcePositionController)",
    )
    ap.add_argument(
        "--kp", type=float, default=None, help="stiffness Nm/rad (default 20)"
    )
    ap.add_argument(
        "--kd",
        type=float,
        default=None,
        help="damping Nm·s/rad. **Also sets approach speed** (≈ budget/kd). The "
        "default is derived from the motor so that approach speed lands at 1.1 "
        "rad/s (MOTOR_APPROACH_SPEED_RADPS)",
    )
    ap.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="UI refresh rate. It does not set the control submit rate — both "
        "controllers are locked to the status stream's phase, one submit per frame",
    )
    ap.add_argument(
        "--step",
        type=float,
        default=0.05,
        help="j/k step size (normalized 0..1, default 0.05)",
    )
    # ---- Impedance mode's two layers of host-side protection ----
    ap.add_argument(
        "--max-position-torque",
        type=float,
        default=None,
        dest="max_position_torque",
        help="torque budget/error clamp: the commanded target is limited to within "
        "±(this/kp) rad of the measured position. When blocked, the command "
        "saturates here and stays there; that is the grip force. Defaults to the "
        "motor's reported **continuous stall rating** (EL05 1.10 / RS00 3.60); "
        "must be below --rated-torque",
    )
    # The default is derived from MOTOR_RATED_TORQUE_NM; do not write a literal
    # again: this was originally hardcoded to 2.0, and aa3d0a9 lowered
    # ImpedanceConfig's upper bound from peak 6.0 to rated 1.8 (that hold is
    # indefinite, so it is capped at rated). The two drifted apart, and impedance
    # mode's default arguments threw ValueError outright.
    ap.add_argument(
        "--rated-torque",
        type=float,
        default=None,
        dest="rated_torque",
        help="torque ceiling: once measured torque hits it, switch to a pure "
        "feedforward hold. Defaults to the motor's reported **rotating rating** "
        "(EL05 1.80 / RS00 5.00); start() checks it against the device's actual "
        "rating",
    )
    # ---- ForcePositionController ----
    ap.add_argument(
        "--grasp-torque",
        type=float,
        default=None,
        dest="grasp_torque",
        help="torque budget Nm = grip force (the error clamp's upper bound; "
        "saturates here when blocked). Defaults to the motor's reported "
        "**continuous stall rating** (EL05 1.10 / RS00 3.60) and cannot exceed it "
        "— start() rejects it outright if it does. A blocked jaw sits at this "
        "torque indefinitely, and nothing times it",
    )
    ap.add_argument(
        "--close-speed",
        type=float,
        default=None,
        dest="close_speed",
        help="close speed rad/s, default 1.1 (MOTOR_APPROACH_SPEED_RADPS). It "
        "**is independent** of --grasp-torque — there used to be a coupling rule "
        "between grasp/close_speed, because the travel damping gain was that ratio "
        "back then; now the ramp sets the speed itself and the budget split sets "
        "the gains, so slow is just slow",
    )
    # ---- Firmware motion safety envelope ----
    ap.add_argument(
        "--show-envelope",
        action="store_true",
        help="print the envelope (stored / actually effective) and exit; read-only",
    )
    ap.add_argument(
        "--set-envelope",
        action="store_true",
        help="fix the envelope to the motor rating the device reports, then "
        "continue. **Writes MCU flash**, persists across power loss; writes nothing "
        "if already correct. When the device reports a spec, cont/peak are set "
        "exactly to the rating, and values that are too low are fixed too; the "
        "stricter existing value is kept only when the spec is unknown",
    )
    args = ap.parse_args()

    log.set_level("warn")
    g, ep = _target.open_follower(args.target)
    print(f"[fw] {g.firmware_version}")
    motor_fw = _motor_firmware_label(g.motor)
    print(f"[motor fw] {motor_fw}")

    # ---- Motion safety envelope: firmware-side, the only layer on the MIT path
    # ---- that cannot be bypassed
    #
    # There are no numeric knobs here, on purpose. What the envelope should hold is
    # not the caller's question: the device knows the rating of the motor it
    # carries, and the firmware clamps to that rating whatever is written. The SDK
    # reads the numbers from the device.
    #
    # What matters is the difference between stored and effective — the firmware
    # logs clamping cont down only on a UART that is not wired to USB, while what
    # reads back is the record in flash. Measured: 0015s stored 1.800 and ran at
    # 1.100 all along, and this console used to show only 1.800.
    a = g.audit_envelope()
    print(f"[envelope] stored    {a.stored}")
    print(
        f"[envelope] effective {a.effective if a.effective else '*** firmware enforces nothing ***'}"
    )
    if not a.ok:
        print(f"[warn] {a.detail}")
        if a.needs_write:
            print(
                "       Fix it once with --set-envelope (writes MCU flash, "
                "persists across power loss)."
            )

    if args.set_envelope:
        w = g.ensure_envelope()
        print(
            f"[envelope] written {w.written}"
            if w.wrote
            else "[envelope] already correct, not written"
        )
        a = g.audit_envelope()

    if args.show_envelope:
        return 0

    cfg = g.get_gripper_config()
    norm_ok = bool(cfg.flags & 0x0001)
    if not norm_ok:
        log.warning("gripper not calibrated — Act[0-1] shows N/A")

    Backend = {"impedance": ImpedanceBackend, "force-position": ForcePositionBackend}[
        args.mode
    ]
    backend = Backend(g, args)

    # When the stored value exceeds the stall rating, also show the number the
    # firmware actually enforces — showing only the value in flash would make it
    # look as if the setting took effect.
    eff = a.effective
    envline = (
        f"  envelope: cont={eff.cont_torque_nm:.3f} "
        f"peak={eff.peak_torque_nm:.3f} Nm  "
        f"temp={eff.temp_derate_start_c or 90}/{eff.temp_wall_c or 100}C  ENFORCED"
        if eff
        else "  envelope: *** NOT ENFORCED — firmware does not clamp kp x error, "
        "I2t and the temperature wall are not in effect either ***"
    )
    # This shows the **effective values**, not the raw CLI values — these defaults
    # are now derived from the device's motor spec (for_spec); when not given on
    # the command line they are None, and formatting them directly would raise
    # TypeError.
    #
    # Gains are shown only in impedance mode: force-position mode's position gains
    # are not configurable (they live in detail::ForcePositionTuning), and printing
    # them in the header would make it look as if they took effect.
    c = backend.cfg
    gains = (
        f"  budget={c.max_position_torque_nm:.2f}Nm kp={c.kp:.1f} kd={c.kd:.2f}"
        f" ({c.max_position_torque_nm / c.kd:.1f}rad/s)"
        if args.mode == "impedance"
        else f"  grasp={c.grasp_torque_nm:.2f}Nm close={c.close_speed_radps:.2f}rad/s"
    )
    head = (
        f"=== Gripper Console [{backend.label}]  {ep.firmware_sn}  "
        f"fw {g.firmware_version}  motor_fw={motor_fw}{gains} ==="
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
        backend.start()  # controller starts its own status stream, seeded at the
        # current position, so there is no jump
        g.motor.enable()
        controller_active = True

        # Wait for the first frame and initialize the local target from the actual
        # opening. The controller's start() has already seeded itself; this only
        # makes the UI and the keys start from the same number — nothing is sent,
        # to avoid one redundant command.
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
            backend.stop()  # stop() sends zero torque first
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

# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Customer operations sharing one native motor-control session."""
from __future__ import annotations

import math
import threading
import time

from . import _taccap_native as _native
from .gripper_types import (
    BusyError,
    FaultError,
    GripperDevice,
    GripperDiagnostics,
    GripperError,
    GripperInfo,
    GripperLimits,
    GripperResult,
    GripperState,
    GripperTiming,
    OperationInterrupted,
)


def _in_range(name: str, value: float, low: float, high: float) -> float:
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite and in [{low}, {high}]")
    return value


def _positive(name, value, high):
    value = _in_range(name, value, 0, high)
    if value == 0:
        raise ValueError(f"{name} must be greater than zero")
    return value


def _timeout(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be finite and positive")
    return value


def _wait_options(wait, timeout):
    if not isinstance(wait, bool):
        raise TypeError("wait must be a bool")
    return _timeout(timeout)


def _firmware_fault(execution):
    if execution.owner == 4 or execution.last_error or execution.flags & 2:
        return f"firmware fault (code {execution.last_error}, owner {execution.owner})"
    return None


class Gripper:
    """Calibrated follower with position, grasp and impedance control.

    Connection is passive. Operations share one native runtime; switching modes
    does not stop it or disable the motor. wait=False remains the default for
    compatibility. Waiting timeout leaves control active. Context exit disables.
    """

    limits = GripperLimits()

    @staticmethod
    def discover() -> tuple[GripperDevice, ...]:
        """Discover followers without enabling motion; pass a result to Gripper()."""
        return tuple(GripperDevice(e.mcu_device, e.firmware_sn, e.side.name.lower())
                     for e in _native.scan_grippers() if e.role == _native.Role.Follower)

    def __init__(self, target: str | GripperDevice | None = None, *,
                 position_tolerance: float = 0.01, closed_tolerance: float = 0.03,
                 settle_time: float = 0.05, torque_limit_nm: float = 0.35,
                 speed_rad_s: float = 0.5, update_hz: int = 100,
                 close_compensation_rad: float = 0.0):
        self._close_compensation_rad = _in_range("close_compensation_rad", close_compensation_rad, 0, 0.05)
        self._torque_limit_nm = _positive("torque_limit_nm", torque_limit_nm, self.limits.max_torque_nm)
        self._speed_rad_s = _positive("speed_rad_s", speed_rad_s, self.limits.max_speed_rad_s)
        if isinstance(update_hz, bool) or not isinstance(update_hz, int) or update_hz not in (20, 25, 40, 50, 100):
            raise ValueError("update_hz must be one of 20, 25, 40, 50, 100")
        self._update_hz = update_hz
        self._position_tolerance = _positive("position_tolerance", position_tolerance, 0.1)
        self._closed_tolerance = _in_range("closed_tolerance", closed_tolerance,
                                          self._position_tolerance, 0.1)
        self._settle_time = _in_range("settle_time", settle_time, 0, 5)
        self._lock = threading.RLock()
        self._controller = None
        self._target = None
        self._operation = None
        self._generation = 0
        self._fault = None
        self._disable_pending = False
        if isinstance(target, GripperDevice):
            target = target.device
        if target is not None and not isinstance(target, str):
            raise TypeError("target must be left/right, a firmware SN, device path or GripperDevice")
        if target and target.startswith("/dev/"):
            path = target
        else:
            devices = self.discover()
            if target is not None:
                devices = tuple(d for d in devices if
                                (d.side == target.lower() if target.lower() in ("left", "right")
                                 else d.firmware_sn == target))
            if len(devices) != 1:
                raise GripperError(f"Expected one follower for {target!r}, found {len(devices)}. "
                                   "Use Gripper.discover() and select a device.")
            path = devices[0].device
        self._device = _native.FollowerGripper(mcu_device=path)
        try:
            version = self._device.firmware_version
            if version is None or (version.major, version.minor, version.patch) < (1, 1, 7):
                raise GripperError("Gripper requires follower firmware >= 1.1.7; use advanced for maintenance")
            if self._close_compensation_rad and (version.major, version.minor, version.patch) < (1, 1, 8):
                raise GripperError("close_compensation_rad requires firmware >= 1.1.8")
            try:
                position_map = self._device.position_map()
            except Exception as exc:
                raise GripperError(
                    f"Cannot read valid follower travel calibration: {exc}. "
                    "Complete follower startup calibration or use advanced maintenance; "
                    "Gripper does not start calibration automatically.") from exc
            self._info = GripperInfo(path, (version.major, version.minor, version.patch),
                                     position_map.max_open_rad - position_map.min_open_rad,
                                     position_map.reverse)
        except BaseException:
            self._device.transport.stop()
            self._device = None
            raise

    @property
    def info(self) -> GripperInfo:
        """Cached connection identity and calibration; does not query the device."""
        return self._info

    def _connected(self):
        if self._device is None:
            raise GripperError("Gripper is closed")
        return self._device

    def _start(self):
        device = self._connected()
        if self._fault:
            raise FaultError(f"{self._fault}; resolve the cause and call clear_fault()")
        if self._controller is None:
            execution = device.motor.execution_status()
            fault = _firmware_fault(execution)
            if fault:
                self._fault = fault
                raise FaultError(f"Cannot start gripper: {fault}; call clear_fault() after resolving it")
            if execution.owner != 0:
                reason = {1: "another control session", 2: "calibration", 3: "firmware update"}
                raise BusyError(f"Cannot start gripper: {reason.get(execution.owner, 'unknown owner')} is active")
            config = _native.ForcePositionConfig()
            config.monitor_execution = True
            config.motor_stream_hz = self._update_hz
            config.close_compensation_rad = self._close_compensation_rad
            config.grasp_torque_nm = self._torque_limit_nm
            config.close_speed_radps = self._speed_rad_s
            config.contact_torque_nm = min(config.contact_torque_nm, self._torque_limit_nm / 2)
            controller = _native.ForcePositionController(device, config)
            try:
                controller.start()
            except BaseException:
                try:
                    controller.stop()
                except Exception as exc:
                    self._disable_pending = True
                    self._fault = f"startup cleanup failed; device state unknown: {exc}"
                    raise
                raise
            self._controller = controller
        state = self._state()
        if state.fault:
            raise FaultError(f"Gripper fault: {state.fault}; resolve the cause and call clear_fault()")
        return self._controller

    def _command(self, operation, method, *args, wait=False, timeout=5):
        timeout = _wait_options(wait, timeout)
        with self._lock:
            controller = self._start()
            getattr(controller, method)(*args)
            self._target = controller.snapshot().target_position
            self._operation = operation
            self._generation += 1
            generation = self._generation
        if wait:
            return self._wait(timeout, generation, result=True)
        return None

    def start(self) -> None:
        """Start feedback and hold the current position; call before a timed loop.

        This enables control. Connection and first-command setup are synchronous.
        Subsequent set_position() only updates the native target; it does not wait
        for serial acknowledgements or motor arrival.
        """
        with self._lock:
            self._start()

    def set_position(self, position: float) -> None:
        """Follow normalized opening (0 closed, 1 open) with connection defaults.

        Keeps bounded torque when an object prevents arrival. Each update replaces
        the target; no target queue or contact latch. The latest target continues
        until replaced/disabled, including when the application stops updating it.
        """
        self.move(position)

    @property
    def timing(self) -> GripperTiming | None:
        """Cached active-session rates/gaps. No serial query; None before start."""
        with self._lock:
            self._connected()
            if self._controller is None:
                return None
            s = self._controller.snapshot()
            elapsed = max(s.elapsed_s, 1e-9)
            return GripperTiming(self._update_hz, s.feedback_frames / elapsed,
                s.command_frames / elapsed, s.feedback_frames, s.command_frames,
                s.max_feedback_gap_ms, s.max_command_gap_ms, s.last_submit_latency_ms,
                s.max_submit_latency_ms, s.execution_telemetry)

    def move(self, position: float, *, torque_nm: float | None = None,
             torque_limit_nm: float | None = None, speed_rad_s: float | None = None,
             wait: bool = False, timeout: float = 5.0) -> GripperResult | None:
        """Position following with optional per-command limits and arrival wait.

        Obstruction keeps bounded torque toward the target. Waiting times out if
        it cannot arrive; control remains active. speed_rad_s limits reference
        slew, not measured speed. torque_nm is a compatibility alias.
        """
        if torque_limit_nm is not None and torque_nm is not None:
            raise ValueError("Specify torque_limit_nm or torque_nm, not both")
        cap = torque_nm if torque_limit_nm is None else torque_limit_nm
        cap = self._torque_limit_nm if cap is None else cap
        position = _in_range("position", position, 0, 1)
        cap = _positive("torque_limit_nm", cap, self.limits.max_torque_nm)
        speed = _positive("speed_rad_s", self._speed_rad_s if speed_rad_s is None else speed_rad_s,
                          self.limits.max_speed_rad_s)
        return self._command("move", "set_position_target", position, cap, speed,
                             wait=wait, timeout=timeout)

    def grasp(self, *, torque_nm: float = 0.35, speed_rad_s: float = 0.5,
              wait: bool = False, timeout: float = 5.0) -> GripperResult | None:
        """Close then hold motor torque; result is inferred contact or no_object."""
        torque = _positive("torque_nm", torque_nm, self.limits.max_torque_nm)
        speed = _positive("speed_rad_s", self._speed_rad_s if speed_rad_s is None else speed_rad_s,
                          self.limits.max_speed_rad_s)
        return self._command("grasp", "set_grasp_target", 0.0, torque, speed,
                             wait=wait, timeout=timeout)

    def release(self, position: float = 1.0, *, torque_limit_nm: float | None = None,
                speed_rad_s: float | None = None, wait: bool = False,
                timeout: float = 5.0) -> GripperResult | None:
        """Position-controlled release; choose an opening wider than the object."""
        position = _in_range("position", position, 0, 1)
        cap = _positive("torque_limit_nm", self._torque_limit_nm if torque_limit_nm is None else torque_limit_nm,
                        self.limits.max_torque_nm)
        speed = _positive("speed_rad_s", self._speed_rad_s if speed_rad_s is None else speed_rad_s,
                          self.limits.max_speed_rad_s)
        return self._command("release", "set_position_target", position, cap, speed,
                             wait=wait, timeout=timeout)

    def impedance(self, position: float, *, kp: float, kd: float,
                  feedforward_torque_nm: float = 0.0, velocity_rad_s: float = 0.0,
                  torque_limit_nm: float = 1.0) -> None:
        """Sustained compliance target; replace it to stream algorithm outputs.

        Position is normalized, kp in Nm/rad, kd in Nm*s/rad. Velocity and
        feedforward torque are signed in the raw motor frame (see info.reversed).
        A bounded impedance target is not a promise to reach position, so it
        has no wait result. kp=kd=0 provides bounded feedforward torque control.
        """
        position = _in_range("position", position, 0, 1)
        kp = _in_range("kp", kp, 0, self.limits.max_kp)
        kd = _in_range("kd", kd, 0, self.limits.max_kd)
        cap = _positive("torque_limit_nm", self._torque_limit_nm if torque_limit_nm is None else torque_limit_nm,
                        self.limits.max_torque_nm)
        torque = _in_range("feedforward_torque_nm", feedforward_torque_nm, -cap, cap)
        velocity = _in_range("velocity_rad_s", velocity_rad_s,
                             -self.limits.max_speed_rad_s, self.limits.max_speed_rad_s)
        self._command("impedance", "set_impedance_target", position, kp, kd, torque, velocity, cap)

    def hold(self, *, wait: bool = False, timeout: float = 5.0) -> GripperResult | None:
        """Cancel the current motion/compliance and hold the observed position."""
        return self._command("hold", "hold_position", wait=wait, timeout=timeout)

    def _state(self):
        device = self._connected()
        if self._controller is None:
            sample = device.motor.read_status()
            execution = device.motor.execution_status()
            fault = _firmware_fault(execution) or self._fault
            if sample.status & 0x0FFA:
                fault = fault or "motor reports a fault"
            if not all(math.isfinite(v) for v in (sample.actual_pos, sample.actual_vel,
                                                   sample.actual_torque, sample.motor_temp_c)):
                fault = fault or "non-finite motor feedback"
            return GripperState(device.rad_to_pos(sample.actual_pos), sample.actual_vel,
                                sample.actual_torque, sample.motor_temp_c, None,
                                "fault" if fault else "idle", 0.0, fault)
        snapshot = self._controller.snapshot()
        obs = snapshot.observation
        fault = snapshot.fault_reason or self._fault
        if snapshot.execution_telemetry:
            fault = fault or _firmware_fault(snapshot.execution)
        if not obs.valid or obs.age_ms >= 350:
            fault = fault or "motor feedback is stale"
        if obs.status & 0x0FFA:
            fault = fault or "motor reports a fault"
        if not all(math.isfinite(v) for v in (obs.position, obs.velocity, obs.torque, obs.motor_temp_c)):
            fault = fault or "non-finite motor feedback"
        if not snapshot.running:
            fault = fault or "controller stopped unexpectedly"
        execution_valid = snapshot.execution_telemetry and snapshot.execution.applied_seq > 0
        ready = (snapshot.command_sequence > 0
                 and snapshot.submitted_sequence == snapshot.command_sequence
                 and obs.seq > snapshot.submission_observation_sequence)
        return GripperState(obs.position, obs.velocity, obs.torque, obs.motor_temp_c,
                            self._target, snapshot.state.name.lower(), obs.age_ms, fault,
                            snapshot.control_mode.name.lower(), snapshot.blocked, obs.seq,
                            snapshot.command_sequence, ready and not fault,
                            self._position_tolerance, snapshot.active_torque_limit_nm,
                            snapshot.execution.torque_cap_nm if execution_valid else None,
                            bool(snapshot.execution.flags & 1) if execution_valid else None,
                            snapshot.target_error_rad if snapshot.control_mode == _native.ForcePositionMode.POSITION else None,
                            self._position_tolerance * self._info.motor_travel_rad)

    @property
    def state(self) -> GripperState:
        """Cached active feedback; idle reads motor and firmware fault status."""
        with self._lock:
            return self._state()

    def diagnostics(self) -> GripperDiagnostics:
        """Explicit firmware query; use outside high-rate observation loops."""
        with self._lock:
            s = self._connected().motor.execution_status()
            return GripperDiagnostics(
                {0: "idle", 1: "host", 2: "homing", 3: "ota", 4: "fault"}.get(s.owner, "unknown"),
                s.last_error, s.torque_cap_nm, bool(s.flags & 1), bool(s.flags & 2),
                s.target_seq, s.applied_seq)

    def wait_result(self, timeout: float = 5.0) -> GripperResult:
        """Wait for the current finite operation; timeout leaves control active."""
        return self._wait(_timeout(timeout), result=True)

    def wait(self, timeout: float = 5.0) -> GripperState:
        """Compatibility: return terminal feedback. Prefer wait_result()."""
        return self._wait(_timeout(timeout), result=False)

    def _outcome(self, state):
        if not state.ready:
            return None
        if self._operation == "grasp":
            if state.reached or state.contact:
                return "no_object" if state.position <= self._closed_tolerance else "contact"
        elif state.blocked:
            return "blocked"
        elif state.reached:
            return "reached"
        return None

    def _wait(self, timeout, generation=None, *, result):
        with self._lock:
            self._connected()
            if self._controller is None:
                raise GripperError("No active operation to wait for")
            if generation is not None and generation != self._generation:
                raise OperationInterrupted("Gripper operation was stopped or replaced")
            if self._operation == "impedance":
                raise GripperError("Impedance is a sustained setpoint; read state or issue move()/hold()")
            if generation is None:
                generation = self._generation
        deadline = time.monotonic() + timeout
        stable_since = None
        previous = None
        stable_seq = None
        while True:
            with self._lock:
                if generation != self._generation:
                    raise OperationInterrupted("Gripper operation was stopped or replaced")
                state = self._state()
                if state.fault:
                    raise FaultError(f"Gripper fault: {state.fault}")
                outcome = self._outcome(state)
                now = time.monotonic()
                if outcome is None or outcome != previous:
                    stable_since = now if outcome else None
                    stable_seq = state.seq
                previous = outcome
                if (outcome and stable_since is not None and now - stable_since >= self._settle_time
                        and (self._settle_time == 0 or state.seq != stable_seq)):
                    # UART submit has no ACK: check firmware ownership/errors
                    # before declaring a terminal result, not on every stream read.
                    execution = self._connected().motor.execution_status()
                    fault = _firmware_fault(execution)
                    if fault:
                        self._fault = fault
                        raise FaultError(f"Gripper fault: {fault}")
                    if execution.owner != 1:
                        raise OperationInterrupted("Firmware no longer owns an active host target")
                    if execution.applied_seq:
                        return GripperResult(self._operation, outcome, state) if result else state
            if time.monotonic() >= deadline:
                error = _native.TimeoutError(
                    f"Gripper {self._operation} timed out after {timeout:g}s; operation remains active. "
                    f"position={state.position:.4f}, target={state.target}, "
                    f"velocity={state.velocity_rad_s:.4f} rad/s, torque={state.torque_nm:.4f} Nm, "
                    f"mode={state.mode}, ready={state.ready}, age={state.age_ms:.1f} ms")
                error.state = state
                raise error
            time.sleep(0.01)

    def disable(self) -> None:
        """Stop control and request motor disable. A held object may be released."""
        with self._lock:
            device = self._connected()
            # Invalidate waiters even if the disable acknowledgement fails.
            self._generation += 1
            self._disable_pending = True
            self._target = None
            self._operation = None
            if self._controller is not None:
                try:
                    self._controller.stop()
                except Exception as exc:
                    self._fault = f"disable failed; device state unknown: {exc}"
                    raise
                finally:
                    self._controller = None
            else:
                try:
                    device.motor.disable()
                except Exception as exc:
                    self._fault = f"disable failed; device state unknown: {exc}"
                    raise
            self._disable_pending = False

    def stop(self) -> None:
        """Compatibility alias for disable(); hold() retains position instead."""
        self.disable()

    def clear_fault(self) -> None:
        """Disable, clear firmware faults, and remain idle until the next command."""
        with self._lock:
            self.disable()
            self._connected().motor.clear_fault()
            execution = self._connected().motor.execution_status()
            fault = _firmware_fault(execution)
            if fault:
                self._fault = fault
                raise FaultError(f"Clear fault did not recover the device: {fault}")
            self._fault = None

    @property
    def advanced(self):
        """Native maintenance/diagnostic API. Disable before reconfiguring."""
        with self._lock:
            return self._connected()

    def close(self) -> None:
        """Close, disabling only this object's active/uncertain control session.

        Passive connections do not stop another owner. Explicit disable() can.
        """
        with self._lock:
            if self._device is None:
                return
            try:
                if self._controller is not None or self._disable_pending:
                    self.disable()
            finally:
                self._device.transport.stop()
                self._device = None

    def __enter__(self):
        self._connected()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

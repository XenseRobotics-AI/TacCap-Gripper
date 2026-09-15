# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Immutable customer observations and results; values use motor units."""
from dataclasses import dataclass
from typing import Literal

ControlMode = Literal["idle", "position", "grasp", "impedance"]


class GripperError(RuntimeError):
    """Customer operation cannot proceed; native I/O exceptions remain separate."""


class FaultError(GripperError):
    """Resolve the cause, clear_fault(), then explicitly request a new operation."""


class BusyError(GripperError):
    """Another session, homing or firmware update owns the motor."""


class OperationInterrupted(GripperError):
    """The operation being waited for was replaced, disabled or closed."""


@dataclass(frozen=True)
class GripperDevice:
    device: str
    firmware_sn: str
    side: str


@dataclass(frozen=True)
class GripperInfo:
    device: str
    firmware_version: tuple[int, int, int]
    motor_travel_rad: float
    reversed: bool
    control_modes: tuple[str, ...] = ("position", "grasp", "impedance")


@dataclass(frozen=True)
class GripperLimits:
    """SDK request limits, not the device's dynamic thermal/torque allowance."""
    position_min: float = 0.0
    position_max: float = 1.0
    max_torque_nm: float = 1.2
    max_speed_rad_s: float = 50.0
    max_kp: float = 500.0
    max_kd: float = 5.0


@dataclass(frozen=True)
class GripperDiagnostics:
    """Explicit firmware query, separate from cached control observations."""
    owner: str
    error_code: int
    effective_torque_limit_nm: float
    target_limited: bool
    output_failed: bool
    target_sequence: int
    applied_sequence: int


@dataclass(frozen=True)
class GripperState:
    """Latest observation. Contact is inferred, not a verified grasp."""
    position: float                 # normalized opening: 0 closed, 1 open
    velocity_rad_s: float            # signed raw motor frame
    torque_nm: float                 # motor estimate, not fingertip force
    temperature_c: float
    target: float | None
    mode: str                       # execution phase, retained for compatibility
    age_ms: float
    fault: str | None
    control_mode: ControlMode = "idle"
    blocked: bool = False
    seq: int = 0
    command_id: int = 0
    ready: bool = False              # command sent, subsequent feedback received
    position_tolerance: float = 0.01
    torque_limit_nm: float | None = None  # requested SDK budget
    effective_torque_limit_nm: float | None = None  # firmware envelope allowance
    target_limited: bool | None = None   # MCU changed the submitted target
    target_error_rad: float | None = None  # includes closing compensation
    motor_tolerance_rad: float = 0.01

    @property
    def position_error(self) -> float | None:
        return None if self.target is None else self.target - self.position

    @property
    def contact(self) -> bool:
        return self.mode == "holding_force" and self.fault is None and self.age_ms < 350

    @property
    def reached(self) -> bool:
        # A compensated target intentionally differs from the calibrated 0..1
        # target. Use its motor error when available, not both coordinate frames.
        position_reached = (abs(self.target_error_rad) <= self.motor_tolerance_rad
                            if self.target_error_rad is not None else
                            self.target is not None and
                            abs(self.position - self.target) <= self.position_tolerance)
        return (self.fault is None and self.age_ms < 350 and self.target is not None
                and not self.blocked and self.control_mode != "impedance"
                and position_reached and abs(self.velocity_rad_s) <= 0.05)

    @property
    def moving(self) -> bool:
        return self.fault is None and self.age_ms < 350 and abs(self.velocity_rad_s) > 0.05


@dataclass(frozen=True)
class GripperResult:
    """Terminal operation observation. Holding continues after this result."""
    operation: Literal["move", "grasp", "release", "hold"]
    status: Literal["reached", "contact", "no_object", "blocked"]
    state: GripperState


@dataclass(frozen=True)
class GripperTiming:
    """Host session statistics, not physical response-time measurements.

    Submission latency ends at the UART write, not MCU execution/motor arrival.
    Only the latest target of an update burst is sent and included in latency.
    """
    configured_hz: int
    feedback_hz: float
    command_hz: float
    feedback_frames: int
    command_frames: int
    max_feedback_gap_ms: float
    max_command_gap_ms: float
    last_submit_latency_ms: float
    max_submit_latency_ms: float
    execution_telemetry: bool

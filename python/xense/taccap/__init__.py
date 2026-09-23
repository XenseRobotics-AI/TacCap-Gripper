"""xense.taccap — TacCap-Gripper Python SDK.

Imported via ``import xense.taccap``. ``xense`` is a PEP 420 namespace
package (no ``__init__.py``), so this subpackage can coexist with any other
``xense.*`` packages installed alongside it.

Top-level surface today:

  - Versioning:       ``__version__``, ``hello()``
  - Protocol enums:   ``Address``, ``FrameType``, ``Cmd``, ``ErrorCode``
  - Wire framing:     ``Frame``, ``FrameParser``, ``pack_frame``,
                       ``crc16_modbus``, ``stuff_data``, ``unstuff_data``
  - Serial transport: ``SerialBus``
  - Exceptions:       ``ProtocolError``, ``CrcError``, ``IoError``,
                       ``TimeoutError``
"""

from . import _taccap_native

# ---- Versioning -------------------------------------------------------------
# Derived, never written here. The one hand-written copy is [project].version in
# pyproject.toml; CMake reads it into PROJECT_VERSION, version.hpp.in bakes it
# into the extension, and we take it from there.
#
# This was a hand-maintained _version.py until 0.2.1, and it had drifted: the
# package said 0.2.0 while the extension, hello() and the wheel metadata all
# said 0.2.1. A version the public API reports wrongly is worse than no version,
# because it reads as authoritative.
__version__ = _taccap_native.__version__
hello = _taccap_native.hello

# ---- Logging (spdlog-backed, shared with C++ core) --------------------------
log = _taccap_native.log

# ---- Enums ------------------------------------------------------------------
Address = _taccap_native.Address
FrameType = _taccap_native.FrameType
Cmd = _taccap_native.Cmd
ErrorCode = _taccap_native.ErrorCode

# ---- Frame constants --------------------------------------------------------
FRAME_HEAD = _taccap_native.FRAME_HEAD
FRAME_TAIL = _taccap_native.FRAME_TAIL
FRAME_ESCAPE = _taccap_native.FRAME_ESCAPE
MIN_FRAME_LEN = _taccap_native.MIN_FRAME_LEN
MAX_FRAME_LEN = _taccap_native.MAX_FRAME_LEN
MAX_PAYLOAD_LEN = _taccap_native.MAX_PAYLOAD_LEN

# ---- Wire framing -----------------------------------------------------------
Frame = _taccap_native.Frame
FrameParser = _taccap_native.FrameParser
pack_frame = _taccap_native.pack_frame
crc16_modbus = _taccap_native.crc16_modbus
stuff_data = _taccap_native.stuff_data
unstuff_data = _taccap_native.unstuff_data

# ---- Serial transport -------------------------------------------------------
SerialBus = _taccap_native.SerialBus

# ---- Async transport (background reader + ACK matching + DATA dispatch) -----
Transport = _taccap_native.Transport
AckResponse = _taccap_native.AckResponse
TransportStats = _taccap_native.TransportStats

# ---- Components (typed wrappers around Transport) ---------------------------
ImuSample = _taccap_native.ImuSample
EncoderSample = _taccap_native.EncoderSample
MotorStatusSample = _taccap_native.MotorStatusSample
MotorProtocol = _taccap_native.MotorProtocol          # V1.7
GripperConfig = _taccap_native.GripperConfig          # V1.7
GripperAutoCalConfig = _taccap_native.GripperAutoCalConfig  # V1.9 power-on auto-cal
GripperEnvelope = _taccap_native.GripperEnvelope       # motion safety envelope (in GripperConfig)
GRIPPER_ENVELOPE_VALID = _taccap_native.GRIPPER_ENVELOPE_VALID
GRIPPER_ENVELOPE_ENFORCE = _taccap_native.GRIPPER_ENVELOPE_ENFORCE
GRIPPER_ENVELOPE_LAYOUT_VERSION = _taccap_native.GRIPPER_ENVELOPE_LAYOUT_VERSION
GripperPosition = _taccap_native.GripperPosition      # raw rad <-> normalized [0,1]
CameraFisheyeCal = _taccap_native.CameraFisheyeCal    # V2.0 fisheye intrinsics + distortion
FisheyeUndistorter = _taccap_native.FisheyeUndistorter  # applies them to frames
# Reference intrinsics for the TC-GU-01 wrist lens, for units whose firmware was
# never calibrated. Approximate by construction — see the C++ header — so every
# path that uses it should say so.
ColorMode = _taccap_native.ColorMode
FISHEYE_FALLBACK_CAL = _taccap_native.FISHEYE_FALLBACK_CAL
is_usable_fisheye_cal = _taccap_native.is_usable_fisheye_cal
FirmwareVersion = _taccap_native.FirmwareVersion
Calibration = _taccap_native.Calibration              # V2.0/V2.1 flash-persisted cal records
GripperObservation = _taccap_native.GripperObservation  # controller latest obs
ImpedanceState = _taccap_native.ImpedanceState        # supervised impedance FSM
ImpedanceConfig = _taccap_native.ImpedanceConfig
ImpedanceSnapshot = _taccap_native.ImpedanceSnapshot
ImpedanceController = _taccap_native.ImpedanceController
MOTOR_RATED_TORQUE_NM = _taccap_native.MOTOR_RATED_TORQUE_NM   # 1.8 Nm, indefinite
MOTOR_PEAK_TORQUE_NM = _taccap_native.MOTOR_PEAK_TORQUE_NM     # 6.0 Nm, transient
ForcePositionState = _taccap_native.ForcePositionState
ForcePositionConfig = _taccap_native.ForcePositionConfig
ForcePositionSnapshot = _taccap_native.ForcePositionSnapshot
ForcePositionController = _taccap_native.ForcePositionController
FORCE_POSITION_MAX_HOLD_TORQUE_NM = _taccap_native.FORCE_POSITION_MAX_HOLD_TORQUE_NM
FORCE_POSITION_MAX_MOTION_TORQUE_NM = _taccap_native.FORCE_POSITION_MAX_MOTION_TORQUE_NM
Diagnostics = _taccap_native.Diagnostics              # firmware UART counters + log control
UartStats   = _taccap_native.UartStats                # Cmd 0x54 payload
LogConfig   = _taccap_native.LogConfig                # Cmd 0x55 payload
LogLevel    = _taccap_native.LogLevel                 # firmware log verbosity
LOG_OUTPUT_NONE = _taccap_native.LOG_OUTPUT_NONE      # firmware default: logging off
LOG_OUTPUT_UART = _taccap_native.LOG_OUTPUT_UART      # MCU DEBUG UART (not on USB)
MotorControlStats = _taccap_native.MotorControlStats  # V1.7
MotorPrivateParam = _taccap_native.MotorPrivateParam  # V1.9+ private-protocol param
MotorStatusExt = _taccap_native.MotorStatusExt        # V2.2 72-byte status (Cmd 0x53)
MotorFaultReport = _taccap_native.MotorFaultReport    # V2.2 64-byte fault report (Cmd 0x52)
MotorVersion = _taccap_native.MotorVersion            # 电机自身固件版本 (Cmd 0x58, 需固件 >= 1.2.6)
MotorStopReason = _taccap_native.MotorStopReason      # V2.2
# V2.2 partial auto-cal writes — patch stall params without a read-modify-write
GripperAutoCalStallParam = _taccap_native.GripperAutoCalStallParam
GripperAutoCalStallParamEx = _taccap_native.GripperAutoCalStallParamEx
KeySample = _taccap_native.KeySample
SensorErrorSample = _taccap_native.SensorErrorSample
CameraFrame = _taccap_native.CameraFrame
IMU = _taccap_native.IMU
Encoder = _taccap_native.Encoder
Motor = _taccap_native.Motor
Key = _taccap_native.Key
Led = _taccap_native.Led                              # V1.9 WS2812 LED control
Ws2812Mode = _taccap_native.Ws2812Mode                # V1.9
Ws2812EffectType = _taccap_native.Ws2812EffectType    # V1.9
SensorErrors = _taccap_native.SensorErrors
Camera = _taccap_native.Camera
OtaSession = _taccap_native.OtaSession
OtaTargetVersion = _taccap_native.OtaTargetVersion
OtaStatus = _taccap_native.OtaStatus
KeyState = _taccap_native.KeyState
crc32_iso_hdlc = _taccap_native.crc32_iso_hdlc

# ---- Aggregate gripper + discovery ------------------------------------------
LeaderGripper = _taccap_native.LeaderGripper
FollowerGripper = _taccap_native.FollowerGripper
GripperEndpoints = _taccap_native.GripperEndpoints
Side = _taccap_native.Side
Role = _taccap_native.Role
ParsedSerial = _taccap_native.ParsedSerial
parse_serial = _taccap_native.parse_serial
scan_grippers = _taccap_native.scan_grippers
find_one = _taccap_native.find_one
find_left = _taccap_native.find_left
find_right = _taccap_native.find_right
find_leader = _taccap_native.find_leader
find_follower = _taccap_native.find_follower
find_leader = _taccap_native.find_leader
find_follower = _taccap_native.find_follower

# ---- Exceptions -------------------------------------------------------------
ProtocolError = _taccap_native.ProtocolError
CrcError = _taccap_native.CrcError
IoError = _taccap_native.IoError
TimeoutError = _taccap_native.TimeoutError

__all__ = [
    "__version__",
    "hello",
    # Logging
    "log",
    # Enums
    "Address",
    "FrameType",
    "Cmd",
    "ErrorCode",
    # Frame constants
    "FRAME_HEAD",
    "FRAME_TAIL",
    "FRAME_ESCAPE",
    "MIN_FRAME_LEN",
    "MAX_FRAME_LEN",
    "MAX_PAYLOAD_LEN",
    # Wire framing
    "Frame",
    "FrameParser",
    "pack_frame",
    "crc16_modbus",
    "stuff_data",
    "unstuff_data",
    # Serial transport
    "SerialBus",
    # Async transport
    "Transport",
    "AckResponse",
    "TransportStats",
    # Components
    "ImuSample",
    "EncoderSample",
    "MotorStatusSample",
    "MotorProtocol",
    "GripperConfig",
    "GripperAutoCalConfig",
    "GripperEnvelope",
    "GRIPPER_ENVELOPE_VALID",
    "GRIPPER_ENVELOPE_ENFORCE",
    "GRIPPER_ENVELOPE_LAYOUT_VERSION",
    "GripperPosition",
    "CameraFisheyeCal",
    "FisheyeUndistorter",
    "ColorMode",
    "FISHEYE_FALLBACK_CAL",
    "is_usable_fisheye_cal",
    "FirmwareVersion",
    "Calibration",
    "GripperObservation",
    "ImpedanceState",
    "ImpedanceConfig",
    "ImpedanceSnapshot",
    "ImpedanceController",
    "MOTOR_RATED_TORQUE_NM",
    "MOTOR_PEAK_TORQUE_NM",
    "ForcePositionState",
    "ForcePositionConfig",
    "ForcePositionSnapshot",
    "ForcePositionController",
    "FORCE_POSITION_MAX_HOLD_TORQUE_NM",
    "FORCE_POSITION_MAX_MOTION_TORQUE_NM",
    "Diagnostics",
    "UartStats",
    "LogConfig",
    "LogLevel",
    "LOG_OUTPUT_NONE",
    "LOG_OUTPUT_UART",
    "MotorControlStats",
    "MotorPrivateParam",
    "MotorStatusExt",
    "MotorFaultReport",
    "MotorVersion",
    "MotorStopReason",
    "GripperAutoCalStallParam",
    "GripperAutoCalStallParamEx",
    "CameraFrame",
    "IMU",
    "Encoder",
    "Motor",
    "Led",
    "Ws2812Mode",
    "Ws2812EffectType",
    "Camera",
    # Aggregate + discovery
    "LeaderGripper",
    "FollowerGripper",
    "GripperEndpoints",
    "Side",
    "Role",
    "ParsedSerial",
    "parse_serial",
    "scan_grippers",
    "find_one",
    "find_left",
    "find_right",
    "find_leader",
    "find_follower",
    "find_leader",
    "find_follower",
    # Exceptions
    "ProtocolError",
    "CrcError",
    "IoError",
    "TimeoutError",
]

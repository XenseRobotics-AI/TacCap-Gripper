// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 protocol payload structures. Layouts mirror firmware
//   Embedded Software/tc-gu-01/App/protocol/protocol_data.h
// 1:1 — when the firmware changes, this file must follow.
//
// All structs are #pragma pack(1) to match the wire format exactly. Size
// invariants are enforced by static_assert; if a layout drift is introduced
// the build fails immediately.

#pragma once

#include <cstddef>
#include <cstdint>

namespace xense::taccap::protocol {

#pragma pack(push, 1)

// ---- System --------------------------------------------------------------

struct FirmwareVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t build;
};

// 16 character SN + NUL terminator on the wire.
struct SnInfo {
    char sn[17];
};

enum class DeviceType : uint8_t {
    Left    = 0,
    Right   = 1,
    Unknown = 0xFF,
};

// ---- IMU -----------------------------------------------------------------

namespace ImuValid {
    constexpr uint16_t Accel = 0x01;
    constexpr uint16_t Gyro  = 0x02;
    constexpr uint16_t Mag   = 0x04;
    constexpr uint16_t Temp  = 0x08;
}

struct ImuData {
    uint32_t timestamp_us;
    uint16_t valid_flag;          // ImuValid::* bits

    int16_t  accel_x, accel_y, accel_z;   // mg (0.001 g)
    int16_t  gyro_x,  gyro_y,  gyro_z;    // 0.01 dps
    int16_t  mag_x,   mag_y,   mag_z;     // 0.01 µT
    int16_t  temperature;                 // 0.01 °C
    uint16_t seq;
};

struct ImuConfig {
    uint16_t sample_rate;     // sensor internal sample rate (Hz); 0 = leave unchanged
    uint16_t odr;             // host output data rate (Hz, ≤ sample_rate); 0 = leave unchanged
    uint8_t  accel_range;     // 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t  gyro_range;      // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
    uint8_t  mag_range;       // 0=4800µT
    uint8_t  filter_enable;
    uint16_t filter_cutoff;
};

// ---- Encoder -------------------------------------------------------------

namespace EncoderStatusBit {
    constexpr uint16_t Ok       = 0x0000;
    constexpr uint16_t Error    = 0x0001;
    constexpr uint16_t Overflow = 0x0002;
}

struct EncoderData {
    uint32_t timestamp_us;
    float    position_rad;
    float    velocity_rad_s;
    uint16_t status;
    uint16_t seq;
};

// Firmware `0086da6` (2026-05-27) dropped `baudrate` / `resolution` / `ratio`
// along with the RS485 Modbus encoder driver, shrinking this 14 -> 5 bytes.
// The SDK kept mirroring the old layout until 2026-08-14, which made BOTH
// Cmd::SetEncoderConfig (firmware NACKs LengthMismatch on a 14-byte request)
// and Cmd::GetEncoderConfig (SDK threw on the 5-byte response) unusable. The
// encoder is SPI MT6816 now — there is no baud rate to configure.
struct EncoderConfig {
    uint8_t  direction;       // 0 = forward, 1 = reversed
    float    offset_rad;      // zero-point offset (rad)
};

// ---- Electronic skin -----------------------------------------------------

constexpr uint8_t ESKIN_MAX_ROWS = 12;
constexpr uint8_t ESKIN_MAX_COLS = 8;

enum class EskinOutputType : uint8_t {
    Adc     = 0,  // uint16_t per cell
    Voltage = 1,  // float per cell (V)
    Force   = 2,  // float per cell (N)
};

struct EskinHeader {
    uint32_t timestamp_us;
    uint8_t  rows;
    uint8_t  cols;
    uint8_t  type;        // EskinOutputType
    uint8_t  reserved;    // alignment padding
    uint16_t seq;
    uint16_t _reserved2;  // wire padding — firmware ESKIN_HEADER_SIZE=12
                          // memcpys 12 bytes, so the wire reserves 2 bytes
                          // here (contents undefined; do not rely on them).
    // Followed by rows*cols*(2 or 4) bytes of cell data.
};

struct EskinConfig {
    uint8_t  rows;
    uint8_t  cols;
    uint8_t  output_type;
    uint16_t sample_rate;
};

// ---- Combined sensor packet (combined-stream mode) -----------------------

namespace SensorValid {
    constexpr uint16_t Imu     = 0x0001;
    constexpr uint16_t Encoder = 0x0002;
    constexpr uint16_t Eskin1  = 0x0004;
    constexpr uint16_t Eskin2  = 0x0008;
}

struct CombinedSensorHeader {
    uint32_t timestamp_us;
    uint16_t valid_mask;
    uint16_t seq;
    // Followed by per-source [type(1) + len(2) + data] sub-packets.
};

// ---- Motor ---------------------------------------------------------------

enum class MotorMode : uint8_t {
    Idle      = 0,
    Position  = 1,
    Velocity  = 2,
    Torque    = 3,
    Impedance = 4,
};

// Motor CAN protocol (V1.7 — Cmd::MotorGetProtocol / MotorSwitchProtocol).
enum class MotorProtocol : uint8_t {
    Private = 0,   // Xense private CAN protocol
    Mit     = 2,   // MIT-style CAN protocol
};

namespace MotorStatusBit {
    constexpr uint16_t Enabled       = 0x0001;
    constexpr uint16_t Fault         = 0x0002;
    constexpr uint16_t Stalled       = 0x0004;
    constexpr uint16_t OverTemp      = 0x0008;
    constexpr uint16_t OverCurrent   = 0x0010;
    constexpr uint16_t OverVolt      = 0x0020;
    constexpr uint16_t UnderVolt     = 0x0040;
    constexpr uint16_t EncoderError  = 0x0080;
    // V2.2 — the firmware now decodes the motor's own fault word into these.
    // They ride in the same 16-bit `status` field the 31-byte layout already
    // carries, so they surface on Cmd::GetMotorStatus and the DATA stream too.
    constexpr uint16_t DriverFault         = 0x0100;
    constexpr uint16_t PositionInitError   = 0x0200;
    constexpr uint16_t HardwareIdError     = 0x0400;
    constexpr uint16_t EncoderUncalibrated = 0x0800;
}

struct MotorPosCtrl {
    float target_pos;     // rad
    float max_vel;        // rad/s
    float max_torque;     // Nm
};

struct MotorVelCtrl {
    float target_vel;     // rad/s
    float max_torque;     // Nm
    float profile_acc;    // rad/s²
};

struct MotorTorqueCtrl {
    float target_torque;  // Nm
    float max_vel;        // rad/s
    float reserved;
};

struct MotorImpedanceCtrl {
    float target_pos;     // rad
    float kp;             // Nm/rad
    float kd;             // Nm·s/rad
    float target_torque;  // Nm (feed-forward)
    float vel;            // V1.7 — feed-forward velocity (rad/s); 0 = firmware
                          // estimates from the position delta. MIT protocol only.
};

// motor_status_t — V1.9 shrank this 40 -> 31 bytes by dropping the current
// fields (actual_current / target_current / current_source). The first 18 bytes
// (actual_pos..status) are byte-identical to the older layout, so position /
// torque / status readings stay correct across firmware versions; only the
// target_* / control_mode tail moved. The SDK targets the V1.9 (31-byte) layout.
struct MotorStatus {
    float    actual_pos;      // rad
    float    actual_vel;      // rad/s
    float    actual_torque;   // Nm
    float    motor_temp;      // °C
    uint16_t status;          // MotorStatusBit::*
    float    target_pos;      // rad   — last applied target
    float    target_vel;      // rad/s
    float    target_torque;   // Nm    — target / feed-forward / clamp
    uint8_t  control_mode;    // MotorMode of the last applied command
};

// ---- Motor spec (Cmd 0x56) ------------------------------------------------
//
// THE DEVICE IS THE AUTHORITY ON ITS OWN MOTOR. MOTOR_RATED_TORQUE_NM and
// MOTOR_PEAK_TORQUE_NM below are EL05 numbers compiled into this SDK, and they
// are wrong the moment the gripper ships with a different actuator: an RS00 is
// rated 5.0 Nm and peaks at 14.0, against EL05's 1.8 / 6.0. Two copies of one
// table, one of them in a repository that does not know which motor is plugged
// in -- so read it from the firmware, whose motor_spec.c is the single source.
//
// A zero field means "the datasheet does not give it" or "not measured on this
// model yet" (most of the thermal fields on RS00..RS06 are still zero). Treat 0
// as UNKNOWN, never as the number zero.
struct __attribute__((packed)) MotorSpec {
    char     name[8];               // model, NUL-padded
    float    p_max_rad;             // position range, symmetric +/-
    float    v_max_rad_s;           // velocity range, symmetric +/-
    float    t_max_nm;              // torque range == peak load
    float    kp_max;
    float    kd_max;
    float    rated_torque_nm;
    float    stall_cont_torque_nm;  // indefinite stall rating; 0 = unknown
    uint8_t  winding_limit_c;       // 0 = unknown
    uint8_t  board_limit_c;         // 0 = unknown
    uint8_t  reserved[2];
};
constexpr std::size_t MOTOR_SPEC_SIZE = 40;

// ---- Auto-calibration diagnostics (Cmd 0x57) ------------------------------
//
// Homing has five failure paths and, before this command existed, NONE of them
// were visible to a host: all five only reach LOG_E on UART7, which is not
// wired to USB on this board. A host could see the aftermath -- stop_reason ==
// StopReason::Emergency -- and nothing about which step failed or why.
//
// That cost real time. Chasing a "gripper closes but never opens" fault we
// could only infer the step by comparing stop_timestamp_ms against the 30 s
// timeout constant. With this report the answer was one sample: state == Open
// while the motor self-reported Reset mode and torque sat at the noise floor.
//
// Read it any time, including while control runs -- it is a read-only snapshot
// and deliberately does not go through the motor-admin gate, because a running
// control loop is exactly when you most want to look.
enum class HomeState : std::uint8_t {
    Idle = 0, Init, ClearFault, Close, CloseBackoff, CloseAverageHold,
    SetZero, PostZero, Open, OpenBackoff, SaveMax, OpenFinalBackoff,
    Done, Error,
};

enum class HomeFail : std::uint8_t {
    None = 0,
    Timeout,        // whole-sequence budget (30 s) ran out
    ClearFault,     // can_motor_clear_fault failed
    SetZero,        // can_motor_set_zero_hold failed
    MaxOpenTooSmall,
    SaveConfig,
};

namespace HomeDiagFlag {
    constexpr std::uint8_t Active    = 0x01;
    constexpr std::uint8_t Done      = 0x02;
    constexpr std::uint8_t Attempted = 0x04;
    constexpr std::uint8_t HasMoved  = 0x08;  // stall monitor saw real motion
}

struct __attribute__((packed)) HomeDiagReport {
    std::uint8_t  version;            // == HOME_DIAG_REPORT_VERSION
    std::uint8_t  state;              // HomeState
    std::uint8_t  fail_reason;        // HomeFail
    std::uint8_t  flags;              // HomeDiagFlag::*
    std::int32_t  fail_ret;           // ret that accompanied the failure, else 0
    std::uint32_t elapsed_ms;         // since homing started
    std::uint32_t state_elapsed_ms;   // in the current phase -- a hang is obvious
    float         vel_cmd;            // last commanded velocity, signed
    float         torque_limit_nm;    // last commanded current limit
    float         stall_threshold_nm; // limit * saturation ratio * margin
    float         last_abs_torque;    // |torque| at the last stall evaluation
    float         last_abs_vel;
    float         peak_vel;           // max |vel| seen in this phase
    float         progress_rad;       // distance covered in this phase
    float         last_pos;
    float         close_pos;          // measured closed limit
    float         open_pos;           // measured open limit
    float         open_sign;          // +1/-1, derived from GripperConfigFlag::Reverse
    std::uint16_t reenable_count;     // times the motor was found in Reset while
                                      // the MCU believed it enabled (see
                                      // can_motor_reconcile_enable)
    std::uint16_t report_miss_count;  // times active reporting was requested but
                                      // never delivered, so the firmware fell
                                      // back to polling. Non-zero means this
                                      // motor does not honour MIT instruction 13
                                      // (needs motor firmware >= 1.0.5.0.4).
};
constexpr std::size_t HOME_DIAG_REPORT_SIZE = 64;
constexpr std::uint8_t HOME_DIAG_REPORT_VERSION = 0x01;

// ---- Extended motor status + fault report (V2.2 — Cmd 0x53 / 0x52) --------
//
// V2.2 grew the firmware's internal motor_status_t to 72 bytes, but deliberately
// did NOT change what Cmd::GetMotorStatus (0x50) or the MotorStatus DATA stream
// emit — both still send the 31-byte prefix. The extra fields are reachable only
// via Cmd::GetMotorStatusExt (0x53). Consequently a 31-byte status frame says
// nothing about firmware age; probe with Cmd::GetVersion instead.
//
// Wire sizes mirror firmware MOTOR_STATUS_*_SIZE. The 59-byte figure is the
// monitor extension without the raw-CAN tail; the firmware never emits it as a
// standalone frame, but the PC tool decodes progressively (31 -> 59 -> 72) and
// the constant is kept here so host code can do the same.
constexpr std::size_t MOTOR_STATUS_LEGACY_SIZE = 31;
constexpr std::size_t MOTOR_STATUS_V2_SIZE     = 59;
constexpr std::size_t MOTOR_STATUS_EXT_SIZE    = 72;

// MotorStatusExt::monitor_version — firmware MOTOR_MONITOR_VERSION.
constexpr uint8_t MOTOR_MONITOR_VERSION = 0x02;

// MotorStatusExt::monitor_flags — health of the firmware's own status/fault
// collection, NOT motor faults. Read these before trusting fault_code:
// StatusValid/FaultValid say whether the corresponding fields were ever filled.
namespace MotorMonitorFlag {
    constexpr uint8_t StatusValid       = 0x01;  // motion status has been received
    constexpr uint8_t FaultValid        = 0x02;  // a full fault reply has been received
    constexpr uint8_t CommStale         = 0x04;  // status feedback has gone stale
    constexpr uint8_t FaultPollPending  = 0x08;  // a fault read is in flight
    constexpr uint8_t StopSnapshotValid = 0x10;  // stop_fault_code/stop_timestamp are set
    constexpr uint8_t FaultLatched      = 0x20;  // latched_fault_code is non-zero
    constexpr uint8_t Warning           = 0x40;  // motor reports a warning
    constexpr uint8_t FaultStale        = 0x80;  // detail fault data stale / never read
}

// MotorStatusExt::monitor_reserved — provenance of the fault data, so a host can
// distinguish "no fault" from "never managed to read one".
namespace MotorMonitorDiag {
    constexpr uint8_t RxSeen       = 0x01;  // a valid cmd-5 reply has been seen
    constexpr uint8_t TimeoutSeen  = 0x02;  // a fault read has timed out at least once
    constexpr uint8_t CanFrameSeen = 0x04;  // a frame resembling a cmd-5 reply was seen
}

// MotorStatusExt::stop_reason / MotorFaultReport::stop_reason.
enum class MotorStopReason : uint8_t {
    None         = 0x00,
    Disable      = 0x01,  // motor was disabled
    Emergency    = 0x02,  // emergency stop
    ClearFault   = 0x03,  // stopped by a clear-fault
    LimitStall   = 0x04,  // limit / stall protection tripped
    ControlError = 0x05,  // firmware control-loop error
    // The slave control task found its cached target stale -- the host stopped
    // sending. Mirrors MOTOR_STOP_REASON_HOST_TIMEOUT.
    //
    // Note this is NOT the actuator's own 0x7028 CAN timeout, which cannot fire
    // on host loss: the MCU keeps re-sending the cached target at 500 Hz, so
    // the motor goes on receiving CAN commands. 0x7028 covers MCU/RTOS death;
    // this covers the host going away.
    HostTimeout  = 0x06,
};

// Bit positions inside the raw 32-bit motor fault word (MotorStatusExt::
// fault_code / latched_fault_code / stop_fault_code, and the MotorFaultReport
// motor_* fields). These come from the RobStride motor itself, not the MCU.
namespace MotorFaultBit {
    constexpr uint32_t OverTemp            = 1u << 0;   // > 135 °C
    constexpr uint32_t DriverChip          = 1u << 1;
    constexpr uint32_t UnderVolt           = 1u << 2;   // < 12 V
    constexpr uint32_t OverVolt            = 1u << 3;   // > 60 V
    constexpr uint32_t PhaseBOverCurrent   = 1u << 4;
    constexpr uint32_t PhaseCOverCurrent   = 1u << 5;
    constexpr uint32_t EncoderUncalibrated = 1u << 7;
    constexpr uint32_t HardwareId          = 1u << 8;
    constexpr uint32_t PositionInit        = 1u << 9;
    constexpr uint32_t StallOverload       = 1u << 14;
    constexpr uint32_t PhaseAOverCurrent   = 1u << 16;
}

// Cmd::GetMotorStatusExt (0x53) response — firmware motor_status_t. Bytes 0..30
// are byte-identical to MotorStatus, so the two can share a decode path.
struct MotorStatusExt {
    // ---- bytes 0..30 — identical to MotorStatus ----
    float    actual_pos;           // rad
    float    actual_vel;           // rad/s
    float    actual_torque;        // Nm
    float    motor_temp;           // °C
    uint16_t status;               // MotorStatusBit::*
    float    target_pos;           // rad
    float    target_vel;           // rad/s
    float    target_torque;        // Nm
    uint8_t  control_mode;         // MotorMode
    // ---- bytes 31..58 — monitor extension ----
    uint8_t  monitor_version;      // == MOTOR_MONITOR_VERSION
    uint8_t  monitor_flags;        // MotorMonitorFlag::*
    uint8_t  stop_reason;          // MotorStopReason
    uint8_t  monitor_reserved;     // MotorMonitorDiag::*
    uint32_t fault_code;           // current motor fault word (MotorFaultBit::*)
    uint32_t latched_fault_code;   // OR of every fault seen since power-on
    uint32_t fault_timestamp_ms;   // MCU tick of the last full fault reply
    uint32_t status_timestamp_ms;  // MCU tick of the last motion-status reply
    uint32_t stop_fault_code;      // fault word latched when the gripper stopped
    uint32_t stop_timestamp_ms;    // MCU tick of the stop snapshot
    // ---- bytes 59..71 — raw cmd-5 CAN reply ----
    uint32_t fault_can_id;         // standard CAN id of the reply, normally 0xFD
    uint8_t  fault_can_dlc;        // raw DLC; a valid reply is ≥ 5
    uint8_t  fault_can_data[8];    // byte0 = motor CAN id, bytes 1..4 = LE fault
};

// MotorFaultReport::version — firmware MOTOR_FAULT_REPORT_VERSION.
constexpr uint8_t MOTOR_FAULT_REPORT_VERSION = 0x01;

// MotorFaultReport::report_flags.
namespace MotorFaultReportFlag {
    constexpr uint8_t Valid          = 0x01;
    constexpr uint8_t ForceRead      = 0x02;  // this report came from a forced read
    constexpr uint8_t MotorValid     = 0x04;  // motor_*_fault_code fields are meaningful
    constexpr uint8_t FwValid        = 0x08;  // firmware_*_fault_code fields are meaningful
    constexpr uint8_t CanEvidence    = 0x10;  // raw CAN evidence is attached
    constexpr uint8_t ReadTimeout    = 0x20;  // the fault read timed out
    constexpr uint8_t PrivatePartial = 0x40;  // partial data under the private protocol
}

// MotorFaultReport::source_mask — which layer raised the fault.
namespace MotorFaultSource {
    constexpr uint8_t Motor      = 0x01;  // the motor's own fault word
    constexpr uint8_t FwCan      = 0x02;  // MCU CAN layer
    constexpr uint8_t FwControl  = 0x04;  // MCU control loop
    constexpr uint8_t FwSystem   = 0x08;  // MCU system level
}

// MotorFaultReport::firmware_fault_code / firmware_latched_fault_code. Unlike
// the motor fault word these are enumerated values, not bit masks — compare for
// equality. Mirrors firmware FW_FAULT_*.
enum class FirmwareFaultCode : uint32_t {
    None                = 0x00000000,
    CanStatusStale      = 0x00000201,
    CanFaultReadTimeout = 0x00000202,
    CanBusOff           = 0x00000203,
    CanErrorPassive     = 0x00000204,
    CanTxTimeout        = 0x00000205,
    CanInvalidFaultFrame = 0x00000207,
    ControlOutput       = 0x00000302,
    ControlDeadline     = 0x00000303,
    ControlTargetTimeout = 0x00000304,
    ControlLimitStall   = 0x00000305,
};

// Cmd::GetMotorFault (0x52) response — firmware motor_fault_report_t. This is
// the diagnostic-grade view: it merges the motor's fault word, the MCU's own
// fault state and the raw CAN evidence into one snapshot. Always ACKs OK (even
// with nothing to report) — check report_flags before reading the fault fields.
struct MotorFaultReport {
    uint8_t  version;                     // == MOTOR_FAULT_REPORT_VERSION
    uint8_t  report_flags;                // MotorFaultReportFlag::*
    uint8_t  source_mask;                 // MotorFaultSource::*
    uint8_t  protocol_mode;               // MotorProtocol in effect
    uint32_t event_seq;                   // increments per new fault event
    uint32_t timestamp_ms;                // MCU tick when this report was built
    uint32_t motor_fault_code;            // MotorFaultBit::*
    uint32_t motor_latched_fault_code;
    uint32_t stop_fault_code;
    uint32_t firmware_fault_code;         // FirmwareFaultCode
    uint32_t firmware_latched_fault_code;
    int32_t  firmware_detail_code;        // driver-level errno, signed
    uint32_t fault_can_id;
    uint8_t  fault_can_dlc;
    uint8_t  monitor_flags;               // MotorMonitorFlag::*
    uint8_t  monitor_reserved;            // MotorMonitorDiag::*
    uint8_t  reserved0;
    uint8_t  fault_can_data[8];
    uint32_t fault_timestamp_ms;
    uint32_t status_timestamp_ms;
    uint8_t  stop_reason;                 // MotorStopReason
    uint8_t  reserved1[3];
};

// ---- Private-protocol single-parameter access (V1.9+ — Cmd 0x38/0x39) -----
// Only valid when the motor CAN protocol is Private; NACKs InvalidParam under
// MIT. The firmware whitelists which index is readable / writable.
namespace MotorPrivateParamType {
    constexpr uint8_t U8  = 1;
    constexpr uint8_t F32 = 2;
}
namespace MotorPrivateParamAccess {
    constexpr uint8_t Read  = 0x01;
    constexpr uint8_t Write = 0x02;
}

struct MotorPrivateParam {   // GET (0x38) response — 8 bytes
    uint16_t index;
    uint8_t  type;        // MotorPrivateParamType::*
    uint8_t  access;      // MotorPrivateParamAccess::* bits
    uint32_t raw_value;   // 4 raw bytes; reinterpret per `type` (u8 or f32)
};

// ---- Follower (slave) gripper config (V1.7 — Cmd::*GripperConfig 0x66/0x67)
constexpr uint32_t GRIPPER_CONFIG_MAGIC   = 0x47525052u;  // "GRPR"
constexpr uint16_t GRIPPER_CONFIG_VERSION = 0x0001u;
namespace GripperConfigFlag {
    constexpr uint16_t Valid   = 0x0001;  // config is valid
    constexpr uint16_t Reverse = 0x0002;  // "open" is the motor's negative dir
}

struct GripperConfig {
    uint32_t magic;          // GRIPPER_CONFIG_MAGIC
    uint16_t version;        // GRIPPER_CONFIG_VERSION
    uint16_t flags;          // GripperConfigFlag::* bits
    float    max_open_rad;   // max open span after zeroing (rad)
    // Raw shaft angle that normalized 0.0 maps to. NOT always zero: the
    // power-on auto-calibration sets the motor zero while the jaw is held
    // loaded at close_stall_torque_nm via a VELOCITY command, and a kd-only
    // MIT frame cannot reproduce that push (measured: 0.359 Nm commanded,
    // 0.213 Nm fed back). Firmware therefore stores a small inset here so
    // that normalized 0.0 is a point control can actually reach. Symmetric
    // with the margin already applied at the open end.
    float    min_open_rad;
    uint8_t  reserved[16];   // GripperEnvelope overlays this — see below
};

// ---- Motion safety envelope (overlays GripperConfig::reserved) ------------
// Carried inside the existing 32-byte GripperConfig record, so Cmd 0x66/0x67
// keep their payload length and no new command was added. On a device that has
// never had one written, reserved is all zeros, so flags is 0 and the envelope
// is simply inactive.
//
// It lives in firmware because the host cannot enforce it in time: this link
// is 100 Hz, phase-locked, and cannot be polled while controlling (an ACK
// collides with the control frames). At 8 rad/s a 30 ms host reaction is
// 0.24 rad of travel — measured, that is enough to cross an object and knock
// it out before torque even rises.
//
// There is deliberately no command-timeout field: an MCU-side timer cannot
// protect against the MCU itself hanging. The motor's own CAN timeout register
// (0x7028) is the right layer for that and is independent of this record.
namespace GripperEnvelopeFlag {
    constexpr uint16_t Valid   = 0x0001;  // the record has been written
    constexpr uint16_t Enforce = 0x0002;  // firmware applies it every cycle

    // Layout version, in the top nibble. Forced by a real incident: removing a
    // float from GripperEnvelope left sizeof at 16 but shifted every field by
    // four bytes. The SDK wrote the new layout, the firmware read the old one,
    // and because the SDK reads back what it itself wrote, get_envelope()
    // looked correct while the clamp behaved as a stale value — peak was set to
    // 2.2 Nm and enforced as 0.5.
    //
    // static_assert on sizeof cannot catch that class: a same-size layout
    // change is exactly its blind spot, and these 16 bytes are a contract
    // between two independently built artifacts that never see each other at
    // compile time. Bump this whenever the layout changes; firmware that does
    // not recognise the version refuses to enforce rather than misreading.
    constexpr uint16_t LayoutVersion = 2;
    constexpr uint16_t LayoutShift   = 12;
    constexpr uint16_t LayoutMask    = 0xF000;
    constexpr uint16_t LayoutBits    = LayoutVersion << LayoutShift;

    inline constexpr uint16_t layout_of(uint16_t flags) noexcept {
        return static_cast<uint16_t>((flags & LayoutMask) >> LayoutShift);
    }
}

struct GripperEnvelope {
    float    cont_torque_nm;      // indefinitely holdable torque, 0 = unlimited
    float    peak_torque_nm;      // motion transient ceiling, 0 = unlimited
    // No speed ceiling here, and that is a measured decision rather than an
    // omission. The only speed an MIT frame can clamp is the velocity
    // FEED-FORWARD, and impedance control runs a pure PD path with that field
    // at zero, so clamping it never fires. The motor's own limit_spd (0x7017)
    // is the right layer, but private-parameter access is refused outright
    // while the motor speaks MIT, and switching protocols needs a power cycle
    // -- there is no runtime path to both.
    //
    // Approach speed is set by peak_torque_nm / kd instead: the jaw accelerates
    // until damping balances the clamped error torque. Measured on firmware
    // 1.1.5 with peak = 1.5 Nm and kd = 1.0: 1.70 rad/s. Lower peak_torque_nm
    // to approach more slowly.
    // Temperature wall, degrees C, 0 = firmware default. Fields rather than
    // firmware constants because they have to be calibrated against the whole
    // assembly's cooling, and every constant change costs a reflash plus a
    // power cycle.
    //
    // Note the vendor's "-20 to 50 C" is an AMBIENT rating, not a ceiling on
    // the module's internal temperature -- an easy and consequential thing to
    // confuse. The real limit is the Class B insulation's 130 C winding rating,
    // and what this sensor reports is board/case temperature, which runs cooler
    // than the winding.
    // Defaults when left 0: derate from 90 C, wall at 100 C.
    //
    // Measured behaviour these numbers come out of, all on firmware 1.1.5:
    //   - equilibrium settles ~6 C under the wall, repeatably: wall 60 -> 55 C
    //     and 1.05 Nm, wall 70 -> 64 C and 1.28 Nm, wall 80 -> 74 C and
    //     1.384 Nm (that last one flat for 120 s).
    //   - the motor's OWN over-temperature protection never fired up to 100 C
    //     case temperature (297 s at a constant 1.65 Nm, fault word all zero),
    //     so in the THERMAL direction this wall is the only line. Not so in the
    //     electrical direction: the motor's undervoltage protection is prompt
    //     (it is what drops the jaw in the customer's report).
    //   - what this sensor reads is board/case temperature. The winding is
    //     hotter by an unmeasured margin; at the 20-40 C typical of such
    //     modules, a 94 C case implies a 114-134 C winding against the Class B
    //     130 C rating. Insulation ageing is cumulative and roughly halves life
    //     per extra 10 C -- it never raises a fault, so it cannot be discovered
    //     by testing for one.
    //   - the wall incidentally caps electrical stress too: a wall of 80 settles
    //     at 1.38 Nm, a wall of 100 at about 1.65 Nm, so sustained current rises
    //     with it. The undervoltage threshold is only bracketed so far -- 4.5 Nm
    //     survived, 6.0 Nm untested, and 6.0 is what the customer's failure hit.
    uint8_t  temp_derate_start_c; // linear derate begins here
    uint8_t  temp_wall_c;         // only the minimum hold torque above here
    uint8_t  reserved0[4];        // write 0
    uint16_t flags;               // GripperEnvelopeFlag::*
};

static_assert(sizeof(GripperEnvelope) == 16,
              "GripperEnvelope must overlay GripperConfig::reserved[16]");

// ---- UART counters (fw 1.1.3 — Cmd::GetUartStats 0x54) --------------------
//
// Exists to answer "which side dropped the bytes". tx_bytes_ok counts only
// what HAL_UART_Transmit accepted, i.e. what actually reached the MCU's
// transmit register. When that disagrees with what the host received, the loss
// happened downstream of the MCU — cable or USB-serial bridge — and no
// firmware change can help. tx_fail_timeout is called out separately because a
// polled transmit that times out mid-frame leaves a short frame on the wire,
// which looks identical to bytes lost in transit.
//
// All counters are free-running since boot; take deltas across a measurement
// window rather than reading absolutes.
struct UartStats {
    uint32_t tx_bytes_ok;       // control port: bytes HAL accepted
    uint32_t tx_calls_ok;       // control port: successful sends (~frames out)
    uint32_t tx_fail_timeout;   // control port: polled-send timeouts (truncates!)
    uint32_t tx_fail_other;     // control port: other send failures
    uint32_t rx_bytes;          // control port: bytes taken by the RXNE ISR
    uint32_t rx_overflow;       // control port: bytes dropped, ring buffer full
    uint32_t debug_tx_bytes;    // DEBUG port bytes out — quantifies logging cost
    uint16_t rb_used;           // control port ring buffer occupancy
    uint16_t rb_free;           // control port ring buffer headroom
    uint32_t log_dropped;       // whole log lines dropped (0 on fw < 1.1.4)
};

// ---- Log configuration (fw 1.1.4 — Cmd::SetLogConfig 0x55) ----------------
//
// Firmware logging is OFF by default and this is the only way to turn it back
// on. It is a diagnostic lever, not a setting to leave enabled: the firmware's
// log sink is a blocking polled UART write (~0.5 ms per line at 921600), so
// every enabled LOG_* call stalls whichever task emitted it. That is what
// livelocked the command channel before it was switched off.
struct LogConfig {
    uint8_t level;        // LogLevel
    uint8_t output_mask;  // LogOutput bitmask
};

enum class LogLevel : uint8_t {
    None    = 0,   // nothing is emitted, whatever output_mask says
    Error   = 1,
    Warn    = 2,
    Info    = 3,
    Debug   = 4,
    Verbose = 5,
};

namespace LogOutput {
    constexpr uint8_t None = 0x00;   // the firmware default
    constexpr uint8_t Uart = 0x01;   // the MCU's DEBUG UART (not exposed over USB)
}

// ---- Follower motor control loop stats (V1.7 — Cmd::GetMotorControlStats 0x51)
struct MotorControlStats {
    uint8_t  running;              // control thread running?
    uint8_t  mode;                 // MotorMode
    uint16_t target_hz;            // requested control rate (Hz)
    uint16_t period_ms;            // control period (ms)
    uint16_t sample_ms;            // stats sampling window (ms)
    float    actual_hz;            // measured control output rate
    uint32_t target_seq;           // host target update sequence
    uint32_t applied_seq;          // last applied target sequence
    uint32_t loop_count;           // cumulative control loops
    uint32_t error_count;          // cumulative control errors
    uint32_t deadline_miss_count;  // cumulative period overruns
    uint32_t timeout_count;        // cumulative comm timeouts (reserved)
    int32_t  last_error;           // last error code
    uint16_t target_age_ms;        // age of current target (ms)
    uint16_t reserved;
    float    target_update_hz;     // host target update rate
};

// ---- Follower power-on auto-calibration config (V1.9 — Cmd 0x68/0x69) -----
// When Enable is set, the firmware auto-calibrates on power-up: it closes until
// the motor stalls at close_stall_torque (that pose becomes zero / fully
// closed), then opens until it stalls at open_stall_torque (that span becomes
// max_open). This automates the manual "zero at close, capture max_open" flow.
//
// V2.2 changed the firmware-side procedure (the wire layout is unchanged):
// stall is confirmed from a single sample held for stall_hold_ms rather than
// averaged over several, the open stall records the frame *before* the trigger
// instead of the fully-jammed pose, and 0.013 rad is subtracted from the saved
// max_open as a safety margin. Expect a slightly smaller max_open than the same
// hardware reported on 1.1.1.
constexpr uint32_t GRIPPER_AUTO_CAL_MAGIC   = 0x4743414Cu;  // "GCAL"
constexpr uint16_t GRIPPER_AUTO_CAL_VERSION = 0x0001u;
namespace GripperAutoCalFlag {
    constexpr uint16_t Valid  = 0x0001;  // config is valid
    constexpr uint16_t Enable = 0x0002;  // run auto-cal on power-up
}

struct GripperAutoCalConfig {
    uint32_t magic;                 // GRIPPER_AUTO_CAL_MAGIC
    uint16_t version;               // GRIPPER_AUTO_CAL_VERSION
    uint16_t flags;                 // GripperAutoCalFlag::* bits
    float    close_stall_torque_nm; // close-to-zero stall torque (Nm)
    float    open_stall_torque_nm;  // open-limit stall torque (Nm)
    float    close_speed_rad_s;     // close speed (rad/s)
    float    open_speed_rad_s;      // open speed (rad/s)
    uint16_t stall_hold_ms;         // stall confirmation time (ms)
    uint16_t startup_delay_ms;      // delay after power-on before auto-cal (ms)
    uint16_t post_zero_delay_ms;    // delay after set-zero before opening (ms)
    uint8_t  close_confirm_count;   // compat only — V2.2 firmware confirms once
    uint8_t  open_confirm_count;    // compat only — V2.2 firmware confirms once
};

// V2.2 — Cmd::SetGripperAutoCalConfig (0x68) now also accepts a short payload
// that patches only the stall-detection fields, leaving speeds, flags and the
// magic/version header at their stored values. Useful for tuning stall torque
// without a read-modify-write round trip.
//
// Accepted request lengths: 10 (GripperAutoCalStallParam), 12 (the same struct
// padded — the firmware ignores the two trailing bytes), 16
// (GripperAutoCalStallParamEx), 32 (full GripperAutoCalConfig) and 36 (the
// pre-V2.2 layout that carried a trailing fast_open_speed_rad_s, ignored).
// The 0x69 read response is always the 32-byte GripperAutoCalConfig.
struct GripperAutoCalStallParam {
    float    close_stall_torque_nm;
    float    open_stall_torque_nm;
    uint16_t stall_hold_ms;
};

struct GripperAutoCalStallParamEx {
    float    close_stall_torque_nm;
    float    open_stall_torque_nm;
    uint16_t stall_hold_ms;
    uint16_t startup_delay_ms;
    uint16_t post_zero_delay_ms;
    uint8_t  close_confirm_count;   // compat only — V2.2 firmware confirms once
    uint8_t  open_confirm_count;    // compat only — V2.2 firmware confirms once
};

// ---- Fisheye camera / leader-encoder calibration (V2.0 / V2.1) ------------
//
// Cmd::CameraFisheyeCal (0x2B) and Cmd::EncoderMaxCal (0x2C) are read/write
// pairs multiplexed on one command: the request payload starts with a CalOp
// byte, and a write appends the parameter body.
//
//   read  request : [op=Read]                              (1 byte)
//   write request : [op=Write][body]                       (1 + sizeof(body))
//   read  response: [op echo=Read][body]                   (1 + sizeof(body))
//   write response: firmware's generic empty-response ACK
//
// NOTE the read *response* leads with the op byte echoed back (0x00), NOT an
// error byte — the firmware packs camera_fisheye_cal_payload_t /
// encoder_max_cal_payload_t verbatim. Reading a parameter that was never
// written NACKs with ErrorCode::CalNotSet rather than returning zeros, so the
// host can tell "uncalibrated" from "calibrated to exactly 0".
//
// The firmware stores both records in internal flash behind a magic/version
// header and does not interpret them: no unit conversion, no range clamping,
// only NaN/Inf rejection (plus max_rad > 0 for the encoder record).
enum class CalOp : uint8_t {
    Read  = 0x00,
    Write = 0x01,
};

// Wire sizes of the full (op + body) request / read-response payloads. Mirror
// firmware CAMERA_FISHEYE_CAL_FULL_SIZE / ENCODER_MAX_CAL_FULL_SIZE.
constexpr std::size_t CAMERA_FISHEYE_CAL_FULL_SIZE = 33;
constexpr std::size_t ENCODER_MAX_CAL_FULL_SIZE    = 5;

// Body of Cmd::CameraFisheyeCal — firmware's `float params[8]`, named. The
// order is fixed by the firmware and must not be reordered.
struct CameraFisheyeCal {
    float fx;   // params[0] — focal length x (px)
    float fy;   // params[1] — focal length y (px)
    float cx;   // params[2] — principal point x (px)
    float cy;   // params[3] — principal point y (px)
    float k1;   // params[4] — fisheye distortion coefficients
    float k2;   // params[5]
    float k3;   // params[6]
    float k4;   // params[7]
};

// ---- WS2812 LED control (V1.9 — Cmd::Ws2812Set 0x0A / Ws2812Effect 0x0B) --
enum class Ws2812Mode : uint8_t {
    Off       = 0,   // all LEDs off
    EffectSet = 1,   // set the effect-layer base color
    Override  = 2,   // direct override color
};

struct Ws2812Set {
    uint8_t  mode;        // Ws2812Mode
    uint8_t  r;           // 0-255
    uint8_t  g;           // 0-255
    uint8_t  b;           // 0-255
    uint8_t  brightness;  // 0-255 (0 = leave unchanged)
    uint16_t blink_ms;    // blink half-period (ms); 0 = no blink
};

enum class Ws2812EffectType : uint8_t {
    None         = 0,
    // The presets' colours and rates are firmware-side and have changed
    // before — 1.2.1 recoloured NormalSolid green -> white and halved the
    // FaultBlink period. Treat the descriptions as "as of leader 1.2.1";
    // the wire values are what this enum pins down.
    NormalSolid  = 1,    // preset: solid white (green before leader 1.2.1)
    NormalBlink  = 2,    // preset: blinking green
    OtaBlink     = 3,    // preset: blinking blue
    FaultBlink   = 4,    // preset: blinking red, 500 ms (1000 ms before 1.2.1)
    Demo         = 5,
    ColorBlink   = 10,   // custom color blink
    ColorBreathe = 11,   // custom color breathe
    HsvCycle     = 12,   // HSV hue cycle
    ColorLerp    = 13,   // two-color LERP fade
};

struct Ws2812Effect {
    uint8_t  effect;      // Ws2812EffectType
    uint8_t  param1;      // effect param 1 (e.g. breathe min brightness)
    uint8_t  param2;      // effect param 2 (e.g. breathe max brightness)
    uint8_t  reserved;    // = 0
    uint16_t period_ms;   // effect period (ms)
    uint8_t  r1;          // color 1 / start color R
    uint8_t  g1;
    uint8_t  b1;
    uint8_t  r2;          // color 2 / end color R (LERP only)
    uint8_t  g2;
    uint8_t  b2;
};

// ---- Stream config -------------------------------------------------------

namespace StreamSrc {
    constexpr uint16_t Imu         = 0x0001;
    constexpr uint16_t Encoder     = 0x0002;
    constexpr uint16_t Eskin1      = 0x0004;
    constexpr uint16_t Eskin2      = 0x0008;
    constexpr uint16_t MotorStatus = 0x0010;
}

enum class StreamMode : uint8_t {
    Separate = 0,
    Combined = 1,
};

enum class StreamInterface : uint8_t {
    Usb  = 0,
    Uart = 1,  // canonical for v1.2 (single USART3 path)
};

struct StreamConfig {
    uint16_t source_mask;     // StreamSrc::* bits
    uint8_t  mode;            // StreamMode
    uint16_t imu_rate;        // Hz
    uint16_t encoder_rate;
    uint16_t eskin_rate;
    uint16_t motor_rate;
    uint8_t  output_iface;    // StreamInterface
};

// ---- ACK / NACK payload --------------------------------------------------

struct AckPayload {
    uint8_t  ack_seq;         // sequence number being ACKed
    uint8_t  error_code;      // ErrorCode::*; Ok for ACK, non-zero for NACK
    uint16_t retry_count;     // firmware-side retry counter
};

// ---- Key status (V1.4 — Cmd::KeyStatus, 0x15) ----------------------------

namespace KeyState {
    constexpr uint8_t SingleClickDown   = 0;
    constexpr uint8_t SingleClickUp     = 1;
    constexpr uint8_t DoubleClick       = 2;
    constexpr uint8_t LongPressDown     = 3;
    constexpr uint8_t LongPressUp       = 4;
}

struct KeyStatusPayload {
    uint8_t key_id;    // K1 = 0
    uint8_t key_state; // KeyState::*
};

// ---- IMU mag-iron calibration (V1.4 — Cmd::SetImuMagCal, 0x26) -----------

struct ImuMagCal {
    float hard_iron[3];     // bx, by, bz in µT (Hard-Iron bias)
    float soft_iron[3][3];  // 3×3 Soft-Iron correction matrix
};

// ---- Calibration result mask (V1.5 — Cmd 0x27/0x28/0x29) -----------------

enum class CalSensorId : uint8_t {
    Imu     = 0,
    ImuMag  = 1,
    Encoder = 2,
    Key     = 3,
    Eskin1  = 4,
    Eskin2  = 5,
    Camera1 = 6,
    Camera2 = 7,
    Camera3 = 8,
    // Max = 9
};

namespace CalMask {
    constexpr uint16_t Imu     = 1u << 0;
    constexpr uint16_t ImuMag  = 1u << 1;
    constexpr uint16_t Encoder = 1u << 2;
    constexpr uint16_t Key     = 1u << 3;
    constexpr uint16_t Eskin1  = 1u << 4;
    constexpr uint16_t Eskin2  = 1u << 5;
    constexpr uint16_t Camera1 = 1u << 6;
    constexpr uint16_t Camera2 = 1u << 7;
    constexpr uint16_t Camera3 = 1u << 8;
}

// CMD_SET_CAL_RESULT (0x27) request payload
struct CalSetPayload {
    uint8_t sensor_id;  // CalSensorId
    uint8_t result;     // 0 = fail/uncalibrated, 1 = success
};

// CMD_SET_ALL_CAL_RESULT (0x28) request payload
struct CalSetAllPayload {
    uint16_t mask;  // CalMask::* bits, 1 = success
};

// CMD_GET_CAL_RESULT (0x29) response payload
struct CalGetResponse {
    uint16_t mask;  // CalMask::* bits, 1 = calibrated
};

// ---- Sensor error report (V1.6 — Cmd::SensorErrorReport, 0x2A) -----------

enum class SensorErrorId : uint8_t {
    Imu      = 0,
    ImuMag   = 1,
    Encoder  = 2,
    Eskin1   = 3,
    Eskin2   = 4,
    Motor    = 5,
    // Max = 6
};

namespace SensorErrCode {
    constexpr uint8_t None        = 0x00;  // recovered
    constexpr uint8_t InitFail    = 0x01;
    constexpr uint8_t CommTimeout = 0x02;
    constexpr uint8_t DataInvalid = 0x03;  // CRC etc
    constexpr uint8_t Offline     = 0x04;
    constexpr uint8_t Overflow    = 0x05;
    constexpr uint8_t Range       = 0x06;
}

struct SensorErrorReport {
    uint8_t  sensor_id;       // SensorErrorId
    uint8_t  error_code;      // SensorErrCode::*
    uint16_t error_count;     // cumulative error count
    uint32_t timestamp_ms;    // firmware HAL_GetTick() — milliseconds, NOT µs
};

// ---- OTA (V1.3 — Cmd::Ota* 0x70-0x75) ------------------------------------

constexpr uint16_t OTA_BLOCK_SIZE   = 1024;  // payload size per OTA_WRITE_BLOCK
constexpr uint32_t OTA_MAX_FW_SIZE  = 456u * 1024u;  // single-bank max

enum class OtaState : uint8_t {
    Idle      = 0,
    Started   = 1,  // OTA_START accepted, awaiting blocks
    Receiving = 2,  // block stream in progress
    Verified  = 3,  // CRC32 passed
    Applying  = 4,  // bank swap in flight
    Error     = 5,  // needs Cmd::OtaAbort to clear
};

// CMD_OTA_START (0x70) request payload — 12 bytes
struct OtaStart {
    uint32_t firmware_size;   // total bytes of firmware image
    uint32_t firmware_crc32;  // ISO-HDLC / zlib.crc32 of the whole image
    uint8_t  target_major;
    uint8_t  target_minor;
    uint8_t  target_patch;
    uint8_t  target_build;
};

// CMD_OTA_WRITE_BLOCK (0x71) request header — followed by `length` bytes
// of raw block data. The full wire payload size = sizeof(OtaWriteBlockHeader)
// + length, where length ≤ OTA_BLOCK_SIZE.
struct OtaWriteBlockHeader {
    uint32_t offset;   // byte offset of this block within the firmware image
    uint16_t length;   // bytes following the header
};

// CMD_OTA_GET_STATUS (0x75) response payload — 8 bytes
struct OtaStatus {
    uint8_t  state;          // OtaState
    uint8_t  error_code;     // last ErrorCode (OtaBusy/OtaNotStarted/…)
    uint32_t bytes_written;  // cumulative bytes accepted
    uint16_t progress_ppt;   // 0–1000 (per-mille)
};

#pragma pack(pop)

// Layout assertions — must match firmware *_PACKET_SIZE / *_HEADER_SIZE
// macros in protocol_data.h. If any of these fail, a wire-format change
// has slipped in and downstream parsing will desync.
static_assert(sizeof(FirmwareVersion)    == 4);
static_assert(sizeof(SnInfo)             == 17);
static_assert(sizeof(ImuData)            == 28);  // IMU_PACKET_SIZE
static_assert(sizeof(ImuConfig)          == 10);
static_assert(sizeof(EncoderData)        == 16);  // ENCODER_PACKET_SIZE
static_assert(sizeof(EncoderConfig)      == 5);   // firmware 0086da6 (was 14)
static_assert(sizeof(EskinHeader)        == 12);  // ESKIN_HEADER_SIZE
static_assert(sizeof(EskinConfig)        == 5);
static_assert(sizeof(CombinedSensorHeader) == 8);
static_assert(sizeof(MotorPosCtrl)       == 12);
static_assert(sizeof(MotorVelCtrl)       == 12);
static_assert(sizeof(MotorTorqueCtrl)    == 12);
static_assert(sizeof(MotorImpedanceCtrl) == 20);  // V1.7 (+ feed-forward vel)
static_assert(sizeof(MotorSpec)          == MOTOR_SPEC_SIZE);
static_assert(sizeof(HomeDiagReport)     == HOME_DIAG_REPORT_SIZE);
static_assert(sizeof(MotorStatus)        == 31);  // V1.9 motor_status_t (was 40)
static_assert(sizeof(MotorStatus)        == MOTOR_STATUS_LEGACY_SIZE);
// V2.2 — 0x53 payload. Its first 31 bytes must stay layout-identical to
// MotorStatus; the firmware memcpys one into the other.
static_assert(sizeof(MotorStatusExt)     == 72);  // V2.2 motor_status_t
static_assert(sizeof(MotorStatusExt)     == MOTOR_STATUS_EXT_SIZE);
static_assert(offsetof(MotorStatusExt, monitor_version) == MOTOR_STATUS_LEGACY_SIZE);
static_assert(offsetof(MotorStatusExt, fault_can_id)    == MOTOR_STATUS_V2_SIZE);
static_assert(sizeof(MotorFaultReport)   == 64);  // V2.2 motor_fault_report_t
static_assert(sizeof(MotorPrivateParam)  == 8);   // V1.9+ private-param GET resp
static_assert(sizeof(GripperConfig)      == 32);  // V1.7 gripper_config_t
static_assert(sizeof(MotorControlStats)  == 48);  // V1.7 motor_control_stats
static_assert(sizeof(UartStats)          == 36);  // fw 1.1.3 uart_stats_packet_t
static_assert(sizeof(LogConfig)          == 2);   // fw 1.1.4 log_config_payload_t
static_assert(sizeof(GripperAutoCalConfig) == 32); // V1.9 gripper_auto_cal_config_t
static_assert(sizeof(GripperAutoCalStallParam)   == 10);  // V2.2 partial 0x68 write
static_assert(sizeof(GripperAutoCalStallParamEx) == 16);  // V2.2 partial 0x68 write
static_assert(sizeof(Ws2812Set)          == 7);   // V1.9 ws2812_set_t
static_assert(sizeof(Ws2812Effect)       == 12);  // V1.9 ws2812_effect_t
// V2.0/V2.1 — body only; the wire payload prepends a CalOp byte, which is why
// the FULL_SIZE constants are one larger.
static_assert(sizeof(CameraFisheyeCal)   == 32);  // camera_fisheye_cal_payload_t.params
static_assert(sizeof(CameraFisheyeCal) + 1 == CAMERA_FISHEYE_CAL_FULL_SIZE);
static_assert(sizeof(float) + 1          == ENCODER_MAX_CAL_FULL_SIZE);
static_assert(sizeof(StreamConfig)       == 12);
static_assert(sizeof(AckPayload)         == 4);
// V1.4+
static_assert(sizeof(KeyStatusPayload)   == 2);
static_assert(sizeof(ImuMagCal)          == 48);  // 3 + 9 floats
// V1.5+
static_assert(sizeof(CalSetPayload)      == 2);
static_assert(sizeof(CalSetAllPayload)   == 2);
static_assert(sizeof(CalGetResponse)     == 2);
// V1.6+
static_assert(sizeof(SensorErrorReport)  == 8);   // SENSOR_ERROR_REPORT_SIZE
// V1.3 OTA
static_assert(sizeof(OtaStart)           == 12);
static_assert(sizeof(OtaWriteBlockHeader) == 6);
static_assert(sizeof(OtaStatus)          == 8);

}  // namespace xense::taccap::protocol

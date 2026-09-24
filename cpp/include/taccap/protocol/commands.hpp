// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 protocol enumerations. Wire values mirror the firmware's
// authoritative definitions in
//   third_party/firmware/tc-gu-01/App/protocol/{protocol_cmd.h,protocol_frame.h}
//   (clone-on-demand, see README "Firmware / PC GUI reference repos").
//
// When in doubt, the firmware source is canonical. The PROTOCOL.md document
// and any host-side Python implementation are secondary.
//
// Tracked firmware protocol: **wire framing V1.8** (global byte stuffing, see
// bus/frame.cpp) + **command set V2.2** (V1.7 motor / CAN-id / gripper-config
// commands; V1.9 WS2812 + private motor params; V2.0/V2.1 fisheye-camera and
// leader-encoder-max calibration; V2.2 follower startup limit-torque, motor
// fault report and extended motor status), plus the two diagnostic commands
// below (0x54 / 0x55), which are NOT part of a V2.x level and are available on
// both roles. All of it is hardware-validated.
//
// Firmware builds carrying command set V2.1: leader (master) 1.2.0,
// follower (slave) 1.1.0. V2.2 adds follower (slave) 1.1.2 — every V2.2 command
// is follower-only, so the leader stayed at 1.2.1 for it.
//
// The diagnostic pair arrived later and on both roles: follower 1.1.3 (0x54)
// and 1.1.4 (0x55), leader 1.2.2 for both. The leader's version moved not for
// those commands alone but because three fixes from the 1.1.x line live in code
// the two roles share, which changed the leader binary while its version
// constant had stayed put.
//
// V2.2 is purely additive — Cmd::GetMotorStatus (0x50) and the MotorStatus DATA
// stream still carry the 31-byte layout. Payload length is therefore NOT a
// firmware-version probe; use Cmd::GetVersion.

#pragma once

#include <cstdint>

namespace xense::taccap::protocol {

// Wire address byte. Firmware has a single ADDR_MCU=0x02; the doc breaks it
// into "MCU left/right" = 0x02/0x03 distinguished by a runtime DEV_TYPE
// setting. We accept all three at the wire layer.
enum class Address : uint8_t {
    PC        = 0x01,
    MCU       = 0x02,  // canonical MCU address; left side when DEV_TYPE=LEFT
    MCU_RIGHT = 0x03,  // when DEV_TYPE=RIGHT
};

// Frame type byte. Matches firmware FRAME_TYPE_* macros.
enum class FrameType : uint8_t {
    CMD_NEED_ACK = 0x00,  // command, host expects ACK back
    CMD_NO_ACK   = 0x01,  // command, no response wanted
    ACK          = 0x02,  // ACK / NACK reply (carries AckPayload)
    DATA         = 0x03,  // unsolicited data push (e.g. streaming sensors)
};

// Application command byte. Mirrors firmware protocol_cmd.h exactly.
enum class Cmd : uint8_t {
    // System (0x00–0x0F)
    Heartbeat       = 0x01,
    GetVersion      = 0x02,
    ResetDevice     = 0x03,
    GetSn           = 0x04,
    SetSn           = 0x05,
    GetDevType      = 0x06,
    SetDevType      = 0x07,
    Ws2812Set       = 0x0A,    // V1.9 — WS2812 base LED control, ws2812_set_t (7B)
    Ws2812Effect    = 0x0B,    // V1.9 — WS2812 effect control, ws2812_effect_t (12B)

    // Sensor reads (0x10–0x1F)
    GetImu          = 0x10,
    GetEncoder      = 0x11,
    GetEskin1       = 0x12,
    GetEskin2       = 0x13,
    GetAllSensors   = 0x14,
    KeyStatus       = 0x15,    // V1.4 — device-side button, payload=key_status_payload_t

    // Stream control + calibration (0x20–0x2F)
    StartStream         = 0x20,
    StopStream          = 0x21,
    SetStreamRate       = 0x22,
    SetStreamMode       = 0x23,
    SetEncoderZero      = 0x24,
    SetImuCal           = 0x25,    // accel/gyro calibration (firmware-side params)
    SetImuMagCal        = 0x26,    // V1.4 — magnetometer hard/soft iron, imu_mag_cal_t (48B)
    SetCalResult        = 0x27,    // V1.5 — write one sensor's cal-success flag, cal_set_payload_t
    SetAllCalResult     = 0x28,    // V1.5 — bulk write cal mask, cal_set_all_payload_t
    GetCalResult        = 0x29,    // V1.5 — read cal-success mask, cal_get_response_t
    SensorErrorReport   = 0x2A,    // V1.6 — DATA-stream-only, sensor_error_report_t (8B)
    CameraFisheyeCal    = 0x2B,    // V2.0 — fisheye intrinsics+distortion R/W (leader + follower)
    EncoderMaxCal       = 0x2C,    // V2.1 — leader encoder max travel angle R/W (leader only)

    // Motor (0x30–0x4F)
    MotorEnable         = 0x30,
    MotorDisable        = 0x31,
    MotorClearFault     = 0x32,
    MotorSetZero        = 0x33,    // V1.7 — zero / mode (0..2-byte payload)
    MotorGetCanId       = 0x34,    // V1.7 — read motor CAN id (resp 1B)
    MotorSetCanId       = 0x35,    // V1.7 — set motor CAN id (req 1B)
    MotorSwitchProtocol = 0x36,    // V1.7 — switch CAN protocol, persist to flash
    MotorGetProtocol    = 0x37,    // V1.7 — query CAN protocol (resp 1B MotorProtocol)
    MotorGetPrivateParam = 0x38,   // V1.9+ — read one private-protocol param (req u16 index)
    MotorSetPrivateParam = 0x39,   // V1.9+ — write one private-protocol param
    MotorSetStartupLimitTorque = 0x3A,  // V2.2 — persist power-on limit torque (req f32)
    MotorGetStartupLimitTorque = 0x3B,  // V2.2 — read it back (resp f32, 4B)
    MotorPosCtrl        = 0x40,
    MotorVelCtrl        = 0x41,
    MotorTorqueCtrl     = 0x42,
    MotorImpedanceCtrl  = 0x43,
    GetMotorStatus      = 0x50,    // resp MotorStatus (31B) — unchanged in V2.2
    GetMotorControlStats = 0x51,   // V1.7 — follower control-loop stats (resp 48B)
    GetMotorFault       = 0x52,    // V2.2 — MotorFaultReport (64B); req 0B (cached)
                                   //        or 1B non-zero (force a CAN read)
    GetMotorStatusExt   = 0x53,    // V2.2 — MotorStatusExt (72B), superset of 0x50
    GetMotorSpec             = 0x56,  // 电机型号规格(40B)
    GetHomeDiag              = 0x57,  // 自动标定诊断(64B)
    GetMotorVersion          = 0x58,  // 电机自身固件版本(8B,仅私有协议)
    // Which actuator this gripper is built around. NOT a query to the motor:
    // the motor cannot answer it (its version frame carries only a version
    // number, neither manual's parameter table has a model field, and private
    // parameters are unreadable under MIT). The MCU keeps the answer in flash.
    //
    // It matters because MIT frames quantise torque and velocity against the
    // model's ranges, so the wrong model is not an error -- it is a fixed
    // scale factor on every frame (EL05 +/-6 Nm vs RS00 +/-14 Nm).
    GetMotorModel            = 0x59,  // 本机记录的电机型号(20B)
    SetMotorModel            = 0x5A,  // 写电机型号(1B),掉电保持,重启生效
    // Diagnostics, firmware 1.1.3+ / 1.1.4+. Present on leader and follower
    // alike: the counters live in the firmware's UART layer, not in a
    // gripper-role-specific subsystem.
    GetUartStats        = 0x54,    // fw 1.1.3 — UartStats (36B); req 0B
    SetLogConfig        = 0x55,    // fw 1.1.4 — req LogConfig (2B), resp 2B

    // Config (0x60–0x6F)
    SetImuConfig        = 0x60,
    GetImuConfig        = 0x61,
    SetEncoderConfig    = 0x62,
    GetEncoderConfig    = 0x63,
    SetEskinConfig      = 0x64,
    GetEskinConfig      = 0x65,
    SetGripperConfig    = 0x66,    // V1.7 — follower open/close limits (req 32B)
    GetGripperConfig    = 0x67,    // V1.7 — read follower config (resp 32B)
    SetGripperAutoCalConfig = 0x68, // V1.9 — power-on auto-cal config (req 32B)
    GetGripperAutoCalConfig = 0x69, // V1.9 — read auto-cal config (resp 32B)

    // OTA upgrade (0x70–0x7F) — added V1.3
    OtaStart            = 0x70,    // ota_start_t (12B)
    OtaWriteBlock       = 0x71,    // ota_write_block_t (6 + len, len ≤ OTA_BLOCK_SIZE)
    OtaVerify           = 0x72,
    OtaApply            = 0x73,
    OtaAbort            = 0x74,
    OtaGetStatus        = 0x75,    // response: ota_status_t (8B)
};

// Error code byte returned in ACK/NACK frames. Mirrors firmware ERR_*.
enum class ErrorCode : uint8_t {
    Ok              = 0x00,
    InvalidCmd      = 0x01,
    InvalidParam    = 0x02,
    LengthMismatch  = 0x03,
    CrcError        = 0x04,
    Timeout         = 0x05,
    MotorFault      = 0x10,
    SensorOffline   = 0x20,
    SysBusy         = 0x30,
    SeqMismatch     = 0x40,
    // V2.0 — a calibration parameter was read before it was ever written.
    // Distinguishes "never calibrated" from "calibrated to exactly zero";
    // returned by Cmd::CameraFisheyeCal / Cmd::EncoderMaxCal reads.
    CalNotSet       = 0x60,
    // OTA error band (V1.3+) — only returned by Cmd::Ota* NACKs.
    OtaBusy         = 0x50,
    OtaNotStarted   = 0x51,
    OtaOffsetErr    = 0x52,
    OtaFlashErr     = 0x53,
    OtaVerifyFail   = 0x54,
    OtaSizeExceed   = 0x55,
};

// to_string helpers (returns a stable C string; never null).
const char* to_string(Address) noexcept;
const char* to_string(FrameType) noexcept;
const char* to_string(Cmd) noexcept;
const char* to_string(ErrorCode) noexcept;

}  // namespace xense::taccap::protocol

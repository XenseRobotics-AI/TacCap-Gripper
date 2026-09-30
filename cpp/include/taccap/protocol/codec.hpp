// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Typed encoders / decoders for TC-GU-01 payloads. Thin layer on top of
// payloads.hpp that hides the raw memcpy and bounds-checks the buffer.
//
// Encoders return wire bytes; decoders take a pointer + length and either
// return std::optional<T> (caller-friendly) or throw ProtocolError when the
// length is wrong. We default to throwing because most call sites already
// know exactly which payload type they're parsing — wrong length is a
// firmware/version-skew bug worth surfacing, not silently dropping.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace xense::taccap::protocol {

// ---- Generic helpers ------------------------------------------------------

template <typename T>
std::vector<uint8_t> encode_pod(const T& value) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "encode_pod requires a trivially-copyable POD");
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &value, sizeof(T));
    return out;
}

// ---- Specific payload encoders --------------------------------------------

std::vector<uint8_t> encode(const MotorPosCtrl&);
std::vector<uint8_t> encode(const MotorVelCtrl&);
std::vector<uint8_t> encode(const MotorTorqueCtrl&);
std::vector<uint8_t> encode(const MotorImpedanceCtrl&);
std::vector<uint8_t> encode(const GripperConfig&);   // V1.7
std::vector<uint8_t> encode(const LogConfig&);       // fw 1.1.4
std::vector<uint8_t> encode(const GripperAutoCalConfig&);  // V1.9
std::vector<uint8_t> encode(const GripperAutoCalStallParam&);    // V2.2 (10B)
std::vector<uint8_t> encode(const GripperAutoCalStallParamEx&);  // V2.2 (16B)
std::vector<uint8_t> encode(const Ws2812Set&);       // V1.9
std::vector<uint8_t> encode(const Ws2812Effect&);    // V1.9
std::vector<uint8_t> encode(const StreamConfig&);
std::vector<uint8_t> encode(const ImuConfig&);
std::vector<uint8_t> encode(const EncoderConfig&);
std::vector<uint8_t> encode(const EskinConfig&);
std::vector<uint8_t> encode_sn(const std::string& sn);   // 17-byte NUL-padded

// V1.4+ — calibration / key
std::vector<uint8_t> encode(const ImuMagCal&);
std::vector<uint8_t> encode(const CalSetPayload&);
std::vector<uint8_t> encode(const CalSetAllPayload&);

// V2.0/V2.1 — Cmd::CameraFisheyeCal / Cmd::EncoderMaxCal multiplex read and
// write on one command via a leading CalOp byte, so they get explicit builders
// instead of an encode() overload (the same struct maps to two wire shapes).
std::vector<uint8_t> encode_camera_fisheye_cal_read();                  // 1 byte
std::vector<uint8_t> encode_camera_fisheye_cal_write(const CameraFisheyeCal&);  // 33 bytes
std::vector<uint8_t> encode_encoder_max_cal_read();                     // 1 byte
std::vector<uint8_t> encode_encoder_max_cal_write(float max_rad);       // 5 bytes

// V1.3+ — OTA. write_block has a variable-length tail so we take the
// data pointer + length explicitly rather than wrap them in a struct.
std::vector<uint8_t> encode(const OtaStart&);
std::vector<uint8_t> encode_ota_write_block(uint32_t offset,
                                            const uint8_t* data,
                                            uint16_t length);

// ---- Specific payload decoders (throw ProtocolError on size mismatch) -----

FirmwareVersion    decode_version(const uint8_t* data, std::size_t len);

// Human-facing firmware version, "MAJOR.MINOR.PATCH" — deliberately WITHOUT
// the build byte. Firmware pins build to 0 and it carries no meaning for
// users, so showing "1.2.1.0" only invited people to type the trailing zero
// into version comparisons. The byte is still on the wire and still readable
// as FirmwareVersion::build for anyone who needs it; this is the one place
// that decides how a version is *presented*, so the format cannot drift
// between the gripper open() logs, OTA logs and the example CLIs.
std::string        version_string(const FirmwareVersion&);
std::string        decode_sn(const uint8_t* data, std::size_t len);
DeviceType         decode_dev_type(const uint8_t* data, std::size_t len);
ImuData            decode_imu(const uint8_t* data, std::size_t len);
ImuConfig          decode_imu_config(const uint8_t* data, std::size_t len);
EncoderData        decode_encoder(const uint8_t* data, std::size_t len);
EncoderConfig      decode_encoder_config(const uint8_t* data, std::size_t len);
EskinHeader        decode_eskin_header(const uint8_t* data, std::size_t len);
EskinConfig        decode_eskin_config(const uint8_t* data, std::size_t len);
MotorStatus        decode_motor_status(const uint8_t* data, std::size_t len);
MotorPrivateParam  decode_motor_private_param(const uint8_t* data, std::size_t len);  // V1.9+
// V2.2 — Cmd::GetMotorStatusExt (0x53) / Cmd::GetMotorFault (0x52). The ext
// decoder accepts the 31- and 59-byte prefixes as well as the full 72 bytes and
// zero-fills whatever the frame stopped short of, so a host that asks for 0x53
// on firmware that only fills part of it still gets a usable struct — check
// monitor_flags rather than the length. Anything below 31 bytes throws.
MotorStatusExt     decode_motor_status_ext(const uint8_t* data, std::size_t len);

// ---- Motor firmware version, as RobStride writes it --------------------------
// The motor's version frame carries 4 raw bytes (high first): an EL05 answers
// {10, 5, 0, 4}, an RS00 {0, 0, 3, 32}. RobStride writes the EL05 one
// "1.0.5.0.4" -- the leading byte 10 split into "1.0" -- and the RS00 one
// "0.0.3.32". These convert between that vendor text and the wire bytes, so a
// factory operator can type what the nameplate says.
//
// Formatting splits the first byte only when it is 10..19, the one case seen
// (EL05). Anything else prints as four plain parts: how the vendor would write
// a first byte of 1..9 or >= 20 is not known, and printing it plainly cannot
// misstate it.
std::string format_motor_fw_version(const uint8_t version[4]);

// Accepts the vendor text ("1.0.5.0.4", "0.0.3.32") or plain raw bytes
// ("10.5.0.4"). Five parts merge the first two as tens and units (the first of
// the two must be 1, the second 0..9, matching what format produces); four
// parts are the bytes as they stand. Throws std::invalid_argument naming the
// problem otherwise.
std::array<uint8_t, 4> parse_motor_fw_version(const std::string& text);
MotorFaultReport   decode_motor_fault_report(const uint8_t* data, std::size_t len);
// V1.7 follower (slave) gripper
GripperConfig      decode_gripper_config(const uint8_t* data, std::size_t len);
GripperAutoCalConfig decode_gripper_auto_cal_config(const uint8_t* data, std::size_t len);  // V1.9
MotorControlStats  decode_motor_control_stats(const uint8_t* data, std::size_t len);
// fw 1.1.3 shipped a 32-byte packet; 1.1.4 appended log_dropped. Both decode
// here, with the missing tail zero-filled, so one SDK build talks to either.
UartStats          decode_uart_stats(const uint8_t* data, std::size_t len);
LogConfig          decode_log_config(const uint8_t* data, std::size_t len);
StreamConfig       decode_stream_config(const uint8_t* data, std::size_t len);
AckPayload         decode_ack(const uint8_t* data, std::size_t len);
// V1.4+
KeyStatusPayload   decode_key_status(const uint8_t* data, std::size_t len);
ImuMagCal          decode_imu_mag_cal(const uint8_t* data, std::size_t len);
// V1.5+
CalGetResponse     decode_cal_get(const uint8_t* data, std::size_t len);
// V2.0/V2.1 — read responses lead with the CalOp byte echoed back (Read), not
// an error byte; these decoders verify that byte and strip it.
CameraFisheyeCal   decode_camera_fisheye_cal(const uint8_t* data, std::size_t len);
float              decode_encoder_max_cal(const uint8_t* data, std::size_t len);
// V1.6+
SensorErrorReport  decode_sensor_error(const uint8_t* data, std::size_t len);
// V1.3 OTA
OtaStatus          decode_ota_status(const uint8_t* data, std::size_t len);

// EskinFrame collapses the wire layout (header + variable-length body) into
// one easy-to-use object. Cell data is stored in either values_u16 (when
// type == EskinOutputType::Adc) or values_f32 (Voltage / Force).
struct EskinFrame {
    EskinHeader              header;
    std::vector<uint16_t>    values_u16;
    std::vector<float>       values_f32;
};
EskinFrame decode_eskin(const uint8_t* data, std::size_t len);

}  // namespace xense::taccap::protocol

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/protocol/codec.hpp>
#include <taccap/error.hpp>

#include <algorithm>
#include <cstring>

namespace xense::taccap::protocol {

namespace {

template <typename T>
T pod_from_bytes(const uint8_t* data, std::size_t len, const char* type_name) {
    if (len != sizeof(T)) {
        throw ProtocolError(
            std::string("decode ") + type_name + ": expected " +
            std::to_string(sizeof(T)) + " bytes, got " + std::to_string(len));
    }
    T out{};
    std::memcpy(&out, data, sizeof(T));
    return out;
}

}  // namespace

// ---- Encoders -------------------------------------------------------------

std::vector<uint8_t> encode(const MotorPosCtrl& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorVelCtrl& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorTorqueCtrl& v)     { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorImpedanceCtrl& v)  { return encode_pod(v); }
std::vector<uint8_t> encode(const GripperConfig& v)       { return encode_pod(v); }
std::vector<uint8_t> encode(const GripperAutoCalConfig& v){ return encode_pod(v); }
std::vector<uint8_t> encode(const GripperAutoCalStallParam& v)  { return encode_pod(v); }
std::vector<uint8_t> encode(const GripperAutoCalStallParamEx& v){ return encode_pod(v); }
std::vector<uint8_t> encode(const Ws2812Set& v)           { return encode_pod(v); }
std::vector<uint8_t> encode(const Ws2812Effect& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const StreamConfig& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const ImuConfig& v)           { return encode_pod(v); }
std::vector<uint8_t> encode(const EncoderConfig& v)       { return encode_pod(v); }
std::vector<uint8_t> encode(const EskinConfig& v)         { return encode_pod(v); }
// V1.4+
std::vector<uint8_t> encode(const ImuMagCal& v)           { return encode_pod(v); }
std::vector<uint8_t> encode(const CalSetPayload& v)       { return encode_pod(v); }
std::vector<uint8_t> encode(const CalSetAllPayload& v)    { return encode_pod(v); }
// V1.3 OTA
std::vector<uint8_t> encode(const OtaStart& v)            { return encode_pod(v); }

std::vector<uint8_t> encode_ota_write_block(uint32_t offset,
                                            const uint8_t* data,
                                            uint16_t length) {
    // Variable-length: 6-byte header (offset + length) followed by `length`
    // bytes of raw firmware data. Mirror firmware ota_write_block_t.
    std::vector<uint8_t> out(sizeof(OtaWriteBlockHeader) + length);
    OtaWriteBlockHeader hdr{};
    hdr.offset = offset;
    hdr.length = length;
    std::memcpy(out.data(), &hdr, sizeof(hdr));
    if (length > 0 && data != nullptr) {
        std::memcpy(out.data() + sizeof(hdr), data, length);
    }
    return out;
}

// V2.0/V2.1 — CalOp-prefixed read/write builders. The body is memcpy'd at
// offset 1, i.e. deliberately unaligned; the firmware does the same.
namespace {

std::vector<uint8_t> encode_cal_read_request() {
    return {static_cast<uint8_t>(CalOp::Read)};
}

template <typename T>
std::vector<uint8_t> encode_cal_write_request(const T& body) {
    std::vector<uint8_t> out(1 + sizeof(T));
    out[0] = static_cast<uint8_t>(CalOp::Write);
    std::memcpy(out.data() + 1, &body, sizeof(T));
    return out;
}

// Read responses lead with the op byte echoed back. Validate it and hand back
// a pointer to the body.
const uint8_t* cal_read_body(const uint8_t* data, std::size_t len,
                             std::size_t full_size, const char* type_name) {
    if (data == nullptr || len != full_size) {
        throw ProtocolError(
            std::string("decode ") + type_name + ": expected " +
            std::to_string(full_size) + " bytes, got " + std::to_string(len));
    }
    if (data[0] != static_cast<uint8_t>(CalOp::Read)) {
        throw ProtocolError(
            std::string("decode ") + type_name + ": expected leading op byte " +
            "0x00 (CalOp::Read), got 0x" +
            std::to_string(static_cast<unsigned>(data[0])));
    }
    return data + 1;
}

}  // namespace

std::vector<uint8_t> encode_camera_fisheye_cal_read() {
    return encode_cal_read_request();
}

std::vector<uint8_t> encode_camera_fisheye_cal_write(const CameraFisheyeCal& v) {
    return encode_cal_write_request(v);
}

std::vector<uint8_t> encode_encoder_max_cal_read() {
    return encode_cal_read_request();
}

std::vector<uint8_t> encode_encoder_max_cal_write(float max_rad) {
    return encode_cal_write_request(max_rad);
}

std::vector<uint8_t> encode_sn(const std::string& sn) {
    std::vector<uint8_t> out(sizeof(SnInfo), 0);
    const std::size_t n = std::min(sn.size(), static_cast<std::size_t>(16));
    std::memcpy(out.data(), sn.data(), n);
    return out;
}

// ---- Decoders -------------------------------------------------------------

FirmwareVersion decode_version(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<FirmwareVersion>(data, len, "FirmwareVersion");
}

std::string version_string(const FirmwareVersion& v) {
    return std::to_string(v.major) + "." + std::to_string(v.minor) + "." +
           std::to_string(v.patch);
}

std::string decode_sn(const uint8_t* data, std::size_t len) {
    if (len == 0) return {};
    // SnInfo wire size is 17 bytes, but earlier docs show queries returning
    // an unbounded ASCII string — accept anything and trim at the first NUL.
    std::size_t end = 0;
    while (end < len && data[end] != 0) ++end;
    return std::string(reinterpret_cast<const char*>(data), end);
}

DeviceType decode_dev_type(const uint8_t* data, std::size_t len) {
    if (len < 1) {
        throw ProtocolError("decode dev_type: empty payload");
    }
    return static_cast<DeviceType>(data[0]);
}

ImuData decode_imu(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<ImuData>(data, len, "ImuData");
}
ImuConfig decode_imu_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<ImuConfig>(data, len, "ImuConfig");
}
EncoderData decode_encoder(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EncoderData>(data, len, "EncoderData");
}
EncoderConfig decode_encoder_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EncoderConfig>(data, len, "EncoderConfig");
}
EskinHeader decode_eskin_header(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EskinHeader>(data, len, "EskinHeader");
}
EskinConfig decode_eskin_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EskinConfig>(data, len, "EskinConfig");
}
MotorStatus decode_motor_status(const uint8_t* data, std::size_t len) {
    // The SDK targets the V1.9 31-byte motor_status_t. The first 18 bytes
    // (pos/vel/torque/temp/status) share the layout of every prior version, so
    // a >=18-byte read always yields correct pose/torque/status; the target_*/
    // control_mode tail is only correct on V1.9 firmware (the older 40-byte
    // layout had current fields in between — a V1.9 SDK reads its target_*
    // from the wrong offsets there, which is the accepted trade-off).
    constexpr std::size_t kPrefix = 18;
    if (data == nullptr || len < kPrefix) {
        throw ProtocolError(
            "decode MotorStatus: expected >= 18 bytes, got " +
            std::to_string(len));
    }
    MotorStatus out{};
    std::memcpy(&out, data, std::min(len, sizeof(MotorStatus)));
    return out;
}

GripperConfig decode_gripper_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<GripperConfig>(data, len, "GripperConfig");
}

MotorPrivateParam decode_motor_private_param(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<MotorPrivateParam>(data, len, "MotorPrivateParam");
}

GripperAutoCalConfig decode_gripper_auto_cal_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<GripperAutoCalConfig>(data, len, "GripperAutoCalConfig");
}

CameraFisheyeCal decode_camera_fisheye_cal(const uint8_t* data, std::size_t len) {
    const uint8_t* body = cal_read_body(data, len, CAMERA_FISHEYE_CAL_FULL_SIZE,
                                        "CameraFisheyeCal");
    CameraFisheyeCal out{};
    std::memcpy(&out, body, sizeof(out));
    return out;
}

float decode_encoder_max_cal(const uint8_t* data, std::size_t len) {
    const uint8_t* body = cal_read_body(data, len, ENCODER_MAX_CAL_FULL_SIZE,
                                        "EncoderMaxCal");
    float out = 0.0f;
    std::memcpy(&out, body, sizeof(out));
    return out;
}

MotorControlStats decode_motor_control_stats(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<MotorControlStats>(data, len, "MotorControlStats");
}

// Firmware 1.1.3 answered 0x54 with 32 bytes; 1.1.4 appended log_dropped for 36.
// Accept either and zero-fill the tail, so one SDK build works against both
// without the caller having to branch on firmware_version().
UartStats decode_uart_stats(const uint8_t* data, std::size_t len) {
    // Every firmware so far: 32 B (1.1.3 .. 1.2.10 / leader 1.2.5), 44 B from
    // follower 1.2.11 / leader 1.2.6 (log_dropped placeholder + rx_errors +
    // rx_rearms). Missing tail fields read 0. Pinned, not derived from sizeof:
    // deriving it rejected every older firmware the moment the struct grew.
    constexpr std::size_t kOldestPacket = 32;
    if (data == nullptr || len < kOldestPacket) {
        throw ProtocolError(
            "decode UartStats: expected >= " + std::to_string(kOldestPacket) +
            " bytes, got " + std::to_string(len));
    }
    UartStats out{};
    std::memcpy(&out, data, std::min(len, sizeof(out)));
    return out;
}

LogConfig decode_log_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<LogConfig>(data, len, "LogConfig");
}

std::vector<uint8_t> encode(const LogConfig& c) { return encode_pod(c); }

// V2.2 — 0x53. Accepts the three documented prefix lengths (31 / 59 / 72) and
// anything in between, zero-filling the tail. A short frame is not an error
// here: the 31-byte prefix is exactly what 0x50 and the DATA stream carry, so
// the same decoder can be pointed at either without a length dance.
MotorStatusExt decode_motor_status_ext(const uint8_t* data, std::size_t len) {
    if (data == nullptr || len < MOTOR_STATUS_LEGACY_SIZE) {
        throw ProtocolError(
            "decode MotorStatusExt: expected >= " +
            std::to_string(MOTOR_STATUS_LEGACY_SIZE) + " bytes, got " +
            std::to_string(len));
    }
    MotorStatusExt out{};
    std::memcpy(&out, data, std::min(len, sizeof(out)));
    return out;
}

MotorFaultReport decode_motor_fault_report(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<MotorFaultReport>(data, len, "MotorFaultReport");
}
StreamConfig decode_stream_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<StreamConfig>(data, len, "StreamConfig");
}
AckPayload decode_ack(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<AckPayload>(data, len, "AckPayload");
}

// V1.4+
KeyStatusPayload decode_key_status(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<KeyStatusPayload>(data, len, "KeyStatusPayload");
}
ImuMagCal decode_imu_mag_cal(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<ImuMagCal>(data, len, "ImuMagCal");
}

// V1.5+
CalGetResponse decode_cal_get(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<CalGetResponse>(data, len, "CalGetResponse");
}

// V1.6+
SensorErrorReport decode_sensor_error(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<SensorErrorReport>(data, len, "SensorErrorReport");
}

// V1.3 OTA
OtaStatus decode_ota_status(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<OtaStatus>(data, len, "OtaStatus");
}

EskinFrame decode_eskin(const uint8_t* data, std::size_t len) {
    if (len < sizeof(EskinHeader)) {
        throw ProtocolError(
            "decode EskinFrame: payload too short for header (" +
            std::to_string(len) + " < " + std::to_string(sizeof(EskinHeader)) + ")");
    }

    EskinFrame out;
    std::memcpy(&out.header, data, sizeof(EskinHeader));

    const std::size_t cells = static_cast<std::size_t>(out.header.rows) *
                              static_cast<std::size_t>(out.header.cols);
    const std::size_t cell_size =
        (out.header.type == static_cast<uint8_t>(EskinOutputType::Adc)) ? 2u : 4u;
    const std::size_t expected = sizeof(EskinHeader) + cells * cell_size;
    if (len != expected) {
        throw ProtocolError(
            "decode EskinFrame: length mismatch (got " + std::to_string(len) +
            ", expected " + std::to_string(expected) + " for " +
            std::to_string(cells) + " cells of " + std::to_string(cell_size) + "B)");
    }

    const uint8_t* body = data + sizeof(EskinHeader);
    if (cell_size == 2) {
        out.values_u16.resize(cells);
        std::memcpy(out.values_u16.data(), body, cells * 2);
    } else {
        out.values_f32.resize(cells);
        std::memcpy(out.values_f32.data(), body, cells * 4);
    }
    return out;
}

}  // namespace xense::taccap::protocol

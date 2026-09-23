// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/imu.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cmath>
#include <cstring>

namespace xense::taccap {

namespace {

// Unit-conversion constants matching the firmware's fixed-point layout
// (see `protocol_data.h`):
//   accel:  raw int16 in mg                 →  m/s²
//   gyro:   raw int16 in 0.01 dps           →  rad/s
//   mag:    raw int16 in 0.01 µT            →  µT
//   temp:   raw int16 in 0.01 °C            →  °C
constexpr float kMgToMps2  = 0.001f * 9.80665f;
constexpr float kCdpsToRps = 0.01f * static_cast<float>(M_PI) / 180.0f;
constexpr float kCuTtoUT   = 0.01f;
constexpr float kCdegToDeg = 0.01f;

}  // namespace

IMU::IMU(bus::Transport& transport) : t_(transport) {}

ImuSample IMU::decode(const std::uint8_t* payload, std::size_t len) {
    ImuSample s{};
    s.host_time = std::chrono::steady_clock::now();
    s.raw       = protocol::decode_imu(payload, len);  // throws on size mismatch

    s.mcu_timestamp_us = s.raw.timestamp_us;
    s.valid_flag       = s.raw.valid_flag;
    s.seq              = s.raw.seq;

    s.accel_mps2 = {
        s.raw.accel_x * kMgToMps2,
        s.raw.accel_y * kMgToMps2,
        s.raw.accel_z * kMgToMps2,
    };
    s.gyro_radps = {
        s.raw.gyro_x * kCdpsToRps,
        s.raw.gyro_y * kCdpsToRps,
        s.raw.gyro_z * kCdpsToRps,
    };
    s.mag_uT = {
        s.raw.mag_x * kCuTtoUT,
        s.raw.mag_y * kCuTtoUT,
        s.raw.mag_z * kCuTtoUT,
    };
    s.temperature_c = s.raw.temperature * kCdegToDeg;
    return s;
}

ImuSample IMU::read_once(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetImu, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("IMU::read_once NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return decode(ack.data.data(), ack.data.size());
}

IMU::SubId IMU::on_data(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::GetImu,
        [cb = std::move(cb)](const bus::Frame& f) {
            // Dispatcher thread context, not the reader -- Transport keeps
            // user code off the read path on purpose. decode() may throw on a
            // malformed frame; swallow here (Transport already counts
            // callback_exc).
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void IMU::off(SubId id) { t_.unsubscribe(id); }

void IMU::set_mag_calibration(const std::array<float, 3>& hard_iron,
                              const std::array<float, 9>& soft_iron_row_major,
                              std::chrono::milliseconds timeout) {
    protocol::ImuMagCal cal{};
    cal.hard_iron[0] = hard_iron[0];
    cal.hard_iron[1] = hard_iron[1];
    cal.hard_iron[2] = hard_iron[2];
    // soft_iron_row_major[r*3 + c] -> cal.soft_iron[r][c]
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cal.soft_iron[r][c] = soft_iron_row_major[r * 3 + c];
        }
    }
    auto ack = t_.send_cmd(protocol::Cmd::SetImuMagCal,
                           protocol::encode(cal), timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("IMU::set_mag_calibration NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

}  // namespace xense::taccap

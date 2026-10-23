// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/imu.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cmath>
#include <cstring>
#include <memory>
#include <random>

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

uint64_t read64(const uint8_t* p) {
    uint64_t v = 0;
    for (unsigned i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
}

}  // namespace

IMU::IMU(bus::Transport& transport) : t_(transport) {}

ImuSample IMU::decode(const std::uint8_t* payload, std::size_t len) {
    ImuSample s{};
    s.host_time = std::chrono::steady_clock::now();
    s.raw       = protocol::decode_imu(payload, len == 44 ? 28 : len);

    s.mcu_timestamp_us = s.raw.timestamp_us;
    if (len == 44) {
        s.mcu_timestamp_us = read64(payload + 28);
        s.boot_session_id = read64(payload + 36);
        s.timestamp_is_64bit = true;
        if (!s.boot_session_id || uint32_t(s.mcu_timestamp_us) != s.raw.timestamp_us)
            throw ProtocolError("IMU V2: invalid session or inconsistent timestamp");
    }
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

ImuSample IMU::Timeline::decode(const uint8_t* p, std::size_t n,
                              std::chrono::steady_clock::time_point arrival) {
    auto s = IMU::decode(p, n);
    s.host_time = arrival;
    if (seen_) {
        const auto gap = std::chrono::duration_cast<std::chrono::microseconds>(s.host_time - host_).count();
        if (s.timestamp_is_64bit) {
            s.timestamp_discontinuity = session_ != s.boot_session_id;
            if (!s.timestamp_discontinuity && s.mcu_timestamp_us < last_time_) {
                s.timestamp_valid = false;
                s.timestamp_discontinuity = true;
                return s; // stale frame: do not move the high-water mark
            }
            legacy_faulted_ = false;
        } else {
            const uint32_t delta = s.raw.timestamp_us - raw_;
            const uint16_t seq_delta = uint16_t(s.seq - seq_);
            // Legacy cannot prove boot identity or reconstruct arbitrary gaps.
            // Never turn a historical 17-second DWT reset into a 71-minute wrap.
            bool ambiguous = legacy_faulted_ || session_ != 0 || gap < 0 || gap >= INT32_MAX ||
                             delta >= INT32_MAX || seq_delta >= 0x8000U;
            if (s.raw.timestamp_us < raw_)
                ambiguous |= raw_ < 0xf0000000U || s.raw.timestamp_us > 0x0fffffffU ||
                             seq_delta == 0 || uint64_t(delta) > uint64_t(gap) + 2000000ULL;
            s.timestamp_valid = !ambiguous;
            s.timestamp_discontinuity = ambiguous;
            legacy_faulted_ = ambiguous;
            if (!ambiguous) s.mcu_timestamp_us = last_time_ + delta;
        }
    }
    seen_ = true;
    last_time_ = s.mcu_timestamp_us;
    raw_ = s.raw.timestamp_us;
    seq_ = s.seq;
    session_ = s.boot_session_id;
    host_ = s.host_time;
    return s;
}

uint16_t IMU::negotiate_time(std::chrono::milliseconds timeout) {
    std::random_device random;
    uint64_t nonce = (uint64_t(random()) << 32) ^ uint64_t(random());
    if (!nonce) nonce = 1;
    std::vector<uint8_t> request(8);
    for (unsigned i = 0; i < 8; ++i) request[i] = uint8_t(nonce >> (8 * i));
    bus::AckResponse ack;
    try {
        ack = t_.send_cmd(protocol::Cmd::TimeSession, request, timeout);
    } catch (const NackError& e) {
        if (e.code() == uint8_t(protocol::ErrorCode::InvalidCmd)) return protocol::StreamSrc::Imu;
        throw;
    }
    const auto error = bus::ack_error_code(ack);
    if (error == protocol::ErrorCode::InvalidCmd) return protocol::StreamSrc::Imu;
    if (error != protocol::ErrorCode::Ok || ack.data.size() != 16 || !read64(ack.data.data() + 8))
        throw ProtocolError("Invalid IMU time-session response");
    return protocol::StreamSrc::ImuV2;
}

ImuSample IMU::read_once(std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(read_mutex_);
    const auto cmd = negotiate_time(timeout) == protocol::StreamSrc::ImuV2
                   ? protocol::Cmd::GetImuV2 : protocol::Cmd::GetImu;
    auto ack = t_.send_cmd(cmd, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("IMU::read_once NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    if (ack.data.size() != (cmd == protocol::Cmd::GetImuV2 ? 44U : 28U))
        throw ProtocolError("IMU response does not match negotiated format");
    return polling_time_.decode(ack.data.data(), ack.data.size());
}

IMU::SubId IMU::on_data(Callback cb) {
    struct SubscriptionState {
        Timeline timeline;
        std::vector<uint8_t> last_payload;
    };
    auto state = std::make_shared<SubscriptionState>();
    auto callback = [cb = std::move(cb), state](const bus::Frame& f) {
            // Dispatcher thread context, not the reader -- Transport keeps
            // user code off the read path on purpose. decode() may throw on a
            // malformed frame; preserve the component's existing policy of
            // dropping malformed frames and containing callback exceptions.
            try {
                if (f.payload.size() != (f.cmd == protocol::Cmd::GetImuV2 ? 44U : 28U))
                    throw ProtocolError("IMU stream format mismatch");
                // Legacy devices may send the identical cache at 1000 Hz
                // although acquisition is 100 Hz. Do not integrate it twice.
                if (f.payload == state->last_payload) return;
                auto sample = state->timeline.decode(f.payload.data(), f.payload.size());
                state->last_payload = f.payload;
                cb(sample);
            } catch (...) {}
        };
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto legacy = t_.subscribe(protocol::Cmd::GetImu, callback);
    try {
        auto v2 = t_.subscribe(protocol::Cmd::GetImuV2, callback);
        try { subscriptions_.emplace(legacy, v2); }
        catch (...) { t_.unsubscribe(v2); throw; }
    } catch (...) { t_.unsubscribe(legacy); throw; }
    return legacy;
}

void IMU::off(SubId id) {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto it = subscriptions_.find(id);
    if (it != subscriptions_.end()) {
        t_.unsubscribe(it->second);
        subscriptions_.erase(it);
    }
    t_.unsubscribe(id);
}

void IMU::calibrate_accel_gyro(std::chrono::milliseconds timeout) {
    for (uint8_t action : {0x01, 0x04}) {
        auto ack = t_.send_cmd(protocol::Cmd::SetImuCal, {action}, timeout,
                               bus::RetryMode::SameSeq);
        if (ack.is_nack) {
            throw ProtocolError(std::string("IMU::calibrate_accel_gyro NACK: ") +
                                protocol::to_string(ack.error_code));
        }
    }
}

void IMU::start_mag_calibration(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::SetImuMagCal, {0x01}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("IMU::start_mag_calibration NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void IMU::stop_mag_calibration(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::SetImuMagCal, {0x02}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("IMU::stop_mag_calibration NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

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

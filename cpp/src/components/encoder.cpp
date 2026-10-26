// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/encoder.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

namespace xense::taccap {

namespace {

// Below this (more negative than -0.1 rad) we surface a warning instead
// of silently clamping — it's the threshold between "expected
// post-zero jitter" and "something is mechanically off".
constexpr float kNegPositionWarnThreshold = -0.1f;

// Don't warn more than once per second per Encoder instance, so a
// 100 Hz stream of bad samples doesn't drown the log.
constexpr std::chrono::seconds kNegPositionWarnInterval{1};

}  // namespace

Encoder::Encoder(bus::Transport& transport) : t_(transport) {}

EncoderSample Encoder::decode(const std::uint8_t* payload, std::size_t len) {
    EncoderSample s{};
    s.host_time          = std::chrono::steady_clock::now();
    s.raw                = protocol::decode_encoder(payload, len);
    s.mcu_timestamp_us   = s.raw.timestamp_us;
    s.position_rad       = s.raw.position_rad;
    s.velocity_rad_s     = s.raw.velocity_rad_s;
    s.status             = s.raw.status;
    s.seq                = s.raw.seq;
    // Normalization is a per-instance setting, so the static decoder leaves
    // `position` unset; normalize() fills it when a map is installed.
    s.position           = std::numeric_limits<float>::quiet_NaN();
    return s;
}

EncoderSample Encoder::read_once(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetEncoder, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Encoder::read_once NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    auto s = decode(ack.data.data(), ack.data.size());
    s.host_time = ack.receive_time;
    normalize(s);
    return s;
}

void Encoder::set_position_map(const GripperPosition& map) {
    if (!map.valid()) {
        throw ProtocolError(
            "Encoder::set_position_map: converter is not valid (travel span "
            "<= 0) — the gripper must be calibrated before normalized "
            "positions mean anything");
    }
    std::lock_guard<std::mutex> lk(pos_map_mu_);
    pos_map_ = map;
}

void Encoder::clear_position_map() {
    std::lock_guard<std::mutex> lk(pos_map_mu_);
    pos_map_ = GripperPosition{};
}

bool Encoder::has_position_map() const {
    std::lock_guard<std::mutex> lk(pos_map_mu_);
    return pos_map_.valid();
}

GripperPosition Encoder::position_map() const {
    std::lock_guard<std::mutex> lk(pos_map_mu_);
    return pos_map_;
}

void Encoder::normalize(EncoderSample& s) const {
    // Clamp first, then normalize, so `position` is derived from the same
    // user-facing value as position_rad (both bottom out at fully closed).
    clamp_negative_(s);

    GripperPosition map;
    {
        std::lock_guard<std::mutex> lk(pos_map_mu_);
        map = pos_map_;
    }
    if (map.valid()) {
        s.position = map.to_position(s.position_rad);
    }
}

void Encoder::clamp_negative_(EncoderSample& s) const {
    if (s.position_rad >= 0.0f) return;

    if (s.position_rad < kNegPositionWarnThreshold) {
        const auto now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        const int64_t interval_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                kNegPositionWarnInterval)
                .count();
        int64_t last_ns = last_neg_warn_ns_.load(std::memory_order_relaxed);
        if (now_ns - last_ns >= interval_ns &&
            last_neg_warn_ns_.compare_exchange_strong(
                last_ns, now_ns,
                std::memory_order_relaxed)) {
            logger()->warn(
                "Encoder reading {:.4f} rad is below clamp threshold "
                "({:.2f} rad) — likely calibration drift or mechanical "
                "issue; clamping to 0 (raw preserved in .raw.position_rad).",
                s.position_rad, kNegPositionWarnThreshold);
        }
    }
    s.position_rad = 0.0f;
}

void Encoder::set_zero(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::SetEncoderZero, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Encoder::set_zero NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

Encoder::SubId Encoder::on_data(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::GetEncoder,
        [this, cb = std::move(cb)](const bus::Frame& f) {
            try {
                auto s = decode(f.payload.data(), f.payload.size());
                s.host_time = f.receive_time;
                normalize(s);
                cb(s);
            } catch (...) {}
        });
}

void Encoder::off(SubId id) { t_.unsubscribe(id); }

}  // namespace xense::taccap

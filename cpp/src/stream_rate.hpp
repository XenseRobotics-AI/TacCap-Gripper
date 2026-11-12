// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Private helper shared by leader_gripper.cpp and follower_gripper.cpp — NOT
// part of the installed public headers.
//
// Legacy tick model for encoder, motor status and low-rate IMU. IMU >100 Hz
// uses real-sample selection on capable firmware; do not apply this model to
// that path or claim a measured hardware rate from the requested value.
//
// Mirrored from third_party/firmware/tc-gu-01,
// App/tasks/task_data_stream.c (hw_v1.1.0 @ bf0a06e):
//
//   - The scheduler runs on a 1 ms tick (STREAM_CYCLE_MS) and divides it by an
//     integer, so only exact divisors of 1000 come out at the requested rate.
//     300 Hz becomes 1000/3 = 333 Hz; 150 Hz becomes 1000/6 = 167 Hz.
//   - Anything above 1000 Hz makes the divider 0, which the firmware rewrites
//     to 10 — so asking for 2000 Hz silently yields 100 Hz, not 1000.
//   - Motor status is capped at STREAM_MOTOR_MAX_RATE_HZ = 100 ("leave bandwidth
//     for the control channel"), so a 200 Hz request silently yields 100 Hz.
//   - **A rate of 0 does not disable a source.** Emission is gated purely on
//     the source_mask bit; rate 0 takes the same divider==0 fallback and
//     streams at 100 Hz. Turning a source off means clearing its mask bit,
//     which is what stream_source_mask() below is for.
//   - None of this is validated: data_stream_start() memcpy's the config and
//     never NACKs, so every one of these adjustments is silent on the wire.

#pragma once

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstdint>
#include <algorithm>

namespace xense::taccap::detail {

// STREAM_CYCLE_MS = 1 → a 1 kHz base tick.
constexpr unsigned kStreamTickHz   = 1000;
// STREAM_MOTOR_MAX_RATE_HZ.
constexpr unsigned kMotorMaxRateHz = 100;
constexpr unsigned kImuMaxRateHz = 1125;
// The firmware's fallback divider whenever the computed one is 0.
constexpr unsigned kStreamFallbackDiv = 10;

// What the firmware will really emit for `requested_hz`.
// `cap_hz` is the source's firmware ceiling, or 0 for the uncapped sources
// (IMU / encoder / eskin).
inline unsigned effective_stream_rate_hz(unsigned requested_hz,
                                         unsigned cap_hz) noexcept {
    unsigned rate = requested_hz;
    if (cap_hz != 0 && (rate == 0 || rate > cap_hz)) rate = cap_hz;
    unsigned div = (rate > 0) ? (kStreamTickHz / rate) : 0;
    if (div == 0) div = kStreamFallbackDiv;
    return kStreamTickHz / div;
}

// Which sources to actually turn on. A rate of 0 means "off", and the only
// way to express that to the firmware is to leave the bit clear — passing
// rate 0 with the bit set would stream at 100 Hz.
inline uint16_t stream_source_mask(unsigned imu_hz, unsigned encoder_hz,
                                   unsigned motor_hz) noexcept {
    uint16_t mask = 0;
    if (imu_hz     > 0) mask |= protocol::StreamSrc::Imu;
    if (encoder_hz > 0) mask |= protocol::StreamSrc::Encoder;
    if (motor_hz   > 0) mask |= protocol::StreamSrc::MotorStatus;
    return mask;
}

// Warn when the firmware will not honour the request. Silent when it will.
inline void warn_if_rate_adjusted(const char* who, const char* source,
                                  unsigned requested_hz, unsigned cap_hz) {
    if (requested_hz == 0) return;   // source is off; nothing to adjust
    const unsigned actual = effective_stream_rate_hz(requested_hz, cap_hz);
    if (actual == requested_hz) return;

    if (cap_hz != 0 && requested_hz > cap_hz) {
        logger()->warn(
            "{}: {} stream requested at {} Hz but the firmware caps it at "
            "{} Hz — you will receive {} Hz. This is silent on the wire "
            "(StartStream is not NACKed).",
            who, source, requested_hz, cap_hz, actual);
    } else {
        logger()->warn(
            "{}: {} stream requested at {} Hz but the firmware divides a "
            "{} Hz tick by an integer — you will receive {} Hz. Pick a "
            "divisor of {} for an exact rate.",
            who, source, requested_hz, kStreamTickHz, actual, kStreamTickHz);
    }
}

// Pushed (Separate-mode) IMU above 100 Hz keeps the uplink busy, so host
// writes overlap uplink frames often. Most shipped grippers keep the CH342
// pull-down on the control UART and lose the overlapped frame; only boards with
// that resistor removed tolerate it. The revision is not visible on the wire,
// so warn instead of refusing. start_exchange() never overlaps and works on both.
inline void warn_if_pushed_high_rate(const char* who, unsigned imu_hz) {
    if (imu_hz <= 100) return;
    logger()->warn(
        "{}: pushed IMU stream above 100 Hz ({} Hz). On grippers that keep the "
        "CH342 pull-down resistor (most shipped units) host commands overlapping "
        "the uplink corrupt frames; use start_exchange() unless this unit has the "
        "resistor removed.",
        who, imu_hz);
}

// Keep the leader default capped at 200 Hz: each unchanged 0x5E reply has
// 504 bytes for records, enough for ten 44-byte IMUs plus their record headers.
// 200 exchanges/s therefore has headroom for 1125 IMUs/s and 100 encoders/s.
inline unsigned leader_default_exchange_hz(unsigned imu_hz, unsigned encoder_hz) {
    const unsigned hz = imu_hz > encoder_hz ? imu_hz : encoder_hz;
    return hz < 1 ? 1 : (hz > 200 ? 200 : hz);
}

// 0x5D carries at most three IMUs. High rates reserve 25% of that capacity
// for jitter/backlog; preserve the old default at <=100 Hz.
inline unsigned follower_default_exchange_hz(unsigned motor_hz, unsigned imu_hz) {
    if (imu_hz <= 100) return std::max({motor_hz, imu_hz, 1u});
    return std::min(500u, std::max(motor_hz, (imu_hz * 4u + 8u) / 9u));
}

}  // namespace xense::taccap::detail

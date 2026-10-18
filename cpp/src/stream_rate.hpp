// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Private helper shared by leader_gripper.cpp and follower_gripper.cpp — NOT
// part of the installed public headers.
//
// A faithful host-side model of the firmware's stream scheduler, so
// start_streaming() can tell the caller what the firmware will *actually*
// emit instead of letting them measure it in production.
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

namespace xense::taccap::detail {

// STREAM_CYCLE_MS = 1 → a 1 kHz base tick.
constexpr unsigned kStreamTickHz   = 1000;
// STREAM_MOTOR_MAX_RATE_HZ.
constexpr unsigned kMotorMaxRateHz = 100;
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

}  // namespace xense::taccap::detail

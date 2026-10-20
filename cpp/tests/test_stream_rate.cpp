// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Pins the host-side model of the firmware's stream scheduler against the
// real thing: third_party/firmware/tc-gu-01, App/tasks/task_data_stream.c
// (hw_v1.1.0 @ bf0a06e). Every expectation below is a line of that file, not
// a guess — if the firmware scheduler changes, these should fail loudly
// rather than have start_streaming() quietly lie about the rate again.

#include <gtest/gtest.h>

#include "stream_rate.hpp"

#include <taccap/protocol/payloads.hpp>

namespace det = xense::taccap::detail;
namespace tp  = xense::taccap::protocol;

// Firmware: div = 1000 / rate, integer. Only divisors of 1000 are exact.
TEST(StreamRate, ExactForDivisorsOfTheTick) {
    for (unsigned hz : {1u, 2u, 4u, 5u, 8u, 10u, 20u, 25u, 50u, 100u,
                        125u, 200u, 250u, 500u, 1000u}) {
        EXPECT_EQ(det::effective_stream_rate_hz(hz, 0), hz) << "hz=" << hz;
    }
}

// The truncation users actually trip over: asking for a non-divisor gets you
// something *faster*, not slower, and nothing tells you.
// effective = 1000 / (1000 / requested), which is always >= requested.
TEST(StreamRate, NonDivisorsRoundUpToTheNextDivider) {
    EXPECT_EQ(det::effective_stream_rate_hz(300, 0), 333u);   // div 3
    EXPECT_EQ(det::effective_stream_rate_hz(150, 0), 166u);   // div 6
    EXPECT_EQ(det::effective_stream_rate_hz(400, 0), 500u);   // div 2
    EXPECT_EQ(det::effective_stream_rate_hz(60,  0), 62u);    // div 16
}

// div = 1000/rate is 0 above the tick, and the firmware rewrites 0 to 10 —
// so overshooting the tick lands you at 100 Hz, not at the ceiling.
TEST(StreamRate, AboveTheTickCollapsesToOneHundred) {
    EXPECT_EQ(det::effective_stream_rate_hz(1001, 0), 100u);
    EXPECT_EQ(det::effective_stream_rate_hz(2000, 0), 100u);
    EXPECT_EQ(det::effective_stream_rate_hz(65535, 0), 100u);
}

// The reason start_streaming() clears mask bits instead of zeroing rates:
// rate 0 is NOT "off" to the firmware, it is the same 100 Hz fallback.
TEST(StreamRate, ZeroRateIsOneHundredHzNotOff) {
    EXPECT_EQ(det::effective_stream_rate_hz(0, 0), 100u);
    EXPECT_EQ(det::effective_stream_rate_hz(0, det::kMotorMaxRateHz), 100u);
}

// STREAM_MOTOR_MAX_RATE_HZ = 100, "leave bandwidth for the control channel".
TEST(StreamRate, MotorStatusIsCappedAtOneHundred) {
    EXPECT_EQ(det::kMotorMaxRateHz, 100u);
    EXPECT_EQ(det::effective_stream_rate_hz(100, det::kMotorMaxRateHz), 100u);
    EXPECT_EQ(det::effective_stream_rate_hz(200, det::kMotorMaxRateHz), 100u)
        << "a 200Hz motor stream request is silently served at 100Hz";
    EXPECT_EQ(det::effective_stream_rate_hz(500, det::kMotorMaxRateHz), 100u);
    // Below the cap the ordinary divider rules still apply.
    EXPECT_EQ(det::effective_stream_rate_hz(50, det::kMotorMaxRateHz), 50u);
    EXPECT_EQ(det::effective_stream_rate_hz(80, det::kMotorMaxRateHz), 83u);
    // Not every non-divisor moves: 1000/30 = 33 and 1000/33 = 30 again, so
    // 30 Hz is a fixed point of the truncation. The rule is
    // effective = 1000 / (1000 / requested), which is >= requested, not
    // "always different".
    EXPECT_EQ(det::effective_stream_rate_hz(30, det::kMotorMaxRateHz), 30u);
}

// ---- source mask ----------------------------------------------------------

TEST(StreamRate, ZeroRateClearsTheSourceBit) {
    EXPECT_EQ(det::stream_source_mask(0, 200, 100),
              tp::StreamSrc::Encoder | tp::StreamSrc::MotorStatus)
        << "imu_hz=0 must drop the IMU bit, not stream it at 100Hz";
    EXPECT_EQ(det::stream_source_mask(100, 0, 0), tp::StreamSrc::Imu);
    EXPECT_EQ(det::stream_source_mask(0, 0, 100), tp::StreamSrc::MotorStatus);
}

TEST(StreamRate, AllSourcesEnabledSetsAllThreeBits) {
    EXPECT_EQ(det::stream_source_mask(100, 100, 100),
              tp::StreamSrc::Imu | tp::StreamSrc::Encoder |
              tp::StreamSrc::MotorStatus);
}

// start_streaming() turns this into a thrown IoError rather than handing the
// firmware a config that starts a stream carrying nothing.
TEST(StreamRate, AllZeroYieldsAnEmptyMask) {
    EXPECT_EQ(det::stream_source_mask(0, 0, 0), 0u);
}

// Eskin is never enabled by the gripper wrappers; the mask must not grow a
// bit nobody asked for.
TEST(StreamRate, NeverEnablesEskin) {
    const uint16_t all = det::stream_source_mask(100, 100, 100);
    EXPECT_EQ(all & (tp::StreamSrc::Eskin1 | tp::StreamSrc::Eskin2), 0u);
}

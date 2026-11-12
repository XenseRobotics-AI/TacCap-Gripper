#include "fake_follower.hpp"
#include "fake_leader.hpp"
#include "stream_rate.hpp"
#include "leader_exchange.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <taccap/error.hpp>
#include <taccap/leader_gripper.hpp>

#include <mutex>
#include <sstream>
#include <vector>

namespace tx = xense::taccap;
namespace tp = tx::protocol;
using namespace taccap_test;

namespace {
std::unique_ptr<tx::LeaderGripper> leader(const Pty& pty) {
    tx::LeaderGripper::Config c;
    c.mcu_device = pty.slave_path();
    c.ack_timeout_ms = 300;
    c.max_retries = 0;
    return std::make_unique<tx::LeaderGripper>(c);
}

// Only attach before starting any worker; remove after closing the gripper.
struct Warnings {
    std::ostringstream text;
    std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink =
        std::make_shared<spdlog::sinks::ostream_sink_mt>(text);
    Warnings() { sink->set_level(spdlog::level::warn); tx::logger()->sinks().push_back(sink); }
    ~Warnings() {
        auto& sinks = tx::logger()->sinks();
        sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
    }
};
}

TEST(ImuHighRate, FollowerStartStreamPreservesWireRatesAndMotorCap) {
    for (unsigned hz : {101u, 500u, 1000u, 1125u}) {
        Pty pty; FakeFollower fw(pty); fw.set_imu_v2_supported(true);
        auto g = open_follower(pty);
        g->start_streaming(100, hz);
        const auto c = fw.stream_config();
        EXPECT_EQ(c.imu_rate, hz);
        EXPECT_EQ(c.motor_rate, 100u);
        EXPECT_EQ(c.source_mask, tp::StreamSrc::ImuV2 | tp::StreamSrc::MotorStatus);
        EXPECT_EQ(c.mode, static_cast<uint8_t>(tp::StreamMode::Separate));
        EXPECT_EQ(tp::encode(c).size(), 12u);
        g->close();
    }
}

TEST(ImuHighRate, FollowerDefaultExchangeDrainsThreeSampleV2BatchesInOrder) {
    Pty pty; FakeFollower fw(pty); fw.set_imu_v2_supported(true);
    fw.set_exchange_imu_count(3);
    auto g = open_follower(pty);
    std::mutex mu;
    std::vector<tx::ImuSample> samples;
    g->imu().on_data([&](const tx::ImuSample& s) { std::lock_guard<std::mutex> lock(mu); samples.push_back(s); });
    g->start_exchange(100, 1125);
    EXPECT_TRUE(wait_for([&] { return g->exchange_stats().imu_samples >= 30; }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(fw.stream_config().imu_rate, 1125u);
    EXPECT_EQ(fw.stream_config().motor_rate, 100u);
    g->close();
    std::lock_guard<std::mutex> lock(mu);
    ASSERT_GE(samples.size(), 3u);
    for (std::size_t i = 1; i < samples.size(); ++i) {
        EXPECT_EQ(static_cast<uint16_t>(samples[i].seq - samples[i - 1].seq), 1u);
        EXPECT_EQ(samples[i].mcu_timestamp_us - samples[i - 1].mcu_timestamp_us, 10000u);
        EXPECT_TRUE(samples[i].timestamp_is_64bit);
        EXPECT_NEAR(samples[i].accel_mps2[2], 9.80665f, .001f);
    }
}

TEST(ImuHighRate, LeaderBatchPreservesLegacyAndV2PayloadsAndCaptureTimes) {
    // Existing 0x5E records: three 44-byte IMUs with distinct capture times,
    // followed by one original 28-byte IMU. No record format extensions.
    std::vector<uint8_t> wire{1, tp::LeaderExchangeRespFlag::Polled, 4, 0, 0, 0, 0, 0};
    for (unsigned i = 0; i < 4; ++i) {
        const bool v2 = i < 3;
        tp::ImuData raw{};
        raw.accel_z = 1000; raw.gyro_x = 100; raw.seq = 40 + i;
        raw.timestamp_us = 2000 + 889 * i;
        wire.push_back(static_cast<uint8_t>(v2 ? tp::Cmd::GetImuV2 : tp::Cmd::GetImu));
        wire.push_back(v2 ? 44 : 28);
        const auto* bytes = reinterpret_cast<const uint8_t*>(&raw);
        wire.insert(wire.end(), bytes, bytes + sizeof(raw));
        if (v2) for (uint64_t value : {UINT64_C(0x200000000) + raw.timestamp_us, UINT64_C(123)})
            for (unsigned b = 0; b < 8; ++b) wire.push_back(static_cast<uint8_t>(value >> (8 * b)));
    }
    tp::LeaderExchangeResponseHeader header{};
    std::vector<tx::bus::Frame> frames;
    ASSERT_TRUE(tx::detail::LeaderExchange::decode(wire, header, frames));
    ASSERT_EQ(frames.size(), 4u);
    for (unsigned i = 0; i < frames.size(); ++i) {
        auto s = tx::IMU::decode(frames[i].payload.data(), frames[i].payload.size());
        EXPECT_EQ(s.seq, 40 + i);
        EXPECT_EQ(s.mcu_timestamp_us, (i < 3 ? UINT64_C(0x200000000) : 0) + 2000 + 889 * i);
        EXPECT_EQ(s.timestamp_is_64bit, i < 3);
        EXPECT_NEAR(s.accel_mps2[2], 9.80665f, .001f);
        EXPECT_NEAR(s.gyro_radps[0], 0.0174532925f, .000001f);
    }
}

TEST(ImuHighRate, FollowerExplicitExchangeHonorsThreeSampleCapacity) {
    for (const auto pair : {std::pair<unsigned,unsigned>{500, 167}, {1000, 334}, {1125, 375}}) {
        Pty pty; FakeFollower fw(pty); fw.set_imu_supported(true);
        auto g = open_follower(pty);
        EXPECT_THROW(g->start_exchange(100, pair.first, pair.second - 1), tx::IoError);
        EXPECT_EQ(fw.imu_queries(), 0u);
        EXPECT_EQ(fw.stream_start_attempts(), 0u);
        g->start_exchange(100, pair.first, pair.second);
        EXPECT_TRUE(g->is_exchanging());
        g->close();
    }
}

TEST(ImuHighRate, FollowerHighRateNackIsVisibleWithoutFallback) {
    for (bool exchange : {false, true}) {
        Pty pty; FakeFollower fw(pty); fw.set_imu_supported(true); fw.set_max_imu_rate(100);
        auto g = open_follower(pty);
        if (exchange) EXPECT_THROW(g->start_exchange(100, 1125), tx::NackError);
        else EXPECT_THROW(g->start_streaming(100, 1125), tx::NackError);
        EXPECT_EQ(fw.stream_start_attempts(), 1u);
        EXPECT_FALSE(g->is_streaming());
        EXPECT_FALSE(g->is_exchanging());
        EXPECT_FALSE(g->transport().half_duplex());
    }
}

TEST(ImuHighRate, FollowerRejectsMoreThanThreeImuSamplesPerReply) {
    Pty pty; FakeFollower fw(pty); fw.set_imu_v2_supported(true); fw.set_exchange_imu_count(4);
    auto g = open_follower(pty);
    std::atomic<unsigned> callbacks{0};
    g->imu().on_data([&](const auto&) { ++callbacks; });
    g->start_exchange(100, 100);
    EXPECT_TRUE(wait_for([&] { return g->exchange_stats().malformed >= 2; }, std::chrono::milliseconds(300)));
    g->close();
    EXPECT_EQ(callbacks.load(), 0u);
}

TEST(ImuHighRate, LeaderRejectsOutOfRangeBeforeChangingStream) {
    Pty pty; FakeLeader fw(pty); auto g = leader(pty);
    EXPECT_THROW(g->start_streaming(1126, 100), tx::IoError);
    EXPECT_THROW(g->start_exchange(65537, 100), tx::IoError);
    EXPECT_EQ(fw.stream_start_attempts(), 0u);
    g->close();
}

TEST(ImuHighRate, LeaderHighRateNackDoesNotRetryAsPushedStream) {
    Pty pty; FakeLeader fw(pty); fw.set_max_imu_rate(100); auto g = leader(pty);
    EXPECT_THROW(g->start_exchange(1125, 100), tx::NackError);
    EXPECT_EQ(fw.stream_start_attempts(), 1u);
    EXPECT_FALSE(g->is_streaming());
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(g->transport().half_duplex());
    g->close();
}

TEST(ImuHighRate, LeaderAcceptsHighRateWithoutLegacyDividerWarning) {
    for (unsigned hz : {150u, 300u, 500u, 1000u, 1001u, 1125u}) {
        Warnings warnings;
        Pty pty; FakeLeader fw(pty); auto g = leader(pty);
        EXPECT_TRUE(g->start_exchange(hz, 100));
        EXPECT_EQ(fw.imu_rate(), hz);
        g->close();
        EXPECT_EQ(warnings.text.str().find("IMU stream requested"), std::string::npos);
    }
}

// Shipped grippers mostly keep the CH342 pull-down on the control UART: a host
// write overlapping a pushed frame corrupts it. The SDK cannot detect the board
// revision, so pushed high-rate streaming warns and points to start_exchange().
TEST(ImuHighRate, PushedHighRateWarnsAboutLinkOverlapOnlyWhenPushed) {
    const std::string marker = "pushed IMU stream above 100 Hz";
    {
        Warnings w;
        Pty pty; FakeLeader fw(pty); auto g = leader(pty);
        g->start_streaming(1125, 100);
        g->close();
        EXPECT_NE(w.text.str().find(marker), std::string::npos);
        EXPECT_NE(w.text.str().find("start_exchange"), std::string::npos);
    }
    {
        Warnings w;
        Pty pty; FakeFollower fw(pty); fw.set_imu_v2_supported(true);
        auto g = open_follower(pty);
        g->start_streaming(100, 1125);
        g->close();
        EXPECT_NE(w.text.str().find(marker), std::string::npos);
    }
    {
        Warnings w;
        Pty pty; FakeLeader fw(pty); auto g = leader(pty);
        g->start_streaming(100, 100);
        g->close();
        Pty pty2; FakeLeader fw2(pty2); auto g2 = leader(pty2);
        EXPECT_TRUE(g2->start_exchange(1125, 100));
        g2->close();
        Pty pty3; FakeFollower fw3(pty3); fw3.set_imu_v2_supported(true);
        auto g3 = open_follower(pty3);
        g3->start_exchange(100, 1125);
        g3->close();
        EXPECT_EQ(w.text.str().find(marker), std::string::npos);
    }
}

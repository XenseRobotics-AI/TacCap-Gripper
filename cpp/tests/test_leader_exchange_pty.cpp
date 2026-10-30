// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// LeaderGripper::start_exchange() against the fake leader: polled mode with
// IMU/encoder/key fan-out to the ordinary subscribers, fallback on firmware
// without 0x5E, backlog draining, a lost reply replayed by same-seq retry
// rather than lost, strict one-request-at-a-time with concurrent queries,
// malformed replies, and stop.

#include "fake_leader.hpp"

#include <taccap/error.hpp>
#include <taccap/leader_gripper.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::FakeLeader;
using taccap_test::Pty;

namespace {

constexpr auto kSecond = std::chrono::milliseconds(1000);

std::unique_ptr<tx::LeaderGripper> open_leader(const Pty& pty) {
    tx::LeaderGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 2;
    return std::make_unique<tx::LeaderGripper>(cfg);
}

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

// Collects delivered sequence numbers so a test can assert "nothing lost".
struct SeqLog {
    std::mutex mu;
    std::vector<uint16_t> seqs;
    void add(uint16_t s) { std::lock_guard<std::mutex> lk(mu); seqs.push_back(s); }
    std::size_t size() { std::lock_guard<std::mutex> lk(mu); return seqs.size(); }
    // Count of missing sequence numbers between first and last delivered.
    unsigned gaps() {
        std::lock_guard<std::mutex> lk(mu);
        unsigned missing = 0;
        for (std::size_t i = 1; i < seqs.size(); ++i) {
            const uint16_t d = static_cast<uint16_t>(seqs[i] - seqs[i - 1]);
            if (d != 1) missing += d == 0 ? 1u : d - 1u;
        }
        return missing;
    }
};

}  // namespace

TEST(LeaderExchangePty, PolledModeFeedsImuEncoderAndKeySubscribersWithoutPush) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);

    SeqLog imu, enc;
    std::atomic<unsigned> keys{0};
    std::atomic<int> last_key{-1};
    g->imu().on_data([&](const tx::ImuSample& s) { imu.add(s.seq); });
    g->encoder().on_data([&](const tx::EncoderSample& s) { enc.add(s.seq); });
    g->key().on_event([&](const tx::KeySample& k) { ++keys; last_key.store(k.key_state); });

    ASSERT_TRUE(g->start_exchange(100, 100));
    EXPECT_TRUE(g->is_exchanging());
    EXPECT_TRUE(g->is_streaming());
    EXPECT_TRUE(fw.polled());
    EXPECT_EQ(fw.stream_mode(), static_cast<uint8_t>(tp::StreamMode::Polled));
    ASSERT_TRUE(wait_for([&] { return imu.size() >= 30 && enc.size() >= 30; }, 2 * kSecond))
        << "imu=" << imu.size() << " enc=" << enc.size();

    fw.press_key(tp::KeyState::DoubleClick);
    ASSERT_TRUE(wait_for([&] { return keys.load() == 1; }, kSecond));
    EXPECT_EQ(last_key.load(), tp::KeyState::DoubleClick);

    EXPECT_EQ(fw.data_frames_pushed(), 0u) << "the leader pushed a DATA frame during the exchange";
    EXPECT_EQ(imu.gaps(), 0u);
    EXPECT_EQ(enc.gaps(), 0u);
    const auto s = g->exchange_stats();
    EXPECT_GE(s.exchanges, 20u);
    EXPECT_EQ(s.timeouts, 0u);
    EXPECT_EQ(s.malformed, 0u);
    EXPECT_EQ(s.not_polled, 0u);
    EXPECT_EQ(s.key_events, 1u);
    EXPECT_EQ(s.records, s.imu_samples + s.encoder_samples + s.key_events + s.sensor_errors + s.other_records);
    EXPECT_EQ(s.records_dropped, 0u);

    g->stop_streaming();
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(g->is_streaming());
    EXPECT_FALSE(fw.polled());
    EXPECT_FALSE(fw.streaming());
    const unsigned after = fw.exchanges();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(fw.exchanges(), after) << "exchange loop kept running after stop_streaming";
}

TEST(LeaderExchangePty, OldFirmwareFallsBackToTheOriginalStream) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    fw.set_exchange_supported(false);
    auto g = open_leader(pty);

    std::atomic<unsigned> imu{0};
    g->imu().on_data([&](const tx::ImuSample&) { ++imu; });
    EXPECT_FALSE(g->start_exchange(100, 100));
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_TRUE(g->is_streaming());
    EXPECT_EQ(fw.stream_mode(), static_cast<uint8_t>(tp::StreamMode::Separate));
    EXPECT_TRUE(wait_for([&] { return imu.load() >= 10; }, kSecond));
    EXPECT_GT(fw.data_frames_pushed(), 0u);
    EXPECT_EQ(fw.exchanges(), 0u);
    g->stop_streaming();
    EXPECT_FALSE(fw.streaming());
}

TEST(LeaderExchangePty, OldFirmwareWithoutFallbackThrowsAndLeavesNothingRunning) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    fw.set_exchange_supported(false);
    auto g = open_leader(pty);

    EXPECT_THROW(g->start_exchange(100, 100, 0, false), tx::ProtocolError);
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(g->is_streaming());
    EXPECT_FALSE(fw.streaming());
    // The link is still usable for ordinary requests.
    EXPECT_NO_THROW(g->transport().send_cmd(tp::Cmd::Heartbeat, {}, std::chrono::milliseconds(200)));
}

TEST(LeaderExchangePty, RejectsBadArgumentsBeforeTouchingTheDevice) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);
    EXPECT_THROW(g->start_exchange(0, 0), tx::IoError);
    EXPECT_THROW(g->start_exchange(100, 100, 501), tx::IoError);
    EXPECT_FALSE(fw.streaming());
    g->start_streaming(100, 100);
    EXPECT_THROW(g->start_exchange(100, 100), tx::IoError) << "must stop streaming first";
    g->stop_streaming();
}

TEST(LeaderExchangePty, BacklogIsFetchedAtOnceWithoutLoss) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);

    SeqLog imu;
    g->imu().on_data([&](const tx::ImuSample& s) { imu.add(s.seq); });
    // 100 Hz samples, 5 Hz exchanges: ~40 records (>504 B) per period.
    ASSERT_TRUE(g->start_exchange(100, 100, 5));
    ASSERT_TRUE(wait_for([&] { return imu.size() >= 120; }, 3 * kSecond)) << imu.size();
    const auto s = g->exchange_stats();
    EXPECT_GT(s.continuation_exchanges, 0u);
    EXPECT_EQ(s.records_dropped, 0u);
    EXPECT_EQ(imu.gaps(), 0u);
    g->stop_streaming();
}

TEST(LeaderExchangePty, LostReplyIsReplayedBySameSeqRetryNotLost) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);

    SeqLog imu;
    g->imu().on_data([&](const tx::ImuSample& s) { imu.add(s.seq); });
    ASSERT_TRUE(g->start_exchange(100, 100, 50));
    ASSERT_TRUE(wait_for([&] { return imu.size() >= 10; }, kSecond));

    const auto timeouts_before = g->transport().stats().ack_timeouts;
    fw.drop_exchange_replies(1);
    ASSERT_TRUE(wait_for([&] { return fw.replays() >= 1; }, kSecond));
    const std::size_t at_replay = imu.size();
    ASSERT_TRUE(wait_for([&] { return imu.size() >= at_replay + 20; }, kSecond));
    EXPECT_GT(g->transport().stats().ack_timeouts, timeouts_before);
    EXPECT_EQ(g->exchange_stats().timeouts, 0u) << "the retry recovered the reply";
    EXPECT_EQ(imu.gaps(), 0u) << "records from the lost reply were lost";
    g->stop_streaming();
}

TEST(LeaderExchangePty, NeverWritesWhileAReplyIsPendingEvenWithConcurrentQueries) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);
    fw.set_exchange_reply_delay_ms(3);
    ASSERT_TRUE(g->start_exchange(100, 100));

    std::atomic<bool> stop{false};
    std::atomic<unsigned> heartbeats{0}, heartbeat_timeouts{0};
    std::thread user([&] {
        while (!stop.load()) {
            try { g->transport().send_cmd(tp::Cmd::Heartbeat, {}, std::chrono::milliseconds(200)); ++heartbeats; }
            catch (...) { ++heartbeat_timeouts; }
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stop.store(true);
    user.join();

    EXPECT_GT(fw.exchanges(), 15u);
    EXPECT_GT(heartbeats.load(), 5u);
    // Only after giving up on a reply may the host write over it.
    const auto gave_up = g->transport().stats().ack_timeouts;
    EXPECT_LE(fw.overlaps(), gave_up) << "host wrote a frame while an exchange reply was pending";
    g->stop_streaming();
}

TEST(LeaderExchangePty, MalformedReplyIsCountedAndDispatchesNothing) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);
    std::atomic<unsigned> imu{0};
    g->imu().on_data([&](const tx::ImuSample&) { ++imu; });
    ASSERT_TRUE(g->start_exchange(100, 100, 20));
    ASSERT_TRUE(wait_for([&] { return imu.load() >= 5; }, kSecond));
    fw.corrupt_next_reply();
    ASSERT_TRUE(wait_for([&] { return g->exchange_stats().malformed >= 1; }, kSecond));
    EXPECT_TRUE(wait_for([&] { return imu.load() >= 30; }, kSecond)) << "exchange did not continue";
    g->stop_streaming();
}

TEST(LeaderExchangePty, CloseEndsTheExchange) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeLeader fw(pty);
    auto g = open_leader(pty);
    ASSERT_TRUE(g->start_exchange(100, 100));
    ASSERT_TRUE(wait_for([&] { return fw.exchanges() >= 3; }, kSecond));
    g->close();
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(fw.polled());
}

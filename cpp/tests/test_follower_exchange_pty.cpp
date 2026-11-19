// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// FollowerGripper::start_exchange() against the fake follower: polled mode,
// telemetry fan-out to the ordinary subscribers, targets riding on exchanges
// instead of no-ACK frames, strict one-request-at-a-time on the link (with
// concurrent user queries), timeout resend, rejection reporting, and a
// ForcePositionController running unmodified on top.

#include "fake_follower.hpp"

#include <taccap/error.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/force_position_controller.hpp>

#include <atomic>
#include <chrono>
#include <thread>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;
using taccap_test::wait_for;

namespace {
constexpr auto kSecond = std::chrono::milliseconds(1000);
}

TEST(FollowerExchangePty, PolledModeFeedsStatusAndImuSubscribersWithoutStreaming) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto g = open_follower(pty);

    std::atomic<unsigned> status{0}, imu{0};
    g->motor().on_status([&](const tx::MotorStatusSample&) { ++status; });
    g->imu().on_data([&](const tx::ImuSample&) { ++imu; });
    g->start_exchange(100, 100);

    EXPECT_TRUE(g->is_exchanging());
    EXPECT_TRUE(g->is_streaming());
    EXPECT_TRUE(fw.polled());
    EXPECT_EQ(fw.stream_config().mode, static_cast<uint8_t>(tp::StreamMode::Polled));
    ASSERT_TRUE(wait_for([&] { return status.load() >= 20 && imu.load() >= 20; }, 2 * kSecond))
        << "status=" << status << " imu=" << imu;
    const auto s = g->exchange_stats();
    EXPECT_GE(s.telemetry_exchanges, 20u);
    EXPECT_EQ(s.timeouts, 0u);
    EXPECT_EQ(s.malformed, 0u);
    EXPECT_EQ(s.motor_samples, s.telemetry_exchanges);

    g->stop_streaming();
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(fw.polled());
    const unsigned after = fw.exchanges();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(fw.exchanges(), after) << "exchange loop kept running after stop_streaming";
}

TEST(FollowerExchangePty, SubmitRidesOnAnExchangeAndGoesOutPromptly) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->start_exchange(10);   // slow telemetry: a target must not wait for the tick

    ASSERT_TRUE(wait_for([&] { return fw.telemetry_exchanges() >= 1; }, kSecond));
    const unsigned targets = fw.exchange_targets();
    const auto t0 = std::chrono::steady_clock::now();
    g->motor().submit_impedance(0.25f, 2.0f, 0.1f, 0.3f, 0.0f);
    ASSERT_TRUE(wait_for([&] { return fw.exchange_targets() > targets; }, kSecond));
    const auto latency = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(latency, std::chrono::milliseconds(30)) << "target waited for a telemetry tick";
    EXPECT_EQ(fw.submit_count(), 0u) << "no-ACK 0x43 frame written during exchange";
    const auto last = fw.last_submit();
    ASSERT_TRUE(last.has_value());
    EXPECT_FLOAT_EQ(last->target_pos, 0.25f);
    EXPECT_FLOAT_EQ(last->kp, 2.0f);
    EXPECT_FLOAT_EQ(last->target_torque, 0.3f);
    // The fake records the target before the host has processed its reply.
    EXPECT_TRUE(wait_for([&] { return g->exchange_stats().last_target_result == 0; }, kSecond));

    g->stop_streaming();
    g->motor().submit_impedance(0.1f, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(wait_for([&] { return fw.submit_count() == 1; }, kSecond))
        << "after stop the target must go out as the ordinary no-ACK frame";
}

TEST(FollowerExchangePty, NeverWritesWhileAReplyIsPendingEvenWithConcurrentQueries) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    fw.set_exchange_reply_delay_ms(3);
    g->start_exchange(100);

    std::atomic<bool> stop{false};
    std::atomic<unsigned> heartbeats{0}, heartbeat_timeouts{0};
    // 1 kHz of user queries, as in the leader test: an unthrottled loop starves
    // the exchange on the (unfair) half-duplex turn lock, a documented limit.
    std::thread user([&] {
        while (!stop.load()) {
            try { g->transport().send_cmd(tp::Cmd::Heartbeat, {}, std::chrono::milliseconds(200)); ++heartbeats; }
            catch (...) { ++heartbeat_timeouts; }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::thread submitter([&] {
        for (int i = 0; i < 200 && !stop.load(); ++i) {
            g->motor().submit_impedance(0.01f * i, 0, 0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stop.store(true);
    submitter.join();
    user.join();

    EXPECT_GT(fw.exchanges(), 15u);
    EXPECT_GT(heartbeats.load(), 5u);
    EXPECT_GT(fw.exchange_targets(), 5u);
    // The host may only write over a pending reply after giving up on it. On
    // an unloaded machine that never happens; a starved test runner can delay
    // the fake past the timeout, and then each overlap must be a timeout.
    const auto gave_up = g->exchange_stats().timeouts + heartbeat_timeouts.load();
    EXPECT_LE(fw.overlaps(), gave_up) << "host wrote a frame while an exchange reply was pending";
    g->stop_streaming();
}

TEST(FollowerExchangePty, TimedOutTargetIsResentUnlessSuperseded) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->start_exchange(20);
    ASSERT_TRUE(wait_for([&] { return fw.telemetry_exchanges() >= 1; }, kSecond));

    // Two lost requests exhaust the same-seq retry (open_follower: max_retries=1),
    // so the exchange times out and the loop resends the target itself.
    fw.drop_exchange_replies(2);
    const unsigned targets = fw.exchange_targets();
    g->motor().submit_impedance(0.5f, 1.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(wait_for([&] { return fw.exchange_targets() >= targets + 2; }, kSecond))
        << "dropped target was not resent";
    ASSERT_TRUE(wait_for([&] { return g->exchange_stats().targets_resent >= 1; }, kSecond));
    const auto s = g->exchange_stats();
    EXPECT_GE(s.timeouts, 1u);
    ASSERT_TRUE(fw.last_submit().has_value());
    EXPECT_FLOAT_EQ(fw.last_submit()->target_pos, 0.5f);
    g->stop_streaming();
}

TEST(FollowerExchangePty, LostReplyIsReplayedBySameSeqRetryNotLost) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto g = open_follower(pty);
    std::mutex m; std::vector<uint16_t> seqs;
    g->imu().on_data([&](const tx::ImuSample& s) { std::lock_guard<std::mutex> lk(m); seqs.push_back(s.seq); });
    g->start_exchange(100, 100, 50);
    ASSERT_TRUE(wait_for([&] { std::lock_guard<std::mutex> lk(m); return seqs.size() >= 10; }, kSecond));

    const auto acks_before = g->transport().stats().ack_timeouts;
    fw.lose_exchange_replies(1);   // telemetry built (IMU consumed) but lost on the wire
    ASSERT_TRUE(wait_for([&] { return fw.exchange_replays() >= 1; }, kSecond)) << "no same-seq retry";
    std::size_t at;
    { std::lock_guard<std::mutex> lk(m); at = seqs.size(); }
    ASSERT_TRUE(wait_for([&] { std::lock_guard<std::mutex> lk(m); return seqs.size() >= at + 20; }, kSecond));
    EXPECT_GT(g->transport().stats().ack_timeouts, acks_before);
    EXPECT_EQ(g->exchange_stats().timeouts, 0u) << "the retry recovered the reply";
    std::lock_guard<std::mutex> lk(m);
    unsigned missing = 0;
    for (std::size_t i = 1; i < seqs.size(); ++i) {
        const uint16_t d = static_cast<uint16_t>(seqs[i] - seqs[i - 1]);
        if (d != 1) missing += d == 0 ? 1u : d - 1u;
    }
    EXPECT_EQ(missing, 0u) << "IMU samples of the lost reply were lost";
    g->stop_streaming();
}

TEST(FollowerExchangePty, RejectedTargetIsCountedAndTelemetryContinues) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    std::atomic<unsigned> status{0};
    g->motor().on_status([&](const tx::MotorStatusSample&) { ++status; });
    g->start_exchange(100);

    fw.reject_next_exchange_target(static_cast<uint8_t>(tp::ErrorCode::SysBusy));
    g->motor().submit_impedance(0.2f, 0, 0, 0, 0);
    ASSERT_TRUE(wait_for([&] { return g->exchange_stats().targets_rejected == 1; }, kSecond));
    EXPECT_EQ(g->exchange_stats().last_target_result, static_cast<uint8_t>(tp::ErrorCode::SysBusy));
    const unsigned before = status.load();
    EXPECT_TRUE(wait_for([&] { return status.load() >= before + 5; }, kSecond));
    g->stop_streaming();
}

TEST(FollowerExchangePty, OldFirmwareNacksAndNothingIsLeftRunning) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_exchange_supported(false);
    auto g = open_follower(pty);
    EXPECT_THROW(g->start_exchange(100), tx::ProtocolError);
    EXPECT_FALSE(g->is_exchanging());
    EXPECT_FALSE(g->is_streaming());
    EXPECT_FALSE(g->transport().half_duplex());
    g->motor().submit_impedance(0.1f, 0, 0, 0, 0);
    EXPECT_TRUE(wait_for([&] { return fw.submit_count() == 1; }, kSecond));
}

TEST(FollowerExchangePty, ArgumentsAndStateAreValidated) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    EXPECT_THROW(g->start_exchange(0, 0), tx::IoError);
    EXPECT_THROW(g->start_exchange(101), tx::IoError);
    EXPECT_THROW(g->start_exchange(100, 1126), tx::IoError);
    EXPECT_THROW(g->start_exchange(100, 0, 501), tx::IoError);
    g->start_streaming(100);
    EXPECT_THROW(g->start_exchange(100), tx::IoError);
    g->stop_streaming();
    g->start_exchange(100);
    EXPECT_TRUE(g->transport().half_duplex());
    g->close();
    EXPECT_FALSE(g->is_exchanging());
}

TEST(FollowerExchangePty, ForcePositionControllerRunsUnmodifiedOverTheExchange) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_exchange_reply_delay_ms(1);
    auto g = open_follower(pty);
    g->start_exchange(100);

    tx::ForcePositionConfig cfg;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(c.running());
    ASSERT_TRUE(wait_for([&] { return fw.exchange_targets() >= 20; }, 2 * kSecond))
        << "controller targets via exchange: " << fw.exchange_targets();
    EXPECT_EQ(fw.submit_count(), 0u);
    EXPECT_EQ(c.state(), tx::ForcePositionState::HoldingPosition);
    EXPECT_TRUE(c.snapshot().observation.valid);
    EXPECT_LE(fw.overlaps(), g->exchange_stats().timeouts);
    c.stop();
    EXPECT_TRUE(g->is_exchanging()) << "controller must not stop a stream it did not start";
    g->stop_streaming();
}

TEST(FollowerExchangePty, WaitingForTheLinkCountsAgainstTheRequestTimeout) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->transport().set_half_duplex(true);
    fw.drop_exchange_replies(1);   // the first request holds the turn for its full 600 ms

    const std::vector<uint8_t> req{tp::EXCHANGE_VERSION, tp::ExchangeFlag::Telemetry, 0, 0};
    std::thread holder([&] {
        EXPECT_THROW(g->transport().send_cmd(tp::Cmd::SlaveExchange, req, std::chrono::milliseconds(600),
                                             tx::bus::RetryMode::Once),
                     tx::TimeoutError);
    });
    ASSERT_TRUE(wait_for([&] { return fw.exchanges() >= 1; }, kSecond));
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_THROW(g->transport().send_cmd(tp::Cmd::Heartbeat, {}, std::chrono::milliseconds(50)),
                 tx::TimeoutError);
    const auto waited = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(waited, std::chrono::milliseconds(300)) << "query waited behind a long request";
    holder.join();
    // Once the turn is free a query goes through normally.
    EXPECT_NO_THROW(g->transport().send_cmd(tp::Cmd::Heartbeat, {}, std::chrono::milliseconds(200)));
    g->transport().set_half_duplex(false);
}

// High-rate exchange (IMU 1125 Hz -> 500 exchanges/s, 2 ms period): a
// controller that answers every status sample with a target must ride on the
// next telemetry exchange instead of adding a target-only one. Measured on a
// real follower (9s, 2026-11-13) the old behaviour ran 1000 exchanges/s --
// 500 telemetry + 500 target-only -- doubling half-duplex line occupancy.
TEST(FollowerExchangePty, TargetsRideOnTheNextTelemetryExchangeAtHighRate) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto g = open_follower(pty);
    std::atomic<unsigned> n{0};
    g->motor().on_status([&](const tx::MotorStatusSample& s) {
        g->motor().submit_impedance(s.actual_pos + 0.001f * float(++n % 10), 1.0f, 0.1f, 0.0f, 0.0f);
    });
    g->start_exchange(100, 1125);      // default 500 exchanges/s
    ASSERT_TRUE(wait_for([&] { return g->exchange_stats().telemetry_exchanges >= 50; }, kSecond));
    const auto s0 = g->exchange_stats();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const auto s1 = g->exchange_stats();
    g->stop_streaming();
    const auto telem = s1.telemetry_exchanges - s0.telemetry_exchanges;
    const auto all = s1.exchanges - s0.exchanges;
    const auto with_target = s1.target_exchanges - s0.target_exchanges;
    ASSERT_GT(telem, 50u);
    EXPECT_LE(all, telem + telem / 10) << "targets went out as extra target-only exchanges";
    EXPECT_GE(with_target, telem * 8 / 10) << "targets were not carried on telemetry exchanges";
    EXPECT_EQ(s1.targets_rejected, 0u);
}

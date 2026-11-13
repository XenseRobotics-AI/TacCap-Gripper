// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// V2.10 MCU trajectory (0x5D flags bit1) against the fake follower: the
// capability probe, ForcePositionController.mcu_trajectory and
// ImpedanceController.mcu_stream (targets carried with the bit, sent when the
// caller sets them, kept alive while held, host frames only for faults), and
// the fallback to the host law on firmware that does not answer the probe.

#include "fake_follower.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>

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

TEST(McuTrajectoryPty, ProbeReportsTheCapabilityOnlyWhileExchanging) {
    {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty);            // 1.2.18-like: ignores bit1
        auto g = open_follower(pty);
        EXPECT_FALSE(g->supports_mcu_trajectory());
        g->start_exchange(100);
        EXPECT_FALSE(g->supports_mcu_trajectory());
        EXPECT_EQ(fw.mcu_targets(), 0u);
        g->stop_streaming();
    }
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);
    EXPECT_FALSE(g->supports_mcu_trajectory()) << "not exchanging yet";
    g->start_exchange(100);
    EXPECT_TRUE(g->supports_mcu_trajectory());
    EXPECT_EQ(fw.exchange_targets(), 0u) << "the probe carries no target";
    g->stop_streaming();
    EXPECT_FALSE(g->supports_mcu_trajectory());
}

TEST(McuTrajectoryPty, ForcePositionSendsCallerTargetsWithTheBitAndNoHostFrames) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100);

    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    cfg.close_speed_radps = 4.0f;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    EXPECT_EQ(c.backend(), tx::ControlBackend::Mcu);
    // Engaged with a hold where the jaw is (fake reports 0.60 rad).
    ASSERT_TRUE(wait_for([&] { return fw.mcu_targets() >= 1; }, kSecond));
    auto hold = fw.last_mcu_position();
    ASSERT_TRUE(hold.has_value());
    EXPECT_NEAR(hold->target_pos, 0.60f, 2e-3f);

    const unsigned before = fw.mcu_targets();
    const auto t0 = std::chrono::steady_clock::now();
    c.set_target(0.2f, 0.8f);
    ASSERT_TRUE(wait_for([&] {
        auto p = fw.last_mcu_position();
        return fw.mcu_targets() > before && p && std::abs(p->target_pos - g->pos_to_rad(0.2f)) < 1e-5f;
    }, kSecond));
    EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(40))
        << "an MCU target must go out at once, not wait for a status frame";
    const auto p = fw.last_mcu_position();
    EXPECT_FLOAT_EQ(p->max_vel, 4.0f);
    EXPECT_FLOAT_EQ(p->max_torque, 0.8f);

    // Held: no host frames, but the target is kept alive (~every 80 ms).
    const unsigned plain = fw.plain_exchange_targets();
    const auto s0 = g->exchange_stats();
    std::this_thread::sleep_for(std::chrono::milliseconds(450));
    const auto s1 = g->exchange_stats();
    EXPECT_EQ(fw.plain_exchange_targets(), plain) << "host frames sent while the MCU runs the move";
    EXPECT_GE(s1.mcu_keepalives - s0.mcu_keepalives, 3u);
    EXPECT_LE(s1.mcu_keepalives - s0.mcu_keepalives, 7u);
    EXPECT_EQ(s1.mcu_unacknowledged, 0u);
    EXPECT_EQ(fw.submit_count(), 0u);
    EXPECT_EQ(c.snapshot().backend, tx::ControlBackend::Mcu);

    c.release();
    ASSERT_TRUE(wait_for([&] {
        auto r = fw.last_mcu_position();
        return r && std::abs(r->target_pos - g->pos_to_rad(1.0f)) < 1e-5f;
    }, kSecond));
    c.stop();
    // stop() sends a plain zero frame, which ends the MCU trajectory.
    ASSERT_TRUE(wait_for([&] { return fw.plain_exchange_targets() > plain; }, kSecond));
    const auto z = fw.last_plain_impedance();
    ASSERT_TRUE(z.has_value());
    EXPECT_EQ(z->kp, 0.0f);
    EXPECT_EQ(z->kd, 0.0f);
    const auto ka = g->exchange_stats().mcu_keepalives;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_EQ(g->exchange_stats().mcu_keepalives, ka) << "keep-alive outlived a plain target";
    g->stop_streaming();
}

TEST(McuTrajectoryPty, ForcePositionStreamGoesOutSampleBySample) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.mcu_targets() >= 1; }, kSecond));
    const auto s0 = g->exchange_stats();
    for (int i = 0; i < 50; ++i) {        // 100 Hz caller stream
        c.set_target(0.5f + 0.002f * float(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto s1 = g->exchange_stats();
    EXPECT_GE(s1.mcu_targets - s0.mcu_targets, 45u) << "stream samples were merged away";
    EXPECT_EQ(s1.mcu_keepalives, s0.mcu_keepalives) << "keep-alive injected into a live stream";
    auto last = fw.last_mcu_position();
    ASSERT_TRUE(last.has_value());
    EXPECT_NEAR(last->target_pos, g->pos_to_rad(0.5f + 0.002f * 49), 1e-5f);
    c.stop();
    g->stop_streaming();
}

TEST(McuTrajectoryPty, ForcePositionStaleStatusFaultsWithAPlainZeroFrame) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    c.set_target(0.3f);
    ASSERT_TRUE(wait_for([&] { return fw.mcu_targets() >= 2; }, kSecond));
    const unsigned plain = fw.plain_exchange_targets();
    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return c.state() == tx::ForcePositionState::Fault; }, kSecond));
    ASSERT_TRUE(wait_for([&] { return fw.plain_exchange_targets() > plain; }, kSecond));
    const auto z = fw.last_plain_impedance();
    ASSERT_TRUE(z.has_value());
    EXPECT_EQ(z->kp, 0.0f);
    EXPECT_EQ(z->target_torque, 0.0f);
    // A caller target in Fault never reaches the follower as an MCU target.
    const unsigned mcu = fw.mcu_targets();
    const auto ka = g->exchange_stats().mcu_keepalives;
    c.set_target(0.9f);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_EQ(fw.mcu_targets(), mcu);
    EXPECT_EQ(g->exchange_stats().mcu_keepalives, ka);
    fw.freeze_stream(false);
    c.stop();
    g->stop_streaming();
}

TEST(McuTrajectoryPty, ForcePositionFallsBackToTheHostLawOnOldFirmware) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);                 // does not answer the probe
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    EXPECT_EQ(c.backend(), tx::ControlBackend::Host);
    ASSERT_TRUE(wait_for([&] { return fw.plain_exchange_targets() >= 10; }, 2 * kSecond));
    EXPECT_EQ(g->exchange_stats().mcu_targets, 0u);
    c.stop();
    g->stop_streaming();
}

TEST(McuTrajectoryPty, ForcePositionWithoutExchangeRunsOnTheHost) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);          // plain streaming, no exchange
    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    EXPECT_EQ(c.backend(), tx::ControlBackend::Host);
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 5; }, 2 * kSecond));
    c.stop();
}

TEST(McuTrajectoryPty, ImpedanceStreamCarriesGainsAndTheClampedTarget) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ImpedanceConfig cfg;
    cfg.mcu_stream = true;
    tx::ImpedanceController c(*g, cfg);
    c.enable_target_trace(true);
    c.start();
    EXPECT_EQ(c.backend(), tx::ControlBackend::Mcu);
    ASSERT_TRUE(wait_for([&] { return fw.mcu_targets() >= 1; }, kSecond));

    // A target within the clamp goes out as given, with the caller's gains.
    c.set_gains(8.0f, 0.4f, 0.1f);
    const float near = g->position_map().to_position(0.60f + 0.05f);
    c.set_target(near);
    ASSERT_TRUE(wait_for([&] {
        auto m = fw.last_mcu_impedance();
        return m && std::abs(m->target_pos - g->pos_to_rad(near)) < 1e-5f;
    }, kSecond));
    auto m = fw.last_mcu_impedance();
    EXPECT_FLOAT_EQ(m->kp, 8.0f);
    EXPECT_FLOAT_EQ(m->kd, 0.4f);
    EXPECT_FLOAT_EQ(m->target_torque, 0.1f);

    // Far away: clamped to actual +- max_position_torque/kp, as the host law.
    c.set_target(0.0f);
    const float limit = cfg.max_position_torque_nm / 8.0f;
    ASSERT_TRUE(wait_for([&] {
        auto x = fw.last_mcu_impedance();
        return x && std::abs(std::abs(x->target_pos - 0.60f) - limit) < 1e-4f;
    }, kSecond)) << "clamp not applied to the MCU target";
    // The jaw follows: the clamped target is resent against the new sample.
    fw.set_status(0.55f, 0.0f, 0.0f);
    ASSERT_TRUE(wait_for([&] {
        auto x = fw.last_mcu_impedance();
        return x && std::abs(std::abs(x->target_pos - 0.55f) - limit) < 1e-4f;
    }, kSecond)) << "clamped target not refreshed with the jaw";
    EXPECT_EQ(fw.plain_exchange_targets(), 0u) << "host frames while Tracking in Mcu";
    const auto rows = c.drain_target_trace();
    ASSERT_FALSE(rows.empty());
    EXPECT_TRUE(std::any_of(rows.begin(), rows.end(), [&](const auto& r) {
        return r.feedback_refresh && r.requested_rad == g->pos_to_rad(0.0f) &&
               std::abs(std::abs(r.limited_rad-r.feedback_rad)-limit) < 1e-4f;
    }));
    for (size_t i=1; i<rows.size(); ++i) {
        EXPECT_EQ(rows[i].seq, rows[i-1].seq + 1);
        EXPECT_GE(rows[i].host_ns, rows[i-1].host_ns);
    }
    c.stop();
    g->stop_streaming();
}

namespace {
template <class Controller, class Config>
void check_watchdog_lifecycle(Config cfg, bool hold) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);
    fw.set_imu_v2_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100, 100);
    Controller c(*g, cfg);
    fw.set_watchdog(hold);
    { Controller ordinary(*g, cfg); EXPECT_THROW(ordinary.start(), tx::ProtocolError); }
    EXPECT_EQ(fw.disable_count(), 0u);
    EXPECT_EQ(fw.enable_count(), 0u);
    EXPECT_EQ(fw.exchange_targets(), 0u);
    c.start_after_timeout();
    ASSERT_EQ(fw.enable_count(), hold ? 1u : 2u);
    c.set_target(0.3f);
    ASSERT_TRUE(wait_for([&] { return fw.mcu_targets() > 0; }, kSecond));
    // A delayed response / transport retry may still be owned by the worker
    // while stop is called. Neither it nor its keepalive may follow disable.
    fw.lose_exchange_replies(1);
    c.set_target(0.7f);
    c.stop();
    const auto cutoff = fw.targets_at_disable();
    const auto stats = g->exchange_stats();
    EXPECT_TRUE(wait_for([&] { return g->exchange_stats().imu_samples > stats.imu_samples + 12; }, kSecond));
    EXPECT_GT(g->exchange_stats().motor_samples, stats.motor_samples);
    EXPECT_EQ(fw.exchange_targets(), cutoff);
    EXPECT_EQ(g->exchange_stats().mcu_keepalives, stats.mcu_keepalives);
    ASSERT_TRUE(fw.last_plain_impedance().has_value());
    EXPECT_FLOAT_EQ(fw.last_plain_impedance()->kp, 0.0f);
}
}

TEST(McuTrajectoryPty, ForcePositionWatchdogRecoveryAndOrderedStop) {
    tx::ForcePositionConfig cfg;
    cfg.mcu_trajectory = true;
    for (bool hold : {true, false})
        check_watchdog_lifecycle<tx::ForcePositionController>(cfg, hold);
}

TEST(McuTrajectoryPty, ImpedanceWatchdogRecoveryAndOrderedStop) {
    tx::ImpedanceConfig cfg;
    cfg.mcu_stream = true;
    for (bool hold : {true, false})
        check_watchdog_lifecycle<tx::ImpedanceController>(cfg, hold);
}

TEST(McuTrajectoryPty, ProtectedPvRequiresExplicitCapabilityAndNeverFallsBack) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_mcu_trajectory_supported(true);  // bit1 does not imply protected P+V
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ImpedanceConfig cfg;
    cfg.protected_pv = true;
    {
        tx::ImpedanceController c(*g, cfg);
        EXPECT_THROW(c.start(), std::logic_error);
        EXPECT_FALSE(c.running());
    }
    EXPECT_EQ(fw.disable_count(), 0u);
    EXPECT_EQ(fw.exchange_targets(), 0u);
    tx::ImpedanceController host(*g);
    host.start();
    EXPECT_THROW(host.set_target(0.5f, 1.0f), std::logic_error);
    host.stop();
}

TEST(McuTrajectoryPty, ProtectedPvTransmitsTheSourcePairAndCannotRenewItsClock) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_reverse(true);
    fw.set_protected_impedance_supported(true);
    fw.set_exchange_status_timestamp_ms(0xfffffff5U);
    auto g = open_follower(pty);
    g->start_exchange(100, 0, 250);
    tx::ImpedanceConfig cfg;
    cfg.protected_pv = true;
    tx::ImpedanceController c(*g, cfg);
    c.enable_target_trace(true);
    c.start();
    EXPECT_EQ(c.backend(), tx::ControlBackend::ProtectedPv);
    c.set_target(0.85f, -7.25f);
    ASSERT_TRUE(wait_for([&] { return fw.last_protected_impedance().has_value(); }, kSecond));
    const auto m = *fw.last_protected_impedance();
    EXPECT_FLOAT_EQ(m.impedance.target_pos, g->pos_to_rad(0.85f));
    EXPECT_FLOAT_EQ(m.impedance.vel, -7.25f);
    EXPECT_FLOAT_EQ(m.torque_budget_nm, cfg.max_position_torque_nm);
    EXPECT_EQ(m.valid_until_ms, 19u);  // MCU-clock wrap
    c.set_gains(18.0f, 1.0f);
    ASSERT_TRUE(wait_for([&] { return fw.last_protected_impedance()->impedance.kp == 18.0f; }, kSecond));
    EXPECT_EQ(fw.last_protected_impedance()->valid_until_ms, m.valid_until_ms);
    const auto before = fw.exchange_targets();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(fw.exchange_targets(), before) << "telemetry renewed the old reference";
    EXPECT_THROW(c.set_target(0.4f, 2.0f), std::runtime_error)
        << "repeated cached status was treated as a fresh sample";
    fw.set_exchange_status_timestamp_ms(60);
    ASSERT_TRUE(wait_for([&] {
        try { c.set_target(0.4f, 0.0f); return true; }
        catch (const std::runtime_error&) { return false; }
    }, kSecond));
    ASSERT_TRUE(wait_for([&] { return fw.last_protected_impedance()->valid_until_ms == 90u; }, kSecond));
    EXPECT_FLOAT_EQ(fw.last_protected_impedance()->impedance.vel, 0.0f);
    const auto rows = c.drain_target_trace();
    ASSERT_GE(rows.size(), 3u);
    EXPECT_FLOAT_EQ(rows.front().velocity_rad_s, -7.25f);
    EXPECT_FLOAT_EQ(rows.front().limited_rad, rows.front().requested_rad);
    EXPECT_EQ(rows.front().valid_until_ms, 19u);
    c.stop();
    EXPECT_EQ(fw.plain_exchange_targets(), 0u);
    EXPECT_EQ(fw.submit_count(), 0u);
    const auto cutoff = fw.targets_at_disable();
    const auto samples = g->exchange_stats().motor_samples;
    ASSERT_TRUE(wait_for([&] { return g->exchange_stats().motor_samples > samples + 4; }, kSecond));
    EXPECT_EQ(fw.exchange_targets(), cutoff);
}

TEST(McuTrajectoryPty, ProtectedPvRejectedTargetFaultsAndDisablesWithoutLegacyMotion) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_protected_impedance_supported(true);
    auto g = open_follower(pty);
    g->start_exchange(100);
    tx::ImpedanceConfig cfg;
    cfg.protected_pv = true;
    tx::ImpedanceController c(*g, cfg);
    c.start();
    fw.reject_next_exchange_target(static_cast<uint8_t>(tp::ErrorCode::InvalidParam));
    c.set_target(0.2f, -3.0f);
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() > 0; }, kSecond));
    EXPECT_EQ(c.state(), tx::ImpedanceState::Fault);
    EXPECT_THROW(c.set_target(0.3f, 1.0f), std::logic_error);
    EXPECT_THROW(c.reset(), std::logic_error);
    EXPECT_EQ(fw.plain_exchange_targets(), 0u);
    EXPECT_EQ(fw.mcu_targets(), 0u);
    c.stop();
}

TEST(McuTrajectoryPty, ProtectedPvWatchdogRecoveryAwaitsNewFeedbackAndPreservesTelemetry) {
    for (bool hold : {true, false}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_protected_impedance_supported(true);
        fw.set_imu_v2_supported(true);
        auto g = open_follower(pty);
        g->start_exchange(100, 100);
        tx::ImpedanceConfig cfg;
        cfg.protected_pv = true;
        tx::ImpedanceController c(*g, cfg);
        fw.set_watchdog(hold);
        EXPECT_THROW(c.start(), tx::ProtocolError);
        EXPECT_EQ(fw.enable_count(), 0u);
        c.start_after_timeout();
        EXPECT_EQ(fw.enable_count(), hold ? 1u : 2u);
        c.set_target(0.30f, -3.0f);
        ASSERT_TRUE(wait_for([&] { return g->exchange_stats().protected_accepted > 0; }, kSecond));
        c.stop();
        const auto stats = g->exchange_stats();
        ASSERT_TRUE(wait_for([&] { return g->exchange_stats().imu_samples > stats.imu_samples + 4; }, kSecond));
        EXPECT_GT(g->exchange_stats().motor_samples, stats.motor_samples);
        EXPECT_EQ(fw.exchange_targets(), fw.targets_at_disable());
        EXPECT_EQ(fw.plain_exchange_targets(), 0u);
    }
}

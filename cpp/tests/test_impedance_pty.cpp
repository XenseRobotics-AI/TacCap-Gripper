// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// PTY-driven tests for ImpedanceController — the parts the pure policy cannot
// see: the status-stream subscription, the submit thread, and the interlocks
// between them.

#include "fake_follower.hpp"

#include <taccap/impedance_controller.hpp>

#include <chrono>
#include <cmath>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;
using taccap_test::wait_for;

namespace {
using Ms = std::chrono::milliseconds;
}  // namespace

TEST(ImpedanceControllerPty, StartSeedsTheTargetWhereTheJawIs) {
    // Starting an already-enabled motor must not step it. The fake reports
    // 0.60 rad, so the first frame has to command that and not 0.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, Ms(1000)));

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    EXPECT_NEAR(s->target_pos, 0.60f, 1e-3f)
        << "start() jumped the target instead of seeding it";
    EXPECT_EQ(c.state(), tx::ImpedanceState::Tracking);
    c.stop();
}

TEST(ImpedanceControllerPty, SubmitsOncePerStatusFrame) {
    // Stream-locked by construction: every write lands in the window the MCU is
    // known to be idle, so submits track frames rather than a free clock.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 5; }, Ms(1500)));
    const unsigned a = fw.submit_count();
    std::this_thread::sleep_for(Ms(200));
    const unsigned b = fw.submit_count();
    // ~10 frames in 200 ms at the fake's 100 Hz; allow a wide band, the point
    // is that it neither stalls nor free-runs far above the frame rate.
    EXPECT_GT(b, a);
    EXPECT_LE(b - a, 40u) << "submitting far above the status rate";
    c.stop();
}

TEST(ImpedanceControllerPty, StaleStreamFaultsWithZeroTorqueAndAnInvalidObservation) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ImpedanceController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; }, Ms(1000)));

    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return c.state() == tx::ImpedanceState::Fault; },
                         Ms(1000)));

    const auto snap = c.snapshot();
    EXPECT_FALSE(snap.observation.valid)
        << "frozen numbers still presented as a live reading";
    EXPECT_FALSE(snap.fault_reason.empty());

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    EXPECT_FLOAT_EQ(s->kp, 0.0f);
    EXPECT_FLOAT_EQ(s->kd, 0.0f);
    EXPECT_FLOAT_EQ(s->target_torque, 0.0f);
    c.stop();
}

TEST(ImpedanceControllerPty, StaleStreamInvalidatesObservationEvenWhenAlreadyFaulted) {
    // Same trap ForcePositionController shipped with: a dropped link faults the
    // policy through the submit path first, so an invalidation gated on "not
    // already faulted" would never run in the one case it exists for.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ImpedanceController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; }, Ms(1000)));

    fw.set_status_word(tp::MotorStatusBit::OverCurrent);
    ASSERT_TRUE(wait_for([&] { return c.state() == tx::ImpedanceState::Fault; },
                         Ms(1000)));
    const std::string reason = c.snapshot().fault_reason;

    fw.freeze_stream(true);
    EXPECT_TRUE(wait_for([&] { return !c.snapshot().observation.valid; }, Ms(1000)));
    EXPECT_EQ(c.snapshot().fault_reason, reason)
        << "the first cause was overwritten by what happened next";
    c.stop();
}

TEST(ImpedanceControllerPty, SnapshotReportsGuardsTogether) {
    // The reason this class exists rather than ControlLoop's independent
    // booleans: one lock, one consistent view.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; }, Ms(1000)));

    c.set_target(0.0f);
    fw.set_status(0.50f, 0.0f, 1.9f);        // blocked and over the ceiling
    // Both guards trip on this sample -- the ceiling on measured torque, the
    // stall clamp on torque-plus-arrested after stall_hold_ms. One snapshot has
    // to show both, and the state has to report the one that wins.
    ASSERT_TRUE(wait_for([&] {
        const auto x = c.snapshot();
        return x.torque_capped && x.stalled;
    }, Ms(2000)));

    const auto s = c.snapshot();
    EXPECT_EQ(s.state, tx::ImpedanceState::TorqueCapped)
        << "the ceiling outranks the clamp and is what pins the output";
    EXPECT_TRUE(s.torque_capped);
    EXPECT_TRUE(s.stalled) << "the clamp underneath was lost from the report";
    EXPECT_GE(s.torque_caps, 1u);
    EXPECT_GE(s.stall_trips, 1u);
    EXPECT_TRUE(s.observation.valid);
    EXPECT_NEAR(s.commanded_torque_nm, 1.8f, 1e-3f);
    c.stop();
}

TEST(ImpedanceControllerPty, CommandsBeforeStartThrow) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceController c(*g);
    EXPECT_THROW(c.set_target(0.5f), std::logic_error);
    EXPECT_THROW(c.set_gains(10.0f, 1.0f), std::logic_error);
    EXPECT_THROW(c.reset(), std::logic_error);
}

TEST(ImpedanceControllerPty, StopCommandsZeroTorque) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ImpedanceController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 3; }, Ms(1000)));
    c.stop();

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    EXPECT_FLOAT_EQ(s->kp, 0.0f);
    EXPECT_FLOAT_EQ(s->kd, 0.0f);
    EXPECT_FLOAT_EQ(s->target_torque, 0.0f);
    EXPECT_FALSE(c.snapshot().observation.valid);
}

// stop() must return when the device has vanished mid-run.
//
// From the field: "阻抗控制器报错卡死". The USB tree on that bench dropped and
// re-enumerated 15 times in 45 minutes, so the controller was repeatedly losing
// its device underneath it -- and a stop() that blocks on join() while the run
// thread is stuck writing to a dead tty is a hang with no way out but SIGKILL.
TEST(ImpedanceControllerPty, StopReturnsAfterTheDeviceDisappears) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    auto fw = std::make_unique<FakeFollower>(pty);
    auto g = open_follower(pty);

    fw->set_status(0.60f, 0.0f, 0.0f);
    tx::ImpedanceConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ImpedanceController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; }, Ms(1000)));

    // Pull the device out from under it.
    fw.reset();
    pty.close_master();

    const auto t0 = std::chrono::steady_clock::now();
    c.stop();
    const auto took = std::chrono::duration_cast<Ms>(
        std::chrono::steady_clock::now() - t0);
    EXPECT_LT(took.count(), 3000)
        << "stop() took " << took.count() << " ms with the device gone";
    EXPECT_FALSE(c.running());
}

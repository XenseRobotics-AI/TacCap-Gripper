// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// PTY-driven tests for ControlLoop's safety layer.
//
// This class exists to stop a measured failure: on firmware 1.1.5 a plain
// impedance step to a blocked target drove torque 0.03 -> 2.41 Nm in 20 ms,
// still climbing toward a ~4.96 Nm plateau, and pushed 0.078 rad THROUGH the
// object. The firmware's own limit-stall guard is not wired into the MIT path
// at all, so every layer that catches it lives here -- the error clamp, the
// stall clamp, the output torque ceiling, and the stream-liveness safe state.
//
// They are driven against the fake follower in fake_follower.hpp, which records
// the actual MIT frames, so these assert on kp/kd/tau_ff rather than on frame
// counts alone.

#include "fake_follower.hpp"

#include <taccap/control_loop.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tx = xense::taccap;

using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;
using taccap_test::wait_for;

namespace {

using Ms = std::chrono::milliseconds;

// The fake reports a 1.2 rad travel, so raw and normalized differ and a test
// cannot pass by conflating them.
constexpr float kTravelRad = 1.2f;

tx::ControlLoop::Config base_config() {
    tx::ControlLoop::Config c;
    c.kp = 20.0f;
    c.kd = 1.0f;
    c.max_position_torque_nm = 1.5f;
    c.rated_torque_nm = 1.8f;
    c.rated_hold_ms   = 20;
    c.stall_torque_nm = 1.2f;
    c.stall_vel_radps = 0.15f;
    c.stall_hold_ms   = 60;
    c.status_timeout_ms = 250;
    return c;
}

}  // namespace

// ---- Error clamp ---------------------------------------------------------
//
// The primary protection, and the one that bounds the APPROACH as well as the
// stall: an unclamped step to a far target demands kp * 1.2 rad = 24 Nm and the
// jaw arrives at several rad/s, which knocks a loose object clean out of the
// gripper without the torque ever rising enough for a post-contact guard to
// notice.
TEST(ControlLoopPty, ErrorClampBoundsTheCommandedPosition) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    tx::ControlLoop loop(*g, base_config());
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    loop.set_target(0.0f);              // fully closed: 0.60 rad away
    ASSERT_TRUE(wait_for([&] {
        auto s = fw.last_submit();
        return s && std::abs(s->target_pos - 0.60f) > 1e-4f;
    }, Ms(1000)));

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    const float limit = 1.5f / 20.0f;   // max_position_torque_nm / kp
    EXPECT_LE(std::abs(s->target_pos - 0.60f), limit + 1e-4f)
        << "commanded " << s->target_pos << " from a measured 0.60 rad: the "
           "error clamp did not bind";
    loop.stop();
}

// ---- Stall guard ---------------------------------------------------------

TEST(ControlLoopPty, StallGuardClampsTheTargetAndStaysReportedWhileItHolds) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    auto cfg = base_config();
    cfg.rated_torque_nm = 0.0f;         // isolate the stall guard from the ceiling
    tx::ControlLoop loop(*g, cfg);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    loop.set_target(0.0f);
    // Blocked: loaded past stall_torque_nm and slower than stall_vel_radps.
    fw.set_status(0.50f, 0.01f, 1.30f);
    ASSERT_TRUE(wait_for([&] { return loop.stalled(); }, Ms(1000)))
        << "stall guard never engaged";
    EXPECT_EQ(loop.stall_trips(), 1u);

    // THE REGRESSION. The clamp works by killing the position error, which
    // drops the torque back under stall_torque_nm -- so the instantaneous stall
    // test stops holding on the very next frame. stalled() must keep reporting
    // the CLAMP, not that test, or a caller polling it sees one blink and then
    // false for the entire time the gripper is still clamped.
    fw.set_status(0.50f, 0.0f, 0.05f);
    std::this_thread::sleep_for(Ms(150));
    EXPECT_TRUE(loop.stalled())
        << "stalled() went false while the clamp was still holding the target";
    EXPECT_EQ(loop.stall_trips(), 1u) << "the clamp re-tripped instead of holding";

    // Commanding back the other way releases it.
    loop.set_target(1.0f);
    EXPECT_TRUE(wait_for([&] { return !loop.stalled(); }, Ms(1000)))
        << "the clamp did not release when the caller backed off";
    loop.stop();
}

// ---- Output torque ceiling ----------------------------------------------

TEST(ControlLoopPty, TorqueCeilingSendsPureFeedForwardWithBothGainsZero) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    tx::ControlLoop loop(*g, base_config());
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    loop.set_target(0.0f);
    fw.set_status(0.50f, 0.0f, 1.9f);   // feedback at/over rated_torque_nm
    ASSERT_TRUE(wait_for([&] { return loop.torque_capped(); }, Ms(1000)))
        << "torque ceiling never engaged";
    ASSERT_TRUE(wait_for([&] {
        auto s = fw.last_submit();
        return s && s->kp == 0.0f && s->kd == 0.0f;
    }, Ms(1000)));

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    // Both gains zero is the whole point: position error cannot contribute to
    // the output at all, so the torque is PINNED at the ceiling rather than
    // bounded by an estimate of it.
    EXPECT_FLOAT_EQ(s->kp, 0.0f);
    EXPECT_FLOAT_EQ(s->kd, 0.0f);
    EXPECT_NEAR(std::abs(s->target_torque), 1.8f, 1e-4f);
    EXPECT_EQ(loop.torque_caps(), 1u);
    loop.stop();
}

TEST(ControlLoopPty, TorqueCeilingReleasesWhenTheJawComesFree) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    tx::ControlLoop loop(*g, base_config());
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    loop.set_target(0.0f);
    fw.set_status(0.50f, 0.0f, 1.9f);
    ASSERT_TRUE(wait_for([&] { return loop.torque_capped(); }, Ms(1000)));

    // A constant tau_ff would keep accelerating a jaw that came free, so
    // rated_release_rad of travel means the obstruction gave way.
    fw.set_status(0.50f - 0.08f, -0.5f, 0.05f);   // > rated_release_rad = 0.05
    EXPECT_TRUE(wait_for([&] { return !loop.torque_capped(); }, Ms(1000)))
        << "the ceiling held after the jaw came free";
    loop.stop();
}

// ---- Stream liveness -----------------------------------------------------

TEST(ControlLoopPty, StaleStreamCommandsZeroTorqueAndStopsDriving) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    auto cfg = base_config();
    cfg.phase = tx::ControlLoop::SubmitPhase::FreeRunning;  // keeps writing blind
    tx::ControlLoop loop(*g, cfg);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));
    loop.set_target(0.0f);
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 5; }, Ms(1000)));

    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] {
        auto s = fw.last_submit();
        return s && s->kp == 0.0f && s->kd == 0.0f && s->target_torque == 0.0f;
    }, Ms(1500))) << "no zero-torque frame after the status stream died";
    EXPECT_FALSE(loop.observation().valid)
        << "observation stayed valid after the stream died";

    // And then it stays off the wire rather than driving from a frozen
    // reference: the error clamp would otherwise keep bounding against an
    // obs_.raw_pos that no longer moves.
    const unsigned settled = fw.submit_count();
    std::this_thread::sleep_for(Ms(200));
    EXPECT_EQ(fw.submit_count(), settled)
        << "kept submitting impedance frames against a dead stream";
    loop.stop();
}

TEST(ControlLoopPty, StaleStreamReleasesALatchedTorqueCeiling) {
    // The worst branch: both of the ceiling's release tests are evaluated
    // against status frames, so a stream that dies while it is engaged would
    // pin the motor at rated_torque_nm with nothing left that could let go.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    fw.set_status(0.60f, 0.0f, 0.0f);
    tx::ControlLoop loop(*g, base_config());
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    loop.set_target(0.0f);
    fw.set_status(0.50f, 0.0f, 1.9f);
    ASSERT_TRUE(wait_for([&] { return loop.torque_capped(); }, Ms(1000)));

    fw.freeze_stream(true);
    EXPECT_TRUE(wait_for([&] { return !loop.torque_capped(); }, Ms(1500)))
        << "a latched torque ceiling survived the loss of the status stream, "
           "with no path left that could ever release it";

    const auto s = fw.last_submit();
    ASSERT_TRUE(s.has_value());
    EXPECT_FLOAT_EQ(s->target_torque, 0.0f);
    loop.stop();
}

// A dead link must not leave a live-looking observation behind.
//
// A failed submit -- which is how a yanked USB cable surfaces, as
// `SerialBus::write: Input/output error` -- breaks straight out of the phase
// loop, so guard_stale_() never runs again and nothing else clears obs_. A
// caller polling observation() would go on reading the last frame as a live
// one, with only age_ms rising to give it away. The same shape was reported
// from the field on ForcePositionController: a console showing a plausible
// position and 1.540 Nm of torque beside age=10553.7ms.
TEST(ControlLoopPty, LinkFailureLeavesNoLiveLookingObservation) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    auto fw = std::make_unique<FakeFollower>(pty);
    auto g = open_follower(pty);

    fw->set_status(0.60f, 0.0f, 0.0f);
    auto cfg = base_config();
    cfg.phase = tx::ControlLoop::SubmitPhase::FreeRunning;
    tx::ControlLoop loop(*g, cfg);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return loop.observation().valid; }, Ms(1000)));

    // Stop the fake first so it is not reading the fd we are about to close.
    fw.reset();
    pty.close_master();

    ASSERT_TRUE(wait_for([&] { return !loop.running(); }, Ms(2000)))
        << "the loop kept running after every submit started failing";
    EXPECT_FALSE(loop.observation().valid)
        << "observation still reads valid after the link died";
    loop.stop();
}

// ---- Config validation ---------------------------------------------------
//
// ControlLoop used to validate nothing at all. That is how `rated_torque_nm`
// came to sit at 2.0 in lerobot-xense's gripper config for ten days: the same
// value threw on ImpedanceController -- which is what finally surfaced it, via
// a crashing example -- while this path accepted it and parked an INDEFINITE
// hold 11% above the motor's nameplate rating. One guarded door and one open
// one is worse than neither, because the open one is silent.
//
// Every bound below is derived from the motor constants rather than written as
// a literal. Writing 1.8f here would reproduce the exact defect these tests
// exist to prevent -- the original 2.0 was a literal that stopped tracking the
// constant when the bound moved.

namespace {

// The smallest float strictly above / below a bound, so "at the limit" and
// "just over it" stay correct if the motor constants ever change.
constexpr float just_over(float v) {
    return std::nextafter(v, std::numeric_limits<float>::infinity());
}

}  // namespace

TEST(ControlLoopConfig, RejectsAnIndefiniteCeilingAboveRatedTorque) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    auto cfg = base_config();
    cfg.rated_torque_nm = just_over(tx::MOTOR_RATED_TORQUE_NM);
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);

    // The peak rating is NOT an acceptable ceiling here even though the motor
    // can produce it: nothing in this class times the hold out.
    cfg.rated_torque_nm = tx::MOTOR_PEAK_TORQUE_NM;
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);
}

TEST(ControlLoopConfig, AcceptsExactlyRatedAndTheDocumentedDisable) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    auto cfg = base_config();
    cfg.rated_torque_nm = tx::MOTOR_RATED_TORQUE_NM;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));

    // 0 disables the ceiling -- documented behaviour, must stay legal. With it
    // off, rated_release_rad/rated_hold_ms are unused and unconstrained.
    cfg.rated_torque_nm   = 0.0f;
    cfg.rated_release_rad = 0.0f;
    cfg.rated_hold_ms     = 0;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));
}

TEST(ControlLoopConfig, BoundsEachFieldByHowLongItsTorqueLasts) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    // The commanded-torque clamp is transient, so it takes the PEAK rating --
    // a different bound from rated_torque_nm, and the point of this case is
    // that the two are not accidentally the same number.
    auto cfg = base_config();
    cfg.max_position_torque_nm = tx::MOTOR_PEAK_TORQUE_NM;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));
    cfg.max_position_torque_nm = just_over(tx::MOTOR_PEAK_TORQUE_NM);
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);

    // Feed-forward is bounded by the PEAK rating here, NOT by rated as it is
    // on ImpedanceController. Two shipped lerobot-xense recipes run -3.0 under
    // that project's own documented 3.5 Nm rail; tightening this to rated
    // would break them at construction. Pinned so the looser bound reads as
    // intentional rather than as an oversight someone should "fix".
    cfg = base_config();
    cfg.feedforward_torque = -3.0f;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));
    cfg.feedforward_torque = -tx::MOTOR_PEAK_TORQUE_NM;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));
    cfg.feedforward_torque = -just_over(tx::MOTOR_PEAK_TORQUE_NM);
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);

    cfg = base_config();
    cfg.kp = -1.0f;
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);

    // ...but kp == 0 is legal here and not on ImpedanceController: the torque
    // ceiling itself submits kp=kd=0 frames, so a pure feed-forward caller is
    // a supported use of this primitive.
    cfg = base_config();
    cfg.kp = 0.0f;
    cfg.kd = 0.0f;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));

    cfg = base_config();
    cfg.motor_stream_hz = 0;
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);
}

TEST(ControlLoopConfig, StallBoundsApplyOnlyWhileTheGuardIsArmed) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    auto cfg = base_config();
    cfg.stall_torque_nm = just_over(tx::MOTOR_RATED_TORQUE_NM);
    EXPECT_THROW(tx::ControlLoop(*g, cfg), std::invalid_argument);

    // StallAction::None is how the guard is switched off -- not
    // stall_torque_nm == 0 -- so with it disarmed those fields are unused and
    // must not be policed.
    cfg.stall_action    = tx::ControlLoop::StallAction::None;
    cfg.stall_torque_nm = 0.0f;
    cfg.stall_vel_radps = 0.0f;
    cfg.stall_hold_ms   = 0;
    EXPECT_NO_THROW(tx::ControlLoop(*g, cfg));
}

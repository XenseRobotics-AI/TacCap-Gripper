// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ImpedancePolicy — the impedance control law and its three guards, driven
// without a gripper.
//
// ControlLoop implements the same law but keeps it inline in its status
// callback, so the only way to reach it is through a pty and a thread. Pulling
// it into a pure state machine is most of the point of ImpedanceController:
// these cases step the guards frame by frame, which is how you pin behaviour
// like "a blocked jaw must still be commanding the full budget 200 frames
// later" -- a contract a pty test can only sample, not step.

#include <gtest/gtest.h>

#include <taccap/impedance_controller.hpp>

#include <chrono>
#include <cmath>

namespace {

using xense::taccap::GripperPosition;
using xense::taccap::ImpedanceConfig;
using xense::taccap::ImpedanceState;
using xense::taccap::MOTOR_RATED_TORQUE_NM;
using xense::taccap::MotorStatusSample;
using xense::taccap::detail::ImpedancePolicy;
using xense::taccap::detail::ImpedanceTuning;

MotorStatusSample sample(float pos, float vel = 0.0f, float torque = 0.0f,
                         uint16_t status = 0) {
    MotorStatusSample s{};
    s.actual_pos = pos;
    s.actual_vel = vel;
    s.actual_torque = torque;
    s.status = status;
    return s;
}

// 1.2 rad of travel, so raw and normalized never coincide by accident.
GripperPosition map() { return GripperPosition::from_travel(1.2f); }

}  // namespace

// ---- Tracking + error clamp ---------------------------------------------

TEST(ImpedancePolicy, StartTracksFromWhereTheJawIs) {
    ImpedancePolicy p(map(), ImpedanceConfig{});
    p.reset(sample(0.6f));
    EXPECT_EQ(p.state(), ImpedanceState::Tracking);
    // Seeded at the current position so enabling and starting cannot step.
    EXPECT_NEAR(p.target_position(), 0.5f, 1e-4f);

    const auto c = p.step(sample(0.6f), std::chrono::steady_clock::now());
    EXPECT_NEAR(c.target_pos, 0.6f, 1e-4f);
    EXPECT_FLOAT_EQ(c.kp, 20.0f);
    EXPECT_FLOAT_EQ(c.kd, 1.0f);
}

TEST(ImpedancePolicy, ErrorClampBoundsTheCommandedPosition) {
    // The primary protection, and the one that bounds the APPROACH as well as
    // the stall: an unclamped step to a far target demands kp * 1.2 rad = 24 Nm
    // and arrives at several rad/s, which knocks a loose object out of the jaws
    // without the torque ever rising enough for a post-contact guard to see it.
    ImpedanceConfig cfg;
    cfg.kp = 20.0f;
    cfg.max_position_torque_nm = 1.5f;
    ImpedancePolicy p(map(), cfg);
    p.reset(sample(1.2f));
    p.set_target(0.0f);                       // 1.2 rad away

    const auto c = p.step(sample(1.2f), std::chrono::steady_clock::now());
    const float limit = cfg.max_position_torque_nm / cfg.kp;   // 0.075 rad
    EXPECT_NEAR(c.target_pos, 1.2f - limit, 1e-4f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.max_position_torque_nm + 1e-3f);
}

TEST(ImpedancePolicy, ErrorClampNeverBindsWhileTheJawKeepsUp) {
    // It has to cost nothing in normal tracking, or it would be a slew limit
    // by another name and would lag every teleop motion.
    ImpedanceConfig cfg;
    ImpedancePolicy p(map(), cfg);
    p.reset(sample(0.60f));
    p.set_target(map().to_position(0.62f));   // 0.02 rad, well inside 0.075

    const auto c = p.step(sample(0.60f), std::chrono::steady_clock::now());
    EXPECT_NEAR(c.target_pos, 0.62f, 1e-4f) << "the clamp bound on a small error";
}

// ---- The clamp IS the protection ----------------------------------------
//
// The contract that replaced the stall guard. A blocked jaw must saturate the
// error clamp and STAY there: no trip, no state change, and above all no
// collapse. The guard used to clamp the effective target onto the jaw's own
// position, which drops the error to ~0 and takes the output with it -- measured
// on hardware as letting go of the object 60 ms after contact at 0.35 Nm.

TEST(ImpedancePolicy, ObstructionSaturatesTheClampAndHoldsThereIndefinitely) {
    ImpedanceConfig cfg;              // budget 1.1 Nm, kp 20 -> 0.055 rad limit
    ImpedancePolicy p(map(), cfg);
    p.reset(sample(0.6f));
    p.set_target(0.0f);               // command hard closed; the jaw is blocked

    const float limit = cfg.max_position_torque_nm / cfg.kp;
    auto now = std::chrono::steady_clock::now();

    // Far longer than the old 60 ms trip window, and the blocked jaw reports
    // exactly the conditions the guard used to fire on: clamp binding, no
    // motion, torque above the old floor.
    float last_cmd_err = 0.0f;
    for (int i = 0; i < 200; ++i) {   // 200 frames @ 5 ms = 1 s
        now += std::chrono::milliseconds(5);
        const auto c = p.step(sample(0.6f, 0.0f, 0.35f), now);
        last_cmd_err = c.target_pos - 0.6f;
        ASSERT_EQ(p.state(), ImpedanceState::Tracking) << "frame " << i;
        ASSERT_FLOAT_EQ(c.kp, cfg.kp) << "frame " << i;   // never goes soft
    }

    // The commanded position still sits a full error-limit away from the jaw, so
    // kp * error is the whole budget -- that is the grip, and it is steady.
    EXPECT_NEAR(std::abs(last_cmd_err), limit, 1e-5f);
    EXPECT_NEAR(p.commanded_torque_nm(), cfg.max_position_torque_nm, 1e-3f);
}

TEST(ImpedancePolicy, BudgetDefaultIsATorqueTheMotorCanHoldForever) {
    // A blocked jaw now sits at the budget with nothing timing it out, so the
    // default has to be holdable indefinitely. Pinned against the rating rather
    // than written as a literal: 1.5 was the old value, chosen while a stall
    // guard was expected to end the hold.
    EXPECT_LE(ImpedanceConfig{}.max_position_torque_nm, MOTOR_RATED_TORQUE_NM);
    EXPECT_FLOAT_EQ(ImpedanceConfig{}.max_position_torque_nm, 1.1f);
}

TEST(ImpedancePolicy, TorqueCeilingPinsTheOutputWithBothGainsZero) {
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.rated_hold_ms = 20;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    const auto loaded = sample(0.50f, 0.0f, 1.9f);   // at/over rated_torque_nm

    for (int i = 0; i < 5; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(loaded, t);
    }
    ASSERT_TRUE(p.torque_capped());
    EXPECT_EQ(p.state(), ImpedanceState::TorqueCapped);
    EXPECT_EQ(p.torque_caps(), 1u);

    t += std::chrono::milliseconds(10);
    const auto c = p.step(loaded, t);
    // With both gains zero, position error cannot contribute to the output at
    // all: the torque is PINNED at the ceiling, not bounded by an estimate.
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    // It holds the BUDGET, not the trip level. The ceiling fired because the
    // motor is already producing at or above rated; answering with rated right
    // back -- forever, nothing times it out -- would pin a sustained torque
    // above the value that was chosen because it can be held forever.
    EXPECT_NEAR(std::abs(c.target_torque), cfg.max_position_torque_nm, 1e-4f);
    EXPECT_LT(cfg.max_position_torque_nm, cfg.rated_torque_nm)
        << "the budget has to sit below the backstop for this to mean anything";
    EXPECT_NEAR(c.target_pos, 0.50f, 1e-4f) << "the frame should carry no error";
}

TEST(ImpedancePolicy, CeilingHoldsTheRatingOnlyWhenThereIsNoBudget) {
    // With the error clamp disabled there is no budget to collapse to, so the
    // rating stands as the hold. Pins the fallback so it cannot silently become
    // "hold 0 Nm", which min() over a disabled clamp would give.
    ImpedanceConfig cfg;
    cfg.max_position_torque_nm = 0.0f;       // clamp off
    ImpedanceTuning tune;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    const auto loaded = sample(0.50f, 0.0f, 1.9f);
    for (int i = 0; i < 5; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(loaded, t);
    }
    ASSERT_TRUE(p.torque_capped());
    t += std::chrono::milliseconds(10);
    const auto c = p.step(loaded, t);
    EXPECT_NEAR(std::abs(c.target_torque), cfg.rated_torque_nm, 1e-4f);
}

TEST(ImpedancePolicy, TorqueCeilingReleasesWhenTheJawComesFree) {
    ImpedanceConfig cfg;
    ImpedancePolicy p(map(), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    for (int i = 0; i < 5; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.50f, 0.0f, 1.9f), t);
    }
    ASSERT_TRUE(p.torque_capped());

    // A constant tau_ff would keep accelerating a jaw that came free.
    t += std::chrono::milliseconds(10);
    p.step(sample(0.50f - 0.08f, -0.5f, 0.05f), t);   // > rated_release_rad
    EXPECT_FALSE(p.torque_capped());
    EXPECT_EQ(p.state(), ImpedanceState::Tracking);
}

TEST(ImpedancePolicy, MotorFaultBitsCommandZeroTorque) {
    ImpedancePolicy p(map(), ImpedanceConfig{});
    p.reset(sample(0.60f));
    const auto c = p.step(
        sample(0.60f, 0.0f, 0.0f, xense::taccap::protocol::MotorStatusBit::OverCurrent),
        std::chrono::steady_clock::now());
    EXPECT_EQ(p.state(), ImpedanceState::Fault);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FALSE(p.fault_reason().empty());
}

TEST(ImpedancePolicy, StalledBitAloneIsNotAFault) {
    // Holding a blocked position is a stall by the motor's own definition.
    // Faulting on it would abort every legitimate hold.
    ImpedancePolicy p(map(), ImpedanceConfig{});
    p.reset(sample(0.60f));
    p.step(sample(0.60f, 0.0f, 0.5f, xense::taccap::protocol::MotorStatusBit::Stalled),
           std::chrono::steady_clock::now());
    EXPECT_NE(p.state(), ImpedanceState::Fault);
}

TEST(ImpedancePolicy, NonFiniteStatusFaults) {
    ImpedancePolicy p(map(), ImpedanceConfig{});
    p.reset(sample(0.60f));
    p.step(sample(std::nanf(""), 0.0f, 0.0f), std::chrono::steady_clock::now());
    EXPECT_EQ(p.state(), ImpedanceState::Fault);
}

TEST(ImpedancePolicy, ResetLeavesFaultAndReseedsTheTarget) {
    ImpedancePolicy p(map(), ImpedanceConfig{});
    p.reset(sample(0.60f));
    p.fail("test");
    ASSERT_EQ(p.state(), ImpedanceState::Fault);

    p.reset(sample(0.30f));
    EXPECT_EQ(p.state(), ImpedanceState::Tracking);
    EXPECT_TRUE(p.fault_reason().empty());
    EXPECT_NEAR(p.target_position(), 0.25f, 1e-4f) << "target must reseed, not jump";
}

// ---- Config validation ---------------------------------------------------

TEST(ImpedanceConfigValidation, BudgetMustSitBelowTheBackstop) {
    // A backstop that fires inside the operating band is mis-set, not
    // protective: at or past the ceiling every normal grasp trips it, the
    // output drops to kp=kd=0 and position control is lost while holding.
    ImpedanceConfig cfg;
    cfg.rated_torque_nm        = 1.1f;
    cfg.max_position_torque_nm = 1.1f;       // equal is already too far
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg.max_position_torque_nm = 1.2f;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg.max_position_torque_nm = 1.0f;
    EXPECT_NO_THROW(ImpedancePolicy(map(), cfg));
    // A disabled ceiling has no pairing to enforce.
    cfg.max_position_torque_nm = 2.0f;
    cfg.rated_torque_nm        = 0.0f;
    EXPECT_NO_THROW(ImpedancePolicy(map(), cfg));
}

TEST(ImpedanceConfigValidation, FeedForwardCountsTowardTheBackstop) {
    // ff is added to the command on every frame, so the sustained ask is
    // budget + |ff|. Checking the budget alone left a hole: at the defaults an
    // ff of 0.8 puts the steady command at 1.9, past the 1.8 backstop, and
    // every normal grasp would trip it -- the same failure the budget check
    // exists to prevent, reached through a different field.
    ImpedanceConfig cfg;                      // budget 1.1, ceiling 1.8
    ASSERT_LT(cfg.max_position_torque_nm, cfg.rated_torque_nm);

    cfg.feedforward_torque = 0.6f;            // 1.7 -- still under
    EXPECT_NO_THROW(ImpedancePolicy(map(), cfg));
    cfg.feedforward_torque = 0.8f;            // 1.9 -- over
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg.feedforward_torque = -0.8f;           // sign must not launder it
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);

    // With the clamp disabled the budget is 0, so ff alone has to clear it.
    cfg.max_position_torque_nm = 0.0f;
    cfg.feedforward_torque     = 1.7f;
    EXPECT_NO_THROW(ImpedancePolicy(map(), cfg));
    cfg.feedforward_torque     = cfg.rated_torque_nm;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
}

TEST(ImpedanceConfigValidation, CeilingIsCappedAtRatedNotPeakTorque) {
    // The hold it produces is indefinite, so the peak rating is the wrong bound.
    ImpedanceConfig cfg;
    cfg.rated_torque_nm = 2.0f;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg.rated_torque_nm = 1.8f;
    EXPECT_NO_THROW(ImpedancePolicy(map(), cfg));
}

TEST(ImpedanceConfigValidation, RejectsUnusableGainsAndRates) {
    ImpedanceConfig cfg;
    cfg.kp = 0.0f;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg = ImpedanceConfig{};
    cfg.motor_stream_hz = 250;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
    cfg = ImpedanceConfig{};
    cfg.status_timeout_ms = 0;
    EXPECT_THROW(ImpedancePolicy(map(), cfg), std::invalid_argument);
}

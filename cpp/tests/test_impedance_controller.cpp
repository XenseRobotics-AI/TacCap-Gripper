// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ImpedancePolicy — the impedance control law and its three guards, driven
// without a gripper.
//
// ControlLoop implements the same law but keeps it inline in its status
// callback, so the only way to reach it is through a pty and a thread. Pulling
// it into a pure state machine is most of the point of ImpedanceController:
// these cases step the guards frame by frame, which is how you pin behaviour
// like "the stall flag must not go false the moment the clamp starts working".

#include <gtest/gtest.h>

#include <taccap/impedance_controller.hpp>

#include <chrono>
#include <cmath>

namespace {

using xense::taccap::GripperPosition;
using xense::taccap::ImpedanceConfig;
using xense::taccap::ImpedanceState;
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

// ---- Stall guard ---------------------------------------------------------

TEST(ImpedancePolicy, StallNeedsTheClampBindingAndArrestedMotion) {
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);                      // clamp limit = 1.5/20 = 0.075 rad

    // Loaded and the clamp binding, but still moving: the impact transient,
    // measured at 1.33 Nm while still running at 1.94 rad/s. Must not trip.
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.55f, 1.94f, 1.33f), t);
    }
    EXPECT_FALSE(p.stalled());
    EXPECT_EQ(p.state(), ImpedanceState::Tracking);

    // Arrested and loaded, but the clamp is NOT binding because the jaw is at
    // the target. That is a jaw holding station, not a stall -- and it is the
    // case an absolute torque trip cannot distinguish.
    p.set_target(map().to_position(0.55f));
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.55f, 0.0f, 1.30f), t);
    }
    EXPECT_FALSE(p.stalled()) << "tripped while sitting at its own target";

    // Arrested with the clamp binding: a real stall.
    p.set_target(0.0f);
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.55f, 0.0f, 0.30f), t);
    }
    EXPECT_TRUE(p.stalled());
}

TEST(ImpedancePolicy, StallTripIsReachableUnderTheErrorClamp) {
    // The regression this replaced. ControlLoop trips the stall guard on an
    // absolute 1.2 Nm of feedback, which the error clamp makes unreachable:
    // measured on 1.1.6 with a genuine 5-second stall at
    // max_position_torque_nm = 0.35, the command saturated at 0.362 Nm and the
    // feedback peaked at 0.271 Nm -- 4.4x below the trip. The loop reported
    // TRACKING the whole time with the jaw hard against the mechanism, leaving
    // it pushing the full clamp torque indefinitely.
    ImpedanceConfig cfg;
    cfg.max_position_torque_nm = 0.35f;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();

    p.reset(sample(0.60f));
    p.set_target(0.0f);
    // The real numbers off the bench: stalled at 0.0807 normalized (0.097 rad)
    // carrying 0.271 Nm.
    const auto stalled = sample(0.097f, 0.0f, 0.271f);
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(stalled, t);
    }
    EXPECT_TRUE(p.stalled())
        << "a genuine stall at the measured feedback torque did not register";
    EXPECT_EQ(p.state(), ImpedanceState::Stalled);
}

TEST(ImpedancePolicy, ArrestedButUnloadedIsNotAStall) {
    // The floor's only job: reject "commanded, but the motor is not actually
    // pushing" (disabled, unpowered).
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.55f, 0.0f, 0.01f), t);   // under the 0.080 Nm floor
    }
    EXPECT_FALSE(p.stalled());
}

TEST(ImpedancePolicy, StallClampsTheTargetAndStaysReportedWhileItHolds) {
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    const auto blocked = sample(0.50f, 0.01f, 1.30f);
    for (int i = 0; i < 6; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(blocked, t);
    }
    ASSERT_TRUE(p.stalled());
    EXPECT_EQ(p.state(), ImpedanceState::Stalled);
    EXPECT_EQ(p.stall_trips(), 1u);

    // THE REGRESSION ControlLoop shipped with. The clamp works by killing the
    // position error, so the torque falls back under the trip on the very next
    // frame -- that IS the clamp working. Reporting the instantaneous test
    // instead of the clamp made the flag blink once and then read false for the
    // whole time the gripper was still clamped.
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.50f, 0.0f, 0.05f), t);
    }
    EXPECT_TRUE(p.stalled())
        << "stalled() went false while the clamp was still holding the target";
    EXPECT_EQ(p.stall_trips(), 1u) << "re-tripped instead of holding";
    EXPECT_EQ(p.state(), ImpedanceState::Stalled);

    // The clamped target must be where the jaw stopped, not where the caller
    // asked for, so kp * error cannot grow.
    t += std::chrono::milliseconds(10);
    const auto c = p.step(sample(0.50f, 0.0f, 0.05f), t);
    EXPECT_NEAR(c.target_pos, 0.50f, 1e-3f);
}

TEST(ImpedancePolicy, StallReleasesWhenTheCallerBacksOff) {
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    const auto blocked = sample(0.50f, 0.0f, 1.30f);
    for (int i = 0; i < 6; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(blocked, t);
    }
    ASSERT_TRUE(p.stalled());

    p.set_target(1.0f);                      // command back the other way
    t += std::chrono::milliseconds(10);
    p.step(sample(0.50f, 0.0f, 0.05f), t);
    EXPECT_FALSE(p.stalled());
    EXPECT_EQ(p.state(), ImpedanceState::Tracking);
}

// ---- Output torque ceiling ----------------------------------------------

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
    EXPECT_NEAR(std::abs(c.target_torque), cfg.rated_torque_nm, 1e-4f);
    EXPECT_NEAR(c.target_pos, 0.50f, 1e-4f) << "the frame should carry no error";
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

TEST(ImpedancePolicy, TorqueCeilingOutranksTheStallClamp) {
    // Both guards can be engaged at once. The ceiling acts on MEASURED torque
    // and pins the output, so it is what the state must report -- but the clamp
    // is still holding underneath and a caller diagnosing a blocked jaw wants
    // to see that too.
    ImpedanceConfig cfg;
    ImpedanceTuning tune;
    tune.stall_hold_ms = 30;
    tune.rated_hold_ms = 20;
    ImpedancePolicy p(map(), cfg, tune);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.60f));
    p.set_target(0.0f);

    const auto jammed = sample(0.50f, 0.0f, 1.9f);
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(jammed, t);
    }
    EXPECT_EQ(p.state(), ImpedanceState::TorqueCapped);
    EXPECT_TRUE(p.torque_capped());
    EXPECT_TRUE(p.stalled()) << "the clamp underneath was lost from the report";
}

// ---- Fault ---------------------------------------------------------------

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

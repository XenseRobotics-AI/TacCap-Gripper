// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <gtest/gtest.h>

#include <taccap/force_position_controller.hpp>

#include <chrono>
#include <cmath>
#include <limits>

namespace {

using xense::taccap::ForcePositionConfig;
using xense::taccap::ForcePositionState;
using xense::taccap::GripperPosition;
using xense::taccap::MotorStatusSample;
using xense::taccap::detail::ForcePositionPolicy;

MotorStatusSample sample(float pos, float vel = 0.0f, float torque = 0.0f,
                         uint16_t status = 0) {
    MotorStatusSample s{};
    s.actual_pos = pos;
    s.actual_vel = vel;
    s.actual_torque = torque;
    s.status = status;
    return s;
}

}  // namespace

TEST(GripperModes, PositionUsesSpeedLimitedReferenceAndSharedTorqueBudget) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_position_target(sample(0.8f), 0.0f, 0.4f, 0.5f, now);
    const auto first = p.step(sample(0.8f), now + std::chrono::milliseconds(10));
    EXPECT_FLOAT_EQ(first.vel, -0.5f);
    EXPECT_GT(-(first.kp * (first.target_pos - 0.8f) + first.kd * first.vel), 0.0f);
    EXPECT_GT(first.kp, 0.0f);
    EXPECT_FLOAT_EQ(first.target_torque, 0.0f);
    const auto moving = p.step(sample(0.8f, 3.0f), now + std::chrono::milliseconds(20));
    const float predicted = moving.kp * (moving.target_pos - 0.8f) + moving.kd * (moving.vel - 3.0f);
    EXPECT_LE(std::abs(predicted), 0.4f + 1e-6f);
    EXPECT_EQ(p.state(), ForcePositionState::MovingPosition);
}

TEST(GripperModes, PositionObstructionRetainsTorqueAndReversalAcrossRates) {
    for (bool reverse : {false, true}) for (float target : {0.0f, 1.0f})
        for (int period_ms : {10, 20, 50}) {
        const float raw = reverse ? -0.5f : 0.5f;
        const float direction = (target == 1.0f ? 1.0f : -1.0f) * (reverse ? -1.0f : 1.0f);
        ForcePositionPolicy p(GripperPosition::from_travel(1.0f, 0.0f, reverse), {});
        const auto now = std::chrono::steady_clock::now();
        const auto loaded = sample(raw, 0.0f, 0.2f);
        p.reset(loaded, now);
        p.set_position_target(loaded, target, 0.35f, 0.5f, now);
        for (int ms = period_ms; ms <= 2000; ms += period_ms) {
            const auto time = now + std::chrono::milliseconds(ms);
            p.set_position_target(loaded, target, 0.35f, 0.5f, time);
            const auto c = p.step(loaded, time);
            const float torque = c.kp * (c.target_pos - raw) + c.kd * c.vel + c.target_torque;
            EXPECT_GT(torque * direction, 0.0f);
            EXPECT_LE(std::abs(torque), 0.35001f);
            EXPECT_EQ(p.state(), ForcePositionState::MovingPosition);
            EXPECT_FLOAT_EQ(p.target_position(), target);
        }
        const auto changed = now + std::chrono::milliseconds(2010);
        p.set_position_target(loaded, 1.0f - target, 0.35f, 0.5f, changed);
        const auto c = p.step(loaded, changed + std::chrono::milliseconds(period_ms));
        EXPECT_LT((c.kp * (c.target_pos - raw) + c.kd * c.vel) * direction, 0.0f);
    }
}

TEST(GripperModes, RepeatedCommandWakesCannotCountCachedFeedbackAsNewContact) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 2;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    const auto loaded = sample(0.5f, 0.0f, 0.2f);
    p.reset(loaded, now);
    p.set_grasp_target(loaded, 0.0f, 0.35f, 0.5f, now);
    p.step(loaded, now, true);
    for (int i = 0; i < 10; ++i) p.step(loaded, now, false);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_EQ(p.contact_count(), 1u);
    p.step(loaded, now, true);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
}

TEST(GripperModes, ArrivalAndManualHoldKeepOperationBudget) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_position_target(sample(0.8f), 0.5f, 0.35f, 0.5f, now);
    p.step(sample(0.5f), now);
    ASSERT_EQ(p.state(), ForcePositionState::HoldingPosition);
    const auto holding = p.step(sample(0.7f), now);
    EXPECT_LE(std::abs(holding.kp * (holding.target_pos - 0.7f)), 0.35f + 1e-6f);
    EXPECT_LE(p.commanded_torque_nm(), 0.35f + 1e-6f);
    p.hold_position(sample(0.7f));
    p.step(sample(0.9f, 4.0f), now);
    EXPECT_LE(p.commanded_torque_nm(), 0.35f + 1e-6f);
    p.set_impedance_target(sample(0.5f), 0.5f, 20.0f, 1.0f, 0.0f, 0.0f, 5.5f, now);
    p.hold_position(sample(0.5f));
    p.step(sample(0.9f), now);
    EXPECT_FLOAT_EQ(p.active_torque_limit_nm(), cfg.hold_torque_limit_nm);
    EXPECT_LE(p.commanded_torque_nm(), cfg.hold_torque_limit_nm);
}

TEST(GripperModes, ImpedanceBoundsCombinedAbsoluteTorqueTerms) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    for (float position : {0.0f, 1.0f}) for (float ff : {-0.2f, 0.0f, 0.2f})
        for (float velocity : {-30.0f, 0.0f, 30.0f}) {
            p.set_impedance_target(sample(0.5f), position, 500.0f, 5.0f, ff, -5.0f, 0.3f, now);
            const auto c = p.step(sample(0.5f, velocity), now);
            const float spring = c.kp * (c.target_pos - 0.5f);
            const float damping = c.kd * (c.vel - velocity);
            EXPECT_LE(std::abs(spring) + std::abs(damping) + std::abs(c.target_torque), 0.30001f);
            EXPECT_NEAR(p.commanded_torque_nm(), std::abs(spring + damping + c.target_torque), 1e-6f);
            EXPECT_EQ(p.state(), ForcePositionState::Impedance);
        }
}

TEST(GripperModes, PureFeedforwardAndModeSwitchesUseSamePolicy) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_grasp_target(sample(0.8f), 0.0f, 0.3f, 0.8f, now);
    EXPECT_FLOAT_EQ(p.step(sample(0.8f), now).vel, -0.8f);
    p.step(sample(0.5f, 0.0f, 0.2f), now);
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);
    p.set_impedance_target(sample(0.5f), 0.4f, 0.0f, 0.0f, 0.2f, 0.0f, 0.4f, now);
    const auto ff = p.step(sample(0.5f), now);
    EXPECT_FLOAT_EQ(ff.target_torque, 0.2f);
    EXPECT_FLOAT_EQ(ff.kp, 0.0f);
    EXPECT_FLOAT_EQ(ff.kd, 0.0f);
    p.set_position_target(sample(0.5f), 1.0f, 0.4f, 0.5f, now);
    EXPECT_EQ(p.state(), ForcePositionState::MovingPosition);
    EXPECT_EQ(p.control_mode(), xense::taccap::ForcePositionMode::Position);
}

TEST(GripperModes, InvalidModeParametersLeaveExistingOperationUntouched) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_grasp_target(sample(0.5f), 0.0f, 0.35f, 0.5f, now);
    for (float invalid : {-1.0f, 51.0f, std::numeric_limits<float>::quiet_NaN()}) {
        EXPECT_THROW(p.set_position_target(sample(0.5f), 0.2f, 0.4f, invalid, now), std::invalid_argument);
        EXPECT_EQ(p.state(), ForcePositionState::Closing);
    }
    EXPECT_THROW(p.set_impedance_target(sample(0.5f), 0.4f, 501.0f, 1.0f, 0.0f, 0.0f, 0.4f, now), std::invalid_argument);
    EXPECT_THROW(p.set_impedance_target(sample(0.5f), 0.4f, 1.0f, 6.0f, 0.0f, 0.0f, 0.4f, now), std::invalid_argument);
    EXPECT_THROW(p.set_impedance_target(sample(0.5f), 0.4f, 1.0f, 1.0f, 0.5f, 0.0f, 0.4f, now), std::invalid_argument);
    EXPECT_THROW(p.set_grasp_target(sample(0.5f), 0.0f, 1.21f, 0.5f, now), std::invalid_argument);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(p.target_position(), 0.0f);
}

TEST(GripperModes, FaultCannotBeClearedByChangingMode) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.fail("test fault");
    EXPECT_THROW(p.set_position_target(sample(0.5f), 1.0f, 0.4f, 0.5f, now), std::logic_error);
    EXPECT_THROW(p.set_grasp_target(sample(0.5f), 0.0f, 0.35f, 0.5f, now), std::logic_error);
    EXPECT_THROW(p.set_impedance_target(sample(0.5f), 0.4f, 1.0f, 1.0f, 0.0f, 0.0f, 0.4f, now), std::logic_error);
    EXPECT_EQ(p.state(), ForcePositionState::Fault);
    EXPECT_EQ(p.fault_reason(), "test fault");
}

TEST(ForcePositionPolicy, ClosingUsesVelocityDampingWithoutPositionSpring) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    cfg.grasp_torque_nm = 0.35f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t0);
    p.set_target(sample(0.8f), cfg.close_position, cfg.grasp_torque_nm, t0);

    const auto c = p.step(sample(0.8f), t0);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(c.target_pos, 0.8f);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FLOAT_EQ(c.vel, -0.5f);
    EXPECT_NEAR(c.kd * std::abs(c.vel), 0.35f, 1e-6f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.motion_torque_limit_nm);
}

TEST(ForcePositionPolicy, ClosingDampingIncludesActualVelocityInTorqueBound) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    cfg.grasp_torque_nm = 0.35f;
    cfg.motion_torque_limit_nm = 1.8f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_target(sample(0.8f), cfg.close_position, cfg.grasp_torque_nm, now);

    const float actual_velocity = 4.0f;  // moving opposite the close target
    const auto c = p.step(sample(0.8f, actual_velocity), now);
    const float predicted = c.kd * (c.vel - actual_velocity);
    EXPECT_LE(std::abs(predicted), cfg.motion_torque_limit_nm + 1e-6f);
    EXPECT_NEAR(std::abs(predicted), p.commanded_torque_nm(), 1e-6f);
}

TEST(ForcePositionPolicy, RuntimeTargetSelectsDirectionAndTorque) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);

    p.set_target(sample(0.8f), 0.4f, 0.6f, now);
    const auto closing = p.step(sample(0.8f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(p.target_position(), 0.4f);
    EXPECT_FLOAT_EQ(p.grasp_torque_nm(), 0.6f);
    EXPECT_FLOAT_EQ(closing.vel, -0.5f);
    EXPECT_NEAR(closing.kd * std::abs(closing.vel), 0.6f, 1e-6f);

    p.set_target(sample(0.2f), 0.6f, 0.4f, now);
    const auto opening = p.step(sample(0.2f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Opening);
    EXPECT_FLOAT_EQ(opening.vel, 0.5f);

    const auto arrived = p.step(sample(0.6f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
    EXPECT_FLOAT_EQ(arrived.target_pos, 0.6f);
}

TEST(ForcePositionPolicy, RuntimeTargetContactUsesDynamicTorque) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_target(sample(0.8f), 0.2f, 0.6f, now);

    const auto holding = p.step(sample(0.5f, 0.0f, 0.6f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.target_torque, -0.6f);
    EXPECT_FLOAT_EQ(p.target_position(), 0.2f);
    EXPECT_FLOAT_EQ(p.hold_position(), 0.5f);
}

TEST(ForcePositionPolicy, RuntimeTargetRejectsUnsafeValues) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);

    EXPECT_THROW(p.set_target(sample(0.5f), -0.1f, 0.3f, now),
                 std::invalid_argument);
    EXPECT_THROW(p.set_target(sample(0.5f), 0.3f, 1.21f, now),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, ConfirmedContactSwitchesToPureBoundedTorqueHold) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.startup_guard_ms = 250;
    cfg.contact_samples = 2;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t0);
    p.set_target(sample(0.8f), cfg.close_position, cfg.grasp_torque_nm, t0);

    const auto after_guard = t0 + std::chrono::milliseconds(300);
    // Below the contact floor: the jaw is arrested but not loaded.
    p.step(sample(0.5f, 0.0f, 0.05f), after_guard);
    p.step(sample(0.5f, 0.0f, 0.05f),
           after_guard + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::Closing);

    p.step(sample(0.5f, 0.0f, 0.12f),
           after_guard + std::chrono::milliseconds(20));
    const auto c = p.step(sample(0.5f, 0.0f, 0.12f),
                          after_guard + std::chrono::milliseconds(30));

    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, -0.35f);
    EXPECT_FLOAT_EQ(c.target_pos, 0.5f);
    EXPECT_LE(std::abs(c.target_torque), cfg.hold_torque_limit_nm);
}

TEST(ForcePositionPolicy, ForceHoldUsesSoftwareCeilingNotMotionLimit) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.2f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_target(sample(0.8f), cfg.close_position, cfg.grasp_torque_nm, now);

    const auto holding = p.step(sample(0.5f, 0.0f, 1.2f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.kp, 0.0f);
    EXPECT_FLOAT_EQ(holding.kd, 0.0f);
    EXPECT_FLOAT_EQ(holding.target_torque, -1.2f);
    EXPECT_LT(std::abs(holding.target_torque), cfg.motion_torque_limit_nm);
}

TEST(ForcePositionPolicy, ReverseMapFlipsCloseTorqueAndVelocity) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f, 0.0f, true), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(-0.8f), t0);
    p.set_target(sample(-0.8f), cfg.close_position, cfg.grasp_torque_nm, t0);

    const auto moving = p.step(sample(-0.8f), t0);
    EXPECT_GT(moving.vel, 0.0f);
    const auto holding = p.step(sample(-0.5f, 0.0f, 0.4f),
                                t0 + std::chrono::milliseconds(1));
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.target_torque, cfg.grasp_torque_nm);
}

TEST(ForcePositionPolicy, InitialPositionHoldUsesConservativeGraspBudget) {
    ForcePositionConfig cfg;
    cfg.position_kp = 20.0f;
    cfg.position_kd = 1.0f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);  // desired hold stays at raw zero

    const auto c = p.step(sample(1.0f), now);
    const float predicted = c.kp * (c.target_pos - 1.0f);
    EXPECT_NEAR(c.target_pos, 0.9825f, 1e-6f);
    EXPECT_NEAR(std::abs(predicted), cfg.grasp_torque_nm, 1e-5f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.hold_torque_limit_nm);
}

TEST(ForcePositionPolicy, FeedbackBetweenHoldAndMotionLimitsIsAllowed) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 1000;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_target(sample(0.5f), cfg.close_position, cfg.grasp_torque_nm, now);

    p.step(sample(0.5f, 0.0f, 3.3f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_TRUE(p.fault_reason().empty());
}

TEST(ForcePositionPolicy, FeedbackOverMotionLimitTransitionsToZeroTorqueFault) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_target(sample(0.5f), cfg.close_position, cfg.grasp_torque_nm, now);

    const auto c = p.step(sample(0.5f, 0.0f, 5.51f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Fault);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FALSE(p.fault_reason().empty());
}

TEST(ForcePositionPolicy, RejectsGraspTorqueAboveMaximum) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 2.0f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveSoftwareMaximum) {
    ForcePositionConfig cfg;
    cfg.hold_torque_limit_nm = 1.21f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsMotionLimitAboveDeviceMaximum) {
    ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 5.51f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveMotionLimit) {
    ForcePositionConfig cfg;
    cfg.hold_torque_limit_nm = 1.0f;
    cfg.motion_torque_limit_nm = 0.8f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

// The regression that motivated mirroring task_canmotor_is_stalled(): the jaw's
// own restoring torque climbs past the threshold part way down an EMPTY close.
// A bare torque test latches there; torque-plus-arrested-motion does not.
TEST(ForcePositionPolicy, MovingJawAtThresholdTorqueIsNotContact) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.close_speed_radps = 0.5f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.9f), t0);
    p.set_target(sample(0.9f), 0.0f, cfg.grasp_torque_nm, t0);

    // Torque at the threshold, but the jaw is still travelling at the
    // commanded speed -- restoring torque, not an object.
    for (int i = 1; i <= 5; ++i) {
        p.step(sample(0.9f - 0.05f * static_cast<float>(i), -0.5f, 0.40f),
               t0 + std::chrono::milliseconds(10 * i));
        EXPECT_EQ(p.state(), ForcePositionState::Closing) << "sample " << i;
    }
}

TEST(ForcePositionPolicy, ArrestedJawAtThresholdTorqueIsContact) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.close_speed_radps = 0.5f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.9f), t0);
    p.set_target(sample(0.9f), 0.0f, cfg.grasp_torque_nm, t0);

    // Firmware rule "torque": |vel| under contact_vel_radps, no travel history
    // needed, so a jaw that starts against the object still latches.
    const auto c = p.step(sample(0.6f, 0.01f, 0.36f),
                          t0 + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, -0.35f);
}

// Firmware rule "velocity": having moved, a collapse to under
// contact_vel_ratio of the commanded speed counts even above contact_vel_radps.
TEST(ForcePositionPolicy, CollapsedVelocityAfterTravelIsContact) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.close_speed_radps = 0.5f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.9f), t0);
    p.set_target(sample(0.9f), 0.0f, cfg.grasp_torque_nm, t0);

    // Travel at full speed first so has_moved arms (peak >= 35% of commanded,
    // progress >= contact_moved_rad).
    p.step(sample(0.70f, -0.5f, 0.10f), t0 + std::chrono::milliseconds(10));
    p.step(sample(0.50f, -0.5f, 0.20f), t0 + std::chrono::milliseconds(20));
    EXPECT_EQ(p.state(), ForcePositionState::Closing);

    // 0.05 rad/s is above contact_vel_radps (0.035) but only 10% of the
    // commanded 0.5 rad/s, and the torque has saturated.
    p.step(sample(0.49f, -0.05f, 0.36f), t0 + std::chrono::milliseconds(30));
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
}

// Same collapse, but the jaw never travelled: has_moved is what stops a
// slow-but-free close from qualifying under the ratio rule.
TEST(ForcePositionPolicy, CollapsedVelocityWithoutTravelIsNotContact) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.close_speed_radps = 0.5f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.9f), t0);
    p.set_target(sample(0.9f), 0.0f, cfg.grasp_torque_nm, t0);

    p.step(sample(0.9f, -0.05f, 0.36f), t0 + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
}

// The threshold is a flat floor, independent of the grasp torque. Deriving it
// from the commanded cap (the firmware's ratio 1.00) is unreachable under a
// kd-only MIT frame: measured on 1.1.5 hardware the command asked 0.359 Nm at
// stall and the feedback saturated at 0.213 Nm.
TEST(ForcePositionPolicy, ContactFloorIsIndependentOfRuntimeGraspTorque) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.9f), t0);
    p.set_target(sample(0.9f), 0.0f, 0.10f, t0);

    const auto c = p.step(sample(0.6f, 0.0f, 0.11f),
                          t0 + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(c.target_torque, -0.10f);
}

TEST(ForcePositionPolicy, RejectsContactTorqueAboveGraspTorque) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.contact_torque_nm = 0.40f;  // unreachable: the close caps at 0.35
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsContactVelocityRatioOutOfRange) {
    ForcePositionConfig cfg;
    cfg.contact_vel_ratio = 1.5f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Replay of a REAL empty-jaw close, follower firmware 1.1.5, 1578 samples
// decimated to the three regimes that matter. Recorded 2026-08-27 on the unit
// whose travel is 1.2076 rad with the Reverse flag set.
//
// This is the test that would have caught both wrong turns taken on the way
// here: first a threshold low enough to be assumed a false-trigger risk, then
// one derived from the commanded torque cap and therefore unreachable. Neither
// survives contact with the actual numbers.
// ---------------------------------------------------------------------------
namespace {
struct Reading { float raw_pos, vel, torque; };

const Reading kEmptyClose1_1_5[] = {
    // --- stationary at the open end, unloaded (2) ---
    {-1.207573f, -0.012211f, +0.054213f},  // pos=1.0000
    {-1.207573f, -0.012211f, +0.054213f},  // pos=1.0000
    // --- free travel (30) ---
    {-1.146808f, +0.549450f, +0.024909f},  // pos=0.9497
    {-1.112666f, +0.451771f, +0.030769f},  // pos=0.9214
    {-1.079291f, +0.500610f, +0.024909f},  // pos=0.8938
    {-1.046302f, +0.427349f, +0.045421f},  // pos=0.8665
    {-1.012927f, +0.402931f, +0.051282f},  // pos=0.8388
    {-0.978786f, +0.354092f, +0.060073f},  // pos=0.8105
    {-0.945027f, +0.402931f, +0.063004f},  // pos=0.7826
    {-0.909735f, +0.378510f, +0.065934f},  // pos=0.7534
    {-0.870991f, +0.402931f, +0.060073f},  // pos=0.7213
    {-0.836082f, +0.402931f, +0.051282f},  // pos=0.6924
    {-0.801173f, +0.427349f, +0.039560f},  // pos=0.6635
    {-0.765497f, +0.451771f, +0.033700f},  // pos=0.6339
    {-0.731740f, +0.500610f, +0.021978f},  // pos=0.6060
    {-0.697214f, +0.500610f, +0.036630f},  // pos=0.5774
    {-0.663073f, +0.451771f, +0.039560f},  // pos=0.5491
    {-0.627781f, +0.427349f, +0.048352f},  // pos=0.5199
    {-0.592872f, +0.451771f, +0.042491f},  // pos=0.4910
    {-0.551826f, +0.598289f, +0.004395f},  // pos=0.4570
    {-0.516150f, +0.598289f, -0.001465f},  // pos=0.4274
    {-0.479707f, +0.500610f, +0.010256f},  // pos=0.3972
    {-0.444031f, +0.500610f, +0.007326f},  // pos=0.3677
    {-0.407970f, +0.476189f, +0.013187f},  // pos=0.3378
    {-0.372295f, +0.427349f, +0.007326f},  // pos=0.3083
    {-0.337386f, +0.427349f, +0.007326f},  // pos=0.2794
    {-0.302094f, +0.427349f, -0.001465f},  // pos=0.2502
    {-0.266802f, +0.427349f, +0.001465f},  // pos=0.2209
    {-0.228058f, +0.280830f, +0.068864f},  // pos=0.1889
    {-0.192765f, +0.329670f, +0.080586f},  // pos=0.1596
    {-0.156705f, +0.354092f, +0.071795f},  // pos=0.1298
    {-0.121030f, +0.402931f, +0.077656f},  // pos=0.1002
    // --- mechanical closed stop (10) ---
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.209524f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.206594f},  // pos=0.0179
    {-0.021674f, -0.012211f, +0.206594f},  // pos=0.0179
};
}  // namespace

TEST(ForcePositionPolicy, RealEmptyCloseLatchesOnlyAtTheMechanicalStop) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    cfg.grasp_torque_nm = 0.35f;
    cfg.startup_guard_ms = 0;   // exercise the floor on the stationary samples
    ForcePositionPolicy p(GripperPosition::from_travel(1.2076f, 0.0f, true), cfg);

    auto t = std::chrono::steady_clock::now();
    const Reading& first = kEmptyClose1_1_5[0];
    p.reset(sample(first.raw_pos, first.vel, first.torque), t);
    p.set_target(sample(first.raw_pos, first.vel, first.torque), 0.0f,
                 cfg.grasp_torque_nm, t);

    std::size_t latched_at = 0;
    const std::size_t n = std::size(kEmptyClose1_1_5);
    for (std::size_t i = 0; i < n; ++i) {
        const Reading& r = kEmptyClose1_1_5[i];
        t += std::chrono::milliseconds(10);
        p.step(sample(r.raw_pos, r.vel, r.torque), t);
        if (!latched_at && p.state() == ForcePositionState::HoldingForce) {
            latched_at = i + 1;
        }
    }

    // Stationary-but-unloaded (torque 0.054 Nm, under the 0.080 floor) and the
    // whole free travel (|vel| never under 0.183 rad/s, five times the gate)
    // must all pass through without latching. Only the mechanical stop counts.
    ASSERT_NE(latched_at, 0u) << "never latched -- the jaw would push forever";
    EXPECT_GT(latched_at, n - 10) << "latched during travel, at sample "
                                  << latched_at << " of " << n;
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
}

TEST(GripperModes, ClosingCompensationCrossesZeroWithinBudgetAndReleases) {
    for (bool reverse : {false, true}) {
        ForcePositionConfig cfg;
        cfg.close_compensation_rad = 0.03f;
        ForcePositionPolicy p(GripperPosition::from_travel(1.3f, 0.0f, reverse), cfg);
        const float sign = reverse ? -1.0f : 1.0f;
        const auto now = std::chrono::steady_clock::now();
        auto measured = sample(0.0f);
        p.reset(measured, now);
        p.set_position_target(measured, 0.0f, 0.35f, 0.5f, now);
        EXPECT_NEAR(p.position_target_rad(), -sign * 0.03f, 1e-6f);
        auto c = p.step(measured, now + std::chrono::milliseconds(10));
        EXPECT_LT((c.kp * c.target_pos + c.kd * c.vel) * sign, 0.0f);
        EXPECT_LE(std::abs(c.kp * c.target_pos), 0.35001f);
        measured.actual_pos = -sign * 0.015f;
        for (int ms = 20; ms <= 300; ms += 10) {
            auto time = now + std::chrono::milliseconds(ms);
            p.set_position_target(measured, 0.0f, 0.35f, 0.5f, time);
            c = p.step(measured, time);
        }
        EXPECT_NEAR(c.target_pos, -sign * 0.03f, 1e-6f);
        p.set_position_target(measured, 1.0f, 0.35f, 0.5f, now + std::chrono::seconds(1));
        EXPECT_NEAR(p.position_target_rad(), sign * 1.3f, 1e-6f);
        c = p.step(measured, now + std::chrono::milliseconds(1010));
        EXPECT_GT((c.kp * (c.target_pos - measured.actual_pos) + c.kd * c.vel) * sign, 0.0f);
        p.hold_position(measured);
        EXPECT_EQ(p.step(measured, now).target_pos, measured.actual_pos);
    }
}

TEST(GripperModes, PositionTrackingVelocitySharesBudgetAndStopsAtEndpoint) {
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), {});
    auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_position_target(sample(0.5f), 1.0f, 0.35f, 0.5f, now);
    for (float v : {-2.0f, 0.0f, 0.5f, 2.0f}) {
        now += std::chrono::milliseconds(10);
        auto c = p.step(sample(0.5f, v), now);
        EXPECT_LE(std::abs(c.kp * (c.target_pos - 0.5f)) + std::abs(c.kd * (c.vel - v)), 0.35001f);
        EXPECT_NEAR(c.vel, 0.5f, 1e-5f);
    }
    auto arrived = p.step(sample(1.0f), now + std::chrono::seconds(1));
    EXPECT_EQ(arrived.vel, 0.0f);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
}

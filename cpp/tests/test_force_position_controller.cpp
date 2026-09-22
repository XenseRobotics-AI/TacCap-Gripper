// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ForcePositionPolicy is ONE CONTROL LAW plus guards. These tests exercise the
// law's bounds and the observations derived from it. The contact state machine
// they used to cover is gone -- see docs/CONTROL_REFACTOR.md and the header.

#include <gtest/gtest.h>

#include <taccap/force_position_controller.hpp>

#include <chrono>
#include <cmath>

namespace {

using xense::taccap::ForcePositionConfig;
using xense::taccap::ForcePositionState;
using xense::taccap::GripperPosition;
using xense::taccap::MotorStatusSample;
using xense::taccap::detail::ForcePositionPolicy;
using xense::taccap::detail::ForcePositionTuning;

MotorStatusSample sample(float pos, float vel = 0.0f, float torque = 0.0f,
                         uint16_t status = 0) {
    MotorStatusSample s{};
    s.actual_pos = pos;
    s.actual_vel = vel;
    s.actual_torque = torque;
    s.status = status;
    return s;
}

// The whole budget backs the position term; the damping term costs nothing at
// stall because the feed-forward follows the ramp, which stops when blocked.
float error_limit_for(const ForcePositionTuning& t, float budget) {
    return budget / t.position_kp;
}

}  // namespace

// ---------------------------------------------------------------------------
// The control law
// ---------------------------------------------------------------------------

// Travel carries the position spring. The old command set kp=0 so that a stall
// signature stayed clean for the contact test; that cost 37% velocity ripple.
TEST(ForcePositionPolicy, TravelCommandsThePositionSpringAndTheFeedForward) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t0);
    p.set_target(sample(0.8f), 0.0f, cfg.grasp_torque_nm, t0);

    // Seeding frame: the ramp starts at the jaw, no time has passed, so the
    // controller commands nothing at all.
    const auto seed = p.step(sample(0.8f), t0);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(seed.kd, 0.0f);
    EXPECT_FLOAT_EQ(seed.vel, 0.0f);
    EXPECT_NEAR(p.commanded_torque_nm(), 0.0f, 1e-5f);

    // The frame after it carries the spring and the feed-forward.
    const auto c = p.step(sample(0.8f), t0 + std::chrono::milliseconds(10));
    EXPECT_FLOAT_EQ(c.kp, tune.position_kp);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FALSE(p.arrived());
    EXPECT_NEAR(c.kd, tune.travel_kd, 1e-6f);
    EXPECT_LT(c.vel, 0.0f);
}

// THE SAFETY INVARIANT. A jaw that cannot move is pushed at exactly the budget:
// no more, which bounds the force on the object, and no less, which is what
// makes a grasp actually hold. Both terms push the same way here, so the split
// between them is what has to add up.
TEST(ForcePositionPolicy, BlockedTravelPushesExactlyTheGraspTorque) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t);
    p.set_target(sample(0.8f), 0.0f, cfg.grasp_torque_nm, t);

    // The bound is tight from the FIRST step, not only in the steady state:
    // the ramp is clamped to the interval where |kp*error + kd*vel_err| is the
    // budget, so the force is right immediately and only the split between the
    // two terms migrates afterwards.
    auto last = p.step(sample(0.8f), t);
    for (int i = 0; i < 40; ++i) {
        t += std::chrono::milliseconds(10);
        last = p.step(sample(0.8f), t);   // jaw does not move at all
        EXPECT_LE(p.commanded_torque_nm(), cfg.grasp_torque_nm + 1e-4f);
        EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f);
    }
    EXPECT_TRUE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);

    // Converged: the ramp settles the error limit ahead and stops, so the whole
    // command has migrated into the position term and the feed-forward is gone.
    EXPECT_NEAR(0.8f - last.target_pos,
                error_limit_for(tune, cfg.grasp_torque_nm), 3e-3f);
    EXPECT_NEAR(last.vel, 0.0f, 3e-2f);
}

// Free travel costs what friction costs, not what the budget allows. The ramp
// stays close to the jaw, so the spring is nowhere near its clamp.
TEST(ForcePositionPolicy, FreeTravelCostsOnlyFrictionNotTheWholeBudget) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t);
    p.set_target(sample(0.8f), 0.0f, cfg.grasp_torque_nm, t);

    float pos = 0.8f;
    p.step(sample(pos, -cfg.close_speed_radps), t);
    for (int i = 0; i < 20; ++i) {              // jaw tracks the ramp exactly
        t += std::chrono::milliseconds(10);
        pos -= cfg.close_speed_radps * 0.010f;
        p.step(sample(pos, -cfg.close_speed_radps), t);
    }
    EXPECT_LT(p.commanded_torque_nm(), cfg.grasp_torque_nm * 0.25f);
    EXPECT_FALSE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
}

// The ramp is what regulates speed: the setpoint advances at close_speed, it
// does not jump to the clamp and let torque balance decide the velocity.
TEST(ForcePositionPolicy, RampAdvancesAtTheCommandedSpeed) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t);
    p.set_target(sample(0.8f), 0.0f, cfg.grasp_torque_nm, t);

    const float pos = 0.8f;
    p.step(sample(pos, -0.5f), t);              // seeds the ramp at 0.8
    t += std::chrono::milliseconds(20);
    const auto c = p.step(sample(pos, -0.5f), t);
    // 20 ms at 0.5 rad/s is 0.010 rad of ramp travel, and the jaw has not moved,
    // so the whole 0.010 shows up as commanded error (still inside the clamp).
    EXPECT_NEAR(pos - c.target_pos, 0.010f, 1e-6f);
    EXPECT_NEAR(c.vel, -0.5f, 1e-5f);        // feed-forward follows the ramp
}

// Arrival is an observation. There is no arrival branch that changes the law --
// the same clamped PD simply stops asking for much once the error is gone.
TEST(ForcePositionPolicy, ArrivalIsReportedAndCostsAlmostNoTorque) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), t0);
    p.set_target(sample(0.5f), 0.5f, cfg.grasp_torque_nm, t0);

    p.step(sample(0.5f), t0);
    EXPECT_TRUE(p.arrived());
    EXPECT_FALSE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
    EXPECT_NEAR(p.commanded_torque_nm(), 0.0f, 1e-5f);
}

TEST(ForcePositionPolicy, RuntimeTargetSelectsDirection) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);

    p.set_target(sample(0.8f), 0.4f, 0.6f, now);
    p.step(sample(0.8f), now);                       // seeds the ramp
    now += std::chrono::milliseconds(10);
    const auto closing = p.step(sample(0.8f, -cfg.close_speed_radps), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(p.target_position(), 0.4f);
    EXPECT_FLOAT_EQ(p.grasp_torque_nm(), 0.6f);
    EXPECT_LT(closing.vel, 0.0f);                    // ramp advancing to close

    // Moving at the commanded speed: a jaw that is merely far from its target
    // also saturates the budget while it accelerates, so `holding` needs the
    // velocity gate to tell that apart from an obstruction.
    p.set_target(sample(0.2f), 0.6f, 0.4f, now);
    now += std::chrono::milliseconds(10);
    p.step(sample(0.2f, cfg.close_speed_radps), now);   // re-seeds after the jump
    now += std::chrono::milliseconds(10);
    const auto opening = p.step(sample(0.2f, cfg.close_speed_radps), now);
    EXPECT_EQ(p.state(), ForcePositionState::Opening);
    EXPECT_FALSE(p.holding());
    EXPECT_GT(opening.vel, 0.0f);

    now += std::chrono::milliseconds(10);
    const auto arrived = p.step(sample(0.6f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
    EXPECT_TRUE(p.arrived());
    EXPECT_NEAR(arrived.target_pos, 0.6f, 1e-4f);
}

TEST(ForcePositionPolicy, ReverseMapFlipsTheCommandedVelocity) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f, 0.0f, true), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(-0.8f), now);
    p.set_target(sample(-0.8f), 0.0f, cfg.grasp_torque_nm, now);

    // The jaw has to be moving: a stationary jaw carrying the full budget is a
    // blocked one by definition, and gets reported as holding regardless of
    // which way the map runs.
    const float v = cfg.close_speed_radps;
    p.step(sample(-0.8f, v), now);
    const auto c = p.step(sample(-0.8f, v), now + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_GT(c.vel, 0.0f);                          // mirrored
}

// A streamed target is the normal case for teleop. Nothing restarts, because
// there is no guard or confirmation window left to restart.
TEST(ForcePositionPolicy, JitteringStreamedTargetDoesNotDisturbAHold) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), t);

    for (int i = 0; i < 40; ++i) {
        t += std::chrono::milliseconds(10);
        const float dither = (i % 2) ? 0.0008f : -0.0008f;   // +/-0.08% of travel
        p.set_target(sample(0.5f), 0.0f + (dither < 0 ? 0.0f : 0.0008f),
                     cfg.grasp_torque_nm, t);
        p.step(sample(0.5f), t);                             // blocked throughout
    }
    EXPECT_TRUE(p.holding());
    EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f);
}

// ---------------------------------------------------------------------------
// Guards and faults (unchanged behaviour)
// ---------------------------------------------------------------------------

TEST(ForcePositionPolicy, SettledHoldIsBoundedByTheGraspBudget) {
    ForcePositionConfig cfg;
    ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg, tune);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);          // target is raw 0.0

    // Jaw dragged a long way off the target: the clamp bounds the request at the
    // grasp budget, NOT at the 6 Nm motion limit. Everything this class commands
    // is bounded by the force the caller asked for.
    const auto c = p.step(sample(1.0f), now);
    const float predicted = c.kp * (c.target_pos - 1.0f);
    EXPECT_LE(std::abs(predicted), cfg.grasp_torque_nm + 1e-5f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.grasp_torque_nm + 1e-5f);
}

// ---------------------------------------------------------------------------
// Closed-endpoint preload
// ---------------------------------------------------------------------------

// The whole point: kp*(target-actual) vanishes AT the target, so without a
// feed-forward term the jaw reaches the closed stop and stops pressing, leaving
// the gear backlash unseated. Measured on hardware: 0.15 Nm of feed-forward
// seats it at raw 0.00000 and 0.50 Nm moves it no further.
TEST(ForcePositionPolicy, ClosedEndpointHoldCarriesThePreload) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);                  // target_position == 0
    const auto c = p.step(sample(0.0f), now);

    EXPECT_TRUE(p.arrived());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
    // Sitting exactly on the target, so the spring contributes nothing and the
    // command is the preload alone -- closing is -raw on a non-reversed map.
    EXPECT_FLOAT_EQ(c.target_torque, -cfg.close_preload_nm);
    EXPECT_NEAR(p.commanded_torque_nm(), cfg.close_preload_nm, 1e-5f);
}

// A mid-stroke hold must NOT press: the preload is for seating against a
// mechanical stop, and applying it anywhere else would drag the jaw off the
// position the caller asked it to hold.
TEST(ForcePositionPolicy, MidStrokeHoldCarriesNoPreload) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_target(sample(0.5f), 0.5f, cfg.grasp_torque_nm, now);
    const auto c = p.step(sample(0.5f), now);

    EXPECT_TRUE(p.arrived());
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_NEAR(p.commanded_torque_nm(), 0.0f, 1e-5f);
}

// The sign comes from the map, not a constant. to_rad() multiplies by dir_, so
// a rising normalized position is a FALLING raw angle when reversed -- closing
// is -dir_. Hard-coding +1 would push a non-reversed gripper OPEN.
TEST(ForcePositionPolicy, PreloadSignFollowsTheMapDirection) {
    ForcePositionConfig cfg;
    const auto now = std::chrono::steady_clock::now();

    ForcePositionPolicy normal(GripperPosition::from_travel(1.0f), cfg);
    normal.reset(sample(0.0f), now);
    EXPECT_LT(normal.step(sample(0.0f), now).target_torque, 0.0f);

    ForcePositionPolicy reversed(GripperPosition::from_travel(1.0f, 0.0f, true),
                                 cfg);
    reversed.reset(sample(0.0f), now);
    EXPECT_GT(reversed.step(sample(0.0f), now).target_torque, 0.0f);
}

// The preload is RESERVED OUT of the grasp budget, not added on top of it, so
// the bound this class has always held still holds at the endpoint: a jaw
// dragged far off the closed target cannot be asked for more than the budget.
TEST(ForcePositionPolicy, PreloadIsReservedOutOfTheGraspBudget) {
    ForcePositionConfig cfg;
    ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg, tune);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);

    const auto c = p.step(sample(1.0f), now);
    const float spring = c.kp * (c.target_pos - 1.0f);
    // Spring alone is capped at budget-preload; spring plus preload at budget.
    EXPECT_LE(std::abs(spring),
              cfg.grasp_torque_nm - cfg.close_preload_nm + 1e-5f);
    EXPECT_LE(std::abs(spring) + std::abs(c.target_torque), cfg.grasp_torque_nm + 1e-5f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.grasp_torque_nm + 1e-5f);
}

TEST(ForcePositionPolicy, ZeroPreloadRestoresThePureSpringHold) {
    ForcePositionConfig cfg;
    cfg.close_preload_nm = 0.0f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);
    const auto c = p.step(sample(0.0f), now);

    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_NEAR(p.commanded_torque_nm(), 0.0f, 1e-5f);
}

TEST(ForcePositionPolicy, DefaultPreloadIsTheMeasuredSeatingTorque) {
    // 0.15 Nm seats the jaw on hardware; 0.25 is that with headroom for
    // friction growth and drift. Above ~0.15 buys heat, never closure.
    EXPECT_FLOAT_EQ(ForcePositionConfig{}.close_preload_nm, 0.25f);
}

TEST(ForcePositionPolicy, RejectsNegativePreload) {
    ForcePositionConfig cfg;
    cfg.close_preload_nm = -0.01f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsPreloadAboveTheGraspBudget) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm  = 0.30f;
    cfg.close_preload_nm = 0.31f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, FeedbackBetweenHoldAndMotionLimitsIsAllowed) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_target(sample(0.5f), 0.0f, cfg.grasp_torque_nm, now);

    p.step(sample(0.5f, 0.0f, 3.3f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_TRUE(p.fault_reason().empty());
}

TEST(ForcePositionPolicy, FeedbackOverMotionLimitTransitionsToZeroTorqueFault) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.set_target(sample(0.5f), 0.0f, cfg.grasp_torque_nm, now);

    const auto c = p.step(sample(0.5f, 0.0f, 6.01f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Fault);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FALSE(p.fault_reason().empty());
    EXPECT_FALSE(p.holding());
}

TEST(ForcePositionPolicy, RuntimeTargetRejectsUnsafeValues) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);

    EXPECT_THROW(p.set_target(sample(0.5f), -0.1f, 0.3f, now),
                 std::invalid_argument);
    EXPECT_THROW(p.set_target(sample(0.5f), 0.3f, 1.81f, now),
                 std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Configuration validation
// ---------------------------------------------------------------------------

TEST(ForcePositionPolicy, DefaultGraspIsTheMeasuredIndefiniteHold) {
    // Set by a thermal measurement, not by the datasheet: 0.6 Nm plateaus at
    // 49 C on a 600 s hold, while the datasheet's 1.1 Nm continuous rating was
    // still climbing 1 C/min after 432 s. See the header.
    EXPECT_FLOAT_EQ(ForcePositionConfig{}.grasp_torque_nm, 1.1f);
    EXPECT_LE(ForcePositionConfig{}.grasp_torque_nm,
              ForcePositionConfig{}.hold_torque_limit_nm);
}

TEST(ForcePositionPolicy, RejectsGraspTorqueAboveMaximum) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.81f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveSoftwareMaximum) {
    ForcePositionConfig cfg;
    cfg.hold_torque_limit_nm = 1.81f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsMotionLimitAboveDeviceMaximum) {
    ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 6.01f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveMotionLimit) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.3f;
    cfg.hold_torque_limit_nm = 1.5f;
    cfg.motion_torque_limit_nm = 1.0f;
    EXPECT_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsNegativeTravelDamping) {
    ForcePositionConfig cfg;
    ForcePositionTuning tune;
    tune.travel_kd = -0.1f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg, tune),
        std::invalid_argument);
}

// A slow close is just a slow close now. The old rule rejected
// close_speed < grasp/5 because the travel damping gain WAS grasp/close_speed
// and would saturate; the ramp regulates speed instead, so the coupling is gone.
TEST(ForcePositionPolicy, SlowCloseWithAStrongGraspIsAccepted) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.1f;
    cfg.close_speed_radps = 0.05f;         // far below the old grasp/5 = 0.22
    EXPECT_NO_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg));
}

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
using protocol_cmd_t = xense::taccap::protocol::MotorImpedanceCtrl;

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

// A jaw blocked SHORT of the target but within one lead of it. The ramp parks
// on the target, so the lead can never fill and kp*error alone is less than the
// budget -- the band where 0015s sat in Closing at 0.5 of a 1.1 Nm budget with
// its fingers compressing ~20 mrad short of zero. The command is topped up to
// the budget with feed-forward, and that is reported as a grasp.
TEST(ForcePositionPolicy, BlockedWithinOneLeadOfTheTargetStillPushesTheBudget) {
    for (const bool reverse : {false, true}) {
        SCOPED_TRACE(reverse ? "reversed map" : "plain map");
        ForcePositionConfig cfg;
        const ForcePositionTuning tune;
        const auto map = GripperPosition::from_travel(1.0f, 0.0f, reverse);
        ForcePositionPolicy p(map, cfg);
        const float close_dir = reverse ? 1.0f : -1.0f;
        // Short of the closed target by more than arrival_eps, less than the lead.
        const float short_by = 0.022f;
        ASSERT_GT(short_by, tune.arrival_eps_rad);
        ASSERT_LT(short_by, error_limit_for(tune, cfg.grasp_torque_nm));
        const float pos = map.to_rad(0.0f) - close_dir * short_by;

        auto t = std::chrono::steady_clock::now();
        p.reset(sample(pos), t);
        p.set_target(sample(pos), 0.0f, cfg.grasp_torque_nm, t);
        protocol_cmd_t last = p.step(sample(pos), t);
        // The push escalates from kp*error at topup_rate_nmps (see the next
        // test), so give it long enough to reach the budget.
        for (int i = 0; i < 60; ++i) {
            t += std::chrono::milliseconds(10);
            last = p.step(sample(pos), t);   // jaw does not move
            EXPECT_LE(p.commanded_torque_nm(), cfg.grasp_torque_nm + 1e-4f);
        }
        EXPECT_FLOAT_EQ(last.target_pos, map.to_rad(0.0f));   // ramp parked
        EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f);
        const float total = tune.position_kp * (last.target_pos - pos) +
                            last.target_torque;
        EXPECT_NEAR(total, close_dir * cfg.grasp_torque_nm, 1e-3f);
        EXPECT_GT(last.target_torque * close_dir, 0.0f) << "top-up pushes closed";
        EXPECT_TRUE(p.holding());
        EXPECT_FALSE(p.arrived());
        EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);

    }
}

// The top-up is escalated, not stepped: a stop short of the target may be
// breakaway friction, and only a push that grows past it tells the two apart.
// Until the whole budget has failed to move the jaw it is not a grasp.
TEST(ForcePositionPolicy, TopUpEscalatesAtTheConfiguredRateBeforeLatching) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const float pos = 0.022f;              // short of 0.0, within one lead
    const float spring = tune.position_kp * pos;
    ASSERT_LT(spring, cfg.grasp_torque_nm);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(pos), t);
    p.set_target(sample(pos), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(pos), t);
    float prev = 0.0f;
    int escalating = 0;
    for (int i = 0; i < 80 && !p.holding(); ++i) {
        t += std::chrono::milliseconds(10);
        const auto c = p.step(sample(pos), t);
        const float now_nm = p.commanded_torque_nm();
        // While the ramp is still running to the target, the travel law's own
        // budget clamp is in charge (a frozen jaw against a moving ramp); the
        // top-up starts once it has parked.
        if (c.target_pos != 0.0f || c.vel != 0.0f) continue;
        if (now_nm > spring + 1e-4f) {
            ++escalating;
            EXPECT_LE(now_nm - std::max(prev, spring),
                      tune.topup_rate_nmps * 0.010f + 1e-4f) << "frame " << i;
            if (!p.holding()) {
                EXPECT_EQ(p.state(), ForcePositionState::Closing) << "frame " << i;
            }
        }
        prev = now_nm;
    }
    const int expected = static_cast<int>(
        (cfg.grasp_torque_nm - spring) / (tune.topup_rate_nmps * 0.010f));
    EXPECT_GE(escalating, expected - 2) << "stepped rather than escalated";
    EXPECT_TRUE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f);
}

// A compliant object pushed INTO the arrival band must keep the grip. Before the
// latch, 0015s on a notebook alternated between a budget push (-0.013 rad, out
// of the band) and a ~0.4 Nm arrival hold (-0.0098, in it) every ~150 ms, with
// the ramp re-seeding to a zero command on every exit.
TEST(ForcePositionPolicy, AGraspPushedIntoTheArrivalBandDoesNotChatter) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float out = 0.013f, in = 0.0098f;
    ASSERT_GT(out, tune.arrival_eps_rad);
    ASSERT_LT(in, tune.arrival_eps_rad);
    p.reset(sample(out), t);
    p.set_target(sample(out), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(out), t);
    for (int i = 0; i < 60; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(out), t);
    }
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);
    for (int i = 0; i < 60; ++i) {
        t += std::chrono::milliseconds(10);
        const float pos = (i / 5) % 2 ? in : out;
        const auto c = p.step(sample(pos), t);
        EXPECT_EQ(p.state(), ForcePositionState::HoldingForce) << "frame " << i;
        EXPECT_FALSE(p.arrived());
        EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f) << "frame " << i;
        EXPECT_NEAR(tune.position_kp * (c.target_pos - pos) + c.target_torque,
                    -cfg.grasp_torque_nm, 1e-3f);
    }
}

// The object is taken away: the jaw runs through the target and past it. The
// latch lets go, and the ordinary hold brings the jaw back onto the target.
TEST(ForcePositionPolicy, PassingTheTargetReleasesTheGraspLatch) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float target = 0.5f, blocked = 0.52f;
    p.reset(sample(blocked), t);
    p.set_target(sample(blocked), target, cfg.grasp_torque_nm, t);
    p.step(sample(blocked), t);
    for (int i = 0; i < 30; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(blocked), t);
    }
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);

    t += std::chrono::milliseconds(10);
    const float past = target - 2.0f * tune.arrival_eps_rad;
    const auto c = p.step(sample(past), t);
    EXPECT_FALSE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::Opening) << "pulled back toward the target";
    EXPECT_GT(tune.position_kp * (c.target_pos - past) + c.target_torque, 0.0f);

    t += std::chrono::milliseconds(10);
    p.step(sample(target), t);
    EXPECT_TRUE(p.arrived());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
}

// A streamed target that jitters inside the arrival band keeps the grasp. The
// teleop follower sends the leader's position every frame; if each tiny change
// cleared the latch, a compliant object would chatter exactly as before.
TEST(ForcePositionPolicy, AJitteringStreamedTargetKeepsTheGraspLatched) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float out = 0.013f, in = 0.0098f;
    p.reset(sample(out), t);
    p.set_target(sample(out), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(out), t);
    for (int i = 0; i < 60; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(out), t);
    }
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);
    for (int i = 0; i < 60; ++i) {
        t += std::chrono::milliseconds(10);
        const float jitter = (i % 3) * 0.002f;          // < arrival_eps_rad
        ASSERT_LT(jitter, tune.arrival_eps_rad);
        p.set_target(sample(out), jitter, cfg.grasp_torque_nm, t);
        const float pos = (i / 5) % 2 ? in : out;
        const auto c = p.step(sample(pos), t);
        EXPECT_EQ(p.state(), ForcePositionState::HoldingForce) << "frame " << i;
        EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f) << "frame " << i;
        // Pinned on the target with no velocity feed-forward: nothing for the
        // motor's own velocity to disagree with (0015s delivered 0.85 of 1.1).
        EXPECT_FLOAT_EQ(c.target_pos, jitter) << "frame " << i;
        EXPECT_FLOAT_EQ(c.vel, 0.0f) << "frame " << i;
    }
}

// The escalation itself must survive a streamed target. Teleop sends the
// leader's position every frame, jittering by a few mrad; if each frame the ramp
// spent catching up restarted the push from the spring, a grasp made while
// streaming would never reach the budget.
TEST(ForcePositionPolicy, AJitteringStreamedTargetStillEscalatesToTheBudget) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float pos = 0.022f;
    p.reset(sample(pos), t);
    p.set_target(sample(pos), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(pos), t);
    for (int i = 0; i < 10; ++i) {              // let the ramp park first
        t += std::chrono::milliseconds(10);
        p.step(sample(pos), t);
    }
    for (int i = 0; i < 80 && !p.holding(); ++i) {
        t += std::chrono::milliseconds(10);
        const float jitter = (i % 3) * 0.002f;
        ASSERT_LT(jitter, tune.arrival_eps_rad);
        p.set_target(sample(pos), jitter, cfg.grasp_torque_nm, t);
        p.step(sample(pos), t);
    }
    EXPECT_TRUE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_NEAR(p.commanded_torque_nm(), cfg.grasp_torque_nm, 1e-3f);
}

// The latch clears on CUMULATIVE target travel since it was taken: a slowly
// streamed target that walks away in small steps releases it.
TEST(ForcePositionPolicy, SmallStepsAddingUpPastTheBandReleaseTheLatch) {
    ForcePositionConfig cfg;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float blocked = 0.02f;
    p.reset(sample(blocked), t);
    p.set_target(sample(blocked), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(blocked), t);
    for (int i = 0; i < 40; ++i) { t += std::chrono::milliseconds(10); p.step(sample(blocked), t); }
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);
    // Walk the target open in 3 mrad steps (each below arrival_eps).
    float tgt = 0.0f;
    bool released = false;
    for (int i = 0; i < 10 && !released; ++i) {
        tgt += 0.003f;
        ASSERT_LT(0.003f, tune.arrival_eps_rad);
        t += std::chrono::milliseconds(20);
        p.set_target(sample(blocked), tgt, cfg.grasp_torque_nm, t);
        p.step(sample(blocked), t);
        released = !p.holding();
    }
    EXPECT_TRUE(released) << "latch survived " << tgt << " rad of cumulative target travel";
    EXPECT_LE(tgt, tune.arrival_eps_rad + 0.004f) << "released only after " << tgt;
}

// A new target is a new approach: the latch does not follow the jaw to it.
TEST(ForcePositionPolicy, ANewTargetClearsTheGraspLatch) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(0.02f), t);
    p.set_target(sample(0.02f), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(0.02f), t);
    for (int i = 0; i < 30; ++i) {
        t += std::chrono::milliseconds(10);
        p.step(sample(0.02f), t);
    }
    ASSERT_EQ(p.state(), ForcePositionState::HoldingForce);
    p.set_target(sample(0.02f), 0.02f, cfg.grasp_torque_nm, t);
    t += std::chrono::milliseconds(10);
    p.step(sample(0.02f), t);
    EXPECT_TRUE(p.arrived());
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
}

// The top-up is for a jaw at rest. One still moving through that band on its
// way in is left to the ramp, or every approach would end in a budget shove.
TEST(ForcePositionPolicy, NoTopUpWhileTheJawIsStillMoving) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    auto t = std::chrono::steady_clock::now();
    const float pos = 0.022f;
    p.reset(sample(pos), t);
    p.set_target(sample(pos), 0.0f, cfg.grasp_torque_nm, t);
    p.step(sample(pos, -cfg.close_speed_radps), t);
    protocol_cmd_t last{};
    for (int i = 0; i < 10; ++i) {
        t += std::chrono::milliseconds(10);
        last = p.step(sample(pos, -cfg.close_speed_radps), t);
    }
    EXPECT_FLOAT_EQ(last.target_torque, 0.0f);
    EXPECT_FALSE(p.holding());
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
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

TEST(ForcePositionPolicy, AcceptsLimitsAboveTheEl05RatingsForOtherMotors) {
    // This USED to throw: the bounds were EL05's 1.8 / 6.0 compiled in, which
    // made an RS00 (5.0 rated, 14.0 range) unable to be given the 3.6 N·m grip
    // it was fitted for. The device's real ratings are enforced in start(),
    // where the device can actually be asked; a config is built long before.
    ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 14.0f;   // RS00 torque range
    cfg.hold_torque_limit_nm   = 5.0f;    // RS00 rated
    cfg.grasp_torque_nm        = 3.6f;    // RS00 continuous stall
    EXPECT_NO_THROW(ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg));
}

TEST(ForcePositionPolicy, StillRejectsATorqueNoMotorCouldMean) {
    // The sanity ceiling catches a unit slip or a stray zero -- it is not a
    // motor rating and must not be read as one.
    ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = xense::taccap::MOTOR_ABSOLUTE_TORQUE_CEILING_NM + 1.0f;
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

// ---------------------------------------------------------------------------
// Closed loop against a plant with friction
// ---------------------------------------------------------------------------

namespace {

// A jaw with inertia, Coulomb friction that sticks, and an optional wall. The
// motor applies the MIT law at 1 kHz; the policy runs at 100 Hz on the sample
// the stream would have delivered. Numbers are fitted to the RS00 follower
// 0088s (2026-09-29): free travel stalls 13-14 mrad short of a mid-stroke
// target under kp*error ~0.2-0.28 Nm, and a 1 Nm push reaches ~1 rad/s in
// ~50 ms.
struct FrictionPlant {
    float inertia = 0.04f;         // kg*m^2, reflected
    float static_nm = 0.30f;
    float kinetic_nm = 0.20f;
    float viscous = 0.40f;         // Nm*s/rad
    float wall_raw = NAN;          // blocks motion below this raw angle (closing)
    float wall_k = 300.0f;         // Nm/rad, contact stiffness
    float pos = 0.0f;
    float vel = 0.0f;

    void advance(const protocol_cmd_t& c, float dt) {
        const float motor = c.kp * (c.target_pos - pos) + c.kd * (c.vel - vel) +
                            c.target_torque;
        float ext = 0.0f;
        if (std::isfinite(wall_raw) && pos < wall_raw) {
            ext = wall_k * (wall_raw - pos) - 2.0f * vel;
        }
        const float drive = motor + ext;
        if (std::abs(vel) < 1e-4f && std::abs(drive) <= static_nm) {
            vel = 0.0f;            // stuck
            return;
        }
        const float sign = std::abs(vel) >= 1e-4f ? (vel > 0 ? 1.0f : -1.0f)
                                                  : (drive > 0 ? 1.0f : -1.0f);
        const float acc = (drive - sign * kinetic_nm - viscous * vel) / inertia;
        const float next = vel + acc * dt;
        vel = (next * sign < 0.0f) ? 0.0f : next;   // friction cannot reverse it
        pos += vel * dt;
    }
};

struct LoopTrace {
    int holding_frames = 0;
    int arrived_frames_tail = 0;   // over the last second
    float max_excursion = 0.0f;    // |pos - target| after first reaching the band
    float final_pos = 0.0f;
    ForcePositionState final_state = ForcePositionState::Idle;
    float final_cmd_nm = 0.0f;
};

// vel_noise: amplitude of the velocity the stream reports around the true one.
// The RS00's MIT feedback is quantized to ~16 mrad/s and reads +-0.09 rad/s on a
// jaw at rest (0088s, 2026-09-29); position does not jitter.
LoopTrace run_loop(ForcePositionPolicy& p, FrictionPlant& plant, float target,
                   float budget, float seconds, float vel_noise = 0.0f) {
    uint32_t lcg = 12345u;
    auto noisy = [&](float v) {
        if (vel_noise <= 0.0f) return v;
        lcg = lcg * 1664525u + 1013904223u;
        const float u = static_cast<float>(lcg >> 8) / 16777216.0f;  // [0,1)
        constexpr float kQuantum = 0.016117f;
        return std::round((v + (2.0f * u - 1.0f) * vel_noise) / kQuantum) * kQuantum;
    };
    auto t = std::chrono::steady_clock::now();
    p.reset(sample(plant.pos), t);
    p.set_target(sample(plant.pos), target, budget, t);
    LoopTrace tr;
    bool reached = false;
    protocol_cmd_t applied{plant.pos, 0.0f, 0.0f, 0.0f, 0.0f};
    const int frames = static_cast<int>(seconds * 100.0f);
    for (int f = 0; f < frames; ++f) {
        const auto cmd = p.step(sample(plant.pos, noisy(plant.vel)), t);
        // One frame of transport latency: this command lands next period.
        for (int k = 0; k < 10; ++k) plant.advance(applied, 0.001f);
        applied = cmd;
        t += std::chrono::milliseconds(10);
        if (p.holding()) ++tr.holding_frames;
        if (f >= frames - 100 && p.arrived()) ++tr.arrived_frames_tail;
        const float err = std::abs(plant.pos - target);
        if (reached) tr.max_excursion = std::max(tr.max_excursion, err);
        if (p.arrived()) reached = true;
    }
    tr.final_pos = plant.pos;
    tr.final_state = p.state();
    tr.final_cmd_nm = p.commanded_torque_nm();
    return tr;
}

}  // namespace

// Free travel on a stiff mechanism stalls outside the arrival band: at kp=20 a
// 0.28 Nm breakaway friction parks the jaw 14 mrad short. That is friction, not
// an object. Treating it as one pushed the whole budget in one frame, shot the
// jaw through the target, and did the same from the other side -- a ~3 Hz limit
// cycle of +-30 mrad that never arrived (0088s, RS00, 2026-09-29).
TEST(ForcePositionPolicy, FrictionStallShortOfTheTargetCreepsInWithoutALimitCycle) {
    for (const float start : {0.25f, 0.55f}) {        // opening and closing
        SCOPED_TRACE(start < 0.4f ? "opening" : "closing");
        ForcePositionConfig cfg;
        cfg.grasp_torque_nm = 1.0f;
        cfg.close_speed_radps = 0.5f;
        ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
        FrictionPlant plant;
        plant.pos = start;
        const auto tr = run_loop(p, plant, 0.4f, cfg.grasp_torque_nm, 4.0f);
        const ForcePositionTuning tune;
        EXPECT_EQ(tr.holding_frames, 0) << "empty jaw reported as a grasp";
        EXPECT_EQ(tr.arrived_frames_tail, 100) << "final pos " << tr.final_pos;
        EXPECT_LE(tr.max_excursion, tune.arrival_eps_rad * 1.5f);
        EXPECT_EQ(tr.final_state, ForcePositionState::HoldingPosition);
    }
}

// The same stall at MOT-04's 0.2 rad/s with the RS00's real velocity feedback.
// "At rest" is |vel| <= 0.25 * close_speed = 0.05 rad/s there, which a jaw
// standing still fails on most frames: the escalation must not be reset by
// velocity noise, or the push never out-grows breakaway and the jaw stays
// parked 14 mrad out -- no limit cycle, but no arrival either.
TEST(ForcePositionPolicy, FrictionStallCreepsInDespiteQuantizedVelocityNoise) {
    for (const float start : {0.25f, 0.55f}) {
        SCOPED_TRACE(start < 0.4f ? "opening" : "closing");
        ForcePositionConfig cfg;
        cfg.grasp_torque_nm = 1.0f;
        cfg.close_speed_radps = 0.2f;
        ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
        FrictionPlant plant;
        plant.pos = start;
        const auto tr = run_loop(p, plant, 0.4f, cfg.grasp_torque_nm, 5.0f, 0.09f);
        const ForcePositionTuning tune;
        EXPECT_EQ(tr.holding_frames, 0) << "empty jaw reported as a grasp";
        EXPECT_EQ(tr.arrived_frames_tail, 100) << "final pos " << tr.final_pos;
        EXPECT_LE(tr.max_excursion, tune.arrival_eps_rad * 1.5f);
        EXPECT_EQ(tr.final_state, ForcePositionState::HoldingPosition);
    }
}

// The same plant closing onto a wall mid-stroke: the ramp runs ahead, the lead
// fills, and the grasp is the budget -- friction handling must not soften it.
TEST(ForcePositionPolicy, FrictionPlantStillGraspsAWallAtTheBudget) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.0f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    FrictionPlant plant;
    plant.pos = 0.6f;
    plant.wall_raw = 0.3f;
    const auto tr = run_loop(p, plant, 0.1f, cfg.grasp_torque_nm, 3.0f);
    EXPECT_EQ(tr.final_state, ForcePositionState::HoldingForce);
    EXPECT_NEAR(tr.final_cmd_nm, cfg.grasp_torque_nm, 1e-3f);
    EXPECT_NEAR(tr.final_pos, 0.3f, 0.01f);
}

// A wall inside one lead of the target -- the pad-compression case the top-up
// exists for (0015s closing to 0.0, fingers meeting ~20-36 mrad early). With
// friction in the plant it must still end at the full budget.
TEST(ForcePositionPolicy, FrictionPlantWallWithinOneLeadStillReachesTheBudget) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.0f;
    const ForcePositionTuning tune;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    FrictionPlant plant;
    plant.pos = 0.3f;
    plant.wall_raw = 0.025f;
    ASSERT_LT(plant.wall_raw, error_limit_for(tune, cfg.grasp_torque_nm));
    const auto tr = run_loop(p, plant, 0.0f, cfg.grasp_torque_nm, 3.0f);
    EXPECT_EQ(tr.final_state, ForcePositionState::HoldingForce);
    EXPECT_NEAR(tr.final_cmd_nm, cfg.grasp_torque_nm, 1e-3f);
}

// A slowly STREAMED target through the friction plant (0.3.10). The jaw lags by
// friction/kp -- more than arrival_eps on the RS00 -- while it tracks, and the
// per-frame unlatch test never fired because each 50 Hz step is a few mrad, so
// the lag escalated to the budget and latched mid-motion: HOLDING_FORCE and
// Closing/Opening alternating, the budget pushed while tracking. Reported by
// tc-gu-01-pc on 0094s (RS00) with a 50 Hz cosine, 2026-09-29. Escalation and
// latch now reset once the target has moved arrival_eps from where the
// escalation began, in total.
TEST(ForcePositionPolicy, SlowStreamedCosineThroughFrictionNeverGrasps) {
    for (const float period_s : {4.0f, 8.0f, 16.0f}) {
        SCOPED_TRACE(period_s);
        ForcePositionConfig cfg;
        cfg.grasp_torque_nm = 3.6f;                 // RS00 budget
        cfg.hold_torque_limit_nm = 5.0f;
        cfg.motion_torque_limit_nm = 14.0f;
        const float travel = 1.2f;
        ForcePositionPolicy p(GripperPosition::from_travel(travel), cfg);
        FrictionPlant plant;
        plant.pos = 0.9f * travel;
        auto t = std::chrono::steady_clock::now();
        p.reset(sample(plant.pos), t);
        protocol_cmd_t applied{plant.pos, 0.0f, 0.0f, 0.0f, 0.0f};
        int holding = 0;
        float max_cmd = 0.0f;
        const int frames = static_cast<int>(period_s * 2.0f * 100.0f);  // two cycles
        for (int f = 0; f < frames; ++f) {
            const float ts = f * 0.01f;
            if (f % 2 == 0) {                        // 50 Hz target stream
                const float tgt = 0.5f + 0.4f * std::cos(6.2831853f * ts / period_s);
                p.set_target(sample(plant.pos, plant.vel), tgt, cfg.grasp_torque_nm, t);
            }
            const auto cmd = p.step(sample(plant.pos, plant.vel), t);
            for (int k = 0; k < 10; ++k) plant.advance(applied, 0.001f);
            applied = cmd;
            t += std::chrono::milliseconds(10);
            if (p.holding()) ++holding;
            max_cmd = std::max(max_cmd, p.commanded_torque_nm());
        }
        EXPECT_EQ(holding, 0) << "frames reported as a grasp while tracking";
        EXPECT_LT(max_cmd, 1.5f) << "the budget was pushed while tracking";
    }
}

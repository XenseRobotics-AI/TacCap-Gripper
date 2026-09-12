// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// PTY-driven tests for ForcePositionController itself, as opposed to the pure
// ForcePositionPolicy state machine pinned in
// test_force_position_controller.cpp. Everything interesting about the
// controller lives in the parts the policy cannot see: the status-stream
// subscription, the submit thread, and the safety interlocks between them.
// Those are exactly the paths a hardware bring-up would exercise last and
// trust most, so they are driven here against a fake follower firmware.

#include "fake_follower.hpp"

#include <taccap/force_position_controller.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <atomic>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;
using taccap_test::wait_for;

// The fake firmware and its helpers live in fake_follower.hpp so the
// ControlLoop tests can drive the same device.

TEST(ForcePositionControllerPty, StartSeedsPositionHoldAndSubmitsPerStatusFrame) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(c.running());

    EXPECT_TRUE(wait_for([&] { return fw.submit_count() >= 3; },
                         std::chrono::milliseconds(1000)))
        << "controller submitted " << fw.submit_count() << " MIT frames";
    EXPECT_EQ(c.state(), tx::ForcePositionState::HoldingPosition);

    const auto snap = c.snapshot();
    EXPECT_TRUE(snap.observation.valid);
    EXPECT_FLOAT_EQ(snap.device_limit_nm, 6.0f);
    EXPECT_LE(snap.commanded_torque_nm, cfg.motion_torque_limit_nm);
    c.stop();
}

TEST(ForcePositionControllerPty, StaleStatusStreamFaultsWithAZeroTorqueCommand) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; },
                         std::chrono::milliseconds(1000)));

    const unsigned before = fw.submit_count();
    fw.freeze_stream(true);

    // Wait for the zero-torque frame to actually reach the wire, not merely for
    // the state to flip. policy_->fail() sets Fault while holding the lock, but
    // the submit happens outside it -- so Fault is observable before the frame
    // exists, and asserting on state alone races with the write. Waiting on the
    // submit counter is what the test is really about anyway.
    ASSERT_TRUE(wait_for([&] {
                             return c.state() == tx::ForcePositionState::Fault &&
                                    fw.submit_count() > before;
                         },
                         std::chrono::milliseconds(2000)))
        << "state " << tx::to_string(c.state()) << ", submits "
        << fw.submit_count() << " (was " << before << ")";

    const auto last = fw.last_submit();
    ASSERT_TRUE(last.has_value());
    EXPECT_FLOAT_EQ(last->kp, 0.0f);
    EXPECT_FLOAT_EQ(last->kd, 0.0f);
    EXPECT_FLOAT_EQ(last->target_torque, 0.0f);
    EXPECT_FALSE(c.snapshot().fault_reason.empty());
    c.stop();
}

// Regression: staleness used to be checked only when the submit thread woke on
// its own timeout, so a caller command (which sets step_requested_) walked
// straight past it and was computed from a stale sample -- real motion ordered
// from data of unknown age. A dead stream must win over the caller.
TEST(ForcePositionControllerPty, SetTargetOnAStaleStreamCommandsZeroTorque) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; },
                         std::chrono::milliseconds(1000)));

    fw.freeze_stream(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // now stale

    const unsigned before = fw.submit_count();
    c.set_target(0.0f, 0.35f);   // ask for a full close on stale data

    ASSERT_TRUE(wait_for([&] { return fw.submit_count() > before; },
                         std::chrono::milliseconds(1000)));
    const auto last = fw.last_submit();
    ASSERT_TRUE(last.has_value());
    EXPECT_EQ(c.state(), tx::ForcePositionState::Fault);
    EXPECT_FLOAT_EQ(last->kp, 0.0f);
    EXPECT_FLOAT_EQ(last->kd, 0.0f);
    EXPECT_FLOAT_EQ(last->target_torque, 0.0f);
    EXPECT_FLOAT_EQ(last->vel, 0.0f);
    c.stop();
}

TEST(ForcePositionControllerPty, StartRefusesADeviceLimitAboveTheMotionLimit) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 1.8f;  // device answers 6.0
    cfg.hold_torque_limit_nm = 1.8f;
    tx::ForcePositionController c(*g, cfg);
    EXPECT_THROW(c.start(), std::runtime_error);
    EXPECT_FALSE(c.running());
}

// The firmware gate. 1.1.6 is where the motion safety envelope landed, and
// everything older has no stall protection at all on the MIT command path --
// a blocked jaw is bounded only by the motor's own 0x700B ceiling, which on
// 24 V browns out the board. Opening such a device must fail loudly rather
// than quietly running one blocked grasp away from that.
TEST(FollowerFirmwareGate, RefusesFirmwareOlderThanTheMinimum) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty, 1, 1, 5);          // one patch short
    EXPECT_THROW(open_follower(pty), xense::taccap::ProtocolError);
}

TEST(FollowerFirmwareGate, AcceptsTheMinimumAndNewer) {
    {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty, 1, 1, 6);
        EXPECT_NO_THROW({ auto g = open_follower(pty); });
    }
    {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty, 1, 2, 0);
        EXPECT_NO_THROW({ auto g = open_follower(pty); });
    }
}

// The escape hatch has to work, or a device that needs upgrading could not be
// inspected first. (OTA itself goes through LeaderGripper and is unaffected.)
TEST(FollowerFirmwareGate, AllowOutdatedFirmwareOpensAnyway) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty, 1, 0, 2);
    tx::FollowerGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 1;
    cfg.allow_outdated_firmware = true;
    EXPECT_NO_THROW({ tx::FollowerGripper g(cfg); });
}

// A dead stream must invalidate the observation, not just the state.
//
// snapshot().observation.valid was set true on the first sample and never set
// back, so a caller polling it saw a live-looking reading -- plausible
// position, plausible torque -- long after the cable came out. Only `state`
// gave it away, which meant "is the gripper still there" could not be answered
// from the observation itself.
TEST(ForcePositionControllerPty, StaleStreamInvalidatesTheObservation) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; },
                         std::chrono::milliseconds(1000)));

    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return !c.snapshot().observation.valid; },
                         std::chrono::milliseconds(1000)))
        << "observation stayed valid after the status stream died";
    EXPECT_EQ(c.state(), tx::ForcePositionState::Fault);
    c.stop();
}

// Caller commands must not put a frame on the wire by themselves.
//
// set_target()/release()/hold_position() used to apply the command and submit
// on the CALLER's thread, at whatever moment that thread called in. The MCU
// drops bytes out of a status frame it is transmitting when host traffic
// overlaps it, so those writes cost telemetry frames. They are now queued and
// applied on the status doorbell instead.
TEST(ForcePositionControllerPty, CallerCommandDoesNotSubmitOffPhase) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 2000;   // keep the stream "alive" while frozen
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; },
                         std::chrono::milliseconds(1000)));

    // Silence the doorbell without going stale, so anything that reaches the
    // wire from here can only have come from the caller's thread.
    fw.freeze_stream(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const unsigned before = fw.submit_count();

    c.set_target(0.0f, 0.35f);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(fw.submit_count(), before)
        << "set_target() submitted on the caller's thread instead of waiting "
           "for the next status frame";

    // And it must not be lost either: the queued target applies as soon as the
    // stream resumes.
    fw.freeze_stream(false);
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() > before; },
                         std::chrono::milliseconds(1000)))
        << "the queued target never reached the wire after the stream resumed";
    c.stop();
}

// A stale stream must invalidate the observation NO MATTER HOW the policy got
// into Fault.
//
// Reported from the field: the console showed a plausible position and a
// 1.540 Nm torque next to `age=10553.7ms` and
// `fault: submit failed: SerialBus::write: Input/output error`. Ten seconds of
// frozen numbers presented as live readings.
//
// The invalidation was gated on `stale && state != Fault`, i.e. it only ran
// when staleness was what CAUSED the fault. When the USB link drops, the submit
// error faults the policy first -- within one status period, well inside the
// 350 ms staleness timeout -- so by the time the stream is judged stale the
// policy is already in Fault, the branch is skipped, and the observation stays
// "valid" forever. The one case the flag exists for is the one case it missed.
TEST(ForcePositionControllerPty, StaleStreamInvalidatesObservationEvenWhenAlreadyFaulted) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; },
                         std::chrono::milliseconds(1000)));

    // Fault the policy through a route that is NOT staleness, exactly as a
    // failed submit does in the field.
    fw.set_status_word(tp::MotorStatusBit::OverCurrent);
    ASSERT_TRUE(wait_for([&] {
        return c.state() == tx::ForcePositionState::Fault;
    }, std::chrono::milliseconds(1000)));
    const std::string reason = c.snapshot().fault_reason;
    ASSERT_FALSE(reason.empty());

    // Now lose the stream while already faulted.
    fw.freeze_stream(true);
    EXPECT_TRUE(wait_for([&] { return !c.snapshot().observation.valid; },
                         std::chrono::milliseconds(1000)))
        << "observation stayed valid: a caller polling it sees frozen numbers "
           "presented as a live reading";

    // The original cause must survive -- "over current" is why this happened,
    // "stream stale" is only what happened next.
    EXPECT_EQ(c.snapshot().fault_reason, reason);
    c.stop();
}

// stop() must return when the device has vanished mid-run -- see the matching
// ImpedanceController case.
TEST(ForcePositionControllerPty, StopReturnsAfterTheDeviceDisappears) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    auto fw = std::make_unique<FakeFollower>(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().observation.valid; },
                         std::chrono::milliseconds(1000)));

    fw.reset();
    pty.close_master();

    const auto t0 = std::chrono::steady_clock::now();
    c.stop();
    const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);
    EXPECT_LT(took.count(), 3000)
        << "stop() took " << took.count() << " ms with the device gone";
    EXPECT_FALSE(c.running());
}

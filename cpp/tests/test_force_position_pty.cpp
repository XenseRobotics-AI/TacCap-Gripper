// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// PTY-driven tests for ForcePositionController itself, as opposed to the pure
// ForcePositionPolicy state machine pinned in
// test_force_position_controller.cpp. Everything interesting about the
// controller lives in the parts the policy cannot see: the status-stream
// subscription, the submit thread, and the safety interlocks between them.
// Those are exactly the paths a hardware bring-up would exercise last and
// trust most, so they are driven here against a fake follower firmware.

#include "pty_helper.hpp"

#include <taccap/force_position_controller.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <taccap/control_loop.hpp>
#include <cmath>
#include <atomic>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::Pty;

namespace {

std::vector<uint8_t> pod_bytes(const void* p, std::size_t n) {
    std::vector<uint8_t> out(n);
    std::memcpy(out.data(), p, n);
    return out;
}

// Minimal follower firmware: answers the handful of commands
// FollowerGripper's constructor and ForcePositionController::start() issue,
// pushes motor-status DATA frames while "streaming", and records the MIT
// impedance frames the controller submits.
class FakeFollower {
public:
    FakeFollower(Pty& pty, uint8_t major = 1, uint8_t minor = 1, uint8_t patch = 6)
        : pty_(pty), fw_major_(major), fw_minor_(minor), fw_patch_(patch) {
        status_.actual_pos = 0.60f;
        status_.control_mode = 0;
        thread_ = std::thread([this] { run_(); });
    }
    ~FakeFollower() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

    // Stop answering the status stream without stopping the responder: this is
    // what a wedged firmware looks like to the host.
    void freeze_stream(bool frozen) { frozen_.store(frozen); }
    void reject_disable(bool rejected) { reject_disable_.store(rejected); }
    void set_execution(uint8_t owner, int16_t error = 0) {
        execution_owner_.store(owner);
        execution_error_.store(error);
    }

    void set_status(float pos, float vel, float torque) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.actual_pos = pos;
        status_.actual_vel = vel;
        status_.actual_torque = torque;
    }

    std::optional<tp::MotorImpedanceCtrl> last_submit() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_submit_;
    }
    unsigned disable_count() const { return disables_.load(); }
    unsigned submit_count() const { return submits_.load(); }

private:
    void run_() {
        auto next_status = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            if (auto f = pty_.expect_frame(5)) handle_(*f);

            const auto now = std::chrono::steady_clock::now();
            if (streaming_.load() && !frozen_.load() && now >= next_status) {
                next_status = now + std::chrono::milliseconds(10);
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_data(0, tp::Cmd::GetMotorStatus,
                               pod_bytes(&status_, sizeof(status_)));
            }
        }
    }

    void handle_(const xense::taccap::bus::Frame& f) {
        switch (f.cmd) {
            case tp::Cmd::GetVersion: {
                const tp::FirmwareVersion v{fw_major_, fw_minor_, fw_patch_, 0};
                pty_.send_response(f.seq, f.cmd, pod_bytes(&v, sizeof(v)));
                return;
            }
            case tp::Cmd::GetSn: {
                const std::string sn = "TCGU01A28Z0001s";
                pty_.send_response(f.seq, f.cmd,
                                   std::vector<uint8_t>(sn.begin(), sn.end()));
                return;
            }
            case tp::Cmd::GetGripperConfig: {
                tp::GripperConfig c{};
                c.magic = 0x47435047UL;
                c.version = 1;
                c.flags = tp::GripperConfigFlag::Valid;
                c.max_open_rad = 1.30f;
                c.min_open_rad = 0.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&c, sizeof(c)));
                return;
            }
            case tp::Cmd::GetGripperAutoCalConfig: {
                tp::GripperAutoCalConfig c{};
                c.magic = 0x4743414CUL;
                c.version = 1;
                c.flags = tp::GripperAutoCalFlag::Valid;
                // The firmware's own tuned numbers, so the controller's
                // advisory cross-check sees a realistic device.
                c.close_stall_torque_nm = 0.35f;
                c.open_stall_torque_nm = 0.35f;
                c.close_speed_rad_s = 0.25f;
                c.open_speed_rad_s = 0.35f;
                c.stall_hold_ms = 30;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&c, sizeof(c)));
                return;
            }
            case tp::Cmd::MotorGetStartupLimitTorque: {
                const float limit = 5.5f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&limit, sizeof(limit)));
                return;
            }
            case tp::Cmd::GetMotorStatus: {
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_response(f.seq, f.cmd,
                                   pod_bytes(&status_, sizeof(status_)));
                return;
            }
            case tp::Cmd::MotorDisable:
                disables_.fetch_add(1);
                if (reject_disable_.load()) pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                else pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::GetMotorExecutionStatus: {
                tp::MotorExecutionStatus s{};
                s.owner = execution_owner_.load(); s.timeout_ms = 500;
                s.last_error = execution_error_.load();
                s.requested[2] = 1.2f; s.applied[2] = 1.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&s, sizeof(s)));
                return;
            }
            case tp::Cmd::StartStream:
                streaming_.store(true);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::StopStream:
                streaming_.store(false);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::MotorImpedanceCtrl: {
                // Fire-and-forget: no ACK, exactly like the real firmware.
                if (f.payload.size() >= sizeof(tp::MotorImpedanceCtrl)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    tp::MotorImpedanceCtrl c{};
                    std::memcpy(&c, f.payload.data(), sizeof(c));
                    last_submit_ = c;
                }
                submits_.fetch_add(1);
                return;
            }
            default:
                pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                return;
        }
    }

    Pty& pty_;
    uint8_t fw_major_, fw_minor_, fw_patch_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> frozen_{false};
    std::atomic<bool> reject_disable_{false};
    std::atomic<uint8_t> execution_owner_{4};
    std::atomic<int16_t> execution_error_{-22};
    std::atomic<unsigned> submits_{0};
    std::atomic<unsigned> disables_{0};
    mutable std::mutex mu_;
    tp::MotorStatus status_{};
    std::optional<tp::MotorImpedanceCtrl> last_submit_;
};

std::unique_ptr<tx::FollowerGripper> open_follower(const Pty& pty) {
    tx::FollowerGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 1;
    return std::make_unique<tx::FollowerGripper>(cfg);
}

// Spin until `pred` or the deadline. Returns whether it held.
template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

}  // namespace

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
    EXPECT_FLOAT_EQ(snap.device_limit_nm, 5.5f);
    EXPECT_LE(snap.commanded_torque_nm, cfg.motion_torque_limit_nm);
    c.stop();
}

TEST(GripperModesPty, SwitchesModesInOneSessionAndReportsSubmissionEvidence) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    c.set_position_target(0.8f, 0.4f, 0.5f);
    const auto position_seq = c.snapshot().command_sequence;
    ASSERT_TRUE(wait_for([&] {
        const auto s = c.snapshot();
        return s.submitted_sequence == position_seq && s.observation.seq > s.submission_observation_sequence;
    }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(c.snapshot().control_mode, tx::ForcePositionMode::Position);
    const auto floor = c.snapshot().submission_observation_sequence;
    const auto frame_count = fw.submit_count();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() > frame_count + 2; }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(c.snapshot().submission_observation_sequence, floor);

    c.set_grasp_target(0.0f, 0.3f, 0.7f);
    ASSERT_TRUE(wait_for([&] {
        const auto frame = fw.last_submit();
        return frame && frame->kp == 0.0f && std::abs(frame->vel + 0.7f) < 1e-5f;
    }, std::chrono::milliseconds(1000)));
    c.set_impedance_target(0.5f, 0.0f, 0.0f, 0.2f, 0.0f, 0.4f);
    ASSERT_TRUE(wait_for([&] {
        const auto frame = fw.last_submit();
        return frame && frame->kp == 0.0f && frame->kd == 0.0f && frame->target_torque == 0.2f;
    }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(c.snapshot().control_mode, tx::ForcePositionMode::Impedance);
    EXPECT_GT(c.snapshot().submitted_sequence, position_seq);
    EXPECT_EQ(fw.disable_count(), 0u);
    c.hold_position();
    EXPECT_EQ(c.snapshot().control_mode, tx::ForcePositionMode::Position);
    EXPECT_FLOAT_EQ(c.snapshot().active_torque_limit_nm, 0.4f);
    c.stop();
    EXPECT_EQ(fw.disable_count(), 1u);
}

TEST(GripperModesPty, NewModesAndResetRejectStaleFaultWithoutSubmitting) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 100;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() > 0; }, std::chrono::milliseconds(1000)));
    const auto count = fw.submit_count();
    const auto sequence = c.snapshot().command_sequence;
    EXPECT_THROW(c.set_position_target(1.0f), std::logic_error);
    EXPECT_THROW(c.set_grasp_target(0.0f), std::logic_error);
    EXPECT_THROW(c.set_impedance_target(0.5f, 1.0f, 1.0f), std::logic_error);
    EXPECT_THROW(c.hold_position(), std::logic_error);
    EXPECT_THROW(c.reset(), std::logic_error);
    EXPECT_EQ(c.snapshot().command_sequence, sequence);
    EXPECT_EQ(c.state(), tx::ForcePositionState::Fault);
    EXPECT_EQ(fw.submit_count(), count);
    c.stop();
}

TEST(GripperModesPty, FirmwareExecutionFaultStopsImpedanceWithFreshMotorStream) {
    Pty pty;
    FakeFollower fw(pty, 1, 1, 7);
    fw.set_execution(1);
    auto g = open_follower(pty);
    tx::ForcePositionConfig cfg;
    cfg.monitor_execution = true;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    c.set_impedance_target(0.5f, 1.0f, 1.0f, 0.0f, 0.0f, 0.4f);
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 25; }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(c.state(), tx::ForcePositionState::Impedance);
    fw.set_execution(4, -23);
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() > 0; }, std::chrono::milliseconds(1000)));
    const auto snapshot = c.snapshot();
    EXPECT_EQ(snapshot.state, tx::ForcePositionState::Fault);
    EXPECT_LT(snapshot.observation.age_ms, 100.0);
    EXPECT_NE(snapshot.fault_reason.find("error=-23"), std::string::npos);
    EXPECT_THROW(c.set_position_target(0.8f), std::logic_error);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    EXPECT_EQ(fw.disable_count(), 1u);
    c.stop();
}

TEST(GripperModesPty, FailedFaultDisableRetriesWithoutTelemetry) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 100;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    fw.reject_disable(true);
    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() >= 1; }, std::chrono::milliseconds(1000)));
    fw.reject_disable(false);
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() >= 2; }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(c.state(), tx::ForcePositionState::Fault);
    c.stop();
}

TEST(ControlRuntimePty, ExplicitStopReportsDisableFailureAfterCleaningUpOwner) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionController c(*g);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    fw.reject_disable(true);
    EXPECT_THROW(c.stop(), tx::ProtocolError);
    EXPECT_FALSE(c.running());
    fw.reject_disable(false);
    EXPECT_NO_THROW(g->motor().disable());
    EXPECT_NO_THROW(c.start());
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
TEST(ForcePositionControllerPty, SetTargetOnAStaleStreamCannotRestartDisabledMotor) {
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
    ASSERT_TRUE(wait_for([&] { return fw.disable_count() > 0; },
                         std::chrono::milliseconds(1000)));

    const unsigned before = fw.submit_count();
    c.set_target(0.0f, 0.35f);   // ask for a full close on stale data

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(fw.submit_count(), before);
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
    cfg.motion_torque_limit_nm = 1.8f;  // device answers 5.5
    cfg.hold_torque_limit_nm = 1.2f;
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

TEST(ControlRuntimePty, ExclusiveControllerAndManualTargetOwnership) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ControlLoop loop(*g);
    tx::ForcePositionController force(*g);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    EXPECT_THROW(force.start(), std::logic_error);
    EXPECT_THROW(g->motor().submit_torque(0.0f, 5.0f), std::logic_error);
    loop.stop();
    EXPECT_NO_THROW(force.start());
    force.stop();
    EXPECT_NO_THROW(g->motor().submit_torque(0.0f, 5.0f));
}

TEST(ControlRuntimePty, FrozenFeedbackStopsControlLoopAndDisables) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ControlLoop::Config cfg;
    cfg.status_timeout_ms = 100;
    tx::ControlLoop loop(*g, cfg);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] { return !loop.running(); }, std::chrono::milliseconds(1000)));
    EXPECT_TRUE(wait_for([&] { return fw.disable_count() >= 1; }, std::chrono::milliseconds(500)));
    const auto count = fw.submit_count();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(fw.submit_count(), count);
    loop.stop();
}

TEST(ControlRuntimePty, StopsOnlyItsOwnStream) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->start_streaming(100);
    {
        tx::ControlLoop loop(*g);
        loop.start();
        ASSERT_TRUE(wait_for([&] { return fw.submit_count() > 0; }, std::chrono::milliseconds(1000)));
    }
    EXPECT_TRUE(g->is_streaming());
    g->stop_streaming();
}

TEST(MotorPty, RejectsInvalidTargetsBeforeSending) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    EXPECT_THROW(g->motor().submit_impedance(NAN, 1, 1, 0, 0), std::invalid_argument);
    EXPECT_THROW(g->motor().submit_velocity(INFINITY, 1, 10), std::invalid_argument);
    EXPECT_THROW(g->motor().submit_torque(5.51f, 5.0f), std::invalid_argument);
    EXPECT_THROW(g->motor().set_position(0.2f, -1, 1), std::invalid_argument);
    EXPECT_THROW(g->motor().submit_impedance(0, -1, 0, 0, 0), std::invalid_argument);
    EXPECT_EQ(fw.submit_count(), 0);
}

TEST(MotorPty, ExecutionStatusReportsFirmwareOwnershipAndTargets) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    const auto status = g->motor().execution_status();
    EXPECT_EQ(status.owner, 4);
    EXPECT_EQ(status.timeout_ms, 500);
    EXPECT_EQ(status.last_error, -22);
    EXPECT_FLOAT_EQ(status.requested[2], 1.2f);
    EXPECT_FLOAT_EQ(status.applied[2], 1.0f);
}

TEST(ControlRuntimePty, ManualDisablePreventsTheNextTickFromReenabling) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ControlLoop loop(*g);
    loop.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; }, std::chrono::milliseconds(1000)));
    g->motor().disable();
    const auto after_disable = fw.submit_count();
    ASSERT_TRUE(wait_for([&] { return !loop.running(); }, std::chrono::milliseconds(1000)));
    EXPECT_EQ(fw.submit_count(), after_disable);
    EXPECT_THROW(g->motor().enable(), std::logic_error);
    loop.stop();
    loop.start();
    EXPECT_TRUE(wait_for([&] { return fw.submit_count() > after_disable; }, std::chrono::milliseconds(1000)));
    loop.stop();
}

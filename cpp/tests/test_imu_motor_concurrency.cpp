// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// User IMU delivery must not block control feedback or hide its receive age.
// Exercise that boundary through a PTY:
// the fake models packet flow, not firmware scheduling or physical motion.

#include "fake_follower.hpp"

#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <limits>
#include <mutex>
#include <thread>

namespace tx = xense::taccap;
namespace tp = tx::protocol;
using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;
using taccap_test::wait_for;

namespace {
using Ms = std::chrono::milliseconds;

template <class Controller> struct ControllerTraits;
template <> struct ControllerTraits<tx::ForcePositionController> {
    using Config = tx::ForcePositionConfig;
    static constexpr auto fault = tx::ForcePositionState::Fault;
};
template <> struct ControllerTraits<tx::ImpedanceController> {
    using Config = tx::ImpedanceConfig;
    static constexpr auto fault = tx::ImpedanceState::Fault;
};

// A deterministic slow subscriber: hold one callback until the test has
// observed the independent safety/close path, without relying on a particular
// scheduler delay. The release guard also unblocks early ASSERT returns.
class ImuCallbackGate {
public:
    void operator()() {
        ++calls;
        std::unique_lock<std::mutex> lock(mu_);
        if (!armed_ || released_) return;
        entered_ = true;
        cv_.notify_all();
        cv_.wait(lock, [&] { return released_; });
        completed.store(true);
    }

    void arm() {
        std::lock_guard<std::mutex> lock(mu_);
        armed_ = true;
    }
    bool wait_until_entered() {
        std::unique_lock<std::mutex> lock(mu_);
        return cv_.wait_for(lock, Ms(2000), [&] { return entered_; });
    }
    void release() {
        {
            std::lock_guard<std::mutex> lock(mu_);
            released_ = true;
        }
        cv_.notify_all();
    }

    std::atomic<unsigned> calls{0};
    std::atomic<bool> completed{false};

private:
    std::mutex mu_;
    std::condition_variable cv_;
    bool armed_ = false;
    bool entered_ = false;
    bool released_ = false;
};

struct ReleaseCallbackOnExit {
    std::shared_ptr<ImuCallbackGate> gate;
    ~ReleaseCallbackOnExit() { gate->release(); }
};

bool is_zero_torque(const std::optional<tp::MotorImpedanceCtrl>& command) {
    return command && command->kp == 0.0f && command->kd == 0.0f &&
           command->target_torque == 0.0f && command->vel == 0.0f;
}

template <class Controller>
void controller_keeps_preexisting_imu_stream() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);  // advancing timestamps and sample IDs
    auto imus = std::make_shared<std::atomic<unsigned>>(0);
    auto g = open_follower(pty);
    const auto imu_sub = g->imu().on_data([imus](const tx::ImuSample&) {
        ++*imus;
    });
    g->start_streaming(100, 100);
    const auto initial_stops = fw.stream_stops();

    Controller controller(*g);
    controller.start();
    ASSERT_TRUE(wait_for([&] {
        return fw.submit_count() >= 8 && imus->load() >= 8;
    }, Ms(2000)));

    // Repeated target updates must keep producing commands and IMU samples.
    // Hold the fake's observed position: it deliberately has no motor model.
    for (unsigned phase = 0; phase < 3; ++phase) {
        const auto submits_before = fw.submit_count();
        const auto imus_before = imus->load();
        controller.set_target(controller.snapshot().observation.position);
        ASSERT_TRUE(wait_for([&] {
            return fw.submit_count() >= submits_before + 8 &&
                   imus->load() >= imus_before + 8;
        }, Ms(2000)));
        EXPECT_TRUE(controller.snapshot().observation.valid);
        EXPECT_NE(controller.state(), ControllerTraits<Controller>::fault);
    }

    const auto stream = fw.stream_config();
    EXPECT_EQ(stream.source_mask,
              tp::StreamSrc::MotorStatus | tp::StreamSrc::ImuV2);
    EXPECT_EQ(stream.motor_rate, 100u);
    EXPECT_EQ(stream.imu_rate, 100u);
    EXPECT_EQ(stream.mode, static_cast<uint8_t>(tp::StreamMode::Separate));
    EXPECT_EQ(fw.stream_stops(), initial_stops);

    controller.stop();
    EXPECT_FALSE(controller.running());
    EXPECT_TRUE(g->is_streaming());
    EXPECT_EQ(fw.stream_stops(), initial_stops)
        << "stopping a borrower must preserve the caller's shared stream";
    EXPECT_EQ(fw.disable_count(), 1u);
    EXPECT_TRUE(is_zero_torque(fw.last_submit()));

    const auto stopped_submits = fw.submit_count();
    const auto imus_before = imus->load();
    ASSERT_TRUE(wait_for([&] { return imus->load() >= imus_before + 8; },
                         Ms(2000)));
    EXPECT_EQ(fw.submit_count(), stopped_submits);
    const auto stats = g->transport().stats();
    EXPECT_EQ(stats.ack_timeouts, 0u);
    EXPECT_EQ(stats.crc_errors, 0u);
    EXPECT_EQ(stats.callback_exceptions, 0u);
    EXPECT_EQ(stats.queue_dropped, 0u);
    g->imu().off(imu_sub);
    g->close();
}

template <class Controller>
void slow_imu_callback_isolated_and_real_loss_times_out() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto gate = std::make_shared<ImuCallbackGate>();
    auto g = open_follower(pty);
    const auto imu_sub = g->imu().on_data([gate](const tx::ImuSample&) {
        (*gate)();
    });
    g->start_streaming(100, 100);
    typename ControllerTraits<Controller>::Config config;
    config.status_timeout_ms = 120;
    Controller controller(*g, config);
    ReleaseCallbackOnExit release_on_exit{gate};
    controller.start();
    ASSERT_TRUE(wait_for([&] {
        return fw.submit_count() >= 4 && gate->calls.load() >= 4;
    }, Ms(2000)));
    ASSERT_TRUE(controller.snapshot().observation.valid);

    gate->arm();
    ASSERT_TRUE(gate->wait_until_entered());
    const auto submits_before = fw.submit_count();
    const auto frames_before = g->transport().stats().frames_received;

    // A blocked user callback must not stall control feedback. Keep it blocked
    // for several status timeouts, while the independent control lane advances.
    EXPECT_NO_THROW((void)g->device().heartbeat(Ms(500)));
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= submits_before + 30; }, Ms(2000)));
    EXPECT_NE(controller.state(), ControllerTraits<Controller>::fault);
    EXPECT_TRUE(controller.snapshot().observation.valid);
    EXPECT_FALSE(gate->completed.load());

    // Real loss, not a user callback, must still trip the existing failsafe.
    // Backlogged user DATA must never refresh the controller on release.
    fw.freeze_stream(true);
    ASSERT_TRUE(wait_for([&] {
        return controller.state() == ControllerTraits<Controller>::fault &&
               fw.submit_count() > submits_before &&
               is_zero_torque(fw.last_submit());
    }, Ms(2000)));
    const auto snapshot = controller.snapshot();
    EXPECT_FALSE(snapshot.observation.valid);
    EXPECT_EQ(snapshot.fault_reason, "motor status stream stale");
    EXPECT_FALSE(gate->completed.load());
    EXPECT_GT(g->transport().stats().frames_received, frames_before);
    EXPECT_EQ(g->transport().stats().ack_timeouts, 0u);

    // Queued frames can resume after the callback returns; the fault must
    // remain latched rather than silently resuming an old motion command.
    const auto imus_before = gate->calls.load();
    gate->release();
    ASSERT_TRUE(wait_for([&] {
        return gate->completed.load() && gate->calls.load() >= imus_before + 8;
    }, Ms(2000)));
    EXPECT_EQ(controller.state(), ControllerTraits<Controller>::fault);
    EXPECT_TRUE(is_zero_torque(fw.last_submit()));
    controller.stop();
    EXPECT_EQ(fw.disable_count(), 1u);
    g->imu().off(imu_sub);
    g->close();
}
}  // namespace

TEST(ImuMotorConcurrency, ForcePositionKeepsPreexistingCombinedStream) {
    controller_keeps_preexisting_imu_stream<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceKeepsPreexistingCombinedStream) {
    controller_keeps_preexisting_imu_stream<tx::ImpedanceController>();
}

TEST(ImuMotorConcurrency, ForcePositionSlowImuIsIsolatedAndRealLossFaults) {
    slow_imu_callback_isolated_and_real_loss_times_out<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceSlowImuIsIsolatedAndRealLossFaults) {
    slow_imu_callback_isolated_and_real_loss_times_out<tx::ImpedanceController>();
}

TEST(ImuMotorConcurrency, CloseJoinsActiveImuCallbackAndReleasesCaptures) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto gate = std::make_shared<ImuCallbackGate>();
    auto capture = std::make_shared<int>(42);
    std::weak_ptr<int> weak_capture = capture;
    auto g = open_follower(pty);
    ReleaseCallbackOnExit release_on_exit{gate};
    g->imu().on_data([gate, capture](const tx::ImuSample&) { (*gate)(); });
    capture.reset();  // only the registered callback now owns the resource
    gate->arm();
    g->start_streaming(100, 100);
    ASSERT_TRUE(gate->wait_until_entered());
    const auto frames_before = g->transport().stats().frames_received;
    ASSERT_TRUE(wait_for([&] {
        return g->transport().stats().frames_received >= frames_before + 8;
    }, Ms(2000)));

    std::atomic<bool> close_returned{false};
    std::thread closer([&] {
        g->close();
        close_returned.store(true);
    });
    // Do not ASSERT while closer is joinable: release and join even when a
    // check fails, so the test itself cannot strand the dispatcher.
    EXPECT_TRUE(wait_for([&] { return !g->transport().is_running(); }, Ms(2000)));
    EXPECT_FALSE(close_returned.load());
    EXPECT_FALSE(gate->completed.load());
    EXPECT_FALSE(weak_capture.expired());
    const auto calls_before_release = gate->calls.load();
    gate->release();
    closer.join();

    EXPECT_TRUE(close_returned.load());
    EXPECT_TRUE(gate->completed.load());
    EXPECT_TRUE(g->is_closed());
    EXPECT_TRUE(weak_capture.expired());
    EXPECT_EQ(gate->calls.load(), calls_before_release)
        << "close must discard queued DATA instead of entering more callbacks";
    EXPECT_GT(g->transport().stats().queue_dropped, 0u);
}

TEST(ImuMotorConcurrency, QueuedUserSamplesKeepReceiveTime) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto gate = std::make_shared<ImuCallbackGate>();
    auto g = open_follower(pty);
    ReleaseCallbackOnExit release_on_exit{gate};
    std::atomic<bool> released{false};
    std::atomic<unsigned> old_motor{0}, old_imu{0};
    const auto motor_sub = g->motor().on_status([&](const tx::MotorStatusSample& s) {
        if (released && std::chrono::steady_clock::now() - s.host_time >= Ms(80)) ++old_motor;
    });
    const auto imu_sub = g->imu().on_data([&](const tx::ImuSample& s) {
        (*gate)();
        if (released && std::chrono::steady_clock::now() - s.host_time >= Ms(80)) ++old_imu;
    });
    gate->arm();
    g->start_streaming(100, 100);
    ASSERT_TRUE(gate->wait_until_entered());
    auto before = g->transport().stats().frames_received;
    ASSERT_TRUE(wait_for([&] { return g->transport().stats().frames_received >= before + 20; }, Ms(1000)));
    fw.freeze_stream(true);
    std::this_thread::sleep_for(Ms(120));
    released = true;
    gate->release();
    EXPECT_TRUE(wait_for([&] { return old_motor >= 5 && old_imu >= 5; }, Ms(1000)));
    g->motor().off(motor_sub);
    g->imu().off(imu_sub);
    g->close();
}

template <class Controller>
void sustained_slow_user_callbacks_do_not_retime_control() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_imu_v2_supported(true);
    auto g = open_follower(pty);
    const auto imu = g->imu().on_data([](const tx::ImuSample&) { std::this_thread::sleep_for(Ms(20)); });
    const auto motor = g->motor().on_status([](const tx::MotorStatusSample&) { std::this_thread::sleep_for(Ms(5)); });
    g->start_streaming(100, 100);
    typename ControllerTraits<Controller>::Config config;
    config.status_timeout_ms = 150;
    Controller controller(*g, config);
    controller.start();
    const auto before = fw.submit_count();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= before + 60; }, Ms(1200)));
    EXPECT_NE(controller.state(), ControllerTraits<Controller>::fault);
    EXPECT_TRUE(controller.snapshot().observation.valid);
    EXPECT_EQ(g->transport().stats().control_queue_overflows, 0u);
    fw.freeze_stream(true);
    // State changes before submit() and the PTY peer consumes it. Observe both
    // within the original deadline instead of racing the asynchronous send.
    EXPECT_TRUE(wait_for([&] {
        return controller.state() == ControllerTraits<Controller>::fault &&
               is_zero_torque(fw.last_submit());
    }, Ms(500)));
    controller.stop();
    g->motor().off(motor);
    g->imu().off(imu);
    g->close();
}

TEST(ImuMotorConcurrency, ForcePositionSustainedSlowCallbacksDoNotHideFeedbackLoss) {
    sustained_slow_user_callbacks_do_not_retime_control<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceSustainedSlowCallbacksDoNotHideFeedbackLoss) {
    sustained_slow_user_callbacks_do_not_retime_control<tx::ImpedanceController>();
}

TEST(ImuMotorConcurrency, ControlQueueOverflowLatchesClosedAndRequiresReopen) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionConfig config;
    config.status_timeout_ms = 200;
    tx::ForcePositionController controller(*g, config);
    controller.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 4; }, Ms(1000)));
    fw.freeze_stream(true);
    // A deliberately faster-than-real-wire burst exercises overload handling,
    // not physical throughput. Each write stays below the PTY buffer size.
    tp::MotorStatus status{};
    status.actual_pos = 0.60f;
    const auto payload = taccap_test::pod_bytes(&status, sizeof(status));
    std::vector<uint8_t> burst;
    for (unsigned i = 0; i < 64; ++i) {
        const auto frame = tx::bus::pack_frame(tp::Address::MCU, uint8_t(i), tp::FrameType::DATA,
                                               tp::Cmd::GetMotorStatus, payload);
        burst.insert(burst.end(), frame.begin(), frame.end());
    }
    for (unsigned i = 0; i < 40 && !g->transport().stats().control_queue_overflows; ++i) {
        pty.send_raw(burst);
        std::this_thread::sleep_for(Ms(5));
    }
    ASSERT_GT(g->transport().stats().control_queue_overflows, 0u);
    EXPECT_GT(g->transport().stats().control_queue_dropped, 0u);
    EXPECT_LE(g->transport().stats().control_queue_high_water, 8u);
    fw.freeze_stream(false);
    ASSERT_TRUE(wait_for([&] {
        return controller.state() == tx::ForcePositionState::Fault &&
               is_zero_torque(fw.last_submit());
    }, Ms(1000)));
    controller.stop();
    const auto stopped_submits = fw.submit_count();
    EXPECT_THROW(controller.start(), tx::IoError);
    EXPECT_FALSE(controller.running());
    EXPECT_FALSE(controller.snapshot().observation.valid);
    EXPECT_EQ(fw.submit_count(), stopped_submits);
    g->close();
}

template <class Controller>
void fault_then_healthy_cannot_hide_safety_evidence() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    Controller controller(*g);
    controller.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 4; }, Ms(1000)));
    fw.freeze_stream(true);
    tp::MotorStatus healthy{};
    healthy.actual_pos = 0.60f;
    for (unsigned kind = 0; kind < 3; ++kind) {
        auto bad = healthy;
        if (kind == 0) bad.status = tp::MotorStatusBit::OverCurrent;
        if (kind == 1) bad.actual_vel = std::numeric_limits<float>::quiet_NaN();
        if (kind == 2) bad.actual_torque = 19.0f;
        auto burst = tx::bus::pack_frame(tp::Address::MCU, 1, tp::FrameType::DATA,
            tp::Cmd::GetMotorStatus, taccap_test::pod_bytes(&bad, sizeof(bad)));
        const auto good = tx::bus::pack_frame(tp::Address::MCU, 2, tp::FrameType::DATA,
            tp::Cmd::GetMotorStatus, taccap_test::pod_bytes(&healthy, sizeof(healthy)));
        burst.insert(burst.end(), good.begin(), good.end());
        pty.send_raw(burst);
        ASSERT_TRUE(wait_for([&] { return controller.state() == ControllerTraits<Controller>::fault; }, Ms(1000)));
        const auto reason = controller.snapshot().fault_reason;
        EXPECT_NE(reason.find(kind == 0 ? "reports a fault" : kind == 1 ? "non-finite" : "torque exceeded"), std::string::npos);
        EXPECT_TRUE(wait_for([&] { return is_zero_torque(fw.last_submit()); }, Ms(1000)));
        EXPECT_EQ(g->transport().stats().control_queue_overflows, 0u);
        controller.reset();  // explicit reset establishes a new receive epoch
    }
    controller.stop();
    g->close();
}

TEST(ImuMotorConcurrency, ForcePositionFaultThenHealthyBurstStaysFaulted) {
    fault_then_healthy_cannot_hide_safety_evidence<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceFaultThenHealthyBurstStaysFaulted) {
    fault_then_healthy_cannot_hide_safety_evidence<tx::ImpedanceController>();
}

template <class Controller>
void initial_fault_ack_is_not_overwritten_by_healthy_stream() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    fw.set_next_polled_status_word(tp::MotorStatusBit::OverCurrent);
    Controller controller(*g);
    controller.start();
    EXPECT_TRUE(wait_for([&] { return controller.state() == ControllerTraits<Controller>::fault; }, Ms(1000)));
    EXPECT_EQ(controller.snapshot().fault_reason, "motor status reports a fault");
    EXPECT_TRUE(wait_for([&] { return is_zero_torque(fw.last_submit()); }, Ms(1000)));
    controller.stop();
    g->close();
}

TEST(ImuMotorConcurrency, ForcePositionInitialFaultAckIsLatched) {
    initial_fault_ack_is_not_overwritten_by_healthy_stream<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceInitialFaultAckIsLatched) {
    initial_fault_ack_is_not_overwritten_by_healthy_stream<tx::ImpedanceController>();
}

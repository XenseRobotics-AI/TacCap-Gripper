// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// IMU and motor DATA share a dispatcher, while ACK matching and controller
// safety timeouts run independently. Exercise that boundary through a PTY:
// the fake models packet flow, not firmware scheduling or physical motion.

#include "fake_follower.hpp"

#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
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
void slow_imu_callback_triggers_safe_timeout() {
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

    // A user callback stalls motor delivery, even though the firmware and
    // reader still run. ACK matching must remain usable during the stall.
    EXPECT_NO_THROW((void)g->device().heartbeat(Ms(500)));
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

TEST(ImuMotorConcurrency, ForcePositionSlowImuCallbackFaultsWithZeroTorque) {
    slow_imu_callback_triggers_safe_timeout<tx::ForcePositionController>();
}

TEST(ImuMotorConcurrency, ImpedanceSlowImuCallbackFaultsWithZeroTorque) {
    slow_imu_callback_triggers_safe_timeout<tx::ImpedanceController>();
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

#include "fake_follower.hpp"
#include <gtest/gtest.h>
#include <taccap/error.hpp>
#include <atomic>
#include <thread>

namespace tx = xense::taccap;
namespace tp = tx::protocol;
using namespace taccap_test;

TEST(FollowerImu, DefaultRemainsMotorOnlyWithoutProbe) {
    Pty pty; FakeFollower fw(pty); auto g = open_follower(pty);
    g->start_streaming();
    EXPECT_EQ(fw.imu_queries(), 0u);
    EXPECT_EQ(fw.stream_config().source_mask, tp::StreamSrc::MotorStatus);
    EXPECT_EQ(fw.stream_config().imu_rate, 0u);
    EXPECT_EQ(fw.stream_config().motor_rate, 100u);
    EXPECT_THROW(g->start_streaming(100, 50), tx::IoError);
}

TEST(FollowerImu, OldFirmwareFailsBeforeChangingStream) {
    Pty pty; FakeFollower fw(pty); auto g = open_follower(pty);
    const auto stops = fw.stream_stops();
    EXPECT_THROW(g->start_streaming(100, 50), tx::ProtocolError);
    EXPECT_EQ(fw.stream_stops(), stops);
    EXPECT_FALSE(g->is_streaming());
    EXPECT_EQ(fw.stream_config().source_mask, 0u);
    EXPECT_NO_THROW(g->start_streaming(100));
}

TEST(FollowerImu, OptInDeliversBothCallbacksWithUnchangedMotorRate) {
    Pty pty; FakeFollower fw(pty); fw.set_imu_supported(true);
    auto g = open_follower(pty);
    std::atomic<unsigned> imus{0}, motors{0};
    auto i = g->imu().on_data([&](const tx::ImuSample& s) {
        if (std::abs(s.accel_mps2[2] - 9.80665f) < .001f) ++imus;
    });
    auto m = g->motor().on_status([&](const auto&) { ++motors; });
    g->start_streaming(100, 50);
    for (int n = 0; n < 100 && (!imus || !motors); ++n)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_GT(imus.load(), 0u); EXPECT_GT(motors.load(), 0u);
    EXPECT_EQ(fw.stream_config().motor_rate, 100u);
    EXPECT_EQ(fw.stream_config().imu_rate, 50u);
    EXPECT_EQ(fw.stream_config().source_mask, tp::StreamSrc::Imu | tp::StreamSrc::MotorStatus);
    EXPECT_EQ(fw.stream_config().mode, static_cast<uint8_t>(tp::StreamMode::Separate));
    g->stop_streaming();
    g->imu().off(i); g->motor().off(m);
    g->close(); // Join dispatcher before callback captures go out of scope.
}

TEST(FollowerImu, ImuOnlyAndInvalidRates) {
    Pty pty; FakeFollower fw(pty); fw.set_imu_supported(true);
    auto g = open_follower(pty);
    EXPECT_THROW(g->start_streaming(0, 0), tx::IoError);
    EXPECT_THROW(g->start_streaming(100, 101), tx::IoError);
    EXPECT_EQ(fw.imu_queries(), 0u);
    g->start_streaming(0, 25);
    EXPECT_EQ(fw.stream_config().source_mask, tp::StreamSrc::Imu);
    EXPECT_EQ(fw.stream_config().motor_rate, 0u);
    EXPECT_THROW(g->start_streaming(), tx::IoError);
}

TEST(FollowerImu, CalibrationSavesOnlyAfterSuccessfulVerification) {
    Pty pty; tx::bus::Transport transport(base_config(pty.slave_path()));
    tx::IMU imu(transport);
    std::vector<uint8_t> actions;
    std::thread device([&] {
        for (int n = 0; n < 2; ++n) {
            auto f = pty.expect_frame();
            if (!f) return;
            EXPECT_EQ(f->cmd, tp::Cmd::SetImuCal);
            actions.push_back(f->payload.at(0));
            pty.send_ack_ok(f->seq, f->cmd);
        }
    });
    EXPECT_NO_THROW(imu.calibrate_accel_gyro());
    device.join();
    EXPECT_EQ(actions, (std::vector<uint8_t>{1, 4}));
}

TEST(FollowerImu, CalibrationNackDoesNotSave) {
    Pty pty; tx::bus::Transport transport(base_config(pty.slave_path()));
    tx::IMU imu(transport);
    std::thread device([&] {
        auto f = pty.expect_frame();
        if (f) pty.send_nack(f->seq, tp::ErrorCode::SysBusy);
        EXPECT_FALSE(pty.expect_frame(50).has_value());
    });
    EXPECT_THROW(imu.calibrate_accel_gyro(), tx::ProtocolError);
    device.join();
}

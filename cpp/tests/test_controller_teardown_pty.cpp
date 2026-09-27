// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Controller teardown against a gripper that is already gone. The destructor
// calls stop() again after an explicit one, and in Python a controller often
// outlives the `with FollowerGripper(...)` block that closed its port. Neither
// may warn "motor disable on stop failed": the motor WAS disabled, and a
// warning that says otherwise sends people chasing a fault that is not there.

#include "capture_log.hpp"
#include "fake_follower.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>

#include <gtest/gtest.h>

using taccap_test::CaptureLog;
using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;

namespace tx = xense::taccap;

template <class Controller>
void stop_then_close_then_destroy() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    CaptureLog log;
    {
        Controller c(*g);
        c.start();
        c.stop();
        g->close();
    }   // destructor: second stop(), port closed
    EXPECT_EQ(log.count("disable on stop failed"), 0u);
    EXPECT_EQ(fw.disable_count(), 1u) << "the explicit stop() disables, once";
}

template <class Controller>
void close_then_destroy_without_stop() {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    CaptureLog log;
    {
        Controller c(*g);
        c.start();
        g->close();
    }   // destructor: first stop(), port already closed
    EXPECT_EQ(log.count("disable on stop failed"), 0u);
}

TEST(ControllerTeardown, ForcePositionSecondStopAfterCloseIsSilent) {
    stop_then_close_then_destroy<tx::ForcePositionController>();
}
TEST(ControllerTeardown, ForcePositionStopAfterCloseIsSilent) {
    close_then_destroy_without_stop<tx::ForcePositionController>();
}
TEST(ControllerTeardown, ImpedanceSecondStopAfterCloseIsSilent) {
    stop_then_close_then_destroy<tx::ImpedanceController>();
}
TEST(ControllerTeardown, ImpedanceStopAfterCloseIsSilent) {
    close_then_destroy_without_stop<tx::ImpedanceController>();
}

// A stopped controller can be started again; stop() must then do its job
// again rather than remembering that it already ran once.
TEST(ControllerTeardown, RestartedControllerStopsAgain) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    tx::ForcePositionController c(*g);
    c.start();
    c.stop();
    c.start();
    ASSERT_TRUE(c.running());
    c.stop();
    EXPECT_FALSE(c.running());
    EXPECT_EQ(fw.disable_count(), 2u);
    EXPECT_FALSE(g->is_streaming()) << "second stop() must still release the stream";
}

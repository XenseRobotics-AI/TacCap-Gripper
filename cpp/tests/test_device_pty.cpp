// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Device (heartbeat / SN / device type) and the exclusive port open, against
// the pty-backed fake follower. No hardware.
//
// The setters are pinned hardest because they are factory operations: a wrong
// SN swaps left and right or routes the wrong role's firmware to the board.
// Each must refuse bad input before anything reaches the wire, and must notice
// when what it wrote is not what the flash holds.

#include "fake_follower.hpp"

#include <taccap/error.hpp>
#include <taccap/follower_gripper.hpp>

#include <cerrno>
#include <gtest/gtest.h>
#include <stdexcept>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;
using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;

TEST(DevicePty, HeartbeatDecodesUptimeAndVersion) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    const auto hb = g->device().heartbeat();
    EXPECT_EQ(hb.uptime_ms, 123456u);
    ASSERT_TRUE(g->firmware_version().has_value());
    EXPECT_EQ(hb.version.major, g->firmware_version()->major);
    EXPECT_EQ(hb.version.minor, g->firmware_version()->minor);
    EXPECT_EQ(hb.version.patch, g->firmware_version()->patch);
}

TEST(DevicePty, TheSnReadAtOpenIsKept) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    EXPECT_EQ(g->firmware_sn(), "TCGU01A28Z0001s");
    EXPECT_EQ(g->device().get_sn(), "TCGU01A28Z0001s");
}

TEST(DevicePty, SetSnRoundTrips) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->device().set_sn("TCGU01A28Z0099s");
    EXPECT_EQ(fw.stored_sn(), "TCGU01A28Z0099s");
    EXPECT_EQ(g->device().get_sn(), "TCGU01A28Z0099s");
}

TEST(DevicePty, SetSnRefusesBadLengthsBeforeTheWire) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    EXPECT_THROW(g->device().set_sn(""), std::invalid_argument);
    EXPECT_THROW(g->device().set_sn("TCGU01A28Z0001s-TOO-LONG"), std::invalid_argument);
    EXPECT_EQ(fw.stored_sn(), "TCGU01A28Z0001s") << "nothing may have been written";
}

TEST(DevicePty, SetSnCatchesAWriteThatDidNotLand) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_corrupt_sn_writes(true);
    auto g = open_follower(pty);
    EXPECT_THROW(g->device().set_sn("TCGU01A28Z0099s"), tx::ProtocolError);
}

TEST(DevicePty, DeviceTypeRoundTripsAndRefusesUnknown) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    EXPECT_EQ(g->device().get_device_type(), tp::DeviceType::Unknown);
    g->device().set_device_type(tp::DeviceType::Right);
    EXPECT_EQ(g->device().get_device_type(), tp::DeviceType::Right);
    EXPECT_THROW(g->device().set_device_type(tp::DeviceType::Unknown),
                 std::invalid_argument);
    EXPECT_EQ(g->device().get_device_type(), tp::DeviceType::Right);
}

// Two handles on one CDC port split its byte stream between them, and both
// then see CRC errors and lost ACKs that read as a link fault. The second open
// must fail cleanly instead, with EBUSY so discovery can report "in use".
TEST(ExclusiveOpen, ASecondHandleOnTheSamePortIsRefused) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto first = open_follower(pty);
    try {
        auto second = open_follower(pty);
        FAIL() << "second open of " << pty.slave_path() << " succeeded";
    } catch (const tx::IoError& e) {
        EXPECT_EQ(e.errno_value(), EBUSY) << e.what();
    }
    // The first handle is unaffected.
    EXPECT_EQ(first->device().get_sn(), "TCGU01A28Z0001s");
}

TEST(ExclusiveOpen, ThePortIsFreeAgainOnceTheHandleCloses) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    { auto first = open_follower(pty); }
    EXPECT_NO_THROW({ auto again = open_follower(pty); });
}

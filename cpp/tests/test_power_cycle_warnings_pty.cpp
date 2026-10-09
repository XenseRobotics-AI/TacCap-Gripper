// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// The warnings that tell a host a stored write needs a power cycle, and that a
// follower is running on an assumed motor model. Both are the ONLY place the
// host learns these things: the firmware logs them on UART7, which is not wired
// to USB, and the commands themselves just ACK OK.

#include "capture_log.hpp"
#include "fake_follower.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/log.hpp>

#include <gtest/gtest.h>


namespace tp = xense::taccap::protocol;
using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;

using taccap_test::CaptureLog;

TEST(PowerCycleWarnings, EveryStoredAdminWriteSaysCut24V) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    CaptureLog log;

    g->motor().switch_protocol(tp::MotorProtocol::Private);
    g->motor().set_startup_limit_torque(6.0f);
    g->motor().set_can_id(0x7F);
    g->motor().set_model(0);

    for (const char* what : {"switch_protocol", "set_startup_limit_torque",
                             "set_can_id", "set_model"}) {
        const std::string tag = std::string("Motor::") + what + ": stored";
        EXPECT_EQ(log.count(tag), 1u) << what;
    }
    // The instruction is the new rule: 24 V only, USB may stay.
    EXPECT_EQ(log.count("unplug the 24 V power cable"), 4u);
    EXPECT_EQ(log.count("USB may stay in"), 4u);
}

TEST(PowerCycleWarnings, OpeningOnTheCompileTimeDefaultModelWarns) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_model_from_flash(false);
    CaptureLog log;
    auto g = open_follower(pty);
    EXPECT_EQ(log.count("compile-time default (EL05)"), 1u);
}

TEST(PowerCycleWarnings, ARecordedModelOpensQuietly) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    CaptureLog log;
    auto g = open_follower(pty);
    EXPECT_EQ(log.count("compile-time default"), 0u);
}

// Below 1.2.11 a follower opens, but warns: one control-UART overrun used to
// silence the command channel for good (0086s, 2026-09-28).
TEST(FirmwareAdvisory, FollowerBelow1211WarnsButOpens) {
    struct V { uint8_t a, b, c; bool warn; };
    for (const V v : {V{1, 2, 5, true}, V{1, 2, 10, true}, V{1, 2, 11, false},
                      V{1, 2, 13, false}}) {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty, v.a, v.b, v.c);
        CaptureLog log;
        EXPECT_NO_THROW({ auto g = open_follower(pty); });
        EXPECT_EQ(log.count("低于 1.2.11") > 0, v.warn)
            << int(v.a) << "." << int(v.b) << "." << int(v.c);
    }
}

// RS00 followers are handed (mirror-mounted gear train): a left one must store
// Reverse 1, a right one Reverse 0. A wrong bit mirrors the normalized
// position, so opening warns; writing flash behind the caller's back is not
// the SDK's call, so it only warns.
TEST(OpenDirectionWarning, ALeftRs00StoredAsRightWarns) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_model_rs00(true);
    fw.set_sn("TCGU01A28Z0089s");    // odd -> left
    fw.set_reverse(false);
    CaptureLog log;
    auto g = open_follower(pty);
    EXPECT_EQ(log.count("open direction does not match the Left hand"), 1u);
    EXPECT_EQ(log.count("0x0003"), 1u) << "the warning names the flags to write";
}

TEST(OpenDirectionWarning, CorrectlyHandedRs00OpensQuietly) {
    for (const bool left : {true, false}) {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty);
        fw.set_model_rs00(true);
        fw.set_sn(left ? "TCGU01A28Z0089s" : "TCGU01A28Z0094s");
        fw.set_reverse(left);
        CaptureLog log;
        auto g = open_follower(pty);
        EXPECT_EQ(log.count("open direction does not match"), 0u) << (left ? "left" : "right");
    }
}

TEST(OpenDirectionWarning, El05IsNotChecked) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);                // EL05
    fw.set_sn("TCGU01A28Z0094s");
    fw.set_reverse(true);
    CaptureLog log;
    auto g = open_follower(pty);
    EXPECT_EQ(log.count("open direction does not match"), 0u);
}

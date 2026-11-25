// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Discovery SN-parsing tests. parse_serial() / side_from_serial() are pure
// string functions, so no hardware or transport is needed here.

#include <gtest/gtest.h>
#include <taccap/discovery.hpp>

namespace d = xense::taccap::discovery;
using Side = d::Side;
using Role = d::Role;

// ---- Full TacCap grammar: grippers (TCGU…, with m/s patch) ----------------

TEST(ParseSerial, GripperLeaderLeft) {
    auto p = d::parse_serial("TCGU01A24Z0001m");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.product, "TCGU01");
    EXPECT_EQ(p.batch, "A24");
    EXPECT_EQ(p.line, 'Z');
    EXPECT_EQ(p.sequence, "0001");
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // seq ends 1 (odd)
    EXPECT_EQ(p.role, Role::Leader);    // patch m
}

TEST(ParseSerial, GripperLeaderRightProductionLine) {
    auto p = d::parse_serial("TCGU01A24A0002m");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.line, 'A');             // production
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // seq ends 2 (even)
    EXPECT_EQ(p.role, Role::Leader);
}

TEST(ParseSerial, GripperFollowerLeft) {
    auto p = d::parse_serial("TCGU01A24Z0003s");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // seq ends 3 (odd)
    EXPECT_EQ(p.role, Role::Follower);  // patch s
}

TEST(ParseSerial, GripperFollowerRight) {
    auto p = d::parse_serial("TCGU01A24A0004s");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // seq ends 4 (even)
    EXPECT_EQ(p.role, Role::Follower);
}

TEST(ParseSerial, SeqEndingZeroIsRight) {
    auto p = d::parse_serial("TCGU01A24Z0010m");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // 0 is even
}

// ---- Full grammar: visuotactile sensors (GSPS…, no patch) -----------------

TEST(ParseSerial, SensorLeftNoRole) {
    auto p = d::parse_serial("GSPS01A24Z0001");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.product, "GSPS01");
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);
    EXPECT_EQ(p.role, Role::Unknown);   // sensors carry no m/s
}

TEST(ParseSerial, SensorRightNoRole) {
    auto p = d::parse_serial("GSPS01A24A0002");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);
    EXPECT_EQ(p.role, Role::Unknown);
}

// ---- Graceful degradation: legacy / empty / loose inputs ------------------

TEST(ParseSerial, LegacySnRecoversSideOnly) {
    auto p = d::parse_serial("SN000002");
    EXPECT_FALSE(p.valid);              // not the TacCap grammar
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // last digit 2 (even)
    EXPECT_EQ(p.role, Role::Unknown);
}

TEST(ParseSerial, EmptyIsUnknownNoSide) {
    auto p = d::parse_serial("");
    EXPECT_FALSE(p.valid);
    EXPECT_FALSE(p.side.has_value());
    EXPECT_EQ(p.role, Role::Unknown);
}

TEST(ParseSerial, LooseTrailingPatchStillGivesRole) {
    // Not full grammar, but a trailing m/s should still surface the role,
    // and the last digit the side.
    auto p = d::parse_serial("weird-0007s");
    EXPECT_FALSE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // 7 odd
    EXPECT_EQ(p.role, Role::Follower);
}

// ---- side_from_serial back-compat + to_string -----------------------------

TEST(SideFromSerial, MatchesParseSideForNewScheme) {
    // The legacy last-digit helper must still agree with the new parser for
    // the new format (patch suffix is a non-digit, so it's skipped).
    EXPECT_EQ(d::side_from_serial("TCGU01A24Z0001m"), Side::Left);
    EXPECT_EQ(d::side_from_serial("TCGU01A24A0002m"), Side::Right);
    EXPECT_EQ(d::side_from_serial("GSPS01A24Z0003"),  Side::Left);
    EXPECT_FALSE(d::side_from_serial("nodigits").has_value());
}

TEST(ToString, RoleAndSide) {
    EXPECT_STREQ(d::to_string(Role::Leader), "Leader");
    EXPECT_STREQ(d::to_string(Role::Follower), "Follower");
    EXPECT_STREQ(d::to_string(Role::Unknown), "Unknown");
    EXPECT_STREQ(d::to_string(Side::Left), "Left");
    EXPECT_STREQ(d::to_string(Side::Right), "Right");
    EXPECT_STREQ(d::to_string(Side::Unknown), "Unknown");
}

// A freshly-constructed endpoint has Unknown side: discovery never guesses a
// side from the CH343 chip SN — it stays Unknown until a firmware source
// (GetSn / GetDevType) resolves it.
TEST(Discovery, GripperEndpointsDefaultSideIsUnknown) {
    d::GripperEndpoints e{};
    EXPECT_EQ(e.side, Side::Unknown);
    EXPECT_EQ(e.role, Role::Unknown);
}

// ---- select_side: the pick find_left()/find_right() make from a scan -------
//
// The bench that motivated the role filter: one host with a left follower, a
// left leader and a right follower. Scan order follows the CH343 serial, so
// a role-less find_left() got the follower only because its serial sorted
// first.

namespace {
d::GripperEndpoints ep(const char* sn, const char* dev) {
    d::GripperEndpoints e{};
    const auto p = d::parse_serial(sn);
    e.side = p.side ? *p.side : Side::Unknown;
    e.role = p.role;
    e.firmware_sn = sn;
    e.mcu_device = dev;
    return e;
}
}  // namespace

TEST(SelectSide, RoleFilterSkipsLeaderScannedFirst) {
    const std::vector<d::GripperEndpoints> scan = {
        ep("TCGU01A28Z0095m", "/dev/a"),   // left leader, scans first
        ep("TCGU01A45A0017s", "/dev/b"),   // left follower
        ep("TCGU01A45A0016s", "/dev/c"),   // right follower
    };
    bool amb = false;
    auto f = d::select_side(scan, Side::Left, Role::Follower, &amb);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->firmware_sn, "TCGU01A45A0017s");
    EXPECT_FALSE(amb);

    auto l = d::select_side(scan, Side::Left, Role::Leader, &amb);
    ASSERT_TRUE(l.has_value());
    EXPECT_EQ(l->firmware_sn, "TCGU01A28Z0095m");

    auto r = d::select_side(scan, Side::Right, Role::Follower, &amb);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->firmware_sn, "TCGU01A45A0016s");
    EXPECT_FALSE(amb);
}

TEST(SelectSide, NoRoleKeepsFirstOnSideAndFlagsAmbiguity) {
    const std::vector<d::GripperEndpoints> scan = {
        ep("TCGU01A28Z0095m", "/dev/a"),
        ep("TCGU01A45A0017s", "/dev/b"),
        ep("TCGU01A45A0016s", "/dev/c"),
    };
    bool amb = false;
    auto l = d::select_side(scan, Side::Left, std::nullopt, &amb);
    ASSERT_TRUE(l.has_value());
    EXPECT_EQ(l->firmware_sn, "TCGU01A28Z0095m");  // unchanged behaviour: first on side
    EXPECT_TRUE(amb);

    auto r = d::select_side(scan, Side::Right, std::nullopt, &amb);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(amb);                              // one unit on the right: no warning
}

TEST(SelectSide, NoMatchForRoleOrSide) {
    const std::vector<d::GripperEndpoints> scan = {ep("TCGU01A28Z0095m", "/dev/a")};
    EXPECT_FALSE(d::select_side(scan, Side::Left, Role::Follower).has_value());
    EXPECT_FALSE(d::select_side(scan, Side::Right, std::nullopt).has_value());
    EXPECT_FALSE(d::select_side({}, Side::Left, std::nullopt).has_value());
}

TEST(SelectSide, UnknownRoleNeverMatchesARoleFilter) {
    // Legacy SN: side recovered, role Unknown. A role filter must not take it.
    const std::vector<d::GripperEndpoints> scan = {ep("TCGU01A24Z0003", "/dev/a")};
    EXPECT_FALSE(d::select_side(scan, Side::Left, Role::Follower).has_value());
    EXPECT_TRUE(d::select_side(scan, Side::Left, std::nullopt).has_value());
}

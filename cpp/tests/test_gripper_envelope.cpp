// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// The motion safety envelope's decision table, tested without a device.
//
// Two things make this worth pinning hard. First, the record is opt-in and the
// firmware's rejections are silent, so every wrong answer here looks like a
// working gripper right up until a blocked jaw browns out the 24 V rail.
// Second, what the device STORES is not what the firmware ENFORCES -- it
// clamps cont down to the installed motor's stall rating and logs that on a
// UART that is not wired to USB. A bench unit was found storing cont=1.800
// while running at 1.100, which is the case reproduced verbatim below.

#include <gtest/gtest.h>

#include <taccap/follower_gripper.hpp>
#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>

#include <cstring>
#include <optional>
#include <vector>

namespace {

namespace tp = xense::taccap::protocol;

using xense::taccap::EnvelopeAudit;
using xense::taccap::ForcePositionConfig;
using xense::taccap::ImpedanceConfig;
using xense::taccap::MOTOR_RATED_TORQUE_NM;
using xense::taccap::MOTOR_STALL_CONT_TORQUE_NM;
namespace Issue = xense::taccap::GripperEnvelopeIssue;
using xense::taccap::detail::audit_envelope;
using xense::taccap::detail::recommend_envelope;
using xense::taccap::detail::repair_envelope;

// The firmware's own EL05 row (App/drivers/motor_spec.c), copied by value so a
// change on either side shows up as a test failure rather than as drift.
tp::MotorSpec el05() {
    tp::MotorSpec s{};
    std::strncpy(s.name, "EL05", sizeof(s.name));
    s.p_max_rad            = 12.57f;
    s.v_max_rad_s          = 50.0f;
    s.t_max_nm             = 6.0f;
    s.kp_max               = 500.0f;
    s.kd_max               = 5.0f;
    s.rated_torque_nm      = 1.8f;
    s.stall_cont_torque_nm = 1.1f;
    s.winding_limit_c      = 135;
    s.board_limit_c        = 103;
    return s;
}

// RS00: real rated torque, NO stall rating in the firmware's table. This is
// not hypothetical -- every RS0x row carries stall_cont_torque_nm == 0.
tp::MotorSpec rs00() {
    tp::MotorSpec s{};
    std::strncpy(s.name, "RS00", sizeof(s.name));
    s.t_max_nm             = 14.0f;
    s.rated_torque_nm      = 5.0f;
    s.stall_cont_torque_nm = 0.0f;
    return s;
}

constexpr uint16_t kCurrentFlags =
    tp::GripperEnvelopeFlag::Valid | tp::GripperEnvelopeFlag::Enforce |
    tp::GripperEnvelopeFlag::LayoutBits;

tp::GripperEnvelope record(float cont, float peak, uint16_t flags = kCurrentFlags) {
    tp::GripperEnvelope e{};
    e.cont_torque_nm = cont;
    e.peak_torque_nm = peak;
    e.flags          = flags;
    return e;
}

// ---- recommend_envelope -----------------------------------------------------

TEST(EnvelopeRecommend, TakesBothNumbersFromTheDeviceSpec) {
    const auto spec = el05();
    const auto r    = recommend_envelope(spec);
    EXPECT_FLOAT_EQ(r.cont_torque_nm, spec.stall_cont_torque_nm);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, spec.rated_torque_nm);
    EXPECT_EQ(r.flags, kCurrentFlags);
}

TEST(EnvelopeRecommend, PeakIsTheRatedTorqueNotTheTorqueRange) {
    // t_max_nm (6.0) is the motor's absolute ceiling AND the firmware's own
    // default for the persisted 0x700B startup limit. Writing it here would
    // make the envelope a no-op relative to a protection that already exists.
    // "peak should obviously be the peak rating" is the edit this guards.
    const auto spec = el05();
    const auto r    = recommend_envelope(spec);
    EXPECT_NE(r.peak_torque_nm, spec.t_max_nm);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, 1.8f);
}

TEST(EnvelopeRecommend, LeavesPeakAboveContSoTheI2tDerateEngages) {
    // The firmware skips the I2t accumulator entirely when peak <= cont.
    EXPECT_GT(recommend_envelope(el05()).peak_torque_nm,
              recommend_envelope(el05()).cont_torque_nm);
}

TEST(EnvelopeRecommend, LeavesTheTemperaturePairAtFirmwareDefault) {
    const auto r = recommend_envelope(el05());
    EXPECT_EQ(r.temp_derate_start_c, 0u);
    EXPECT_EQ(r.temp_wall_c, 0u);
}

TEST(EnvelopeRecommend, FallsBackToCompiledConstantsWithNoSpec) {
    const auto r = recommend_envelope(std::nullopt);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, MOTOR_RATED_TORQUE_NM);
    EXPECT_FLOAT_EQ(r.cont_torque_nm, MOTOR_STALL_CONT_TORQUE_NM);
}

TEST(EnvelopeRecommend, TakesPeakFromAnRs00ButNotCont) {
    // The mix the two from_device flags exist for: rated is real, the stall
    // rating is absent, so cont silently comes from the EL05 fallback.
    const auto a = audit_envelope(record(0.0f, 0.0f, 0), rs00());
    EXPECT_TRUE(a.peak_from_device);
    EXPECT_FALSE(a.cont_from_device);
    EXPECT_FLOAT_EQ(a.recommended.peak_torque_nm, 5.0f);
    EXPECT_FLOAT_EQ(a.recommended.cont_torque_nm, MOTOR_STALL_CONT_TORQUE_NM);
    EXPECT_NE(a.detail.find("stall rating"), std::string::npos)
        << "the fallback must say so: " << a.detail;
}

TEST(EnvelopeRecommend, DoesNotInventAMarginOnASmallActuator) {
    // rated below the stall fallback: min() keeps cont from exceeding peak,
    // and we report the empty I2t band rather than widening peak to open one.
    tp::MotorSpec small{};
    std::strncpy(small.name, "TINY", sizeof(small.name));
    small.rated_torque_nm = 1.0f;
    const auto r = recommend_envelope(small);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, 1.0f);
    EXPECT_FLOAT_EQ(r.cont_torque_nm, 1.0f);
    EXPECT_TRUE(audit_envelope(r, small).issues & Issue::PeakNotAboveCont);
}

// ---- audit_envelope: gating -------------------------------------------------

TEST(EnvelopeAuditGating, FactoryRecordReportsOnlyNotWritten) {
    // flags == 0 also fails the layout comparison. Reporting both would read
    // as two unrelated faults when there is only one.
    const auto a = audit_envelope(tp::GripperEnvelope{}, el05());
    EXPECT_EQ(a.issues, Issue::NotWritten);
    EXPECT_FALSE(a.effective.has_value())
        << "a factory device enforces nothing at all -- not even the temp wall";
    EXPECT_TRUE(a.needs_write());
    EXPECT_FALSE(a.ok());
}

TEST(EnvelopeAuditGating, StoredButNotEnforcedIsIgnoredByTheFirmware) {
    const auto a = audit_envelope(
        record(1.1f, 1.8f, tp::GripperEnvelopeFlag::Valid |
                               tp::GripperEnvelopeFlag::LayoutBits),
        el05());
    EXPECT_EQ(a.issues, Issue::NotEnforced);
    EXPECT_FALSE(a.effective.has_value());
}

TEST(EnvelopeAuditGating, LayoutMismatchStopsBeforeReadingAnyField) {
    // The nibble exists because a field-order change kept sizeof at 16 and
    // shifted every value: the numbers looked right and meant something else.
    const uint16_t v1 = tp::GripperEnvelopeFlag::Valid |
                        tp::GripperEnvelopeFlag::Enforce |
                        static_cast<uint16_t>(1u << 12);
    const auto a = audit_envelope(record(0.0f, 0.0f, v1), el05());
    EXPECT_EQ(a.issues, Issue::LayoutMismatch);
    EXPECT_FALSE(a.effective.has_value());
    EXPECT_FALSE(a.issues & Issue::PeakUnlimited)
        << "field bytes must not be judged when the layout is not ours";
}

// ---- audit_envelope: values -------------------------------------------------

TEST(EnvelopeAuditValues, StoredContAboveStallRatingIsNotWhatIsEnforced) {
    // TCGU01A28Z0015s as found on the bench: stored 1.800/3.000, enforced
    // 1.100. Nothing on the host could see the difference before this.
    const auto a = audit_envelope(record(1.8f, 3.0f), el05());
    EXPECT_TRUE(a.issues & Issue::ContAboveStallRating);
    ASSERT_TRUE(a.effective.has_value());
    EXPECT_FLOAT_EQ(a.effective->cont_torque_nm, 1.1f);
    EXPECT_FLOAT_EQ(a.effective->peak_torque_nm, 3.0f)
        << "the firmware clamps cont, not peak";
    EXPECT_FLOAT_EQ(a.stored.cont_torque_nm, 1.8f) << "stored must stay as read";
}

TEST(EnvelopeAuditValues, WithoutASpecTheClampIsNotGuessed) {
    const auto a = audit_envelope(record(1.8f, 3.0f), std::nullopt);
    EXPECT_FALSE(a.issues & Issue::ContAboveStallRating);
    ASSERT_TRUE(a.effective.has_value());
    EXPECT_FLOAT_EQ(a.effective->cont_torque_nm, 1.8f);
}

TEST(EnvelopeAuditValues, ZeroPeakLeavesTheCommandPathUnbounded) {
    // Prints as ENFORCED; the firmware's position-error clamp returns the
    // target untouched, so kp*error has no ceiling at all.
    const auto a = audit_envelope(record(1.1f, 0.0f), el05());
    EXPECT_TRUE(a.issues & Issue::PeakUnlimited);
    EXPECT_TRUE(a.needs_write());
}

TEST(EnvelopeAuditValues, ZeroContMeansNoSustainedCeiling) {
    EXPECT_TRUE(audit_envelope(record(0.0f, 1.8f), el05()).issues &
                Issue::ContUnlimited);
}

TEST(EnvelopeAuditValues, ADeliberatelyTighterEnvelopeIsNotAFault) {
    const auto a = audit_envelope(record(0.6f, 1.5f), el05());
    EXPECT_TRUE(a.ok()) << a.detail;
    EXPECT_FALSE(a.needs_write());
    ASSERT_TRUE(a.effective.has_value());
    EXPECT_FLOAT_EQ(a.effective->cont_torque_nm, 0.6f);
}

TEST(EnvelopeAuditValues, TheRecommendationAuditsClean) {
    EXPECT_TRUE(audit_envelope(recommend_envelope(el05()), el05()).ok());
}

TEST(EnvelopeAuditValues, PeakNotAboveContIsReportedButNotRepaired) {
    const auto a = audit_envelope(record(1.1f, 1.1f), el05());
    EXPECT_TRUE(a.issues & Issue::PeakNotAboveCont);
    EXPECT_FALSE(a.needs_write()) << "tighter than recommended is not a hole";
    EXPECT_FALSE(a.ok());
}

// ---- repair_envelope --------------------------------------------------------

TEST(EnvelopeRepair, NeverWidensADeliberatelyTighterRecord) {
    // Only the missing Enforce bit is repaired. Raising 0.6 to 1.1 would
    // quietly undo an operator's tightening for a delicate task.
    const auto a = audit_envelope(
        record(0.6f, 1.5f, tp::GripperEnvelopeFlag::Valid |
                               tp::GripperEnvelopeFlag::LayoutBits),
        el05());
    const auto r = repair_envelope(a);
    EXPECT_FLOAT_EQ(r.cont_torque_nm, 0.6f);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, 1.5f);
    EXPECT_EQ(r.flags, kCurrentFlags);
}

TEST(EnvelopeRepair, ReplacesBothInventedNumbersOnTheBenchRecord) {
    const auto r = repair_envelope(audit_envelope(record(1.8f, 3.0f), el05()));
    EXPECT_FLOAT_EQ(r.cont_torque_nm, 1.1f);
    EXPECT_FLOAT_EQ(r.peak_torque_nm, 1.8f);
}

TEST(EnvelopeRepair, WritesTheRecommendationWholesaleWhenNothingIsTrustworthy) {
    const uint16_t v1 = tp::GripperEnvelopeFlag::Valid |
                        tp::GripperEnvelopeFlag::Enforce |
                        static_cast<uint16_t>(1u << 12);
    const auto r = repair_envelope(audit_envelope(record(0.3f, 0.4f, v1), el05()));
    EXPECT_FLOAT_EQ(r.cont_torque_nm, 1.1f)
        << "a record whose layout we cannot read must not seed the repair";
    EXPECT_FLOAT_EQ(r.peak_torque_nm, 1.8f);
}

TEST(EnvelopeRepair, CarriesTemperaturesThroughIncludingZero) {
    auto stored               = record(1.1f, 1.8f, tp::GripperEnvelopeFlag::Valid |
                                                       tp::GripperEnvelopeFlag::LayoutBits);
    stored.temp_derate_start_c = 70;
    stored.temp_wall_c         = 80;
    const auto r = repair_envelope(audit_envelope(stored, el05()));
    EXPECT_EQ(r.temp_derate_start_c, 70u);
    EXPECT_EQ(r.temp_wall_c, 80u);
}

TEST(EnvelopeRepair, ACarriedPeakNeverCollapsesTheI2tBand) {
    // peak 0.5 is "stricter" but would sit under cont and switch the derate
    // off. The recommendation's peak is restored instead.
    const auto r = repair_envelope(audit_envelope(record(1.1f, 0.5f), el05()));
    EXPECT_GT(r.peak_torque_nm, r.cont_torque_nm);
}

TEST(EnvelopeRepair, ConvergesInOneStep) {
    const std::vector<tp::GripperEnvelope> records{
        tp::GripperEnvelope{},
        record(1.8f, 3.0f),
        record(1.1f, 0.0f),
        record(0.0f, 1.8f),
        record(0.6f, 1.5f, tp::GripperEnvelopeFlag::Valid |
                               tp::GripperEnvelopeFlag::LayoutBits),
        record(0.3f, 0.4f, tp::GripperEnvelopeFlag::Valid |
                               tp::GripperEnvelopeFlag::Enforce |
                               static_cast<uint16_t>(1u << 12)),
    };
    for (const auto& stored : records) {
        const auto repaired = repair_envelope(audit_envelope(stored, el05()));
        EXPECT_FALSE(audit_envelope(repaired, el05()).needs_write())
            << "repair must not need a second pass";
    }
}

// ---- cross-layer invariants -------------------------------------------------
//
// Getting either of these backwards silently derates every default grasp.

TEST(EnvelopeCrossLayer, FirmwarePeakSitsAtOrAboveTheHostErrorClamp) {
    // The host's clamp must bind first, or the envelope stops being a backstop
    // and becomes the primary controller.
    EXPECT_GE(recommend_envelope(el05()).peak_torque_nm,
              ImpedanceConfig{}.max_position_torque_nm);
}

TEST(EnvelopeCrossLayer, FirmwareContSitsAtOrAboveTheDefaultGrasp) {
    // Otherwise the default force-position hold lives permanently inside the
    // I2t derate and never delivers the grip it reports.
    EXPECT_GE(recommend_envelope(el05()).cont_torque_nm,
              ForcePositionConfig{}.grasp_torque_nm);
}

}  // namespace

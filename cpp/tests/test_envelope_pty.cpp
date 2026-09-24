// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ensure_envelope() against a fake device — the parts the pure policy cannot
// see: that it writes once and then stops, that it does not take the travel
// calibration down with it, and that a device which cannot answer 0x56 still
// ends up protected.

#include "fake_follower.hpp"

#include <taccap/follower_gripper.hpp>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::FakeFollower;
using taccap_test::Pty;
using taccap_test::open_follower;

namespace Issue = xense::taccap::GripperEnvelopeIssue;

namespace {

constexpr uint16_t kCurrentFlags =
    tp::GripperEnvelopeFlag::Valid | tp::GripperEnvelopeFlag::Enforce |
    tp::GripperEnvelopeFlag::LayoutBits;

}  // namespace

TEST(EnvelopePty, FactoryDeviceIsAuditedAsEnforcingNothing) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    const auto a = g->audit_envelope();
    EXPECT_EQ(a.issues, Issue::NotWritten);
    EXPECT_FALSE(a.effective.has_value());
    EXPECT_EQ(a.motor_model, "EL05");
    EXPECT_TRUE(a.peak_from_device);
    EXPECT_TRUE(a.cont_from_device);
    EXPECT_EQ(fw.config_writes(), 0u) << "an audit must never write";
}

TEST(EnvelopePty, EnsureWritesTheDeviceDerivedRecord) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    const auto w = g->ensure_envelope();
    ASSERT_TRUE(w.wrote);
    EXPECT_EQ(fw.config_writes(), 1u);

    const auto stored = fw.stored_envelope();
    EXPECT_FLOAT_EQ(stored.cont_torque_nm, 1.1f);  // EL05 stall rating
    EXPECT_FLOAT_EQ(stored.peak_torque_nm, 1.8f);  // EL05 rotating rating
    EXPECT_EQ(stored.flags, kCurrentFlags);
}

TEST(EnvelopePty, EnsureIsIdempotent) {
    // The central claim. Reading the returned struct cannot establish it --
    // only the device-side write counter can.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    ASSERT_TRUE(g->ensure_envelope().wrote);
    const unsigned after_first = fw.config_writes();

    const auto second = g->ensure_envelope();
    EXPECT_FALSE(second.wrote);
    EXPECT_EQ(fw.config_writes(), after_first)
        << "a second ensure_envelope() wrote flash again";
    EXPECT_TRUE(second.before.ok()) << second.before.detail;
}

TEST(EnvelopePty, TheWritePreservesTheTravelCalibration) {
    // set_envelope() republishes the whole GripperConfig. If it ever assembled
    // one from scratch, max_open_rad would go to zero and the gripper would
    // read as having no travel -- silently, on the next power cycle.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    ASSERT_TRUE(g->ensure_envelope().wrote);

    const auto cfg = fw.stored_config();
    EXPECT_FLOAT_EQ(cfg.max_open_rad, 1.30f);
    EXPECT_EQ(cfg.magic, tp::GRIPPER_CONFIG_MAGIC);
    EXPECT_TRUE(cfg.flags & tp::GripperConfigFlag::Valid);
}

TEST(EnvelopePty, ARecordStoringMoreThanIsEnforcedIsRepairedDownwards) {
    // The bench unit's state: stored 1.800/3.000, enforced 1.100.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);

    tp::GripperEnvelope inflated{};
    inflated.cont_torque_nm = 1.8f;
    inflated.peak_torque_nm = 3.0f;
    inflated.flags          = kCurrentFlags;
    fw.set_stored_envelope(inflated);

    auto g = open_follower(pty);

    const auto before = g->audit_envelope();
    EXPECT_TRUE(before.issues & Issue::ContAboveStallRating);
    ASSERT_TRUE(before.effective.has_value());
    EXPECT_FLOAT_EQ(before.effective->cont_torque_nm, 1.1f);

    ASSERT_TRUE(g->ensure_envelope().wrote);
    EXPECT_FLOAT_EQ(fw.stored_envelope().cont_torque_nm, 1.1f);
    EXPECT_FLOAT_EQ(fw.stored_envelope().peak_torque_nm, 1.8f);
    EXPECT_TRUE(g->audit_envelope().ok());
}

TEST(EnvelopePty, ATighterRecordIsLeftAloneExceptForItsFlags) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);

    tp::GripperEnvelope tight{};
    tight.cont_torque_nm = 0.6f;
    tight.peak_torque_nm = 1.5f;
    tight.flags = tp::GripperEnvelopeFlag::Valid |
                  tp::GripperEnvelopeFlag::LayoutBits;  // Enforce missing
    fw.set_stored_envelope(tight);

    auto g = open_follower(pty);
    ASSERT_TRUE(g->ensure_envelope().wrote);

    const auto stored = fw.stored_envelope();
    EXPECT_FLOAT_EQ(stored.cont_torque_nm, 0.6f)
        << "ensure_envelope() raised a deliberately tightened grip";
    EXPECT_FLOAT_EQ(stored.peak_torque_nm, 1.5f);
    EXPECT_EQ(stored.flags, kCurrentFlags);
}

TEST(EnvelopePty, ADeviceThatCannotAnswerTheSpecStillGetsAnEnvelope) {
    // No envelope at all is the brown-out failure; a conservative one derived
    // from the compiled-in ratings is strictly better than nothing.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_spec_supported(false);
    auto g = open_follower(pty);

    const auto w = g->ensure_envelope();
    ASSERT_TRUE(w.wrote);
    EXPECT_FALSE(w.before.peak_from_device);
    EXPECT_FALSE(w.before.cont_from_device);
    EXPECT_TRUE(w.before.motor_model.empty());
    EXPECT_FLOAT_EQ(fw.stored_envelope().cont_torque_nm,
                    tx::MOTOR_STALL_CONT_TORQUE_NM);
    EXPECT_FLOAT_EQ(fw.stored_envelope().peak_torque_nm,
                    tx::MOTOR_RATED_TORQUE_NM);
}

TEST(EnvelopePty, WithoutASpecAnInflatedRecordIsNotJudged) {
    // The firmware would still clamp it, but the host cannot know the rating,
    // so it must not rewrite a record it cannot evaluate.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    fw.set_spec_supported(false);

    tp::GripperEnvelope inflated{};
    inflated.cont_torque_nm = 1.8f;
    inflated.peak_torque_nm = 3.0f;
    inflated.flags          = kCurrentFlags;
    fw.set_stored_envelope(inflated);

    auto g = open_follower(pty);
    const auto a = g->audit_envelope();
    EXPECT_FALSE(a.issues & Issue::ContAboveStallRating);
    EXPECT_FALSE(a.needs_write());
    EXPECT_EQ(g->ensure_envelope().wrote, false);
    EXPECT_EQ(fw.config_writes(), 0u);
}

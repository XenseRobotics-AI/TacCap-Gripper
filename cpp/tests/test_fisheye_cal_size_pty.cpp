// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// End-to-end tests for the V2.10 fisheye calibration resolution (Cmd 0x2B ops
// 0x02 / 0x03), against the fake follower in both of its firmware personas:
// V2.10 (firmware 1.3.0-rc1 / 1.3.1), which records the size, and older firmware, which
// answers op=0x02 with InvalidParam.
//
// What matters here is the seam between the two: the capability probe, the
// fallbacks on old firmware (legacy read with an unknown size; legacy write
// only for 640x480), and resolve_fisheye()'s use of a recorded, inferred or
// uninferable size.

#include "fake_follower.hpp"

#include <gtest/gtest.h>
#include <taccap/components/calibration.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;
using taccap_test::FakeFollower;
using taccap_test::open_follower;
using taccap_test::Pty;
using taccap_test::base_config;

namespace {

constexpr uint8_t kRead       = 0x00;
constexpr uint8_t kWrite      = 0x01;
constexpr uint8_t kReadSized  = tp::CAMERA_FISHEYE_CAL_OP_READ_SIZED;
constexpr uint8_t kWriteSized = tp::CAMERA_FISHEYE_CAL_OP_WRITE_SIZED;

const tp::CameraFisheyeCal kVga{213.0303f, 212.7928f, 321.4f, 239.95f,
                                -0.0172f, 0.0091f, -0.0146f, 0.0051f};
const tp::CameraFisheyeCal kHd{477.2f, 476.9f, 968.3f, 563.1f,
                               -0.021f, 0.0061f, -0.0042f, 0.0011f};

bool same(const tp::CameraFisheyeCal& a, const tp::CameraFisheyeCal& b) {
    return std::memcmp(&a, &b, sizeof(a)) == 0;
}

}  // namespace

// ---- Capability probe -----------------------------------------------------

TEST(FisheyeCalSizePty, NewFirmwareSupportsTheSizeEvenWhenUncalibrated) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    // Never written: the sized read answers CalNotSet, which is still V2.10.
    EXPECT_TRUE(g->calibration().supports_fisheye_size());
    EXPECT_EQ(fw.fisheye_ops(), std::vector<uint8_t>{kReadSized});
}

TEST(FisheyeCalSizePty, OldFirmwareInvalidParamMeansUnsupported) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_size_supported(false);
    fw.set_fisheye_record(kVga);
    auto g = open_follower(pty);
    EXPECT_FALSE(g->calibration().supports_fisheye_size());
}

TEST(FisheyeCalSizePty, TheAnswerIsCached) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_size_supported(false);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    EXPECT_FALSE(cal.supports_fisheye_size());
    EXPECT_FALSE(cal.supports_fisheye_size());
    // read_fisheye_record() also skips the probe once the answer is known.
    fw.set_fisheye_record(kVga);
    ASSERT_TRUE(cal.read_fisheye_record().has_value());
    EXPECT_EQ(fw.fisheye_ops(), (std::vector<uint8_t>{kReadSized, kRead}));
}

TEST(FisheyeCalSizePty, ATransportFailureIsNotCached) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    // Nobody answers the first probe (nor its retries).
    EXPECT_THROW((void)cal.supports_fisheye_size(), tx::IoError);
    // Drop the unanswered attempts still queued on the pty.
    while (pty.expect_frame(20)) {}

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                                 {static_cast<uint8_t>(tp::ErrorCode::CalNotSet)});
    });
    bool supported = false;
    EXPECT_NO_THROW(supported = cal.supports_fisheye_size());
    fw.join();
    EXPECT_TRUE(supported);
}

// ---- read_fisheye_record --------------------------------------------------

TEST(FisheyeCalSizePty, ReadRecordOnNewFirmwareCarriesTheSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    const auto r = g->calibration().read_fisheye_record();
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(same(r->calibration, kHd));
    EXPECT_EQ(r->width, 1920u);
    EXPECT_EQ(r->height, 1080u);
    EXPECT_TRUE(r->size_known());
    // One round trip: the read doubles as the probe.
    EXPECT_EQ(fw.fisheye_ops(), std::vector<uint8_t>{kReadSized});
}

TEST(FisheyeCalSizePty, ReadRecordOnNewFirmwareCanReportAnUnknownSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_record(kVga);   // e.g. written with op 0x01
    auto g = open_follower(pty);
    const auto r = g->calibration().read_fisheye_record();
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->size_known());
}

TEST(FisheyeCalSizePty, ReadRecordOnOldFirmwareFallsBackToTheLegacyRead) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_size_supported(false);
    fw.set_fisheye_record(kVga);
    auto g = open_follower(pty);
    const auto r = g->calibration().read_fisheye_record();
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(same(r->calibration, kVga));
    EXPECT_EQ(r->width, 0u);
    EXPECT_EQ(r->height, 0u);
    EXPECT_EQ(fw.fisheye_ops(), (std::vector<uint8_t>{kReadSized, kRead}));
}

TEST(FisheyeCalSizePty, ReadRecordIsEmptyWhenNeverCalibrated) {
    for (bool sized : {true, false}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_size_supported(sized);
        auto g = open_follower(pty);
        EXPECT_FALSE(g->calibration().read_fisheye_record().has_value())
            << "sized=" << sized;
    }
}

// ---- write_fisheye_sized ---------------------------------------------------

TEST(FisheyeCalSizePty, WriteSizedOnNewFirmwareRecordsTheSize) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    g->calibration().write_fisheye_sized(kHd, 1920, 1080);
    ASSERT_TRUE(fw.stored_fisheye().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye(), kHd));
    EXPECT_EQ(fw.stored_fisheye_size(), (std::pair<uint16_t, uint16_t>{1920, 1080}));
    EXPECT_EQ(fw.fisheye_ops(), (std::vector<uint8_t>{kReadSized, kWriteSized}));

    // And it reads back with its size.
    const auto r = g->calibration().read_fisheye_record();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->width, 1920u);
}

TEST(FisheyeCalSizePty, LegacyWriteOnNewFirmwareLeavesTheSizeUnknown) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    g->calibration().write_fisheye(kVga);
    EXPECT_EQ(fw.stored_fisheye_size(), (std::pair<uint16_t, uint16_t>{0, 0}));
}

TEST(FisheyeCalSizePty, WriteSizedOnOldFirmwareFallsBackForVga) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_size_supported(false);
    auto g = open_follower(pty);
    g->calibration().write_fisheye_sized(kVga, 640, 480);
    ASSERT_TRUE(fw.stored_fisheye().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye(), kVga));
    EXPECT_EQ(fw.fisheye_ops(), (std::vector<uint8_t>{kReadSized, kWrite}));
}

TEST(FisheyeCalSizePty, WriteSizedOnOldFirmwareRefuses1080p) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_size_supported(false);
    auto g = open_follower(pty);
    try {
        g->calibration().write_fisheye_sized(kHd, 1920, 1080);
        FAIL() << "expected Error";
    } catch (const tx::ProtocolError&) {
        FAIL() << "a refusal is the host's decision, not a firmware NACK";
    } catch (const tx::Error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("cannot record the calibration resolution"), std::string::npos) << msg;
        EXPECT_NE(msg.find("1.3.1"), std::string::npos) << msg;
    }
    // Nothing was written: only the probe went out.
    EXPECT_FALSE(fw.stored_fisheye().has_value());
    EXPECT_EQ(fw.fisheye_ops(), std::vector<uint8_t>{kReadSized});
}

TEST(FisheyeCalSizePty, WriteSizedRejectsImplausibleValuesBeforeSending) {
    Pty pty;
    FakeFollower fw(pty);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    // The firmware's own example: 1080p params (cx ~ 968) labelled 640x480.
    EXPECT_THROW(cal.write_fisheye_sized(kHd, 640, 480), tx::Error);
    EXPECT_THROW(cal.write_fisheye_sized(kVga, 0, 480), tx::Error);
    EXPECT_THROW(cal.write_fisheye_sized(kVga, 640, 5000), tx::Error);
    auto zero_f = kVga;
    zero_f.fx = 0.0f;
    EXPECT_THROW(cal.write_fisheye_sized(zero_f, 640, 480), tx::Error);
    auto inf_k = kVga;
    inf_k.k1 = std::numeric_limits<float>::infinity();
    EXPECT_THROW(cal.write_fisheye_sized(inf_k, 640, 480), tx::Error);
    EXPECT_TRUE(fw.fisheye_ops().empty());
    EXPECT_FALSE(fw.stored_fisheye().has_value());
}

TEST(FisheyeCalSizePty, WriteSizedFirmwareRejectionThrowsProtocolError) {
    // A V2.10 firmware NACK (raw pty, so the reply is under test control).
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::vector<uint8_t> ops;
    std::thread fw([&]() {
        // Probe: supported, uncalibrated.
        auto f = pty.expect_frame();
        if (!f) return;
        ops.push_back(f->payload.at(0));
        pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                          {static_cast<uint8_t>(tp::ErrorCode::CalNotSet)});
        // Sized write: flash write failed.
        f = pty.expect_frame();
        if (!f) return;
        ops.push_back(f->payload.at(0));
        pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                          {static_cast<uint8_t>(tp::ErrorCode::SysBusy)});
    });
    EXPECT_THROW(cal.write_fisheye_sized(kHd, 1920, 1080), tx::ProtocolError);
    fw.join();
    EXPECT_EQ(ops, (std::vector<uint8_t>{kReadSized, kWriteSized}));
}

// ---- resolve_fisheye ---------------------------------------------------------

TEST(FisheyeCalSizePty, ResolveUsesTheRecordedSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    const auto r = g->calibration().resolve_fisheye();
    EXPECT_FALSE(r.is_reference);
    EXPECT_TRUE(r.reason.empty());
    EXPECT_TRUE(same(r.calibration, kHd));
    EXPECT_EQ(r.calibrated_size, cv::Size(1920, 1080));
    EXPECT_FALSE(r.size_inferred);
}

TEST(FisheyeCalSizePty, ResolveInfersAnUnknownSize) {
    for (bool sized : {true, false}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_size_supported(sized);
        fw.set_fisheye_record(kHd);   // no size recorded
        auto g = open_follower(pty);
        const auto r = g->calibration().resolve_fisheye();
        EXPECT_FALSE(r.is_reference) << r.reason;
        EXPECT_TRUE(same(r.calibration, kHd));
        EXPECT_EQ(r.calibrated_size, cv::Size(1920, 1080));
        EXPECT_TRUE(r.size_inferred);
    }
}

TEST(FisheyeCalSizePty, ResolveFallsBackWhenTheSizeCannotBeInferred) {
    Pty pty;
    FakeFollower fw(pty);
    auto odd = kHd;
    odd.cx = 600.0f;
    odd.cy = 400.0f;
    fw.set_fisheye_record(odd);
    auto g = open_follower(pty);
    const auto r = g->calibration().resolve_fisheye();
    EXPECT_TRUE(r.is_reference);
    EXPECT_TRUE(same(r.calibration, tx::FISHEYE_FALLBACK_CAL));
    EXPECT_NE(r.reason.find("resolution is unknown and could not be inferred"),
              std::string::npos) << r.reason;
    EXPECT_EQ(r.calibrated_size, cv::Size(640, 480));
    EXPECT_FALSE(r.size_inferred);
}

TEST(FisheyeCalSizePty, ResolveReferenceFallbackIs640x480) {
    Pty pty;
    FakeFollower fw(pty);   // never calibrated
    auto g = open_follower(pty);
    const auto r = g->calibration().resolve_fisheye();
    EXPECT_TRUE(r.is_reference);
    EXPECT_EQ(r.reason, "the wrist lens has never been calibrated");
    EXPECT_EQ(r.calibrated_size, cv::Size(640, 480));
    EXPECT_FALSE(r.size_inferred);
}

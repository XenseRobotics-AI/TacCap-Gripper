// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// End-to-end tests for the V2.11 dual fisheye calibration slots (Cmd 0x2B:
// op 0x03 routed by size, new op 0x04 read-slot, tightened plausibility),
// against the fake follower's three firmware personas: V2.11 (1.3.0-rc4 /
// 1.3.1, two slots), V2.10 (1.3.0-rc1..rc3, one sized record) and older
// firmware (no size at all).
//
// The seams under test: the slot probe on every generation, slot isolation
// (an HD write never touches VGA), the single-record emulation of
// read_fisheye_slot() on older firmware, 0.4.7-era calls still behaving on
// V2.11, and resolve_fisheye_for() picking the slot for the camera's size.

#include "fake_follower.hpp"
#include "wrist_fisheye.hpp"

#include <gtest/gtest.h>
#include <taccap/components/calibration.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;
using taccap_test::FakeFollower;
using taccap_test::open_follower;
using taccap_test::Pty;
using Fw = FakeFollower::FisheyeFirmware;

namespace {

constexpr uint8_t kRead       = 0x00;
constexpr uint8_t kReadSized  = tp::CAMERA_FISHEYE_CAL_OP_READ_SIZED;
constexpr uint8_t kWriteSized = tp::CAMERA_FISHEYE_CAL_OP_WRITE_SIZED;
constexpr uint8_t kReadSlot   = tp::CAMERA_FISHEYE_CAL_OP_READ_SLOT;

const tp::CameraFisheyeCal kVga{213.0303f, 212.7928f, 321.4f, 239.95f,
                                -0.0172f, 0.0091f, -0.0146f, 0.0051f};
const tp::CameraFisheyeCal kHd{477.2f, 476.9f, 968.3f, 563.1f,
                               -0.021f, 0.0061f, -0.0042f, 0.0011f};

bool same(const tp::CameraFisheyeCal& a, const tp::CameraFisheyeCal& b) {
    return std::memcmp(&a, &b, sizeof(a)) == 0;
}

using Ops = std::vector<uint8_t>;

// Sends one raw 0x2B request, bypassing every host-side check, and returns
// the firmware's error code (Ok on success).
tp::ErrorCode raw_cal(tx::FollowerGripper& g, const std::vector<uint8_t>& payload) {
    try {
        (void)g.transport().send_cmd(tp::Cmd::CameraFisheyeCal, payload,
                                     std::chrono::milliseconds(300));
        return tp::ErrorCode::Ok;
    } catch (const tx::NackError& e) {
        return static_cast<tp::ErrorCode>(e.code());
    }
}

}  // namespace

// ---- Capability probe on all three generations ----------------------------

TEST(FisheyeDualSlotPty, ProbeOnV211EvenWhenUncalibrated) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    EXPECT_TRUE(cal.supports_fisheye_slots());
    // Cached, and implies the sized ops without a second probe.
    EXPECT_TRUE(cal.supports_fisheye_slots());
    EXPECT_TRUE(cal.supports_fisheye_size());
    EXPECT_EQ(fw.fisheye_ops(), Ops{kReadSlot});
}

TEST(FisheyeDualSlotPty, ProbeOnV210IsFalse) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_10);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    EXPECT_FALSE(g->calibration().supports_fisheye_slots());
    EXPECT_FALSE(g->calibration().supports_fisheye_slots());
    EXPECT_EQ(fw.fisheye_ops(), Ops{kReadSlot});
}

TEST(FisheyeDualSlotPty, ProbeOnOldFirmwareIsFalse) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::Legacy);
    auto g = open_follower(pty);
    EXPECT_FALSE(g->calibration().supports_fisheye_slots());
    EXPECT_EQ(fw.fisheye_ops(), Ops{kReadSlot});
}

TEST(FisheyeDualSlotPty, NoProbeOnceTheSizeIsKnownUnsupported) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::Legacy);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    EXPECT_FALSE(cal.supports_fisheye_size());
    EXPECT_FALSE(cal.supports_fisheye_slots());
    EXPECT_EQ(fw.fisheye_ops(), Ops{kReadSized});
}

// ---- Slot reads and isolation on V2.11 -------------------------------------

TEST(FisheyeDualSlotPty, ReadSlotReadsEachSlot) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    fw.set_fisheye_hd_record(kHd);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto vga = cal.read_fisheye_slot(640, 480);
    ASSERT_TRUE(vga.has_value());
    EXPECT_TRUE(same(vga->calibration, kVga));
    EXPECT_EQ(vga->width, 640u);
    EXPECT_EQ(vga->height, 480u);
    const auto hd = cal.read_fisheye_slot(1920, 1080);
    ASSERT_TRUE(hd.has_value());
    EXPECT_TRUE(same(hd->calibration, kHd));
    EXPECT_EQ(hd->width, 1920u);
    EXPECT_EQ(hd->height, 1080u);
    EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSlot, kReadSlot}));
}

TEST(FisheyeDualSlotPty, EmptySlotIsNullopt) {
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        fw.set_fisheye_record(kVga, 640, 480);
        auto g = open_follower(pty);
        EXPECT_FALSE(g->calibration().read_fisheye_slot(1920, 1080).has_value());
    }
    {
        // HD set, VGA empty: every VGA read (0x00, 0x02, 0x04) is CalNotSet.
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        fw.set_fisheye_hd_record(kHd);
        auto g = open_follower(pty);
        auto& cal = g->calibration();
        EXPECT_FALSE(cal.read_fisheye_slot(640, 480).has_value());
        EXPECT_FALSE(cal.read_fisheye_record().has_value());
        EXPECT_FALSE(cal.read_fisheye().has_value());
        EXPECT_TRUE(cal.read_fisheye_slot(1920, 1080).has_value());
    }
}

TEST(FisheyeDualSlotPty, UnsizedVgaSlotReadsBackWithAnUnknownSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    cal.write_fisheye(kVga);   // op 0x01: VGA slot, size unknown
    const auto r = cal.read_fisheye_slot(640, 480);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(same(r->calibration, kVga));
    EXPECT_FALSE(r->size_known());
}

TEST(FisheyeDualSlotPty, ReadSlotRefusesASizeWithNoSlot) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    try {
        (void)g->calibration().read_fisheye_slot(1280, 720);
        FAIL() << "expected Error";
    } catch (const tx::ProtocolError&) {
        FAIL() << "a host-side refusal, not a firmware NACK";
    } catch (const tx::Error& e) {
        EXPECT_NE(std::string(e.what()).find("only 640x480 and 1920x1080"),
                  std::string::npos) << e.what();
    }
    EXPECT_TRUE(fw.fisheye_ops().empty());
}

TEST(FisheyeDualSlotPty, HdWriteDoesNotClobberVga) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    cal.write_fisheye_sized(kHd, 1920, 1080);
    ASSERT_TRUE(fw.stored_fisheye().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye(), kVga));
    EXPECT_EQ(fw.stored_fisheye_size(), (std::pair<uint16_t, uint16_t>{640, 480}));
    ASSERT_TRUE(fw.stored_fisheye_hd().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), kHd));
    // A slot-size write needs only the size probe, not the slot probe.
    EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSized, kWriteSized}));

    // And a VGA write leaves HD alone.
    auto vga2 = kVga;
    vga2.fx = 220.0f;
    cal.write_fisheye_sized(vga2, 640, 480);
    EXPECT_TRUE(same(*fw.stored_fisheye(), vga2));
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), kHd));
}

// ---- 0.4.7-era calls on V2.11 ------------------------------------------------

TEST(FisheyeDualSlotPty, LegacyCallsAddressTheVgaSlot) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    auto& cal = g->calibration();

    // An HD-only unit: the 0.4.7 read (op 0x02) sees the empty VGA slot. This
    // is the confusing-but-harmless case the CHANGELOG describes.
    cal.write_fisheye_sized(kHd, 1920, 1080);
    EXPECT_FALSE(cal.read_fisheye_record().has_value());
    EXPECT_TRUE(cal.resolve_fisheye().is_reference);

    cal.write_fisheye_sized(kVga, 640, 480);
    const auto r = cal.read_fisheye_record();
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(same(r->calibration, kVga));
    EXPECT_EQ(r->width, 640u);
    const auto res = cal.resolve_fisheye();
    EXPECT_FALSE(res.is_reference) << res.reason;
    EXPECT_TRUE(same(res.calibration, kVga));
    EXPECT_EQ(res.calibrated_size, cv::Size(640, 480));
    // The HD slot survived all of it.
    ASSERT_TRUE(fw.stored_fisheye_hd().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), kHd));
}

// ---- read_fisheye_slot on single-record firmware ----------------------------

TEST(FisheyeDualSlotPty, ReadSlotOnV210MatchesTheRecordedSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_10);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto hd = cal.read_fisheye_slot(1920, 1080);
    ASSERT_TRUE(hd.has_value());
    EXPECT_TRUE(same(hd->calibration, kHd));
    EXPECT_FALSE(cal.read_fisheye_slot(640, 480).has_value());
    EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSlot, kReadSized, kReadSized}));
}

TEST(FisheyeDualSlotPty, ReadSlotOnV210TreatsAnUnsizedRecordAsVga) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_10);
    fw.set_fisheye_record(kVga);   // written with op 0x01
    auto g = open_follower(pty);
    const auto r = g->calibration().read_fisheye_slot(640, 480);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->size_known());
    EXPECT_FALSE(g->calibration().read_fisheye_slot(1920, 1080).has_value());
}

TEST(FisheyeDualSlotPty, ReadSlotOnOldFirmwareIsTheLegacyRecordAtVgaOnly) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::Legacy);
    fw.set_fisheye_record(kVga);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto r = cal.read_fisheye_slot(640, 480);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(same(r->calibration, kVga));
    EXPECT_FALSE(r->size_known());
    EXPECT_FALSE(cal.read_fisheye_slot(1920, 1080).has_value());
    EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSlot, kReadSized, kRead, kRead}));
}

// ---- resolve_fisheye_for ---------------------------------------------------

TEST(FisheyeDualSlotPty, ResolveForPicksTheSlotForTheCameraSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    fw.set_fisheye_hd_record(kHd);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto hd = cal.resolve_fisheye_for(1920, 1080);
    EXPECT_FALSE(hd.is_reference) << hd.reason;
    EXPECT_TRUE(same(hd.calibration, kHd));
    EXPECT_EQ(hd.calibrated_size, cv::Size(1920, 1080));
    EXPECT_FALSE(hd.size_inferred);
    const auto vga = cal.resolve_fisheye_for(640, 480);
    EXPECT_FALSE(vga.is_reference) << vga.reason;
    EXPECT_TRUE(same(vga.calibration, kVga));
    EXPECT_EQ(vga.calibrated_size, cv::Size(640, 480));
}

TEST(FisheyeDualSlotPty, ResolveForAnEmptySlotFallsBackToTheReference) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    auto g = open_follower(pty);
    const auto r = g->calibration().resolve_fisheye_for(1920, 1080);
    EXPECT_TRUE(r.is_reference);
    // The 1920x1080 reference, not the 640x480 one (a different sensor crop).
    EXPECT_TRUE(same(r.calibration, tx::FISHEYE_FALLBACK_CAL_HD));
    EXPECT_NE(r.reason.find("never been calibrated at 1920x1080"), std::string::npos)
        << r.reason;
    EXPECT_EQ(r.calibrated_size, cv::Size(1920, 1080));
}

TEST(FisheyeDualSlotPty, ResolveForInfersAnUnsizedVgaSlot) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga);   // VGA slot, size unknown
    auto g = open_follower(pty);
    const auto r = g->calibration().resolve_fisheye_for(640, 480);
    EXPECT_FALSE(r.is_reference) << r.reason;
    EXPECT_TRUE(r.size_inferred);
    EXPECT_EQ(r.calibrated_size, cv::Size(640, 480));
}

TEST(FisheyeDualSlotPty, ResolveForOnV210UsesTheRecordOnlyAtItsSize) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_10);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto hd = cal.resolve_fisheye_for(1920, 1080);
    EXPECT_FALSE(hd.is_reference) << hd.reason;
    EXPECT_TRUE(same(hd.calibration, kHd));
    const auto vga = cal.resolve_fisheye_for(640, 480);
    EXPECT_TRUE(vga.is_reference);
    EXPECT_TRUE(same(vga.calibration, tx::FISHEYE_FALLBACK_CAL));
    EXPECT_NE(vga.reason.find("belongs to 1920x1080, not 640x480"), std::string::npos)
        << vga.reason;
}

TEST(FisheyeDualSlotPty, ResolveForOnOldFirmwareInfers) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::Legacy);
    fw.set_fisheye_record(kVga);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto vga = cal.resolve_fisheye_for(640, 480);
    EXPECT_FALSE(vga.is_reference) << vga.reason;
    EXPECT_TRUE(vga.size_inferred);
    const auto hd = cal.resolve_fisheye_for(1920, 1080);
    EXPECT_TRUE(hd.is_reference);
    EXPECT_NE(hd.reason.find("(inferred)"), std::string::npos) << hd.reason;
}

// ---- Tightened plausibility -----------------------------------------------

TEST(FisheyeDualSlotPty, HostRefusesAVgaSetLabelled1080p) {
    for (Fw gen : {Fw::V2_11, Fw::V2_10, Fw::Legacy}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(gen);
        auto g = open_follower(pty);
        // V2.10's rule alone would accept it: cx = 321 lies inside 1920.
        ASSERT_TRUE(tx::fisheye_cal_size_plausible(kVga, 1920, 1080));
        try {
            g->calibration().write_fisheye_sized(kVga, 1920, 1080);
            FAIL() << "expected Error";
        } catch (const tx::ProtocolError&) {
            FAIL() << "a host-side refusal, not a firmware NACK";
        } catch (const tx::Error& e) {
            EXPECT_NE(std::string(e.what()).find("within 15% of the frame centre"),
                      std::string::npos) << e.what();
        }
        EXPECT_TRUE(fw.fisheye_ops().empty());
    }
}

TEST(FisheyeDualSlotPty, FakeRejectsAVgaSetLabelled1080pWhenTheHostIsBypassed) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_hd_record(kHd);
    auto g = open_follower(pty);
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_write_sized(kVga, 1920, 1080)),
              tp::ErrorCode::InvalidParam);
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), kHd));
    EXPECT_FALSE(fw.stored_fisheye().has_value());
}

TEST(FisheyeDualSlotPty, CentreWindowIsInclusive) {
    EXPECT_TRUE(tx::fisheye_cal_slot_plausible(kVga, 640, 480));
    EXPECT_TRUE(tx::fisheye_cal_slot_plausible(kHd, 1920, 1080));
    auto edge = kVga;
    edge.cx = 224.0f;   // 320 - 0.15 * 640
    edge.cy = 312.0f;   // 240 + 0.15 * 480
    EXPECT_TRUE(tx::fisheye_cal_slot_plausible(edge, 640, 480));
    auto out = edge;
    out.cx = 223.5f;
    EXPECT_FALSE(tx::fisheye_cal_slot_plausible(out, 640, 480));
    EXPECT_FALSE(tx::fisheye_cal_slot_plausible(kVga, 1280, 720));

    // Host and fake agree on the boundary.
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    EXPECT_NO_THROW(g->calibration().write_fisheye_sized(edge, 640, 480));
    EXPECT_THROW(g->calibration().write_fisheye_sized(out, 640, 480), tx::Error);
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_write_sized(out, 640, 480)),
              tp::ErrorCode::InvalidParam);
    EXPECT_TRUE(same(*fw.stored_fisheye(), edge));
}

TEST(FisheyeDualSlotPty, OtherSizesAreRefusedOnV211Only) {
    const tp::CameraFisheyeCal k720{320.0f, 320.0f, 641.0f, 359.0f, 0, 0, 0, 0};
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        auto g = open_follower(pty);
        try {
            g->calibration().write_fisheye_sized(k720, 1280, 720);
            FAIL() << "expected Error";
        } catch (const tx::ProtocolError&) {
            FAIL() << "a host-side refusal, not a firmware NACK";
        } catch (const tx::Error& e) {
            EXPECT_NE(std::string(e.what()).find("640x480 and for 1920x1080 only"),
                      std::string::npos) << e.what();
        }
        EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSized, kReadSlot}));
        EXPECT_FALSE(fw.stored_fisheye().has_value());
        // The fake refuses it too.
        EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_write_sized(k720, 1280, 720)),
                  tp::ErrorCode::InvalidParam);
    }
    {
        // V2.10 keeps its looser rule: any plausible size up to 4096.
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_10);
        auto g = open_follower(pty);
        g->calibration().write_fisheye_sized(k720, 1280, 720);
        EXPECT_EQ(fw.stored_fisheye_size(), (std::pair<uint16_t, uint16_t>{1280, 720}));
        EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSized, kReadSlot, kWriteSized}));
    }
}

// ---- Op 0x04 errors on the wire --------------------------------------------

TEST(FisheyeDualSlotPty, ReadSlotWireErrors) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_read_slot(1280, 720)),
              tp::ErrorCode::InvalidParam);
    EXPECT_EQ(raw_cal(*g, {kReadSlot, 0x80, 0x02}), tp::ErrorCode::LengthMismatch);
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_read_slot(640, 480)),
              tp::ErrorCode::CalNotSet);
}

TEST(FisheyeDualSlotPty, ReadSlotEncodingAndDecoding) {
    EXPECT_EQ(tp::encode_camera_fisheye_cal_read_slot(1920, 1080),
              (std::vector<uint8_t>{0x04, 0x80, 0x07, 0x38, 0x04}));
    std::vector<uint8_t> reply{kReadSlot};
    reply.resize(tp::CAMERA_FISHEYE_CAL_SIZED_SIZE);
    std::memcpy(reply.data() + 1, &kHd, sizeof(kHd));
    reply[33] = 0x80; reply[34] = 0x07; reply[35] = 0x38; reply[36] = 0x04;
    const auto d = tp::decode_camera_fisheye_cal_slot(reply.data(), reply.size());
    EXPECT_TRUE(same(d.calibration, kHd));
    EXPECT_EQ(d.width, 1920u);
    EXPECT_EQ(d.height, 1080u);
    // The read-sized op byte is not accepted as a slot reply.
    reply[0] = kReadSized;
    EXPECT_THROW(tp::decode_camera_fisheye_cal_slot(reply.data(), reply.size()),
                 tx::ProtocolError);
}

// ---- Op 0x05 clear slot ------------------------------------------------------

namespace {
constexpr uint8_t kClearSlot = tp::CAMERA_FISHEYE_CAL_OP_CLEAR_SLOT;
}  // namespace

TEST(FisheyeDualSlotPty, ClearHdLeavesVga) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    fw.set_fisheye_hd_record(kHd);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    cal.clear_fisheye_slot(1920, 1080);
    EXPECT_FALSE(fw.stored_fisheye_hd().has_value());
    EXPECT_FALSE(cal.read_fisheye_slot(1920, 1080).has_value());
    const auto vga = cal.read_fisheye_slot(640, 480);
    ASSERT_TRUE(vga.has_value());
    EXPECT_TRUE(same(vga->calibration, kVga));
    EXPECT_EQ(vga->width, 640u);
    EXPECT_EQ(fw.fisheye_ops(), (Ops{kReadSlot, kClearSlot, kReadSlot, kReadSlot}));
}

TEST(FisheyeDualSlotPty, ClearVgaDropsItsSizeAndReadsNone) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kVga, 640, 480);
    fw.set_fisheye_hd_record(kHd);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    cal.clear_fisheye_slot(640, 480);
    EXPECT_FALSE(fw.stored_fisheye().has_value());
    EXPECT_EQ(fw.stored_fisheye_size(), (std::pair<uint16_t, uint16_t>{0, 0}));
    EXPECT_FALSE(cal.read_fisheye_slot(640, 480).has_value());
    EXPECT_FALSE(cal.read_fisheye_record().has_value());
    EXPECT_FALSE(cal.read_fisheye().has_value());
    ASSERT_TRUE(fw.stored_fisheye_hd().has_value());
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), kHd));
}

TEST(FisheyeDualSlotPty, ClearingAnEmptySlotIsOk) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    EXPECT_NO_THROW(g->calibration().clear_fisheye_slot(1920, 1080));
    EXPECT_NO_THROW(g->calibration().clear_fisheye_slot(640, 480));
}

TEST(FisheyeDualSlotPty, ClearIsRefusedHostSideWithoutSlots) {
    for (Fw gen : {Fw::V2_10, Fw::Legacy}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(gen);
        fw.set_fisheye_record(kVga, 640, 480);
        auto g = open_follower(pty);
        try {
            g->calibration().clear_fisheye_slot(640, 480);
            FAIL() << "expected Error";
        } catch (const tx::ProtocolError&) {
            FAIL() << "a host-side refusal, not a firmware NACK";
        } catch (const tx::Error& e) {
            EXPECT_NE(std::string(e.what()).find("cannot clear one"), std::string::npos)
                << e.what();
        }
        // Only the probe went out; the record is intact.
        EXPECT_EQ(fw.fisheye_ops(), Ops{kReadSlot});
        EXPECT_TRUE(fw.stored_fisheye().has_value());
    }
}

TEST(FisheyeDualSlotPty, ClearRefusesASizeWithNoSlot) {
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    EXPECT_THROW(g->calibration().clear_fisheye_slot(1280, 720), tx::Error);
    EXPECT_TRUE(fw.fisheye_ops().empty());
    // The fake answers the wire errors like the slot read.
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_clear_slot(1280, 720)),
              tp::ErrorCode::InvalidParam);
    EXPECT_EQ(raw_cal(*g, {kClearSlot, 0x80, 0x02, 0xE0}), tp::ErrorCode::LengthMismatch);
    EXPECT_EQ(tp::encode_camera_fisheye_cal_clear_slot(640, 480),
              (std::vector<uint8_t>{0x05, 0x80, 0x02, 0xE0, 0x01}));
}

// ---- Wrist undistortion decision (install_wrist_undistorter) -----------------
// detail::resolve_wrist_fisheye() is the camera-free half of
// install_wrist_undistorter(): own calibration at the camera size -> use it;
// none at all -> reference; own calibration only at the other size -> throw.

namespace {

void expect_other_size_refusal(tx::Calibration& cal, cv::Size camera,
                               const std::string& want, const std::string& have) {
    try {
        (void)tx::detail::resolve_wrist_fisheye(cal, camera, "TestGripper");
        FAIL() << "expected Error";
    } catch (const tx::ProtocolError& e) {
        FAIL() << "a host-side refusal, not a firmware NACK: " << e.what();
    } catch (const tx::Error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("runs at " + want), std::string::npos) << msg;
        EXPECT_NE(msg.find("calibration is for " + have), std::string::npos) << msg;
        EXPECT_NE(msg.find("calibrate this unit at " + want +
                           ", or run the camera at " + have),
                  std::string::npos) << msg;
    }
}

}  // namespace

TEST(WristFisheyeDecisionPty, UsesTheCalibrationForTheCameraSize) {
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        fw.set_fisheye_record(kVga, 640, 480);
        fw.set_fisheye_hd_record(kHd);
        auto g = open_follower(pty);
        auto& cal = g->calibration();
        const auto hd = tx::detail::resolve_wrist_fisheye(cal, {1920, 1080}, "T");
        EXPECT_FALSE(hd.is_reference);
        EXPECT_TRUE(same(hd.calibration, kHd));
        const auto vga = tx::detail::resolve_wrist_fisheye(cal, {640, 480}, "T");
        EXPECT_FALSE(vga.is_reference);
        EXPECT_TRUE(same(vga.calibration, kVga));
    }
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_10);
        fw.set_fisheye_record(kHd, 1920, 1080);
        auto g = open_follower(pty);
        const auto hd =
            tx::detail::resolve_wrist_fisheye(g->calibration(), {1920, 1080}, "T");
        EXPECT_FALSE(hd.is_reference);
        EXPECT_TRUE(same(hd.calibration, kHd));
        EXPECT_NO_THROW(tx::FisheyeUndistorter(hd.calibration, hd.calibrated_size,
                                               cv::Size(1920, 1080)));
    }
}

TEST(WristFisheyeDecisionPty, NoCalibrationAnywhereFallsBackToTheReference) {
    for (Fw gen : {Fw::V2_11, Fw::V2_10}) {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(gen);
        auto g = open_follower(pty);
        auto& cal = g->calibration();
        const auto vga = tx::detail::resolve_wrist_fisheye(cal, {640, 480}, "T");
        EXPECT_TRUE(vga.is_reference);
        EXPECT_TRUE(same(vga.calibration, tx::FISHEYE_FALLBACK_CAL));
        EXPECT_NO_THROW(tx::FisheyeUndistorter(vga.calibration, vga.calibrated_size,
                                               cv::Size(640, 480)));
        // A 1080p camera gets the 1920x1080 reference, which the undistorter
        // accepts at that size (it was measured there; nothing is rescaled).
        const auto hd = tx::detail::resolve_wrist_fisheye(cal, {1920, 1080}, "T");
        EXPECT_TRUE(hd.is_reference);
        EXPECT_TRUE(same(hd.calibration, tx::FISHEYE_FALLBACK_CAL_HD));
        EXPECT_EQ(hd.calibrated_size, cv::Size(1920, 1080));
        EXPECT_NO_THROW(tx::FisheyeUndistorter(hd.calibration, hd.calibrated_size,
                                               cv::Size(1920, 1080)));
    }
}

TEST(WristFisheyeDecisionPty, CalibrationOnlyAtTheOtherSizeThrowsOnV211) {
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        fw.set_fisheye_record(kVga, 640, 480);   // VGA only
        auto g = open_follower(pty);
        expect_other_size_refusal(g->calibration(), {1920, 1080}, "1920x1080",
                                  "640x480");
    }
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_11);
        fw.set_fisheye_hd_record(kHd);           // HD only
        auto g = open_follower(pty);
        expect_other_size_refusal(g->calibration(), {640, 480}, "640x480",
                                  "1920x1080");
    }
}

TEST(WristFisheyeDecisionPty, CalibrationOnlyAtTheOtherSizeThrowsOnV210) {
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_10);
        fw.set_fisheye_record(kHd, 1920, 1080);
        auto g = open_follower(pty);
        expect_other_size_refusal(g->calibration(), {640, 480}, "640x480",
                                  "1920x1080");
    }
    {
        Pty pty;
        FakeFollower fw(pty);
        fw.set_fisheye_firmware(Fw::V2_10);
        fw.set_fisheye_record(kVga, 640, 480);
        auto g = open_follower(pty);
        expect_other_size_refusal(g->calibration(), {1920, 1080}, "1920x1080",
                                  "640x480");
    }
}

// ---- Firmware facts confirmed against tc-gu-01 #34 (247671f) -----------------

TEST(FisheyeDualSlotPty, CentreWindowBoundaryMatchesTheFirmwareHostTests) {
    // The firmware's own host tests: 320 + 96 accepted, 320 + 97 refused.
    auto hi = kVga;
    hi.cx = 416.0f;
    auto over = kVga;
    over.cx = 417.0f;
    EXPECT_TRUE(tx::fisheye_cal_slot_plausible(hi, 640, 480));
    EXPECT_FALSE(tx::fisheye_cal_slot_plausible(over, 640, 480));
    auto hd_lo = kHd;
    hd_lo.cy = 378.0f;   // 540 - 0.15 * 1080
    auto hd_under = kHd;
    hd_under.cy = 377.0f;
    EXPECT_TRUE(tx::fisheye_cal_slot_plausible(hd_lo, 1920, 1080));
    EXPECT_FALSE(tx::fisheye_cal_slot_plausible(hd_under, 1920, 1080));

    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    auto g = open_follower(pty);
    EXPECT_NO_THROW(g->calibration().write_fisheye_sized(hi, 640, 480));
    EXPECT_THROW(g->calibration().write_fisheye_sized(over, 640, 480), tx::Error);
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_write_sized(over, 640, 480)),
              tp::ErrorCode::InvalidParam);
    EXPECT_NO_THROW(g->calibration().write_fisheye_sized(hd_lo, 1920, 1080));
    EXPECT_EQ(raw_cal(*g, tp::encode_camera_fisheye_cal_write_sized(hd_under, 1920, 1080)),
              tp::ErrorCode::InvalidParam);
    EXPECT_TRUE(same(*fw.stored_fisheye(), hi));
    EXPECT_TRUE(same(*fw.stored_fisheye_hd(), hd_lo));
}

TEST(FisheyeDualSlotPty, MigratedV210RecordStaysInTheVgaSlotWithAnUnknownSize) {
    // A 1920x1080 record written by V2.10 rc1..rc3 is left in the VGA slot by
    // the upgrade (never moved to HD) and reports 0/0 on 0x02 and 0x04.
    Pty pty;
    FakeFollower fw(pty);
    fw.set_fisheye_firmware(Fw::V2_11);
    fw.set_fisheye_record(kHd, 1920, 1080);
    auto g = open_follower(pty);
    auto& cal = g->calibration();
    const auto vga = cal.read_fisheye_slot(640, 480);
    ASSERT_TRUE(vga.has_value());
    EXPECT_TRUE(same(vga->calibration, kHd));
    EXPECT_FALSE(vga->size_known());
    EXPECT_FALSE(cal.read_fisheye_slot(1920, 1080).has_value());
    const auto rec = cal.read_fisheye_record();   // op 0x02
    ASSERT_TRUE(rec.has_value());
    EXPECT_FALSE(rec->size_known());

    // Inferred from cx ~ 968 as 1920x1080, so not usable at 640x480 ...
    const auto r = cal.resolve_fisheye_for(640, 480);
    EXPECT_TRUE(r.is_reference);
    EXPECT_EQ(r.mismatched_size, cv::Size(1920, 1080));
    EXPECT_NE(r.reason.find("belongs to 1920x1080 (inferred), not 640x480"),
              std::string::npos) << r.reason;
    // ... and the wrist undistortion refuses rather than use the reference.
    expect_other_size_refusal(cal, {640, 480}, "640x480", "1920x1080");
    // A 1080p camera has no HD slot calibration either: the VGA slot record is
    // not promoted to HD, so this also refuses (unit calibrated, other slot).
    const auto hd = cal.resolve_fisheye_for(1920, 1080);
    EXPECT_TRUE(hd.is_reference);
    EXPECT_TRUE(hd.mismatched_size.empty());
}

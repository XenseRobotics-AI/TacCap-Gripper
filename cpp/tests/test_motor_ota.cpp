// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// MotorOtaSession against a simulated RobStride bootloader. No hardware.
//
// The fake applies the session's (mask, value) to its own reply exactly as the
// follower firmware does, so a mask that would not admit the real reply fails
// here as NoReply rather than passing by construction.
//
// What is pinned: the frame sequence and payloads from RobStride's "OTA 协议
// 说明" (20251114) and OTA.py, both readings where those two disagree (failure
// code 0x0F vs 0xF0, data reply type 11 vs 13), and the resume paths.

#include <taccap/error.hpp>
#include <taccap/motor_ota.hpp>

#include <cstring>
#include <gtest/gtest.h>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace tc = xense::taccap;
namespace tp = xense::taccap::protocol;

namespace {

constexpr uint8_t kCan = 0x7F;
constexpr uint8_t kHost = 0xFD;
const std::array<uint8_t, 8> kUid{0x3F, 0x8F, 0x38, 0x00, 0x90, 0x23, 0xB7, 0x11};

struct Sent {
    uint8_t mode;
    uint16_t data16;
    std::vector<uint8_t> data;
};

struct FakeBootloader {
    // Knobs.
    uint8_t fail_code = 0x0F;
    uint8_t data_reply_mode = 13;
    std::map<uint32_t, uint32_t> resume_once;  // at pack -> ask to resume at
    std::map<uint32_t, int> silent_times;      // at pack -> drop this many replies
    std::optional<uint32_t> end_fail_once;     // end fails once, reporting this position
    bool refuse_start = false;
    int start_silent = 0;                      // drop this many start replies
    bool silent = false;

    // Observed.
    std::vector<Sent> sent;
    std::vector<uint8_t> flash;
    uint32_t info_size = 0, info_packs = 0;

    tp::MotorCanXferResp reply(uint32_t id, const uint8_t* d, uint8_t dlc,
                               uint32_t mask, uint32_t value) {
        tp::MotorCanXferResp r{};
        if ((id & mask) != value) {
            r.status = tp::motor_can_xfer_status::NoReply;
            return r;
        }
        r.status = tp::motor_can_xfer_status::Ok;
        r.ext_id = id;
        r.dlc = dlc;
        if (dlc) std::memcpy(r.data, d, dlc);
        return r;
    }
    uint32_t id(uint8_t mode, uint8_t result, uint8_t dst = kHost) {
        return (uint32_t(mode) << 24) | (uint32_t(result) << 16) | (uint32_t(kCan) << 8) | dst;
    }

    tp::MotorCanXferResp operator()(uint32_t ext_id, const std::vector<uint8_t>& data,
                                    uint16_t, uint32_t mask, uint32_t value) {
        const uint8_t mode = (ext_id >> 24) & 0x1F;
        const uint16_t d16 = (ext_id >> 8) & 0xFFFF;
        EXPECT_EQ(ext_id & 0xFF, kCan);
        sent.push_back({mode, d16, data});
        tp::MotorCanXferResp none{};
        none.status = tp::motor_can_xfer_status::NoReply;
        if (silent) return none;

        uint8_t zero8[8] = {0};
        switch (mode) {
        case 0:
            EXPECT_EQ(d16, kHost);
            return reply(id(0, 0, 0xFE), kUid.data(), 8, mask, value);
        case 11:
            if (start_silent > 0) {
                --start_silent;
                return none;
            }
            EXPECT_EQ(d16, kHost);
            EXPECT_EQ(data, std::vector<uint8_t>(kUid.begin(), kUid.end()));
            return reply(id(11, refuse_start ? fail_code : 0), zero8, 8, mask, value);
        case 12:
            EXPECT_EQ(d16, kHost);
            std::memcpy(&info_size, data.data(), 4);
            std::memcpy(&info_packs, data.data() + 4, 4);
            flash.assign(info_packs * 8, 0);
            return reply(id(12, 0), zero8, 8, mask, value);
        case 13: {
            EXPECT_EQ(data.size(), 8u);
            if (auto it = silent_times.find(d16); it != silent_times.end() && it->second > 0) {
                --it->second;
                return none;
            }
            if (auto it = resume_once.find(d16); it != resume_once.end()) {
                uint8_t r[8] = {uint8_t(it->second), uint8_t(it->second >> 8)};
                resume_once.erase(it);
                return reply(id(data_reply_mode, fail_code), r, 8, mask, value);
            }
            std::memcpy(&flash[d16 * 8], data.data(), 8);
            return reply(id(data_reply_mode, 0), zero8, 8, mask, value);
        }
        case 14: {
            EXPECT_EQ(d16, 0);
            uint32_t n = 0;
            std::memcpy(&n, data.data(), 4);
            EXPECT_EQ(n, info_packs);
            if (end_fail_once) {
                uint8_t r[8] = {};
                std::memcpy(r, &*end_fail_once, 4);
                end_fail_once.reset();
                return reply(id(14, fail_code), r, 8, mask, value);
            }
            return reply(id(14, 0), zero8, 8, mask, value);
        }
        }
        ADD_FAILURE() << "unexpected mode " << int(mode);
        return none;
    }
};

std::vector<uint8_t> make_image(size_t n) {
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = uint8_t(i * 7 + 3);
    return v;
}

// The flash contents the motor should end up with: the image, padded with 0xFF.
std::vector<uint8_t> padded(const std::vector<uint8_t>& img) {
    auto v = img;
    v.resize((img.size() + 7) / 8 * 8, 0xFF);
    return v;
}

tc::MotorOtaSession session(FakeBootloader& fb) {
    tc::MotorOtaSession::Options o;
    o.max_attempts = 3;
    return tc::MotorOtaSession(
        [&fb](uint32_t id, const std::vector<uint8_t>& d, uint16_t t, uint32_t m, uint32_t v) {
            return fb(id, d, t, m, v);
        },
        kCan, o);
}

std::vector<uint8_t> modes(const FakeBootloader& fb) {
    std::vector<uint8_t> m;
    for (const auto& s : fb.sent) m.push_back(s.mode);
    return m;
}

}  // namespace

TEST(MotorOta, ReadsUidFromTheReplyAddressedTo0xFE) {
    FakeBootloader fb;
    EXPECT_EQ(session(fb).read_uid(), kUid);
}

TEST(MotorOta, HappyPathFollowsTheDocumentedSequence) {
    FakeBootloader fb;
    const auto img = make_image(20);  // 3 packs, last one padded
    std::vector<std::pair<uint32_t, uint32_t>> progress;
    session(fb).update_from_bytes(img, [&](uint32_t a, uint32_t b) { progress.push_back({a, b}); });

    EXPECT_EQ(modes(fb), (std::vector<uint8_t>{0, 11, 12, 13, 13, 13, 14}));
    EXPECT_EQ(fb.info_size, 20u);
    EXPECT_EQ(fb.info_packs, 3u);
    EXPECT_EQ(fb.flash, padded(img));
    ASSERT_FALSE(progress.empty());
    EXPECT_EQ(progress.back(), std::make_pair(3u, 3u));
}

TEST(MotorOta, AcceptsDataRepliesTypedElevenAsThePdfTableSays) {
    FakeBootloader fb;
    fb.data_reply_mode = 11;
    const auto img = make_image(64);
    session(fb).update_from_bytes(img);
    EXPECT_EQ(fb.flash, padded(img));
}

class FailCode : public ::testing::TestWithParam<uint8_t> {};

// 0x0F per the PDF and the Qt sample, 0xF0 per OTA.py.
TEST_P(FailCode, ADataFailureRewindsToTheMotorsIndex) {
    FakeBootloader fb;
    fb.fail_code = GetParam();
    fb.resume_once[5] = 3;
    const auto img = make_image(80);  // 10 packs
    session(fb).update_from_bytes(img);

    std::vector<uint16_t> data_idx;
    for (const auto& s : fb.sent)
        if (s.mode == 13) data_idx.push_back(s.data16);
    EXPECT_EQ(data_idx, (std::vector<uint16_t>{0, 1, 2, 3, 4, 5, 3, 4, 5, 6, 7, 8, 9}));
    EXPECT_EQ(fb.flash, padded(img));
}

INSTANTIATE_TEST_SUITE_P(BothDocumentedCodes, FailCode, ::testing::Values(0x0F, 0xF0));

TEST(MotorOta, RetriesAFrameThatGotNoReply) {
    FakeBootloader fb;
    fb.silent_times[2] = 2;  // two drops, third attempt lands
    const auto img = make_image(40);
    session(fb).update_from_bytes(img);
    EXPECT_EQ(fb.flash, padded(img));
}

TEST(MotorOta, GivesUpWithTimeoutWhenTheMotorStaysSilent) {
    FakeBootloader fb;
    fb.silent_times[2] = 3;
    EXPECT_THROW(session(fb).update_from_bytes(make_image(40)), tc::TimeoutError);
}

TEST(MotorOta, NoUidMeansNothingElseIsSent) {
    FakeBootloader fb;
    fb.silent = true;
    EXPECT_THROW(session(fb).update_from_bytes(make_image(40)), tc::TimeoutError);
    for (const auto& s : fb.sent) EXPECT_EQ(s.mode, 0);
}

TEST(MotorOta, AnEndFailureResendsFromThePositionItReports) {
    FakeBootloader fb;
    fb.end_fail_once = 7u;
    const auto img = make_image(80);
    session(fb).update_from_bytes(img);
    EXPECT_EQ(modes(fb).back(), 14);
    int ends = 0;
    for (const auto& s : fb.sent) ends += (s.mode == 14);
    EXPECT_EQ(ends, 2);
    EXPECT_EQ(fb.flash, padded(img));
}

// Measured on 0086s: the motor restarts into its bootloader before answering
// start, ~4 s in. Five silent attempts is past the per-frame budget of 3 but
// inside start's own budget of 6.
TEST(MotorOta, StartGetsItsOwnLargerRetryBudget) {
    FakeBootloader fb;
    fb.start_silent = 5;
    const auto img = make_image(40);
    session(fb).update_from_bytes(img);
    EXPECT_EQ(fb.flash, padded(img));
}

TEST(MotorOta, RefusedStartStopsBeforeAnyData) {
    FakeBootloader fb;
    fb.refuse_start = true;
    EXPECT_THROW(session(fb).update_from_bytes(make_image(40)), tc::ProtocolError);
    EXPECT_EQ(modes(fb), (std::vector<uint8_t>{0, 11}));
}

TEST(MotorOta, AResumeIndexPastTheImageIsRejected) {
    FakeBootloader fb;
    fb.resume_once[1] = 999;
    EXPECT_THROW(session(fb).update_from_bytes(make_image(40)), tc::ProtocolError);
}

TEST(MotorOta, ImageSizeIsBoundedByTheSixteenBitPackIndex) {
    FakeBootloader fb;
    EXPECT_THROW(session(fb).update_from_bytes({}), std::invalid_argument);
    EXPECT_THROW(session(fb).update_from_bytes(
                     std::vector<uint8_t>(tc::MotorOtaSession::kMaxImageSize + 1)),
                 std::invalid_argument);
    EXPECT_TRUE(fb.sent.empty());
}

TEST(MotorOta, PreflightNeedsAMotor) {
    FakeBootloader fb;
    EXPECT_THROW(session(fb).preflight("RS00"), std::logic_error);
}

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <cstring>
#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>

#include <random>
#include <string>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

namespace {

std::vector<uint8_t> random_payload(std::size_t len, uint32_t seed = 1) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 255);
    std::vector<uint8_t> out(len);
    for (auto& b : out) b = static_cast<uint8_t>(dist(rng));
    return out;
}

void expect_frame_eq(const tb::Frame& got, tp::Address addr, uint8_t seq,
                     tp::FrameType type, tp::Cmd cmd,
                     const std::vector<uint8_t>& payload) {
    EXPECT_EQ(got.addr, addr);
    EXPECT_EQ(got.seq, seq);
    EXPECT_EQ(got.type, type);
    EXPECT_EQ(got.cmd, cmd);
    EXPECT_EQ(got.payload, payload);
}

}  // namespace

TEST(Frame, PackParseRoundtripEmptyPayload) {
    auto wire = tb::pack_frame(tp::Address::PC, 7,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    // V1.8 byte stuffing may grow the frame past MIN_FRAME_LEN if the body
    // (incl. CRC) contains 0xAA/0x55/0x7D.
    EXPECT_GE(wire.size(), tb::MIN_FRAME_LEN);
    EXPECT_EQ(wire.front(), tb::FRAME_HEAD);
    EXPECT_EQ(wire.back(),  tb::FRAME_TAIL);

    auto out = tb::try_parse_frame(wire.data(), wire.size());
    ASSERT_EQ(out.status, tb::ParseStatus::Success);
    EXPECT_EQ(out.consumed, wire.size());
    expect_frame_eq(out.frame, tp::Address::PC, 7,
                    tp::FrameType::CMD_NEED_ACK, tp::Cmd::GetVersion, {});
}

TEST(Frame, PackParseRoundtripVariousSizes) {
    const std::size_t sizes[] = {0, 1, 4, 8, 28, 256, tb::MAX_PAYLOAD_LEN};
    for (std::size_t n : sizes) {
        const auto payload = random_payload(n, /*seed=*/123u + n);
        auto wire = tb::pack_frame(tp::Address::MCU, 42,
                                   tp::FrameType::DATA, tp::Cmd::GetImu,
                                   payload);
        // >= because byte stuffing can only grow the wire frame.
        ASSERT_GE(wire.size(), tb::MIN_FRAME_LEN + n);

        auto out = tb::try_parse_frame(wire.data(), wire.size());
        ASSERT_EQ(out.status, tb::ParseStatus::Success) << "size=" << n;
        EXPECT_EQ(out.consumed, wire.size());
        expect_frame_eq(out.frame, tp::Address::MCU, 42,
                        tp::FrameType::DATA, tp::Cmd::GetImu, payload);
    }
}

TEST(Frame, PackRejectsOversizedPayload) {
    std::vector<uint8_t> big(tb::MAX_PAYLOAD_LEN + 1, 0);
    EXPECT_THROW(
        tb::pack_frame(tp::Address::PC, 0, tp::FrameType::DATA,
                       tp::Cmd::GetImu, big),
        xense::taccap::ProtocolError);
}

TEST(Frame, ParseSignalsNeedMoreDataOnTruncation) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    // Feed all but the last byte: should ask for more.
    auto out = tb::try_parse_frame(wire.data(), wire.size() - 1);
    EXPECT_EQ(out.status, tb::ParseStatus::NeedMoreData);
}

TEST(Frame, ParseResyncsOnGarbagePrefix) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NO_ACK,
                               tp::Cmd::GetVersion);
    std::vector<uint8_t> noisy = {0x00, 0xFF, 0x42, 0xAA};  // ends on a HEAD lure
    noisy.insert(noisy.end(), wire.begin(), wire.end());

    auto out = tb::try_parse_frame(noisy.data(), noisy.size());
    EXPECT_EQ(out.status, tb::ParseStatus::Resync);
    // Should skip past the leading non-HEAD bytes (3 of them) so the next
    // try lands on a HEAD.
    EXPECT_EQ(out.consumed, 3u);
}

TEST(Frame, ParseResyncsOnBadCrc) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    // Flip a CRC byte.
    wire[wire.size() - 2] ^= 0x01;

    auto out = tb::try_parse_frame(wire.data(), wire.size());
    EXPECT_EQ(out.status, tb::ParseStatus::Resync);
    EXPECT_EQ(out.consumed, 1u);  // drop one byte and rescan
}

TEST(FrameParser, EmitsSingleCompleteFrame) {
    auto wire = tb::pack_frame(tp::Address::PC, 9,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16));
    tb::FrameParser p;
    p.feed(wire);
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_EQ(f.payload.size(), 16u);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, EmitsMultipleFramesFromOneFeed) {
    auto a = tb::pack_frame(tp::Address::PC, 1,
                            tp::FrameType::DATA, tp::Cmd::GetImu,
                            random_payload(28, 1));
    auto b = tb::pack_frame(tp::Address::MCU, 2,
                            tp::FrameType::DATA, tp::Cmd::GetEncoder,
                            random_payload(16, 2));
    auto c = tb::pack_frame(tp::Address::PC, 3,
                            tp::FrameType::ACK, tp::Cmd::GetVersion,
                            random_payload(4, 3));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());
    stream.insert(stream.end(), c.begin(), c.end());

    tb::FrameParser p;
    p.feed(stream);
    EXPECT_EQ(p.pending(), 3u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetImu);
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetVersion);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, HandlesByteAtATimeFeed) {
    auto wire = tb::pack_frame(tp::Address::PC, 11,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetEncoder,
                               random_payload(8, 99));
    tb::FrameParser p;
    for (auto b : wire) {
        p.feed(&b, 1);
    }
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.seq, 11u);
    EXPECT_EQ(f.payload.size(), 8u);
}

TEST(FrameParser, ResyncsAcrossGarbagePlusValidFrame) {
    // Mix of pre-frame junk (incl. a false HEAD whose LEN field parses
    // as a plausible 773 bytes — would stall the parser indefinitely
    // before the V1.6 rewind fix in FrameParser::drain_).
    std::vector<uint8_t> garbage = {0xDE, 0xAD, 0xBE, 0xEF, 0xAA, 0x42, 0x55};
    auto wire = tb::pack_frame(tp::Address::PC, 5,
                               tp::FrameType::DATA, tp::Cmd::GetImu,
                               random_payload(28, 5));
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), garbage.begin(), garbage.end());
    stream.insert(stream.end(), wire.begin(), wire.end());

    tb::FrameParser p;
    p.feed(stream);
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetImu);
    EXPECT_EQ(f.payload.size(), 28u);
}

// Multiple false HEADs back-to-back, all with plausible (but bogus)
// length fields, followed by a real frame at the end. Exercises the
// rewind path repeatedly within a single drain_() call.
TEST(FrameParser, ResyncsAcrossMultipleFalseHeads) {
    auto wire = tb::pack_frame(tp::Address::PC, 9,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16, 17));
    std::vector<uint8_t> stream;
    // 5 false HEADs each followed by a "plausible" 7-byte fake header
    // → length = (varied), tail check / CRC check will both miss → parser
    // walks past each.
    for (uint8_t i = 0; i < 5; ++i) {
        stream.insert(stream.end(),
            {0xAA, /*addr*/ static_cast<uint8_t>(0x10 + i),
                   /*seq */ i,
                   /*type*/ 0xFF,
                   /*cmd */ 0xEE,
                   /*lenL*/ static_cast<uint8_t>(0x20 + i),
                   /*lenH*/ 0x00});
    }
    stream.insert(stream.end(), wire.begin(), wire.end());

    tb::FrameParser p;
    p.feed(stream);
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_EQ(f.payload.size(), 16u);
}

// A 0xAA (HEAD) byte inside the payload: V1.8 stuffing escapes it on the
// wire (so it never appears raw inside a frame), and it must round-trip back
// to 0xAA after unstuffing. Confirms the parser keys on the real frame and
// the payload survives stuffing.
TEST(FrameParser, IgnoresHeadByteInsideValidPayload) {
    std::vector<uint8_t> payload(40);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(i);
    payload[15] = 0xAA;
    payload[16] = 0x05;   // would parse as a small LEN if HEAD wins
    payload[17] = 0x00;
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::DATA, tp::Cmd::GetImu,
                               payload);

    tb::FrameParser p;
    p.feed(wire);
    ASSERT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetImu);
    ASSERT_EQ(f.payload.size(), payload.size());
    EXPECT_EQ(f.payload[15], 0xAA);  // 0xAA preserved inside payload
}

// Regression guard for the V1.3 frame-cap bump: confirm a >1 KB OTA
// write-block-sized frame still pack/parse round-trips.
TEST(FrameParser, OtaWriteBlockSizedFrameRoundtrips) {
    // 6-byte OtaWriteBlock header + 1024-byte block = 1030-byte payload,
    // well past the V1.2 cap of 502 and within the V1.3 cap of 2294.
    std::vector<uint8_t> payload(1030);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i * 13 + 7);
    }
    auto wire = tb::pack_frame(tp::Address::PC, 42,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::OtaWriteBlock,
                               payload);

    tb::FrameParser p;
    p.feed(wire);
    ASSERT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::OtaWriteBlock);
    ASSERT_EQ(f.payload.size(), payload.size());
    EXPECT_EQ(std::memcmp(f.payload.data(), payload.data(), payload.size()), 0);
}

TEST(FrameParser, DropsBadCrcFrameButContinues) {
    auto bad = tb::pack_frame(tp::Address::PC, 1,
                              tp::FrameType::DATA, tp::Cmd::GetImu,
                              random_payload(8, 1));
    bad[bad.size() - 2] ^= 0xFF;  // corrupt CRC

    auto good = tb::pack_frame(tp::Address::PC, 2,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16, 2));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), bad.begin(),  bad.end());
    stream.insert(stream.end(), good.begin(), good.end());

    tb::FrameParser p;
    p.feed(stream);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, BoundsResyncBuffer) {
    tb::FrameParser p(/*max_buffered=*/64);
    // Feed 1024 bytes of pure garbage; buffer must not exceed 64 (rough
    // upper bound: drain_ trims after each feed).
    std::vector<uint8_t> garbage(1024, 0xCC);
    p.feed(garbage);
    EXPECT_LE(p.buffered_bytes(), 64u);
    EXPECT_EQ(p.pending(), 0u);
}

// ---------------------------------------------------------------------------
// Damage accounting. Everything the parser throws away must land in exactly
// one counter, so a caller can tell "the firmware sent fewer frames" apart
// from "we lost bytes on the host".
// ---------------------------------------------------------------------------

TEST(FrameParser, CountsCrcErrorSeparatelyFromResync) {
    auto bad = tb::pack_frame(tp::Address::PC, 1,
                              tp::FrameType::DATA, tp::Cmd::GetImu,
                              random_payload(8, 1));
    bad[bad.size() - 2] ^= 0xFF;  // framing stays valid, CRC does not

    auto good = tb::pack_frame(tp::Address::PC, 2,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16, 2));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), bad.begin(),  bad.end());
    stream.insert(stream.end(), good.begin(), good.end());

    tb::FrameParser p;
    p.feed(stream);

    EXPECT_EQ(p.stats().crc_errors, 1u);
    // The whole corrupt frame is discarded: one byte at the failed CRC, then
    // the scan to the next real HEAD.
    EXPECT_EQ(p.stats().resync_bytes, bad.size());
    EXPECT_EQ(p.stats().overflow_bytes, 0u);
}

// Garbage that never looks like a frame is resync, not a CRC error — the two
// must not be conflated, since only the latter implies lost bytes.
TEST(FrameParser, PureGarbageCountsAsResyncNotCrc) {
    auto good = tb::pack_frame(tp::Address::PC, 3,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16, 3));
    std::vector<uint8_t> stream(32, 0xCC);   // no HEAD anywhere
    stream.insert(stream.end(), good.begin(), good.end());

    tb::FrameParser p;
    p.feed(stream);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_EQ(p.stats().crc_errors, 0u);
    EXPECT_EQ(p.stats().resync_bytes, 32u);
}

TEST(FrameParser, CountsOverflowBytesWhenResyncBufferIsCapped) {
    tb::FrameParser p(/*max_buffered=*/64);
    p.feed(std::vector<uint8_t>(1024, 0xCC));

    EXPECT_LE(p.buffered_bytes(), 64u);
    // 1024 bytes of headless garbage: consumed by resync, capped, or both —
    // but nothing may go unaccounted.
    EXPECT_EQ(p.stats().resync_bytes + p.stats().overflow_bytes +
                  p.buffered_bytes(),
              1024u);
}

// Counters are diagnostics, not buffer state: reset() clears the bytes but
// must not erase the history a caller is polling.
TEST(FrameParser, ResetKeepsDamageCounters) {
    auto bad = tb::pack_frame(tp::Address::PC, 1,
                              tp::FrameType::DATA, tp::Cmd::GetImu,
                              random_payload(8, 4));
    bad[bad.size() - 2] ^= 0xFF;
    tb::FrameParser p;
    p.feed(bad);
    ASSERT_EQ(p.stats().crc_errors, 1u);

    p.reset();
    EXPECT_EQ(p.buffered_bytes(), 0u);
    EXPECT_EQ(p.stats().crc_errors, 1u);
}

namespace {

// Real follower bytes captured before SDK parsing on 2026-10-26, motor
// disabled; no pack_frame()/SDK encoder is used to construct these fixtures.
// Evidence: dist/debug/runs/imu-link-fix-504134a-20261026/logs/idle-uart.strace
// SHA-256: 9c939e9728d2e20b67a448536fb6db06cf89ec8a75d1cce627c96cb7c5010afb
// RX offsets below refer to the concatenated successful read() buffers.
// These bytes prove parser behavior, not firmware timing or loss origin.
std::vector<uint8_t> captured_hex(const char* text) {
    const std::string hex(text);
    std::vector<uint8_t> bytes;
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

struct CapturedFrame {
    const char* name;
    std::vector<uint8_t> wire;
    uint8_t seq;
    tp::FrameType type;
    tp::Cmd cmd;
    std::vector<uint8_t> payload;
};

std::vector<CapturedFrame> captured_good_frames() {
    return {
        // Line 30, RX offset 307, 69 wire bytes; extended motor status.
        {"motor DATA", captured_hex(
            "aa025703503b00c05987bf001048bc0010c0ba000004420000262887bf"
            "000000000000000004021301850000000000000000db587b00e85a7b00"
            "00000000af2d78005b0a55"),
         0x57, tp::FrameType::DATA, tp::Cmd::GetMotorStatus, captured_hex(
            "c05987bf001048bc0010c0ba000004420000262887bf0000000000000000"
            "04021301850000000000000000db587b00e85a7b0000000000af2d7800")},
        // Line 7627, RX offset 245546, 55 wire bytes; payload contains 7d75.
        {"IMU V2 DATA", captured_hex(
            "aa022803172c00016a749d0f000300ffffe803f4ff1200faff7d75f65f"
            "ecb211360e1e7c016a749d060000002bf06c202a29f306c7dd55"),
         0x28, tp::FrameType::DATA, tp::Cmd::GetImuV2, captured_hex(
            "016a749d0f000300ffffe803f4ff1200faff55f65fecb211360e1e7c01"
            "6a749d060000002bf06c202a29f306")},
        // Line 29, RX offset 248, 59 wire bytes; CRC contains escaped 0x55.
        {"control stats ACK", captured_hex(
            "aa020a025130000004f4010200ffff00000000344a0100324a0100fe71"
            "0600000000002200000000000000000000000000000000000000637d7555"),
         0x0a, tp::FrameType::ACK, tp::Cmd::GetMotorControlStats, captured_hex(
            "0004f4010200ffff00000000344a0100324a0100fe7106000000000022"
            "00000000000000000000000000000000000000")},
    };
}

std::vector<uint8_t> captured_short_motor_frame() {
    // Lines 55 and 58 (unfinished/resumed read), RX offset 1138, 67 wire
    // bytes. LEN is 59 but the unstuffed body is 65 bytes instead of 67.
    // This is the captured damage, not a test-side deletion from a good frame.
    return captured_hex(
        "aa025e03503b00c05987bf001048bc0010c0ba000004420000262887bf"
        "00000000000000000402130185000000000000db587b00245b7b000000"
        "0000af2d78008f5155");
}

std::vector<uint8_t> captured_stream(const std::vector<CapturedFrame>& frames,
                                     bool include_damage = false) {
    std::vector<uint8_t> stream;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        stream.insert(stream.end(), frames[i].wire.begin(), frames[i].wire.end());
        if (include_damage && i == 0) {
            const auto bad = captured_short_motor_frame();
            stream.insert(stream.end(), bad.begin(), bad.end());
        }
    }
    return stream;
}

void expect_captured_frames(tb::FrameParser& parser,
                            const std::vector<CapturedFrame>& expected) {
    ASSERT_EQ(parser.pending(), expected.size());
    for (const auto& fixture : expected) {
        SCOPED_TRACE(fixture.name);
        tb::Frame frame;
        ASSERT_TRUE(parser.try_pop(frame));
        expect_frame_eq(frame, tp::Address::MCU, fixture.seq, fixture.type,
                        fixture.cmd, fixture.payload);
    }
    tb::Frame extra;
    EXPECT_FALSE(parser.try_pop(extra));
    EXPECT_EQ(parser.buffered_bytes(), 0u);
    EXPECT_EQ(parser.stats().overflow_bytes, 0u);
}

}  // namespace

TEST(HardwareFrameReplay, ValidFramesAtEverySingleSplit) {
    for (const auto& fixture : captured_good_frames()) {
        SCOPED_TRACE(fixture.name);
        for (std::size_t split = 0; split <= fixture.wire.size(); ++split) {
            SCOPED_TRACE(split);
            tb::FrameParser parser;
            parser.feed(fixture.wire.data(), split);
            EXPECT_EQ(parser.pending(), split == fixture.wire.size() ? 1u : 0u);
            parser.feed(fixture.wire.data() + split, fixture.wire.size() - split);
            expect_captured_frames(parser, {fixture});
            EXPECT_EQ(parser.stats().resync_bytes, 0u);
            EXPECT_EQ(parser.stats().crc_errors, 0u);
        }
    }
}

TEST(HardwareFrameReplay, CoalescedValidFramesAtEverySingleSplit) {
    const auto fixtures = captured_good_frames();
    const auto stream = captured_stream(fixtures);
    // Includes one coalesced read and every boundary across DATA and ACK,
    // including a split immediately after ESC in payload or CRC.
    for (std::size_t split = 0; split <= stream.size(); ++split) {
        SCOPED_TRACE(split);
        tb::FrameParser parser;
        parser.feed(stream.data(), split);
        parser.feed(stream.data() + split, stream.size() - split);
        expect_captured_frames(parser, fixtures);
        EXPECT_EQ(parser.stats().resync_bytes, 0u);
        EXPECT_EQ(parser.stats().crc_errors, 0u);
    }
}

TEST(HardwareFrameReplay, ValidFramesByteByByte) {
    const auto fixtures = captured_good_frames();
    const auto stream = captured_stream(fixtures);
    tb::FrameParser parser;
    for (const auto byte : stream) parser.feed(&byte, 1);
    expect_captured_frames(parser, fixtures);
    EXPECT_EQ(parser.stats().resync_bytes, 0u);
    EXPECT_EQ(parser.stats().crc_errors, 0u);
}

TEST(HardwareFrameReplay, CapturedShortMotorFrameIsRejectedBeforeCrc) {
    const auto bad = captured_short_motor_frame();
    ASSERT_EQ(bad.size(), 67u);
    const auto result = tb::try_parse_frame(bad.data(), bad.size());
    EXPECT_EQ(result.status, tb::ParseStatus::Resync);
    EXPECT_EQ(result.reason, tb::ParseReason::BadLength);
    tb::FrameParser parser;
    parser.feed(bad);
    EXPECT_EQ(parser.pending(), 0u);
    EXPECT_EQ(parser.buffered_bytes(), 0u);
    EXPECT_EQ(parser.stats().resync_bytes, bad.size());
    EXPECT_EQ(parser.stats().crc_errors, 0u)
        << "zero CRC errors cannot establish intact input when LEN already fails";
}

TEST(HardwareFrameReplay, RecoversAfterCapturedDamageAtEverySingleSplit) {
    const auto fixtures = captured_good_frames();
    const auto stream = captured_stream(fixtures, true);
    for (std::size_t split = 0; split <= stream.size(); ++split) {
        SCOPED_TRACE(split);
        tb::FrameParser parser;
        parser.feed(stream.data(), split);
        parser.feed(stream.data() + split, stream.size() - split);
        expect_captured_frames(parser, fixtures);
        EXPECT_EQ(parser.stats().resync_bytes, captured_short_motor_frame().size());
        EXPECT_EQ(parser.stats().crc_errors, 0u);
    }
}

TEST(HardwareFrameReplay, RecoversAfterCapturedDamageByteByByte) {
    const auto fixtures = captured_good_frames();
    const auto stream = captured_stream(fixtures, true);
    tb::FrameParser parser;
    for (const auto byte : stream) parser.feed(&byte, 1);
    expect_captured_frames(parser, fixtures);
    EXPECT_EQ(parser.stats().resync_bytes, captured_short_motor_frame().size());
    EXPECT_EQ(parser.stats().crc_errors, 0u);
}

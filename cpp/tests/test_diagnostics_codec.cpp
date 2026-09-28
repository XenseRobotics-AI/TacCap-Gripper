// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Wire-format tests for the diagnostics payloads (Cmd 0x54 / 0x55).
//
// The one that earns its keep is the short-packet case: firmware 1.1.3 answers
// 0x54 with 32 bytes and 1.1.4 with 36. A decoder that insists on 36 silently
// stops working against the older firmware, which is exactly the kind of
// breakage that shows up as "the tool prints nothing" rather than as an error.

#include <gtest/gtest.h>

#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstring>
#include <vector>

namespace {

namespace tp = xense::taccap::protocol;

TEST(DiagnosticsCodec, PayloadSizesMatchFirmware) {
    EXPECT_EQ(sizeof(tp::UartStats), 44u);
    EXPECT_EQ(sizeof(tp::LogConfig), 2u);
}

TEST(DiagnosticsCodec, DecodesThe36BytePacket) {
    tp::UartStats in{};
    in.tx_bytes_ok     = 246417;
    in.tx_calls_ok     = 6013;
    in.tx_fail_timeout = 0;
    in.tx_fail_other   = 0;
    in.rx_bytes        = 451666;
    in.rx_overflow     = 0;
    in.debug_tx_bytes  = 2459464;
    in.rb_used         = 0;
    in.rb_free         = 8191;
    in.log_dropped     = 7500;

    std::vector<uint8_t> wire(36);
    std::memcpy(wire.data(), &in, wire.size());

    const auto out = tp::decode_uart_stats(wire.data(), wire.size());
    EXPECT_EQ(out.tx_bytes_ok, in.tx_bytes_ok);
    EXPECT_EQ(out.tx_calls_ok, in.tx_calls_ok);
    EXPECT_EQ(out.rx_bytes, in.rx_bytes);
    EXPECT_EQ(out.debug_tx_bytes, in.debug_tx_bytes);
    EXPECT_EQ(out.rb_free, in.rb_free);
    EXPECT_EQ(out.log_dropped, in.log_dropped);
}

// Firmware 1.1.3: same layout, minus the trailing log_dropped.
TEST(DiagnosticsCodec, AcceptsThe32ByteFirmware113Packet) {
    tp::UartStats in{};
    in.tx_bytes_ok    = 51;
    in.tx_calls_ok    = 3;
    in.rx_bytes       = 40;
    in.debug_tx_bytes = 71703;
    in.rb_free        = 8191;
    in.log_dropped    = 0xDEADBEEF;   // must NOT survive: it is not on the wire

    in.rx_errors      = 0xDEADBEEF;
    in.rx_rearms      = 0xDEADBEEF;
    std::vector<uint8_t> wire(32);
    std::memcpy(wire.data(), &in, wire.size());

    const auto out = tp::decode_uart_stats(wire.data(), wire.size());
    EXPECT_EQ(out.tx_bytes_ok, 51u);
    EXPECT_EQ(out.debug_tx_bytes, 71703u);
    EXPECT_EQ(out.rb_free, 8191u);
    EXPECT_EQ(out.log_dropped, 0u) << "the tail must be zero-filled, not garbage";
    EXPECT_EQ(out.rx_errors, 0u);
    EXPECT_EQ(out.rx_rearms, 0u);
}

// Follower 1.2.11 / leader 1.2.6: the receive-stall counters on the tail.
TEST(DiagnosticsCodec, DecodesThe44BytePacketWithReceiveCounters) {
    tp::UartStats in{};
    in.rx_bytes  = 123456;
    in.rx_errors = 3;
    in.rx_rearms = 2;
    std::vector<uint8_t> wire(44);
    std::memcpy(wire.data(), &in, wire.size());
    const auto out = tp::decode_uart_stats(wire.data(), wire.size());
    EXPECT_EQ(out.rx_bytes, 123456u);
    EXPECT_EQ(out.rx_errors, 3u);
    EXPECT_EQ(out.rx_rearms, 2u);
}

TEST(DiagnosticsCodec, RejectsAnythingShorterThanTheOldPacket) {
    std::vector<uint8_t> wire(31, 0);
    EXPECT_THROW(tp::decode_uart_stats(wire.data(), wire.size()),
                 xense::taccap::ProtocolError);
    EXPECT_THROW(tp::decode_uart_stats(nullptr, 36),
                 xense::taccap::ProtocolError);
}

TEST(DiagnosticsCodec, LogConfigRoundTrips) {
    const tp::LogConfig in{static_cast<uint8_t>(tp::LogLevel::Debug), tp::LogOutput::Uart};
    const auto wire = tp::encode(in);
    ASSERT_EQ(wire.size(), 2u);
    EXPECT_EQ(wire[0], 4u);
    EXPECT_EQ(wire[1], 1u);

    const auto out = tp::decode_log_config(wire.data(), wire.size());
    EXPECT_EQ(out.level, in.level);
    EXPECT_EQ(out.output_mask, in.output_mask);
}

TEST(DiagnosticsCodec, LogOutputNoneIsTheFirmwareDefault) {
    EXPECT_EQ(tp::LogOutput::None, 0u);
    EXPECT_EQ(static_cast<uint8_t>(tp::LogLevel::None), 0u);
}

}  // namespace

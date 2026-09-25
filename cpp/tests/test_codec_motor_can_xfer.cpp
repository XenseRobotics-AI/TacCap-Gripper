// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// The raw extended-frame transfer (Cmd 0x5B, follower firmware 1.2.8+). Pure
// codec — no hardware.
//
// Worth pinning byte by byte because this is what motor OTA rides on: a field
// that lands one byte off sends firmware data to the wrong CAN id, or makes
// the MCU wait for a reply that can never match. Neither fails loudly -- the
// first shows up as a motor that ignores the upgrade, the second as NoReply.
//
// Offsets transcribed from the firmware's App/protocol/protocol_data.h
// (motor_can_xfer_req_t / motor_can_xfer_resp_t). scripts/check_protocol_drift.py
// compares the structs by compiled sizeof; this file pins the offsets.

#include <cstddef>
#include <gtest/gtest.h>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tp = xense::taccap::protocol;

TEST(MotorCanXferCodec, CommandSitsAtTheFirmwareOpcode) {
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::MotorCanExtXfer), 0x5B);
}

TEST(MotorCanXferCodec, WireSizes) {
    EXPECT_EQ(sizeof(tp::MotorCanXferReq), 24u);
    EXPECT_EQ(sizeof(tp::MotorCanXferResp), 16u);
}

TEST(MotorCanXferCodec, RequestFieldsLandOnTheFirmwareOffsets) {
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, ext_id), 0u);
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, data), 4u);
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, dlc), 12u);
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, timeout_ms), 14u);
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, match_mask), 16u);
    EXPECT_EQ(offsetof(tp::MotorCanXferReq, match_value), 20u);
}

TEST(MotorCanXferCodec, ResponseFieldsLandOnTheFirmwareOffsets) {
    EXPECT_EQ(offsetof(tp::MotorCanXferResp, status), 0u);
    EXPECT_EQ(offsetof(tp::MotorCanXferResp, dlc), 1u);
    EXPECT_EQ(offsetof(tp::MotorCanXferResp, elapsed_ms), 2u);
    EXPECT_EQ(offsetof(tp::MotorCanXferResp, ext_id), 4u);
    EXPECT_EQ(offsetof(tp::MotorCanXferResp, data), 8u);
}

TEST(MotorCanXferCodec, CommandHasAName) {
    EXPECT_STREQ(tp::to_string(tp::Cmd::MotorCanExtXfer), "MotorCanExtXfer");
}

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// V2.0 / V2.1 protocol codec tests: the CalOp-multiplexed fisheye-camera
// (Cmd 0x2B) and leader-encoder-max (Cmd 0x2C) calibration payloads. Pure
// codec + the normalization converter — no hardware needed.
//
// The wire bytes here are transcribed from firmware docs/PROTOCOL.md §6.22 /
// §6.23 and App/protocol/protocol_handler.c (camera_fisheye_cal_do_read /
// encoder_max_cal_do_read), which are canonical.

#include <cmath>
#include <cstring>
#include <gtest/gtest.h>
#include <taccap/error.hpp>
#include <taccap/gripper_position.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tp = xense::taccap::protocol;
using xense::taccap::GripperPosition;
using xense::taccap::ProtocolError;

namespace {

// Build the read-response payload the firmware emits: the CalOp byte echoed
// back, followed by the record body.
template <typename T>
std::vector<uint8_t> make_read_response(const T& body) {
    std::vector<uint8_t> out(1 + sizeof(T));
    out[0] = static_cast<uint8_t>(tp::CalOp::Read);
    std::memcpy(out.data() + 1, &body, sizeof(T));
    return out;
}

}  // namespace

// ---- Wire constants (mirror firmware protocol_cmd.h / protocol_data.h) -----
TEST(CodecV21, CommandAndErrorWireValues) {
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::CameraFisheyeCal), 0x2Bu);
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::EncoderMaxCal),    0x2Cu);
    EXPECT_EQ(static_cast<uint8_t>(tp::ErrorCode::CalNotSet),  0x60u);
    EXPECT_EQ(static_cast<uint8_t>(tp::CalOp::Read),           0x00u);
    EXPECT_EQ(static_cast<uint8_t>(tp::CalOp::Write),          0x01u);
}

TEST(CodecV21, StructSizes) {
    // Body is 8 floats; the wire payload prepends the op byte.
    EXPECT_EQ(sizeof(tp::CameraFisheyeCal),      32u);
    EXPECT_EQ(tp::CAMERA_FISHEYE_CAL_FULL_SIZE,  33u);
    EXPECT_EQ(tp::ENCODER_MAX_CAL_FULL_SIZE,     5u);
}

TEST(CodecV21, ToStringCoversNewNames) {
    EXPECT_STREQ(tp::to_string(tp::Cmd::CameraFisheyeCal), "CameraFisheyeCal");
    EXPECT_STREQ(tp::to_string(tp::Cmd::EncoderMaxCal),    "EncoderMaxCal");
    EXPECT_STREQ(tp::to_string(tp::ErrorCode::CalNotSet),  "CalNotSet");
}

// ---- Fisheye camera calibration (Cmd 0x2B) --------------------------------

TEST(CodecV21, FisheyeReadRequestIsOpByteOnly) {
    auto wire = tp::encode_camera_fisheye_cal_read();
    ASSERT_EQ(wire.size(), 1u);
    EXPECT_EQ(wire[0], static_cast<uint8_t>(tp::CalOp::Read));
}

TEST(CodecV21, FisheyeWriteRequestIsOpPlusEightFloats) {
    tp::CameraFisheyeCal cal{320.5f, 321.25f, 319.0f, 239.5f,
                             -0.031f, 0.0072f, -0.0013f, 0.00021f};
    auto wire = tp::encode_camera_fisheye_cal_write(cal);
    ASSERT_EQ(wire.size(), tp::CAMERA_FISHEYE_CAL_FULL_SIZE);
    EXPECT_EQ(wire[0], static_cast<uint8_t>(tp::CalOp::Write));

    // Params start at offset 1 — deliberately unaligned, matching firmware.
    float params[8];
    std::memcpy(params, wire.data() + 1, sizeof(params));
    EXPECT_FLOAT_EQ(params[0], 320.5f);    // fx
    EXPECT_FLOAT_EQ(params[1], 321.25f);   // fy
    EXPECT_FLOAT_EQ(params[2], 319.0f);    // cx
    EXPECT_FLOAT_EQ(params[3], 239.5f);    // cy
    EXPECT_FLOAT_EQ(params[4], -0.031f);   // k1
    EXPECT_FLOAT_EQ(params[5], 0.0072f);   // k2
    EXPECT_FLOAT_EQ(params[6], -0.0013f);  // k3
    EXPECT_FLOAT_EQ(params[7], 0.00021f);  // k4
}

TEST(CodecV21, FisheyeDecodeStripsEchoedOpByte) {
    tp::CameraFisheyeCal cal{600.0f, 601.0f, 320.0f, 240.0f,
                             0.1f, -0.2f, 0.3f, -0.4f};
    auto resp = make_read_response(cal);
    ASSERT_EQ(resp.size(), tp::CAMERA_FISHEYE_CAL_FULL_SIZE);

    auto out = tp::decode_camera_fisheye_cal(resp.data(), resp.size());
    EXPECT_FLOAT_EQ(out.fx, 600.0f);
    EXPECT_FLOAT_EQ(out.fy, 601.0f);
    EXPECT_FLOAT_EQ(out.cx, 320.0f);
    EXPECT_FLOAT_EQ(out.cy, 240.0f);
    EXPECT_FLOAT_EQ(out.k1, 0.1f);
    EXPECT_FLOAT_EQ(out.k2, -0.2f);
    EXPECT_FLOAT_EQ(out.k3, 0.3f);
    EXPECT_FLOAT_EQ(out.k4, -0.4f);
}

TEST(CodecV21, FisheyeEncodeDecodeRoundtrip) {
    tp::CameraFisheyeCal cal{1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f, 7.5f, 8.5f};
    auto wire = tp::encode_camera_fisheye_cal_write(cal);
    // A write request and a read response share their body layout; only the
    // leading op byte differs, so patch it to Read to reuse the decoder.
    wire[0] = static_cast<uint8_t>(tp::CalOp::Read);
    auto out = tp::decode_camera_fisheye_cal(wire.data(), wire.size());
    EXPECT_EQ(0, std::memcmp(&cal, &out, sizeof(cal)));
}

TEST(CodecV21, FisheyeDecodeRejectsWrongLength) {
    std::vector<uint8_t> too_short(tp::CAMERA_FISHEYE_CAL_FULL_SIZE - 1, 0);
    EXPECT_THROW(tp::decode_camera_fisheye_cal(too_short.data(), too_short.size()),
                 ProtocolError);

    // A 1-byte error response must not be mistaken for data.
    std::vector<uint8_t> nack{static_cast<uint8_t>(tp::ErrorCode::CalNotSet)};
    EXPECT_THROW(tp::decode_camera_fisheye_cal(nack.data(), nack.size()),
                 ProtocolError);
}

TEST(CodecV21, FisheyeDecodeRejectsWrongOpByte) {
    tp::CameraFisheyeCal cal{};
    auto resp = make_read_response(cal);
    resp[0] = static_cast<uint8_t>(tp::CalOp::Write);
    EXPECT_THROW(tp::decode_camera_fisheye_cal(resp.data(), resp.size()),
                 ProtocolError);
}

// ---- Leader encoder max travel angle (Cmd 0x2C) ---------------------------

// ---- V2.10: fisheye params with their calibration resolution ---------------
// Wire bytes from firmware docs/PROTOCOL.md §6.22.1.

TEST(CodecV210, SizedWireConstants) {
    EXPECT_EQ(tp::CAMERA_FISHEYE_CAL_OP_READ_SIZED,  0x02u);
    EXPECT_EQ(tp::CAMERA_FISHEYE_CAL_OP_WRITE_SIZED, 0x03u);
    EXPECT_EQ(tp::CAMERA_FISHEYE_CAL_SIZED_SIZE,     37u);
    EXPECT_EQ(sizeof(tp::CameraFisheyeCalSized),     36u);
}

TEST(CodecV210, SizedReadRequestIsOpByteOnly) {
    auto wire = tp::encode_camera_fisheye_cal_read_sized();
    ASSERT_EQ(wire.size(), 1u);
    EXPECT_EQ(wire[0], 0x02u);
}

TEST(CodecV210, SizedWriteIsOpEightFloatsThenWidthHeightLittleEndian) {
    const tp::CameraFisheyeCal cal{477.1f, 476.8f, 968.3f, 563.2f,
                                   -0.021f, 0.0061f, -0.0042f, 0.0011f};
    auto wire = tp::encode_camera_fisheye_cal_write_sized(cal, 1920, 1080);
    ASSERT_EQ(wire.size(), tp::CAMERA_FISHEYE_CAL_SIZED_SIZE);
    EXPECT_EQ(wire[0], 0x03u);
    tp::CameraFisheyeCal params{};
    std::memcpy(&params, wire.data() + 1, sizeof(params));
    EXPECT_EQ(0, std::memcmp(&cal, &params, sizeof(cal)));
    // "... [32字节 params] 80 07 38 04" — 1920 = 0x0780, 1080 = 0x0438.
    EXPECT_EQ(wire[33], 0x80u);
    EXPECT_EQ(wire[34], 0x07u);
    EXPECT_EQ(wire[35], 0x38u);
    EXPECT_EQ(wire[36], 0x04u);
}

TEST(CodecV210, SizedEncodeDecodeRoundtrip) {
    const tp::CameraFisheyeCal cal{1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f, 7.5f, 8.5f};
    auto wire = tp::encode_camera_fisheye_cal_write_sized(cal, 640, 480);
    // Write request and read response share the body; only the op differs.
    wire[0] = tp::CAMERA_FISHEYE_CAL_OP_READ_SIZED;
    const auto out = tp::decode_camera_fisheye_cal_sized(wire.data(), wire.size());
    EXPECT_EQ(0, std::memcmp(&cal, &out.calibration, sizeof(cal)));
    EXPECT_EQ(out.width,  640u);
    EXPECT_EQ(out.height, 480u);
}

TEST(CodecV210, SizedDecodePassesUnknownSizeThrough) {
    const tp::CameraFisheyeCal cal{213.0f, 212.8f, 321.4f, 239.95f, 0, 0, 0, 0};
    auto wire = tp::encode_camera_fisheye_cal_write_sized(cal, 0, 0);
    wire[0] = tp::CAMERA_FISHEYE_CAL_OP_READ_SIZED;
    const auto out = tp::decode_camera_fisheye_cal_sized(wire.data(), wire.size());
    EXPECT_EQ(out.width,  0u);
    EXPECT_EQ(out.height, 0u);
    EXPECT_FLOAT_EQ(out.calibration.cx, 321.4f);
}

TEST(CodecV210, SizedDecodeRejectsWrongLengthAndOp) {
    // A legacy 33-byte read response is not a sized one.
    auto legacy = make_read_response(tp::CameraFisheyeCal{});
    EXPECT_THROW(tp::decode_camera_fisheye_cal_sized(legacy.data(), legacy.size()),
                 ProtocolError);
    std::vector<uint8_t> nack{static_cast<uint8_t>(tp::ErrorCode::InvalidParam)};
    EXPECT_THROW(tp::decode_camera_fisheye_cal_sized(nack.data(), nack.size()),
                 ProtocolError);
    // Right length, but the op echoed is the legacy read's.
    auto wire = tp::encode_camera_fisheye_cal_write_sized(tp::CameraFisheyeCal{}, 1, 1);
    wire[0] = static_cast<uint8_t>(tp::CalOp::Read);
    EXPECT_THROW(tp::decode_camera_fisheye_cal_sized(wire.data(), wire.size()),
                 ProtocolError);
}

TEST(CodecV21, EncoderMaxReadRequestIsOpByteOnly) {
    auto wire = tp::encode_encoder_max_cal_read();
    ASSERT_EQ(wire.size(), 1u);
    EXPECT_EQ(wire[0], static_cast<uint8_t>(tp::CalOp::Read));
}

TEST(CodecV21, EncoderMaxWriteRequestIsOpPlusFloat) {
    auto wire = tp::encode_encoder_max_cal_write(1.3f);
    ASSERT_EQ(wire.size(), tp::ENCODER_MAX_CAL_FULL_SIZE);
    EXPECT_EQ(wire[0], static_cast<uint8_t>(tp::CalOp::Write));
    float v = 0.0f;
    std::memcpy(&v, wire.data() + 1, sizeof(v));
    EXPECT_FLOAT_EQ(v, 1.3f);
}

TEST(CodecV21, EncoderMaxDecodeStripsEchoedOpByte) {
    auto resp = make_read_response(1.3f);
    ASSERT_EQ(resp.size(), tp::ENCODER_MAX_CAL_FULL_SIZE);
    EXPECT_FLOAT_EQ(tp::decode_encoder_max_cal(resp.data(), resp.size()), 1.3f);
}

TEST(CodecV21, EncoderMaxDecodeRejectsWrongLength) {
    std::vector<uint8_t> nack{static_cast<uint8_t>(tp::ErrorCode::CalNotSet)};
    EXPECT_THROW(tp::decode_encoder_max_cal(nack.data(), nack.size()),
                 ProtocolError);
}

// ---- Leader normalization map built from the encoder max travel angle ------

TEST(CodecV21, FromTravelMapsClosedToZeroAndOpenToOne) {
    auto m = GripperPosition::from_travel(1.3f);
    ASSERT_TRUE(m.valid());
    EXPECT_FLOAT_EQ(m.max_open_rad(), 1.3f);
    EXPECT_FLOAT_EQ(m.min_open_rad(), 0.0f);
    EXPECT_FALSE(m.reverse());

    EXPECT_FLOAT_EQ(m.to_position(0.0f),  0.0f);   // fully closed
    EXPECT_FLOAT_EQ(m.to_position(1.3f),  1.0f);   // fully open
    EXPECT_FLOAT_EQ(m.to_position(0.65f), 0.5f);   // midpoint

    EXPECT_FLOAT_EQ(m.to_rad(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(m.to_rad(1.0f), 1.3f);
    EXPECT_FLOAT_EQ(m.to_rad(0.5f), 0.65f);
}

TEST(CodecV21, FromTravelClampsOutsideTheCalibratedSpan) {
    auto m = GripperPosition::from_travel(1.3f);
    EXPECT_FLOAT_EQ(m.to_position(-0.4f), 0.0f);  // past closed
    EXPECT_FLOAT_EQ(m.to_position(2.0f),  1.0f);  // past open
    EXPECT_FLOAT_EQ(m.to_rad(-1.0f), 0.0f);
    EXPECT_FLOAT_EQ(m.to_rad(9.0f),  1.3f);
}

TEST(CodecV21, FromTravelIsInvalidForNonPositiveSpan) {
    EXPECT_FALSE(GripperPosition::from_travel(0.0f).valid());
    EXPECT_FALSE(GripperPosition::from_travel(-1.0f).valid());
    // Degenerate span: max must exceed min, not just be positive.
    EXPECT_FALSE(GripperPosition::from_travel(0.5f, 0.5f).valid());
    EXPECT_TRUE(GripperPosition::from_travel(0.5f, 0.1f).valid());
}

TEST(CodecV21, FromTravelHonoursReverse) {
    auto m = GripperPosition::from_travel(1.0f, 0.0f, /*reverse=*/true);
    ASSERT_TRUE(m.valid());
    EXPECT_TRUE(m.reverse());
    EXPECT_FLOAT_EQ(m.to_position(-1.0f), 1.0f);  // opens negative
    EXPECT_FLOAT_EQ(m.to_rad(1.0f),      -1.0f);
}

// A round-trip through the real firmware read path: wire bytes -> max_rad ->
// converter -> normalized position. This is the whole leader link in one test.
TEST(CodecV21, EncoderMaxWireBytesDriveTheNormalizedPosition) {
    auto resp = make_read_response(1.3f);
    const float max_rad = tp::decode_encoder_max_cal(resp.data(), resp.size());
    auto m = GripperPosition::from_travel(max_rad);
    ASSERT_TRUE(m.valid());
    EXPECT_NEAR(m.to_position(0.975f), 0.75f, 1e-6f);
}

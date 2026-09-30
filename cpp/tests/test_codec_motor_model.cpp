// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// The motor-model record (Cmd 0x59 / 0x5A, follower firmware 1.2.7+). Pure
// codec — no hardware.
//
// Why this layout is worth pinning byte by byte: the MCU quantises every MIT
// torque and velocity against the recorded model's ranges, so reading this
// record wrong does not fail, it rescales every frame. An EL05-configured MCU
// driving an RS00 turns a commanded 1.1 N·m into 2.57 N·m (14/6) and reports
// feedback back at 0.43x, which keeps the host's own torque ceiling from ever
// tripping. Nothing in the stream, the counters or the status word looks wrong.
//
// Offsets transcribed from the firmware's App/protocol/protocol_data.h
// (motor_model_t). scripts/check_protocol_drift.py compares the two structs by
// real compiled sizeof; this file pins the field offsets that sizeof cannot see.

#include <array>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <gtest/gtest.h>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tp = xense::taccap::protocol;

namespace {

// id=1 (RS00), name "RS00", from_flash=1, t_max=14.0, v_max=33.0
std::vector<uint8_t> rs00_record() {
    std::vector<uint8_t> b(tp::MOTOR_MODEL_SIZE, 0);
    b[0] = 1;                                   // id
    std::memcpy(&b[1], "RS00", 4);              // name[8], NUL-padded
    b[9] = 1;                                   // from_flash
    const float t = 14.0f, v = 33.0f;
    std::memcpy(&b[10], &t, 4);
    std::memcpy(&b[14], &v, 4);
    return b;
}

}  // namespace

TEST(MotorModelCodec, CommandsSitAtTheFirmwareOpcodes) {
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::GetMotorModel), 0x59);
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::SetMotorModel), 0x5A);
}

TEST(MotorModelCodec, WireSizeIsTwentyBytes) {
    EXPECT_EQ(sizeof(tp::MotorModel), tp::MOTOR_MODEL_SIZE);
    EXPECT_EQ(tp::MOTOR_MODEL_SIZE, 20u);
}

TEST(MotorModelCodec, FieldsLandOnTheFirmwareOffsets) {
    const auto bytes = rs00_record();
    tp::MotorModel m{};
    std::memcpy(&m, bytes.data(), sizeof(m));

    EXPECT_EQ(m.id, 1u);
    EXPECT_STREQ(std::string(m.name, ::strnlen(m.name, sizeof(m.name))).c_str(),
                 "RS00");
    EXPECT_EQ(m.from_flash, 1u);
    EXPECT_FLOAT_EQ(m.t_max_nm, 14.0f);
    EXPECT_FLOAT_EQ(m.v_max_rad_s, 33.0f);
}

TEST(MotorModelCodec, TheRangesAreWhatDistinguishTheTwoActuators) {
    // Not decoration: this ratio IS the failure mode. If a gripper reports EL05
    // ranges while carrying an RS00, every commanded torque is understated by
    // this factor and every reading overstated by its inverse.
    const auto bytes = rs00_record();
    tp::MotorModel rs00{};
    std::memcpy(&rs00, bytes.data(), sizeof(rs00));

    constexpr float kEl05TMax = 6.0f;
    constexpr float kEl05VMax = 50.0f;
    EXPECT_NEAR(rs00.t_max_nm / kEl05TMax, 2.333f, 1e-3f);
    EXPECT_NEAR(rs00.v_max_rad_s / kEl05VMax, 0.660f, 1e-3f);
}

TEST(MotorModelCodec, AnUnprovisionedDeviceSaysSo) {
    // from_flash == 0 means the MCU fell back to its compile-time default. That
    // is a legitimate state, not an error -- but it is the state in which the
    // ranges are an assumption rather than a record, so a caller that cares
    // must be able to tell it apart from a provisioned device.
    std::vector<uint8_t> b(tp::MOTOR_MODEL_SIZE, 0);
    std::memcpy(&b[1], "EL05", 4);
    const float t = 6.0f, v = 50.0f;
    std::memcpy(&b[10], &t, 4);
    std::memcpy(&b[14], &v, 4);

    tp::MotorModel m{};
    std::memcpy(&m, b.data(), sizeof(m));
    EXPECT_EQ(m.from_flash, 0u);
    EXPECT_EQ(m.id, 0u) << "EL05 keeps id 0; ids are persisted and never reused";
}

// Follower 1.2.10 turned reserved[0] into `autodetected`. Byte 18, right after
// v_max -- if it slid, the flag would read garbage from the padding byte.
TEST(MotorModelCodec, AutodetectedSitsAtByte18) {
    EXPECT_EQ(offsetof(tp::MotorModel, autodetected), 18u);
    std::vector<uint8_t> b(tp::MOTOR_MODEL_SIZE, 0);
    b[18] = 1;
    tp::MotorModel m{};
    std::memcpy(&m, b.data(), sizeof(m));
    EXPECT_EQ(m.autodetected, 1u);
    EXPECT_EQ(m.default_envelope, 0u);
}

// Follower 1.2.12 turned the last reserved byte into `default_envelope`.
TEST(MotorModelCodec, DefaultEnvelopeSitsAtByte19) {
    EXPECT_EQ(offsetof(tp::MotorModel, default_envelope), 19u);
    std::vector<uint8_t> b(tp::MOTOR_MODEL_SIZE, 0);
    b[19] = 1;
    tp::MotorModel m{};
    std::memcpy(&m, b.data(), sizeof(m));
    EXPECT_EQ(m.default_envelope, 1u);
    EXPECT_EQ(m.autodetected, 0u);
}

// ---- Motor firmware version: vendor text <-> wire bytes ----------------------
// RobStride writes the EL05's {10,5,0,4} as "1.0.5.0.4" (the 10 split) and the
// RS00's {0,0,3,32} as "0.0.3.32". Operators type what the nameplate says.

TEST(MotorFwVersionText, FormatsTheWayTheVendorWritesIt) {
    const uint8_t el05[4] = {10, 5, 0, 4};
    const uint8_t rs00[4] = {0, 0, 3, 32};
    EXPECT_EQ(tp::format_motor_fw_version(el05), "1.0.5.0.4");
    EXPECT_EQ(tp::format_motor_fw_version(rs00), "0.0.3.32");
}

TEST(MotorFwVersionText, UnknownLeadingBytesPrintPlainNotSplit) {
    const uint8_t a[4] = {3, 1, 0, 0};
    const uint8_t b[4] = {25, 1, 0, 0};
    EXPECT_EQ(tp::format_motor_fw_version(a), "3.1.0.0");
    EXPECT_EQ(tp::format_motor_fw_version(b), "25.1.0.0");
}

TEST(MotorFwVersionText, ParsesVendorFormAndRawForm) {
    using A = std::array<uint8_t, 4>;
    EXPECT_EQ(tp::parse_motor_fw_version("1.0.5.0.4"), (A{10, 5, 0, 4}));
    EXPECT_EQ(tp::parse_motor_fw_version("10.5.0.4"), (A{10, 5, 0, 4}));
    EXPECT_EQ(tp::parse_motor_fw_version("0.0.3.32"), (A{0, 0, 3, 32}));
}

TEST(MotorFwVersionText, RoundTripsEveryLeadingByte) {
    for (int b0 = 0; b0 < 256; ++b0) {
        const uint8_t v[4] = {static_cast<uint8_t>(b0), 7, 0, 250};
        const auto back = tp::parse_motor_fw_version(tp::format_motor_fw_version(v));
        EXPECT_EQ(back[0], b0) << tp::format_motor_fw_version(v);
        EXPECT_EQ(back[3], 250);
    }
}

TEST(MotorFwVersionText, RejectsWhatIsNotAVersion) {
    for (const char* bad : {"", "1.0.5", "1.0.5.0.4.1", "2.0.5.0.4", "1.10.5.0.4",
                            "0.0.3.256", "a.b.c.d", "0..3.32", "0.0.3.32 "}) {
        EXPECT_THROW(tp::parse_motor_fw_version(bad), std::invalid_argument) << bad;
    }
}

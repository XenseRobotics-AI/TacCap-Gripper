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

#include <cstring>
#include <gtest/gtest.h>
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

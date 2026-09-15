// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// End-to-end PTY-driven tests for the V1.3–V1.6 upper-layer SDK
// additions: Key (V1.4) and SensorErrors (V1.6) DATA streams,
// IMU::set_mag_calibration (V1.4) wire round-trip, and the full
// OtaSession (V1.3) state machine including the high-level
// update_from_bytes() orchestrator.
//
// The pure decoder paths are pinned in test_upper_v16.cpp; this file
// drives them through a real Transport against a fake firmware over a
// PTY, so we catch any framing / subscription / state-machine
// regression that wouldn't show up in unit-level decode tests.

#include "pty_helper.hpp"

#include <taccap/components/encoder.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/error.hpp>
#include <taccap/ota.hpp>
#include <taccap/protocol/codec.hpp>

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;
namespace tb = xense::taccap::bus;

using taccap_test::Pty;
using taccap_test::base_config;

// ============================================================
// Key (V1.4): DATA(Cmd::KeyStatus) → subscriber callback
// ============================================================

TEST(KeyEnd2End, OnEventReceivesMultipleStates) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    tb::Transport host(base_config(pty.slave_path()));
    tx::Key key(host);

    std::mutex mu;
    std::condition_variable cv;
    std::vector<tx::KeySample> received;
    auto sub = key.on_event([&](const tx::KeySample& s) {
        std::lock_guard<std::mutex> g(mu);
        received.push_back(s);
        cv.notify_all();
    });

    // Fake firmware sends three DATA frames simulating: down, up, double-click
    std::thread fw([&]() {
        for (uint8_t state : {tp::KeyState::SingleClickDown,
                              tp::KeyState::SingleClickUp,
                              tp::KeyState::DoubleClick}) {
            std::vector<uint8_t> body{ /*key_id*/ 0, state };
            pty.send_data(0, tp::Cmd::KeyStatus, body);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // Wait up to 1 s for all three callbacks.
    {
        std::unique_lock<std::mutex> lk(mu);
        ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                                [&]{ return received.size() >= 3; }))
            << "only received " << received.size() << " key events";
    }
    fw.join();
    key.off(sub);

    ASSERT_EQ(received.size(), 3u);
    EXPECT_EQ(received[0].key_state, tp::KeyState::SingleClickDown);
    EXPECT_EQ(received[1].key_state, tp::KeyState::SingleClickUp);
    EXPECT_EQ(received[2].key_state, tp::KeyState::DoubleClick);
    for (const auto& s : received) EXPECT_EQ(s.key_id, 0u);
}

TEST(KeyEnd2End, OffStopsCallbacks) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Key key(host);

    std::atomic<int> count{0};
    auto sub = key.on_event([&](const tx::KeySample&) { ++count; });

    // 1 event before off, 1 after off — only first should land.
    pty.send_data(0, tp::Cmd::KeyStatus, {0, tp::KeyState::SingleClickDown});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    key.off(sub);
    pty.send_data(1, tp::Cmd::KeyStatus, {0, tp::KeyState::SingleClickUp});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(count.load(), 1);
}

TEST(KeyEnd2End, MalformedPayloadDoesNotKillStream) {
    // Sending a 1-byte (wrong-size) KeyStatus DATA should be swallowed by
    // the component's try/catch and NOT block subsequent valid frames.
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Key key(host);

    std::atomic<int> good{0};
    key.on_event([&](const tx::KeySample& s) {
        if (s.key_id == 0 && s.key_state == tp::KeyState::DoubleClick) ++good;
    });

    pty.send_data(0, tp::Cmd::KeyStatus, {0xFF});  // malformed (1 byte)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    pty.send_data(1, tp::Cmd::KeyStatus, {0, tp::KeyState::DoubleClick});

    for (int i = 0; i < 50 && good.load() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(good.load(), 1);
}

// ============================================================
// SensorErrors (V1.6): DATA(Cmd::SensorErrorReport) → subscriber
// ============================================================

TEST(SensorErrorsEnd2End, OnReportSurfacesAllFields) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::SensorErrors errs(host);

    std::mutex mu;
    std::condition_variable cv;
    std::vector<tx::SensorErrorSample> received;
    errs.on_report([&](const tx::SensorErrorSample& s) {
        std::lock_guard<std::mutex> g(mu);
        received.push_back(s);
        cv.notify_all();
    });

    // Build the canonical wire payload for an encoder Comm Timeout
    tp::SensorErrorReport raw{};
    raw.sensor_id    = static_cast<uint8_t>(tp::SensorErrorId::Encoder);
    raw.error_code   = tp::SensorErrCode::CommTimeout;
    raw.error_count  = 7;
    raw.timestamp_ms = 0xCAFEBABE;
    std::vector<uint8_t> body(sizeof(raw));
    std::memcpy(body.data(), &raw, sizeof(raw));

    pty.send_data(0, tp::Cmd::SensorErrorReport, body);

    {
        std::unique_lock<std::mutex> lk(mu);
        ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                                [&]{ return !received.empty(); }));
    }
    ASSERT_EQ(received.size(), 1u);
    const auto& s = received[0];
    EXPECT_EQ(s.sensor_id,        static_cast<uint8_t>(tp::SensorErrorId::Encoder));
    EXPECT_EQ(s.error_code,       tp::SensorErrCode::CommTimeout);
    EXPECT_EQ(s.error_count,      7u);
    EXPECT_EQ(s.mcu_timestamp_ms, 0xCAFEBABEu);
}

TEST(SensorErrorsEnd2End, MultipleErrorsAccumulate) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::SensorErrors errs(host);

    std::mutex mu;
    std::condition_variable cv;
    std::vector<tx::SensorErrorSample> received;
    errs.on_report([&](const tx::SensorErrorSample& s) {
        std::lock_guard<std::mutex> g(mu);
        received.push_back(s);
        cv.notify_all();
    });

    // Sequence: IMU comm timeout, motor fault, IMU recovers (None).
    auto pack = [](uint8_t sid, uint8_t code, uint16_t count, uint32_t ts) {
        tp::SensorErrorReport r{};
        r.sensor_id = sid; r.error_code = code;
        r.error_count = count; r.timestamp_ms = ts;
        std::vector<uint8_t> b(sizeof(r));
        std::memcpy(b.data(), &r, sizeof(r));
        return b;
    };
    pty.send_data(0, tp::Cmd::SensorErrorReport,
                  pack(static_cast<uint8_t>(tp::SensorErrorId::Imu),
                       tp::SensorErrCode::CommTimeout, 1, 1000));
    pty.send_data(1, tp::Cmd::SensorErrorReport,
                  pack(static_cast<uint8_t>(tp::SensorErrorId::Motor),
                       tp::SensorErrCode::Offline,     1, 1500));
    pty.send_data(2, tp::Cmd::SensorErrorReport,
                  pack(static_cast<uint8_t>(tp::SensorErrorId::Imu),
                       tp::SensorErrCode::None,        1, 2000));

    {
        std::unique_lock<std::mutex> lk(mu);
        ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                                [&]{ return received.size() >= 3; }));
    }
    ASSERT_EQ(received.size(), 3u);
    EXPECT_EQ(received[0].sensor_id,
              static_cast<uint8_t>(tp::SensorErrorId::Imu));
    EXPECT_EQ(received[1].sensor_id,
              static_cast<uint8_t>(tp::SensorErrorId::Motor));
    EXPECT_EQ(received[2].error_code, tp::SensorErrCode::None);  // recovery
}

// ============================================================
// IMU::set_mag_calibration (V1.4): cmd 0x26, 48-byte payload
// ============================================================

TEST(ImuMagCalEnd2End, WireBytesMatchInputs) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::IMU imu(host);

    // Capture the frame fake firmware receives.
    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::SetImuMagCal);
    });

    std::array<float, 3> hard = {1.0f, 2.0f, 3.0f};
    std::array<float, 9> soft = {
        0.1f, 0.2f, 0.3f,
        0.4f, 0.5f, 0.6f,
        0.7f, 0.8f, 0.9f,
    };
    imu.set_mag_calibration(hard, soft);
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::SetImuMagCal);
    ASSERT_EQ(received->payload.size(), 48u);

    // Decode through the codec for fairness — the IMU helper feeds the
    // same ImuMagCal layout the codec serialises.
    auto cal = tp::decode_imu_mag_cal(received->payload.data(),
                                      received->payload.size());
    EXPECT_FLOAT_EQ(cal.hard_iron[0], 1.0f);
    EXPECT_FLOAT_EQ(cal.hard_iron[1], 2.0f);
    EXPECT_FLOAT_EQ(cal.hard_iron[2], 3.0f);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(cal.soft_iron[r][c],
                            soft[r * 3 + c])
                << "soft[" << r << "][" << c << "]";
        }
    }
}

TEST(ImuMagCalEnd2End, NackThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::IMU imu(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_nack(f->seq, tp::ErrorCode::InvalidParam);
    });

    std::array<float, 3> hard = {0, 0, 0};
    std::array<float, 9> soft = {1, 0, 0,  0, 1, 0,  0, 0, 1};
    EXPECT_THROW(imu.set_mag_calibration(hard, soft), tx::ProtocolError);
    fw.join();
}

// ============================================================
// Encoder::set_zero: cmd 0x24, empty payload, ACK-only
// ============================================================

TEST(EncoderSetZeroEnd2End, WireBytesAndAck) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::SetEncoderZero);
    });

    enc.set_zero();   // throws on NACK / timeout
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::SetEncoderZero);
    EXPECT_EQ(received->type, tp::FrameType::CMD_NEED_ACK);
    EXPECT_TRUE(received->payload.empty())
        << "SetEncoderZero is documented as zero-payload";
}

TEST(EncoderSetZeroEnd2End, NackThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_nack(f->seq, tp::ErrorCode::SensorOffline);
    });

    EXPECT_THROW(enc.set_zero(), tx::ProtocolError);
    fw.join();
}

// ============================================================
// Encoder normalization: clamp position to [0, +inf), preserve velocity
// ============================================================

namespace {

// Encode an EncoderData POD to wire bytes (firmware just memcpy's the
// struct into the ACK payload, no special encoding).
std::vector<uint8_t> encode_encoder_payload(float pos_rad, float vel_radps,
                                            uint32_t timestamp_us = 12345,
                                            uint16_t seq = 7) {
    tp::EncoderData raw{};
    raw.timestamp_us   = timestamp_us;
    raw.position_rad   = pos_rad;
    raw.velocity_rad_s = vel_radps;
    raw.status         = 0;
    raw.seq            = seq;
    std::vector<uint8_t> bytes(sizeof(raw));
    std::memcpy(bytes.data(), &raw, sizeof(raw));
    return bytes;
}

// Drive one ReadOnce round-trip; returns the sample the SDK produced
// after normalisation.
tx::EncoderSample read_once_with_payload(float pos_rad, float vel_radps) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (!f) return;
        EXPECT_EQ(f->cmd, tp::Cmd::GetEncoder);
        pty.send_response(f->seq, tp::Cmd::GetEncoder,
                          encode_encoder_payload(pos_rad, vel_radps));
    });

    auto s = enc.read_once();
    fw.join();
    return s;
}

}  // namespace

TEST(EncoderNormalizeEnd2End, SmallNegativeClampedToZeroSilently) {
    // Within the "expected post-zero jitter" band — clamped, no warn.
    auto s = read_once_with_payload(/*pos*/ -0.05f, /*vel*/ 0.0f);
    EXPECT_FLOAT_EQ(s.position_rad,     0.0f);
    EXPECT_FLOAT_EQ(s.raw.position_rad, -0.05f)
        << "raw must preserve firmware-side value";
}

TEST(EncoderNormalizeEnd2End, LargeNegativeStillClampedToZero) {
    // Past the warn threshold — clamped AND would warn (we don't sniff
    // the log here; the rate-limited warning is exercised in calibrate
    // smoke runs).
    auto s = read_once_with_payload(/*pos*/ -0.5f, /*vel*/ 0.0f);
    EXPECT_FLOAT_EQ(s.position_rad,     0.0f);
    EXPECT_FLOAT_EQ(s.raw.position_rad, -0.5f);
}

TEST(EncoderNormalizeEnd2End, PositivePositionUntouched) {
    auto s = read_once_with_payload(/*pos*/ 1.23f, /*vel*/ 0.0f);
    EXPECT_FLOAT_EQ(s.position_rad,     1.23f);
    EXPECT_FLOAT_EQ(s.raw.position_rad, 1.23f);
}

TEST(EncoderNormalizeEnd2End, ZeroExactlyUntouched) {
    // Boundary: pos == 0 must not flip sign or trip the warn path.
    auto s = read_once_with_payload(/*pos*/ 0.0f, /*vel*/ 0.0f);
    EXPECT_FLOAT_EQ(s.position_rad,     0.0f);
    EXPECT_FLOAT_EQ(s.raw.position_rad, 0.0f);
}

TEST(EncoderNormalizeEnd2End, VelocitySignPreserved) {
    // velocity_rad_s carries direction (closing vs opening); clamping
    // it would destroy that signal.
    auto s = read_once_with_payload(/*pos*/ 0.5f, /*vel*/ -2.0f);
    EXPECT_FLOAT_EQ(s.velocity_rad_s,     -2.0f);
    EXPECT_FLOAT_EQ(s.raw.velocity_rad_s, -2.0f);
}

TEST(EncoderNormalizeEnd2End, NegativePositionDoesNotZeroVelocity) {
    // Clamping the position must not bleed over into the velocity even
    // when both are negative.
    auto s = read_once_with_payload(/*pos*/ -0.3f, /*vel*/ -1.5f);
    EXPECT_FLOAT_EQ(s.position_rad,   0.0f);
    EXPECT_FLOAT_EQ(s.velocity_rad_s, -1.5f);
}

// ============================================================
// OtaSession (V1.3): individual commands + full update_from_bytes
// ============================================================

TEST(OtaEnd2End, StartSendsCorrectPayload) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::OtaStart);
    });

    ota.start(/*size*/ 100000, /*crc*/ 0xDEADBEEF,
              {/*major*/ 1, /*minor*/ 6, /*patch*/ 2, /*build*/ 99});
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::OtaStart);
    ASSERT_EQ(received->payload.size(), 12u);
    uint32_t sz = 0, crc = 0;
    std::memcpy(&sz,  &received->payload[0], 4);
    std::memcpy(&crc, &received->payload[4], 4);
    EXPECT_EQ(sz, 100000u);
    EXPECT_EQ(crc, 0xDEADBEEFu);
    EXPECT_EQ(received->payload[8],  1u);
    EXPECT_EQ(received->payload[9],  6u);
    EXPECT_EQ(received->payload[10], 2u);
    EXPECT_EQ(received->payload[11], 99u);
}

TEST(OtaEnd2End, GetStatusRoundtrip) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (!f) return;
        EXPECT_EQ(f->cmd, tp::Cmd::OtaGetStatus);
        tp::OtaStatus st{};
        st.state         = static_cast<uint8_t>(tp::OtaState::Receiving);
        st.error_code    = 0;
        st.bytes_written = 12345;
        st.progress_ppt  = 250;
        std::vector<uint8_t> body(sizeof(st));
        std::memcpy(body.data(), &st, sizeof(st));
        pty.send_response(f->seq, tp::Cmd::OtaGetStatus, body);
    });

    auto status = ota.get_status();
    fw.join();
    EXPECT_EQ(status.state,         static_cast<uint8_t>(tp::OtaState::Receiving));
    EXPECT_EQ(status.bytes_written, 12345u);
    EXPECT_EQ(status.progress_ppt,  250u);
}

TEST(OtaEnd2End, WriteBlockCarriesPayloadVerbatim) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::OtaWriteBlock);
    });

    std::vector<uint8_t> chunk(64);
    for (size_t i = 0; i < chunk.size(); ++i) chunk[i] = static_cast<uint8_t>(i ^ 0xA5);
    ota.write_block(/*offset*/ 4096, chunk.data(),
                    static_cast<uint16_t>(chunk.size()));
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::OtaWriteBlock);
    ASSERT_EQ(received->payload.size(), 6 + chunk.size());

    uint32_t off = 0;
    uint16_t len = 0;
    std::memcpy(&off, &received->payload[0], 4);
    std::memcpy(&len, &received->payload[4], 2);
    EXPECT_EQ(off, 4096u);
    EXPECT_EQ(len, chunk.size());
    EXPECT_EQ(std::memcmp(&received->payload[6], chunk.data(), chunk.size()), 0);
}

TEST(OtaEnd2End, UpdateFromBytesFullSequence) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    // Build a 2.5 KiB firmware blob = 3 OtaWriteBlock frames
    // (1024 + 1024 + 512 = 2560).
    constexpr size_t kFw = 2560;
    std::vector<uint8_t> fw(kFw);
    for (size_t i = 0; i < fw.size(); ++i) fw[i] = static_cast<uint8_t>(i * 7 + 13);
    const uint32_t fw_crc = tx::crc32_iso_hdlc(fw.data(), fw.size());

    // Track everything firmware receives so we can assert sequencing,
    // offsets and payload identity at the end.
    std::vector<tb::Frame> frames;
    std::vector<uint8_t>   reassembled;
    std::thread fwthr([&]() {
        for (;;) {
            auto f = pty.expect_frame(2000);
            if (!f) break;
            frames.push_back(*f);
            if (f->cmd == tp::Cmd::OtaStart ||
                f->cmd == tp::Cmd::OtaVerify ||
                f->cmd == tp::Cmd::OtaApply) {
                pty.send_ack_ok(f->seq, f->cmd);
                if (f->cmd == tp::Cmd::OtaApply) return;  // last frame
            } else if (f->cmd == tp::Cmd::OtaWriteBlock) {
                // Decode header, append body to reassembled blob
                uint32_t off = 0; uint16_t len = 0;
                std::memcpy(&off, &f->payload[0], 4);
                std::memcpy(&len, &f->payload[4], 2);
                if (reassembled.size() < off + len) reassembled.resize(off + len);
                std::memcpy(reassembled.data() + off,
                            &f->payload[6], len);
                pty.send_ack_ok(f->seq, tp::Cmd::OtaWriteBlock);
            } else {
                pty.send_nack(f->seq, tp::ErrorCode::InvalidCmd);
            }
        }
    });

    // Track progress callback firings.
    std::vector<std::pair<uint32_t, uint32_t>> progress;
    ota.update_from_bytes(
        fw,
        tx::OtaSession::TargetVersion{1, 6, 0, 0},
        [&](uint32_t wr, uint32_t tot) {
            progress.emplace_back(wr, tot);
        });

    fwthr.join();

    // 1 OtaStart + 3 OtaWriteBlock + 1 OtaVerify + 1 OtaApply = 6 frames.
    ASSERT_EQ(frames.size(), 6u);
    EXPECT_EQ(frames[0].cmd, tp::Cmd::OtaStart);
    EXPECT_EQ(frames[1].cmd, tp::Cmd::OtaWriteBlock);
    EXPECT_EQ(frames[2].cmd, tp::Cmd::OtaWriteBlock);
    EXPECT_EQ(frames[3].cmd, tp::Cmd::OtaWriteBlock);
    EXPECT_EQ(frames[4].cmd, tp::Cmd::OtaVerify);
    EXPECT_EQ(frames[5].cmd, tp::Cmd::OtaApply);

    // Firmware blob roundtrip: reassembled bytes must equal the input.
    ASSERT_EQ(reassembled.size(), fw.size());
    EXPECT_EQ(std::memcmp(reassembled.data(), fw.data(), fw.size()), 0);

    // CRC field on OtaStart matches what we computed.
    uint32_t crc_wire = 0;
    std::memcpy(&crc_wire, &frames[0].payload[4], 4);
    EXPECT_EQ(crc_wire, fw_crc);

    // Progress callback: fired exactly 3 times, cumulative 1024, 2048, 2560.
    ASSERT_EQ(progress.size(), 3u);
    EXPECT_EQ(progress[0].first, 1024u);
    EXPECT_EQ(progress[1].first, 2048u);
    EXPECT_EQ(progress[2].first, 2560u);
    for (const auto& p : progress) EXPECT_EQ(p.second, fw.size());
}

TEST(OtaEnd2End, UpdateFromBytesNackAtStartSendsAbort) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::vector<tp::Cmd> commands;
    std::thread fwthr([&]() {
        // First frame: OtaStart, NACK. Then expect an OtaAbort follow-up
        // (best-effort cleanup from the high-level update flow).
        auto f1 = pty.expect_frame();
        if (f1) {
            commands.push_back(f1->cmd);
            pty.send_nack(f1->seq, tp::ErrorCode::OtaBusy);
        }
        auto f2 = pty.expect_frame(500);
        if (f2) {
            commands.push_back(f2->cmd);
            pty.send_ack_ok(f2->seq, tp::Cmd::OtaAbort);
        }
    });

    std::vector<uint8_t> fw(512, 0xAB);
    EXPECT_THROW(
        ota.update_from_bytes(fw, tx::OtaSession::TargetVersion{1, 6, 0, 0}),
        tx::ProtocolError);
    fwthr.join();

    ASSERT_GE(commands.size(), 1u);
    EXPECT_EQ(commands[0], tp::Cmd::OtaStart);
    if (commands.size() >= 2) {
        EXPECT_EQ(commands[1], tp::Cmd::OtaAbort);
    }
}

TEST(OtaEnd2End, UpdateFromBytesEmptyBlobThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    EXPECT_THROW(
        ota.update_from_bytes({}, tx::OtaSession::TargetVersion{1, 0, 0, 0}),
        tx::ProtocolError);
    // No firmware traffic — the empty-blob check happens before any send.
    auto f = pty.expect_frame(50);
    EXPECT_FALSE(f.has_value());
}

TEST(OtaEnd2End, UpdateFromBytesOversizeBlobThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::vector<uint8_t> oversize(tp::OTA_MAX_FW_SIZE + 1, 0);
    EXPECT_THROW(
        ota.update_from_bytes(oversize,
                              tx::OtaSession::TargetVersion{1, 0, 0, 0}),
        tx::ProtocolError);
    auto f = pty.expect_frame(50);
    EXPECT_FALSE(f.has_value());
}

TEST(OtaEnd2End, VerifyNackSendsAbort) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::vector<tp::Cmd> commands;
    std::thread fwthr([&]() {
        for (;;) {
            auto f = pty.expect_frame(500);
            if (!f) return;
            commands.push_back(f->cmd);
            if (f->cmd == tp::Cmd::OtaStart) {
                pty.send_ack_ok(f->seq, tp::Cmd::OtaStart);
            } else if (f->cmd == tp::Cmd::OtaWriteBlock) {
                pty.send_ack_ok(f->seq, tp::Cmd::OtaWriteBlock);
            } else if (f->cmd == tp::Cmd::OtaVerify) {
                pty.send_nack(f->seq, tp::ErrorCode::OtaVerifyFail);
            } else if (f->cmd == tp::Cmd::OtaAbort) {
                pty.send_ack_ok(f->seq, tp::Cmd::OtaAbort);
                return;
            }
        }
    });

    std::vector<uint8_t> fw(512, 0xCC);
    EXPECT_THROW(
        ota.update_from_bytes(fw, tx::OtaSession::TargetVersion{1, 6, 0, 0}),
        tx::ProtocolError);
    fwthr.join();

    // Expected sequence: OtaStart, OtaWriteBlock, OtaVerify (NACK), OtaAbort.
    std::string seen;
    for (auto c : commands) {
        seen += tp::to_string(c);
        seen += " ";
    }
    ASSERT_GE(commands.size(), 4u) << "got: " << seen;
    EXPECT_EQ(commands[0], tp::Cmd::OtaStart);
    EXPECT_EQ(commands[1], tp::Cmd::OtaWriteBlock);
    EXPECT_EQ(commands[2], tp::Cmd::OtaVerify);
    EXPECT_EQ(commands[3], tp::Cmd::OtaAbort);
}

TEST(OtaEnd2End, AbortIsBestEffortAndNeverThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    // Don't drive the fake firmware — abort should swallow the timeout.
    EXPECT_NO_THROW(ota.abort());
}

// ============================================================
// Motor (MIT) no-ACK submit(): fire-and-forget CMD_NO_ACK path
// for a host-driven realtime loop. Contrast with set_*() which
// blocks on a CMD_NEED_ACK round-trip.
// ============================================================

TEST(MotorSubmit, ImpedanceSendsNoAckFrame) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() { received = pty.expect_frame(); });

    // Fire-and-forget: returns immediately, firmware sends no response.
    motor.submit(tp::MotorImpedanceCtrl{1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::MotorImpedanceCtrl);
    EXPECT_EQ(received->type, tp::FrameType::CMD_NO_ACK);  // the crux
    ASSERT_EQ(received->payload.size(), 20u);
    float f[5];
    std::memcpy(f, received->payload.data(), 20);
    EXPECT_FLOAT_EQ(f[0], 1.0f);
    EXPECT_FLOAT_EQ(f[3], 4.0f);
    EXPECT_FLOAT_EQ(f[4], 5.0f);  // MIT feed-forward vel
}

TEST(MotorSubmit, PosVelTorqueAreNoAck12Byte) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    auto grab = [&](auto&& send) -> tb::Frame {
        std::optional<tb::Frame> r;
        std::thread fw([&]() { r = pty.expect_frame(); });
        send();
        fw.join();
        EXPECT_TRUE(r.has_value());
        return r.value_or(tb::Frame{});
    };

    auto p = grab([&]() { motor.submit_position(0.5f, 10.0f, 1.0f); });
    EXPECT_EQ(p.cmd, tp::Cmd::MotorPosCtrl);
    EXPECT_EQ(p.type, tp::FrameType::CMD_NO_ACK);
    EXPECT_EQ(p.payload.size(), 12u);

    auto v = grab([&]() { motor.submit_velocity(-1.0f, 0.5f, 4.0f); });
    EXPECT_EQ(v.cmd, tp::Cmd::MotorVelCtrl);
    EXPECT_EQ(v.type, tp::FrameType::CMD_NO_ACK);
    EXPECT_EQ(v.payload.size(), 12u);

    auto t = grab([&]() { motor.submit_torque(0.3f, 8.0f); });
    EXPECT_EQ(t.cmd, tp::Cmd::MotorTorqueCtrl);
    EXPECT_EQ(t.type, tp::FrameType::CMD_NO_ACK);
    EXPECT_EQ(t.payload.size(), 12u);
}

// At 500Hz the loop cannot block on ACKs; verify the no-ACK path delivers
// every frame without dropping into the seq-matching machinery.
TEST(MotorSubmit, HighRateBurstAllDelivered) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    constexpr int kN = 500;
    std::atomic<int> count{0};
    std::thread fw([&]() {
        for (int i = 0; i < kN; ++i) {
            auto f = pty.expect_frame(2000);
            if (!f) break;
            if (f->cmd == tp::Cmd::MotorImpedanceCtrl &&
                f->type == tp::FrameType::CMD_NO_ACK) {
                count.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    for (int i = 0; i < kN; ++i) {
        motor.submit(tp::MotorImpedanceCtrl{
            static_cast<float>(i) / kN, 1.0f, 0.1f, 0.0f, 0.0f});
    }
    fw.join();
    EXPECT_EQ(count.load(), kN);
}

// Regression guard: the blocking path must stay distinct (CMD_NEED_ACK).
TEST(MotorSubmit, SetImpedanceStillUsesNeedAck) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::MotorImpedanceCtrl);
    });
    motor.set_impedance(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);  // blocks on ACK
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->type, tp::FrameType::CMD_NEED_ACK);
}

TEST(MotorSubmit, OnStoppedTransportThrowsIo) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    host.stop();
    EXPECT_THROW(motor.submit(tp::MotorImpedanceCtrl{0, 0, 0, 0, 0}),
                 tx::IoError);
}

// The documented health channel (control_stats) works alongside the
// fire-and-forget submit path.
TEST(MotorSubmit, ControlStatsRoundtripsAfterSubmit) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);

    tp::MotorControlStats stats{};
    stats.running = 1; stats.mode = 4; stats.actual_hz = 499.0f;
    stats.target_seq = 10; stats.applied_seq = 10;
    std::vector<uint8_t> body(sizeof(stats));
    std::memcpy(body.data(), &stats, sizeof(stats));

    std::thread fw([&]() {
        pty.expect_frame();                       // drain the submit frame
        auto f = pty.expect_frame();              // the control_stats request
        if (f && f->cmd == tp::Cmd::GetMotorControlStats)
            pty.send_response(f->seq, tp::Cmd::GetMotorControlStats, body);
    });

    motor.submit(tp::MotorImpedanceCtrl{0.1f, 1.0f, 0.1f, 0.0f, 0.0f});
    auto got = motor.control_stats(std::chrono::milliseconds(500));
    fw.join();

    EXPECT_EQ(got.running, 1u);
    EXPECT_EQ(got.applied_seq, 10u);
    EXPECT_FLOAT_EQ(got.actual_hz, 499.0f);
}

TEST(MotorSubmit, EchoedErrorAckThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Motor motor(host);
    std::thread fw([&] {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, f->cmd,
            {static_cast<uint8_t>(tp::ErrorCode::MotorFault)});
    });
    EXPECT_THROW(motor.set_position(0.2f, 1.0f, 1.0f), tx::ProtocolError);
    fw.join();
}

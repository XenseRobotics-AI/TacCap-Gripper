// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Motor firmware update — reflashing the RobStride motor module itself, not the
// gripper MCU (that is ota.hpp). Rides on Motor::can_ext_xfer (Cmd 0x5B,
// follower firmware 1.2.8+): the MCU relays each CAN frame and waits for the
// motor's reply, the sequencing lives here.
//
// RobStride's OTA protocol ("OTA 协议说明", 20251114), communication types:
//
//    0  get device id  -> 64-bit MCU UID          (reply goes to 0xFE)
//   11  OTA start      payload = that UID
//   12  OTA info       payload = image size (LE u32) + pack count (LE u32)
//   13  OTA data       id bits 23..8 = pack index, payload = 8 image bytes
//   14  OTA end        payload = pack count (LE u32)
//
// Every frame is answered before the next is sent. Replies carry the result in
// id bits 23..16 (0 = ok), the motor id in 15..8 and the host id in 7..0. A
// failed data reply carries the index to resume from in data[0..1]; a failed
// end reply carries it in data[0..3].
//
// Where RobStride's own documents disagree, this accepts both readings:
//   - the failure code is 0x0F in the PDF and the Qt sample, 0xF0 in OTA.py
//     -> any non-zero result is a failure;
//   - the data-frame reply type is 11 in the PDF table, 13 in OTA.py
//     -> either is accepted while sending data.
//
// MEASURED on 0086s (RS00, motor 0.0.3.22, follower firmware 1.2.8):
//   - under MIT the motor does not answer type 0 at all (3/3 NoReply, 200 ms),
//     under PRIVATE it answers in 2 ms. So the motor must be on the PRIVATE
//     protocol, which costs a 24 V power cycle each way. And a gripper OTA
//     switches the motor back to MIT, so do motor OTA after it, not before;
//   - one relayed round trip is 2.0 ms (p95 2.03 ms) host to host, so a 93 KB
//     image (11,686 packs) takes ~23 s.
//
// MEASURED, first real flash (0086s, RS00, 0.0.3.22 -> 0.0.3.32, 2026-09-25):
//   - start answered only on the third attempt, ~4 s in -- the motor restarts
//     into its bootloader before replying (see Options::start_attempts);
//   - 11,686 data packs, 0 resumes, end acknowledged, 27.6 s in total;
//   - after end the motor restarted on the MIT protocol, not the private one it
//     was flashed from, and the follower MCU followed it there without a power
//     cycle. So a version check right after the update cannot work (the
//     version frame needs private); it takes another protocol switch. After a
//     power cycle the follower's auto-calibration completed normally.
//   One unit, one run: the MIT-after-update behaviour is not yet known to hold
//   for other models or versions.
//
// THE PROTOCOL DOES NOT CHECK THE MOTOR MODEL. The image carries no model
// field and the motor accepts whatever it is sent, so an RS00 image goes into
// an EL05 without complaint. Check the model before calling update_*() --
// preflight() does it when the session was built on a Motor.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace xense::taccap {

class Motor;

class MotorOtaSession {
public:
    // One relayed CAN transfer: send (ext_id, data), return the first reply
    // matching (id & mask) == value within timeout_ms. Motor::can_ext_xfer has
    // this shape; tests substitute a simulated bootloader.
    using Xfer = std::function<protocol::MotorCanXferResp(
        uint32_t ext_id, const std::vector<uint8_t>& data, uint16_t timeout_ms,
        uint32_t match_mask, uint32_t match_value)>;

    // Called after each acknowledged data frame and once more at the end.
    using ProgressCallback = std::function<void(uint32_t packs_done, uint32_t packs_total)>;

    struct Options {
        uint8_t  host_id = 0xFD;              // matches the gripper firmware's master id
        uint16_t handshake_timeout_ms = 2000; // uid / start / info / end, per attempt
        uint16_t data_timeout_ms = 500;       // one data frame, per attempt
        int      max_attempts = 3;            // per frame, on no reply
        // The start frame gets its own budget: the motor answers it only after
        // restarting into its bootloader, measured at ~4 s (0086s, RS00,
        // 0.0.3.22 -> 0.0.3.32: attempts 1 and 2 at 2 s each got nothing, the
        // third landed). Three attempts left 2 s of margin; six leaves 8 s.
        int      start_attempts = 6;
        int      max_resumes = 64;            // motor-requested rewinds, whole update
    };

    // Largest image the protocol can address: pack indices are 16 bits wide.
    static constexpr uint32_t kMaxImageSize = 0x80000;

    MotorOtaSession(Motor& motor, uint8_t motor_can_id, Options opts);
    MotorOtaSession(Motor& motor, uint8_t motor_can_id);
    MotorOtaSession(Xfer xfer, uint8_t motor_can_id, Options opts);
    MotorOtaSession(Xfer xfer, uint8_t motor_can_id);

    // Refuse to proceed unless the motor is on the PRIVATE protocol and the
    // gripper's recorded motor model is `expected_model` ("RS00", "EL05").
    // Only possible on a session built on a Motor; throws otherwise.
    void preflight(const std::string& expected_model);

    // Communication type 0. Read-only; the UID is what `start` must quote.
    std::array<uint8_t, 8> read_uid();

    // The whole sequence: uid -> start -> info -> data x N -> end. Throws
    // TimeoutError when the motor stops answering, ProtocolError when it
    // reports a failure it cannot recover from. The motor restarts after end.
    void update_from_bytes(const std::vector<uint8_t>& image,
                           ProgressCallback on_progress = {});
    void update_from_file(const std::string& path,
                          ProgressCallback on_progress = {});

private:
    struct Reply {
        uint8_t mode;
        uint8_t result;
        protocol::MotorCanXferResp raw;
    };

    uint32_t make_id(uint8_t mode, uint16_t data16) const;
    Reply exchange(uint8_t mode, uint16_t data16, const std::vector<uint8_t>& payload,
                   uint16_t timeout_ms, uint32_t mask, uint32_t value, const char* what);
    Reply handshake(uint8_t mode, uint16_t data16, const std::vector<uint8_t>& payload,
                    const char* what);

    Xfer     xfer_;
    Motor*   motor_ = nullptr;
    uint8_t  can_id_;
    Options  opts_;
};

}  // namespace xense::taccap

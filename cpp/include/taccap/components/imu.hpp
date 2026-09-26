// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// IMU component: typed wrapper around Cmd::GetImu (one-shot read) and
// FrameType::DATA frames carrying ImuData (continuous stream after
// LeaderGripper::start_streaming()).
//
// Unit conversions applied here so callers see SI-friendly values (m/s²,
// rad/s, µT, °C). Raw fixed-point fields stay accessible via raw().

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

struct ImuSample {
    // Wall-clock(-ish) host receive time. Useful for cross-stream alignment
    // since the MCU's `timestamp_us` is relative to its own boot.
    std::chrono::steady_clock::time_point host_time;

    uint32_t mcu_timestamp_us;   // MCU-side timestamp (us since MCU boot)
    uint16_t valid_flag;         // protocol::ImuValid::* bits
    uint16_t seq;                // MCU-side packet sequence

    // Converted physical values.
    std::array<float, 3> accel_mps2;   // m/s²    (raw mg → mg * 0.001 * 9.80665)
    std::array<float, 3> gyro_radps;   // rad/s   (raw 0.01 dps → * 0.01 * π/180)
    std::array<float, 3> mag_uT;       // µT      (raw 0.01 µT → * 0.01)
    float                temperature_c;// °C      (raw 0.01 °C → * 0.01)

    // Original packed payload (28 bytes), kept for users who want to bypass
    // the unit conversion or feed it back into a serializer.
    protocol::ImuData raw;
};

class IMU {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const ImuSample&)>;

    explicit IMU(bus::Transport& transport);

    // Synchronous one-shot read (sends CMD_NEED_ACK GetImu, waits for ACK).
    // Throws on timeout / NACK / size mismatch.
    ImuSample read_once(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Subscribe to streaming DATA frames. The Transport keeps streaming
    // running independently — see LeaderGripper::start_streaming().
    SubId on_data(Callback cb);
    void  off(SubId id);

    // V1.4 — write magnetometer hard-iron offsets + soft-iron correction
    // matrix to firmware (Cmd::SetImuMagCal, 0x26). `hard_iron` is the
    // 3-axis bias in µT; `soft_iron` is the 3×3 correction matrix in
    // ROW-major order (same layout the firmware stores after MotionCal).
    // Throws on NACK / timeout. Wire layout pinned by tests.
    void set_mag_calibration(const std::array<float, 3>& hard_iron,
                             const std::array<float, 9>& soft_iron_row_major,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Magnetometer calibration session (Cmd 0x26 with a 1-byte payload; leader
    // firmware only -- a follower NACKs InvalidCmd).
    //
    // start: the firmware flags calibration and STARTS AN IMU STREAM ON ITS OWN
    //   (separate mode, IMU only, rate code 5) so the host can collect raw
    //   magnetometer samples through on_data() while the gripper is rotated.
    //   It does not go through start_streaming(), so the gripper's own
    //   streaming state does not know about it.
    // stop:  stops that stream and clears the flag, discarding the session.
    //
    // Finish a session by fitting hard/soft iron on the host and calling
    // set_mag_calibration(): the firmware applies it, persists it, stops the
    // stream and clears the flag itself -- no stop needed after that.
    void start_mag_calibration(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});
    void stop_mag_calibration(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Decode a wire-format ImuData payload into an ImuSample. Public so
    // tests (and curious users) can exercise the unit conversion.
    static ImuSample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap

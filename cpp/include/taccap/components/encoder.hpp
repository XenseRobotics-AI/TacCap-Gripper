// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Encoder component: typed wrapper around Cmd::GetEncoder (one-shot) and
// FrameType::DATA frames carrying EncoderData (continuous stream).
//
// On the leader gripper this reads the dedicated encoder hardware. On the
// follower gripper the same telemetry is also available through MotorStatus
// (handled separately by the Motor component).
//
// Optional normalization: install a GripperPosition map (see
// set_position_map()) and every sample — one-shot and streamed — additionally
// carries EncoderSample::position, the opening normalized to [0, 1] with
// 0 = fully closed and 1 = fully open. position_rad keeps its meaning either
// way; position is NaN while no map is installed. LeaderGripper wires this up
// automatically when constructed with Config::normalize_position.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/gripper_position.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>

namespace xense::taccap {

struct EncoderSample {
    std::chrono::steady_clock::time_point host_time;
    uint32_t mcu_timestamp_us;
    uint16_t status;         // protocol::EncoderStatusBit::* bits (Ok/Error/Overflow)
    uint16_t seq;
    float    position_rad;
    float    velocity_rad_s;

    // Opening normalized to [0, 1] — 0 = fully closed, 1 = fully open.
    // Filled only while the Encoder has a position map installed
    // (Encoder::set_position_map); NaN otherwise. position_rad above always
    // stays in radians regardless.
    float    position;

    protocol::EncoderData raw;
};

class Encoder {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const EncoderSample&)>;

    explicit Encoder(bus::Transport& transport);

    EncoderSample read_once(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Latch the gripper's *current* encoder reading as the new zero
    // position. Firmware snapshots the raw count it sees the instant
    // it processes the command, so the gripper MUST already be held at
    // the desired zero pose (e.g. fully closed) before the call.
    // Throws ProtocolError on NACK or transport timeout.
    void set_zero(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    SubId on_data(Callback cb);
    void  off(SubId id);

    // ---- Normalized position (0 = closed, 1 = open) ------------------------
    // Install / remove the converter used to fill EncoderSample::position.
    // Takes effect for read_once() and for every on_data() subscriber, so a
    // running stream switches over without re-subscribing. Thread-safe: the
    // transport's dispatcher thread may be normalizing samples concurrently.
    //
    // A map that isn't valid() is rejected (throws ProtocolError) — installing
    // an uncalibrated converter would silently report 0 forever.
    void set_position_map(const GripperPosition& map);
    void clear_position_map();
    bool has_position_map() const;
    // Copy of the installed map; !valid() when none is installed.
    GripperPosition position_map() const;

    static EncoderSample decode(const std::uint8_t* payload, std::size_t len);

private:
    // Post-process a freshly decoded sample: clamp the user-facing
    // position_rad to >= 0 (calibration drift / mechanical slop can
    // make "fully closed" report slightly negative), emit a rate-
    // limited warning if the underlying drift exceeds a threshold, and
    // fill `position` when a position map is installed.
    // `raw.position_rad` is left untouched so callers that want the
    // firmware-side value still have it.
    void normalize(EncoderSample& s) const;

    // Clamp position_rad to >= 0, warning (rate-limited) past the drift
    // threshold. Split out of normalize() so the clamp happens before the
    // normalized value is derived from it.
    void clamp_negative_(EncoderSample& s) const;

    bus::Transport& t_;
    // Guards pos_map_. Held only for a struct copy, so the ~100-500 Hz
    // decode path never contends meaningfully with a set/clear.
    mutable std::mutex pos_map_mu_;
    GripperPosition    pos_map_;   // !valid() == no normalization
    // Last warn timestamp (steady_clock ns since epoch) for the
    // "encoder reading too negative" message. Atomic so the on_data
    // worker thread and a concurrent read_once() caller don't both
    // fire warnings within the same throttle window.
    mutable std::atomic<int64_t> last_neg_warn_ns_{0};
};

}  // namespace xense::taccap

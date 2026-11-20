// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Calibration component: host-side access to the calibration parameters the
// firmware persists in its internal flash.
//
//   - Fisheye camera intrinsics + distortion  (Cmd::CameraFisheyeCal 0x2B,
//     command set >= V2.0) — supported on BOTH leader and follower.
//     Command set V2.10 (firmware 1.3.0-rc1 / 1.3.1, both roles) also records the
//     capture resolution the params belong to (read_fisheye_record /
//     write_fisheye_sized); older firmware leaves it unknown.
//   - Leader encoder max travel angle          (Cmd::EncoderMaxCal 0x2C,
//     command set >= V2.1, i.e. leader >= 1.2.0) — LEADER ONLY; the follower
//     has no MT6816 encoder and NACKs with ErrorCode::InvalidCmd.
//
// The firmware is a dumb store for both records: it persists the bytes, does
// no unit conversion and no range clamping, and rejects only NaN/Inf (plus
// max_rad <= 0). Interpreting the values is the host's job — this SDK uses
// encoder max travel to build a LeaderGripper's normalized-position map (see
// taccap/gripper_position.hpp), and hands the fisheye parameters straight to
// the caller's undistortion pipeline.
//
// Reading a parameter that was never written NACKs with ErrorCode::CalNotSet
// instead of returning zeros, so "never calibrated" is distinguishable from
// "calibrated to exactly 0". The read methods surface that as an empty
// std::optional — it is an expected state, not an error. Every other NACK
// still throws ProtocolError.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <chrono>
#include <optional>

namespace xense::taccap {

class Calibration {
public:
    explicit Calibration(bus::Transport& transport);

    // ---- Fisheye camera (leader + follower) --------------------------------
    // Returns std::nullopt when the firmware has never been given fisheye
    // parameters (ErrorCode::CalNotSet).
    std::optional<protocol::CameraFisheyeCal> read_fisheye(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Ask for this unit's wrist calibration, falling back to the SDK's
    // reference values when it cannot supply one.
    //
    // Prefer this over read_fisheye() unless you specifically need to know what
    // the flash holds. read_fisheye() reports the firmware's answer — including
    // the all-zero record an uncalibrated unit returns instead of a NACK, which
    // remaps every frame to black if handed straight to FisheyeUndistorter.
    // This applies the policy once, so callers that own the wrist UVC device
    // themselves get the same decision the SDK makes internally rather than
    // re-deriving it.
    //
    // The calibration resolution comes from read_fisheye_record(). A record
    // without one (written by write_fisheye() or by pre-V2.10 firmware) gets
    // infer_fisheye_cal_size()'s answer, with size_inferred set and a warning
    // logged; a record whose size cannot be inferred either is treated as
    // unusable and the reference calibration (640x480) stands in.
    //
    // Never throws for a missing or unusable calibration — that is a fallback,
    // reported through ResolvedFisheyeCal::reason. A transport failure still
    // propagates.
    ResolvedFisheyeCal resolve_fisheye(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(200));

    // Persists to MCU flash (survives power cycles). Throws ProtocolError if
    // any value is NaN/Inf (firmware returns InvalidParam) or the flash write
    // fails (SysBusy).
    //
    // Writes the 8 params only (CalOp::Write). On V2.10 firmware this marks the
    // calibration resolution UNKNOWN; use write_fisheye_sized() to record it.
    void write_fisheye(const protocol::CameraFisheyeCal& cal,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ---- Fisheye calibration resolution (V2.10: firmware 1.3.0-rc1 / 1.3.1)
    // True when the firmware stores the calibration resolution (answers the
    // sized read, op 0x02, with Ok or CalNotSet); false when it rejects that op
    // as InvalidParam, which is how older firmware answers an unknown op. The
    // answer is cached for the life of this object; a transport failure
    // propagates and is not cached, and so is any other NACK.
    bool supports_fisheye_size(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // The params plus the resolution they belong to. Uses the sized read when
    // the firmware supports it, else the legacy read with width/height 0/0
    // (unknown). Returns std::nullopt when never calibrated (CalNotSet).
    std::optional<FisheyeCalRecord> read_fisheye_record(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Persist the params together with the capture resolution they were
    // measured at. Throws Error before sending anything when the values fail
    // fisheye_cal_size_plausible() (the firmware's own rule). On V2.10 firmware
    // this is the sized write (op 0x03); a NACK throws ProtocolError, as
    // write_fisheye() does. Older firmware cannot record a size: a 640x480
    // calibration then falls back to the legacy write_fisheye(), and any other
    // resolution throws Error asking for a firmware upgrade.
    void write_fisheye_sized(const protocol::CameraFisheyeCal& cal,
                             int width, int height,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ---- Leader encoder max travel angle (leader only) ---------------------
    // The shaft angle (rad) at which the gripper is fully open, measured from
    // the encoder zero (fully closed). Returns std::nullopt when never
    // calibrated. Throws ProtocolError(InvalidCmd) on a follower.
    std::optional<float> read_encoder_max_rad(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Firmware rejects NaN/Inf and max_rad <= 0 with InvalidParam.
    void write_encoder_max_rad(float max_rad,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

private:
    // Sends the sized read (op 0x02). Returns false when the firmware rejects
    // the op as InvalidParam (pre-V2.10); otherwise fills `out`, with
    // std::nullopt for CalNotSet. Any other NACK throws ProtocolError.
    bool read_fisheye_sized_(std::chrono::milliseconds timeout,
                             std::optional<FisheyeCalRecord>& out);

    // Records whether the firmware answered the sized read: -1 not yet asked,
    // 0 unsupported (pre-V2.10), 1 supported. Set only from a real answer.
    void note_fisheye_size_support_(bool supported) noexcept {
        fisheye_size_support_.store(supported ? 1 : 0);
    }

    bus::Transport&  t_;
    std::atomic<int> fisheye_size_support_{-1};
};

}  // namespace xense::taccap

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
//     Command set V2.11 (firmware 1.3.0-rc4 / 1.3.1) keeps one record per
//     capture resolution -- a VGA (640x480) and an HD (1920x1080) slot
//     (read_fisheye_slot / resolve_fisheye_for).
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
#include <string>

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

    // resolve_fisheye() for a camera running at width x height: this unit's
    // calibration FOR THAT RESOLUTION, or the reference values with a reason.
    //
    // On V2.11 firmware with a slot size (640x480 / 1920x1080) this reads that
    // slot (read_fisheye_slot), so a 1080p camera gets the HD calibration.
    // Otherwise it reads the single record (read_fisheye_record), resolves its
    // size exactly as resolve_fisheye() does (recorded, else inferred), and
    // uses it only when that size equals width x height.
    //
    // When nothing matches, the reference calibration stands in, reported
    // through `reason` as resolve_fisheye() does. When that is only because
    // this unit's own usable calibration belongs to another resolution, that
    // resolution is in ResolvedFisheyeCal::mismatched_size. The reference exists at
    // 640x480 ONLY: for any other requested size its calibrated_size still
    // reads 640x480, so a FisheyeUndistorter built for the camera refuses it
    // rather than rectifying with wrong intrinsics.
    //
    // Never throws for a missing or mismatched calibration; a transport failure
    // propagates.
    ResolvedFisheyeCal resolve_fisheye_for(
        int width, int height,
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
    // fisheye_cal_size_plausible() (the V2.10 firmware rule) or, for 640x480 and
    // 1920x1080, fisheye_cal_slot_plausible() (V2.11's tighter rule: principal
    // point within 15% of the frame centre) -- the latter whatever the firmware
    // generation. On V2.10/V2.11 firmware this is the sized write (op 0x03); a
    // NACK throws ProtocolError, as write_fisheye() does. On V2.11 the write
    // lands in the slot for its size and leaves the other slot untouched; any
    // size other than 640x480 / 1920x1080 throws Error there (V2.11 has no
    // record for it). Older firmware cannot record a size: a 640x480
    // calibration then falls back to the legacy write_fisheye(), and any other
    // resolution throws Error asking for a firmware upgrade.
    void write_fisheye_sized(const protocol::CameraFisheyeCal& cal,
                             int width, int height,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ---- Dual calibration slots (V2.11: firmware 1.3.0-rc4 / 1.3.1) --------
    // True when the firmware keeps a separate calibration per resolution (a VGA
    // and an HD slot). Probes with the slot read (op 0x04) at 640x480: firmware
    // without slots rejects it as an unknown op (InvalidParam; LengthMismatch is
    // treated the same, since V2.11 never answers it to a well-formed request),
    // V2.11 answers with the record or CalNotSet. Firmware already known not to
    // record a size (supports_fisheye_size() == false) is answered without a
    // round trip. Cached like supports_fisheye_size(); a transport failure or
    // any other NACK propagates and is not cached.
    bool supports_fisheye_slots(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // The calibration for one capture resolution. width x height must be
    // 640x480 or 1920x1080 -- anything else throws Error before sending.
    // Returns std::nullopt when there is no calibration for that resolution.
    //
    //   V2.11 : the slot read (op 0x04). The VGA slot may come back with
    //           width/height 0/0 when its params were written without a size
    //           (write_fisheye(), or older firmware before an upgrade); the HD
    //           slot always carries 1920x1080.
    //   V2.10 : one record. Returned when its recorded size equals the request;
    //           a record with no size counts as the VGA slot (see below).
    //   older : the legacy record, returned (with size 0/0) for 640x480 only.
    //
    // "No size means VGA" mirrors V2.11 itself, where ops 0x00/0x01 -- the only
    // way to write params without a size -- address the VGA slot, and op 0x02
    // reports that slot with 0/0. So the same flash content answers the same
    // way across generations. The 0/0 in the result keeps the size visibly
    // unconfirmed; resolve_fisheye_for() checks it against the principal point
    // before trusting it.
    std::optional<FisheyeCalRecord> read_fisheye_slot(
        int width, int height,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Erase the calibration for one capture resolution (V2.11 only); the slot
    // then reads as never calibrated. width x height must be 640x480 or
    // 1920x1080. Clearing VGA also drops its size record; clearing an empty
    // slot succeeds; the other slot is never touched. Throws Error before
    // sending anything on a size with no slot, or on firmware without slots
    // (supports_fisheye_slots() == false) -- there is no single-record
    // equivalent. A firmware NACK throws ProtocolError.
    void clear_fisheye_slot(int width, int height,
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

    // Sends the slot read (op 0x04). Returns false when the firmware rejects
    // the op (no slots); otherwise fills `out`, std::nullopt for CalNotSet.
    bool read_fisheye_slot_(int width, int height, std::chrono::milliseconds timeout,
                            std::optional<FisheyeCalRecord>& out);

    // The shared tail of resolve_fisheye() / resolve_fisheye_for(): turn what
    // was read (or why the read failed) into a ResolvedFisheyeCal.
    static ResolvedFisheyeCal resolve_record_(const std::optional<FisheyeCalRecord>& record,
                                              std::string reason);

    // Records whether the firmware answered the sized read: -1 not yet asked,
    // 0 unsupported (pre-V2.10), 1 supported. Set only from a real answer.
    void note_fisheye_size_support_(bool supported) noexcept {
        fisheye_size_support_.store(supported ? 1 : 0);
    }

    bus::Transport&  t_;
    std::atomic<int> fisheye_size_support_{-1};
    // Same encoding for the V2.11 slot read (op 0x04).
    std::atomic<int> fisheye_slot_support_{-1};
};

}  // namespace xense::taccap

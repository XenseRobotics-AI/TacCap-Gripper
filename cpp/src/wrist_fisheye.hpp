// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Private helper shared by leader_gripper.cpp and follower_gripper.cpp — NOT
// part of the installed public headers.
//
// Both grippers wire the wrist camera's fisheye undistortion identically:
// read the intrinsics the firmware persisted, build the remap tables, hand
// them to the Camera. The interesting part is the failure policy, and having
// one copy of it keeps leader and follower from drifting apart on what counts
// as a warning versus an error.

#pragma once

#include <taccap/components/calibration.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>

#include <memory>
#include <string>

namespace xense::taccap::detail {

// The wrist undistortion's calibration decision for a camera at `camera`,
// split out of install_wrist_undistorter() so it is testable without a camera.
//
//   - this unit has a usable calibration for the camera's size (V2.11 slot, or
//     a single record whose recorded / inferred size matches) -> use it;
//   - this unit has NO usable calibration at any resolution -> the reference,
//     as resolve_fisheye_for() returns it (only usable on a 640x480 camera;
//     for any other size the FisheyeUndistorter refuses it);
//   - this unit HAS a usable calibration, but only for another resolution ->
//     throws Error naming both sizes. Falling back to the reference there would
//     silently trade the unit's own calibration for an approximate one, which
//     is exactly what the dual slots (tc-gu-01 #34) exist to remove.
//
// resolve_fisheye_for() itself stays informational (it returns the reference
// with a "belongs to AxB" reason); the stricter rule lives here. The extra
// reads happen only on the fallback path.
inline ResolvedFisheyeCal resolve_wrist_fisheye(Calibration& cal, cv::Size camera,
                                                const char* who) {
    auto resolved = cal.resolve_fisheye_for(camera.width, camera.height);
    if (!resolved.is_reference) return resolved;

    // Where does this unit's own calibration live, if anywhere? Either the
    // record just read belongs to another size (e.g. a 1920x1080 V2.10 record
    // left in the V2.11 VGA slot, which reports 0/0 and infers as 1080p), or
    // another slot holds one.
    cv::Size other    = resolved.mismatched_size;
    bool     inferred = false;
    if (other.empty()) {
        const cv::Size slots[] = {
            {FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT},
            {FISHEYE_HD_CALIB_WIDTH, FISHEYE_HD_CALIB_HEIGHT},
        };
        for (const auto& s : slots) {
            if (s == camera) continue;
            const auto there = cal.resolve_fisheye_for(s.width, s.height);
            if (there.is_reference) continue;
            other    = s;
            inferred = there.size_inferred;
            break;
        }
    }
    if (!other.empty()) {
        const std::string want = std::to_string(camera.width) + "x" +
                                 std::to_string(camera.height);
        const std::string have = std::to_string(other.width) + "x" +
                                 std::to_string(other.height);
        throw Error(std::string(who) + ": the wrist camera runs at " + want +
                    ", but this unit's fisheye calibration is for " + have +
                    (inferred ? " (inferred)" : "") +
                    " only. Intrinsics do not carry over to another resolution "
                    "or aspect ratio (640x480 and 1920x1080 are different sensor "
                    "crops), and the SDK reference would silently replace this "
                    "unit's own calibration — calibrate this unit at " + want +
                    ", or run the camera at " + have + ".");
    }
    return resolved;
}

// Installs fisheye undistortion on `cam` from resolve_wrist_fisheye(): this
// unit's own calibration for the camera's configured size, or
// FISHEYE_FALLBACK_CAL with a warning when the unit has none at all.
//
// Falls back with a warning (every unit shares the lens, so the reference
// numbers beat raw fisheye; see FISHEYE_FALLBACK_CAL for what that costs):
//   - firmware has no fisheye record yet     -> read_fisheye() == nullopt
//   - firmware answers with an empty record  -> !is_usable_fisheye_cal()
//   - firmware predates command set V2.0     -> ProtocolError(InvalidCmd)
//
// Propagates (a config bug the caller must fix, not something to paper over):
//   - the unit has a usable calibration, but only for another resolution than
//     the camera runs at (e.g. a 1920x1080 camera with only a VGA calibration,
//     or a single V2.10 record sized for the other resolution);
//   - the camera runs at a resolution with no calibration AND there is no
//     reference for it either (references exist at 640x480 and 1920x1080
//     only; FISHEYE_FALLBACK_CAL / FISHEYE_FALLBACK_CAL_HD). Any rectification
//     would be quietly wrong; intrinsics are never rescaled to fit.
inline void install_wrist_undistorter(Calibration& cal,
                                      Camera&      cam,
                                      float        balance,
                                      const char*  who) {
    // The read-and-fall-back policy lives in Calibration::resolve_fisheye_for,
    // so this path and callers that own the wrist UVC device themselves make the
    // same decision. Resolved for the camera's configured size, so a 1080p
    // camera gets the HD slot on V2.11 firmware.
    const cv::Size size{cam.config().width, cam.config().height};
    const auto resolved = resolve_wrist_fisheye(cal, size, who);
    if (resolved.is_reference) {
        logger()->warn("{}: wrist fisheye undistortion is using the SDK's REFERENCE "
                       "intrinsics because {}. Rectification will be approximate — "
                       "lens placement varies per assembly, so the principal point "
                       "drifts. Run python/examples/fisheye_cal.py set-fisheye to "
                       "store this unit's own calibration.", who, resolved.reason);
    }

    auto u = std::make_shared<const FisheyeUndistorter>(
        resolved.calibration, resolved.calibrated_size, size, balance);
    cam.set_undistorter(u);
    logger()->info("{}: wrist fisheye undistortion on ({}x{}, balance={:.2f}, "
                   "focal_scale={:.3f}, calibration={}, calibrated at {}x{} ({}))",
                   who, size.width, size.height, u->balance(), u->focal_scale(),
                   resolved.is_reference ? "SDK reference" : "this unit",
                   resolved.calibrated_size.width, resolved.calibrated_size.height,
                   resolved.is_reference    ? "reference"
                   : resolved.size_inferred ? "inferred"
                                            : "recorded");
}

}  // namespace xense::taccap::detail

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

namespace xense::taccap::detail {

// Installs fisheye undistortion on `cam`, always — from this unit's own
// calibration when the firmware holds one, and from FISHEYE_FALLBACK_CAL with a
// warning when it does not.
//
// Falls back with a warning (every unit shares the lens, so the reference
// numbers beat raw fisheye; see FISHEYE_FALLBACK_CAL for what that costs):
//   - firmware has no fisheye record yet     -> read_fisheye() == nullopt
//   - firmware answers with an empty record  -> !is_usable_fisheye_cal()
//   - firmware predates command set V2.0     -> ProtocolError(InvalidCmd)
//   - the calibration stored for this unit belongs to another resolution
//     than the camera runs at (resolve_fisheye_for)
//
// Propagates (a config bug the caller must fix, not something to paper over):
//   - the camera runs at a resolution with no calibration AND the reference
//     does not apply either (it exists at 640x480 only) -- e.g. a 1920x1080
//     camera on a unit with no HD calibration. Any rectification would be
//     quietly wrong; intrinsics are never rescaled to fit.
inline void install_wrist_undistorter(Calibration& cal,
                                      Camera&      cam,
                                      float        balance,
                                      const char*  who) {
    // The read-and-fall-back policy lives in Calibration::resolve_fisheye, so
    // this path and callers that own the wrist UVC device themselves make the
    // same decision. It used to be spelled out here, and a second copy grew in a
    // downstream consumer — with the two already disagreeing about why a read
    // fails.
    //
    // Resolved for the camera's configured size, so a 1080p camera gets the HD
    // slot on V2.11 firmware, and a calibration stored for the other resolution
    // falls back to the reference with a warning instead of being refused.
    const cv::Size size{cam.config().width, cam.config().height};
    const auto resolved = cal.resolve_fisheye_for(size.width, size.height);
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

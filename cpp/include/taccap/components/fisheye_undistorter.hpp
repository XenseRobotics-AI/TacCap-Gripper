// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Fisheye undistortion for the wrist camera, driven by the intrinsics the
// firmware persists in flash (Cmd::CameraFisheyeCal 0x2B, command set >= V2.0;
// read them with Calibration::read_fisheye()).
//
// This class is deliberately protocol-free: it takes a plain
// protocol::CameraFisheyeCal value and knows nothing about transports or
// grippers, so it is equally usable on frames that never came from this SDK
// (e.g. an external camera service that owns the wrist UVC device — which is
// the common case, since Gripper::Config::open_cameras defaults to false).
//
// The remap tables are built once in the constructor and reused for every
// frame; rebuilding them per frame would dominate the cost.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace xense::taccap {

// The 8 intrinsic/distortion floats only hold at the capture resolution they
// were measured at. 640x480 (4:3) and 1920x1080 (16:9) are DIFFERENT sensor
// crops, not one image at two scales, so intrinsics cannot be rescaled between
// them — nor to any other size. Firmware V2.10 (1.3.0-rc1 / 1.3.1)
// therefore records the calibration resolution next to the params
// (Calibration::read_fisheye_record / write_fisheye_sized), and the undistorter
// refuses any frame that is not at that exact resolution.
//
// 640x480 remains the resolution of the SDK's reference calibration and of the
// legacy constructor below, which predates the recorded size.
inline constexpr int FISHEYE_CALIB_WIDTH  = 640;
inline constexpr int FISHEYE_CALIB_HEIGHT = 480;

// A fisheye record as the firmware holds it: the params plus the capture
// resolution they belong to. width/height 0/0 means the size is UNKNOWN — the
// record was written without one (CalOp::Write, or firmware older than V2.10)
// — and must not be assumed to be 640x480; see infer_fisheye_cal_size().
struct FisheyeCalRecord {
    protocol::CameraFisheyeCal calibration{};
    uint16_t                   width  = 0;
    uint16_t                   height = 0;

    bool size_known() const noexcept { return width != 0 && height != 0; }
};

// The plausibility rule the firmware applies to a write-sized (op 0x03), so a
// write can be refused host-side with a useful message rather than a bare
// InvalidParam: every value finite, fx > 0, fy > 0, 1 <= width, height <= 4096,
// and the principal point strictly inside the frame (0 < cx < width,
// 0 < cy < height).
bool fisheye_cal_size_plausible(const protocol::CameraFisheyeCal& cal,
                                int width, int height);

// Best-effort calibration resolution for a record whose size is unknown.
//
// The rule, shared verbatim with the PC calibration tools so every consumer
// infers the same answer:
//   - candidates are exactly 640x480 and 1920x1080;
//   - a candidate W x H matches when the calibration is usable
//     (is_usable_fisheye_cal) AND |cx - W/2| <= 0.10 * W AND
//     |cy - H/2| <= 0.10 * H, i.e. the principal point sits within 10% of the
//     frame centre on each axis;
//   - the size is returned only when EXACTLY one candidate matches; zero or
//     several matches return std::nullopt.
// A principal point far from both centres is not "probably 640x480": it means
// the record cannot be placed, and the caller should ask for a recalibration.
std::optional<cv::Size> infer_fisheye_cal_size(const protocol::CameraFisheyeCal& cal);

// `balance` interpolates the output focal length between these two, matching
// the PC tool's fisheye_focal_scale_for_balance()
// (tc-gu-01-pc/camera_calibration/calibration/workflow.py). balance = 0 keeps
// the calibrated focal length — a natural, reference-like view; balance = 1
// shortens it to 0.70x for the widest field of view, at the cost of more
// black border around the edges.
// A reference calibration for the TC-GU-01 wrist lens, measured on a sample
// unit at 640x480. It exists so a gripper whose firmware was never calibrated
// can still deliver an approximately rectified stream instead of raw fisheye —
// every unit carries the same lens and sensor, so the shared numbers are far
// closer to correct than no rectification at all. They hold at 640x480 ONLY.
//
// It is NOT a substitute for calibrating a unit. Lens placement varies between
// assemblies, so the principal point in particular drifts per unit; anything
// that measures in pixels off a rectified frame should be calibrated properly.
// Every path that falls back to this warns, and says how to store a real one.
inline constexpr protocol::CameraFisheyeCal FISHEYE_FALLBACK_CAL{
    /* fx */ 213.0303f, /* fy */ 212.7928f,
    /* cx */ 321.4000f, /* cy */ 239.9500f,
    /* k1 */ -0.0172f,  /* k2 */ 0.0091f,
    /* k3 */ -0.0146f,  /* k4 */ 0.0051f,
};

// A record read back from flash can be present but empty: an uncalibrated unit
// answers with all-zero params rather than a NACK, and building remap tables
// from fx = fy = 0 maps every pixel outside the source image, so the "rectified"
// frame comes out uniformly black. Observed on firmware 1.1.1. Treat such a
// record as absent.
// The outcome of asking a gripper for its wrist calibration: what to rectify
// with, and — when the unit could not supply one — why the reference values are
// standing in. `reason` is empty exactly when `is_reference` is false.
//
// This exists so the policy has one home. It is applied on both sides of the
// same decision: install_wrist_undistorter() uses it when the SDK owns the wrist
// UVC device, and callers that own the device themselves (the common case, since
// Config::open_cameras defaults to false) get the identical answer instead of
// re-deriving it and drifting.
//
// `calibrated_size` is the capture resolution `calibration` belongs to: the
// size the firmware recorded, or — for a record written without one —
// infer_fisheye_cal_size()'s answer, flagged by `size_inferred`. The reference
// calibration is always 640x480 with size_inferred false.
struct ResolvedFisheyeCal {
    protocol::CameraFisheyeCal calibration;
    bool                       is_reference;
    std::string                reason;
    cv::Size                   calibrated_size{FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT};
    bool                       size_inferred = false;
};

inline bool is_usable_fisheye_cal(const protocol::CameraFisheyeCal& c) {
    return std::isfinite(c.fx) && std::isfinite(c.fy) && c.fx > 0.0f && c.fy > 0.0f;
}

inline constexpr double FISHEYE_NATURAL_FOCAL_SCALE = 1.00;
inline constexpr double FISHEYE_WIDE_FOCAL_SCALE    = 0.70;

class FisheyeUndistorter {
public:
    // `calibrated_size` is the resolution `cal` was measured at (from
    // FisheyeCalRecord / ResolvedFisheyeCal::calibrated_size); `frame_size` is
    // what the camera actually delivers. They must be EQUAL — throws Error
    // otherwise, because intrinsics do not carry over to another resolution or
    // aspect ratio; recalibrate at the capture resolution instead. `balance` is
    // clamped to [0,1] rather than rejected — it is a taste knob, not a
    // correctness one.
    FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                       cv::Size calibrated_size,
                       cv::Size frame_size,
                       float    balance = 0.0f);

    // Legacy form, from before the firmware recorded a calibration size: `cal`
    // is taken to be a 640x480 calibration, so this throws Error unless `size`
    // is 640x480. Prefer the form above with the record's own size.
    FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                       cv::Size size,
                       float    balance = 0.0f);

    // Rectify one frame. `src` must match the size this was built for;
    // mismatches throw Error rather than silently producing garbage.
    // Any cv::Mat type cv::remap accepts works — BGR8 in the SDK's own path.
    cv::Mat apply(const cv::Mat& src) const;

    cv::Size size()        const noexcept { return size_; }
    float    balance()     const noexcept { return balance_; }
    double   focal_scale() const noexcept { return focal_scale_; }

    // The camera matrix the rectified frames are expressed in — i.e. the
    // calibrated K with fx/fy scaled by focal_scale(). Callers that do their
    // own geometry on rectified pixels (projection, hand-eye) need THIS
    // matrix, not the raw one from the firmware.
    const cv::Mat& new_camera_matrix() const noexcept { return new_k_; }

private:
    cv::Size size_;
    float    balance_     = 0.0f;
    double   focal_scale_ = FISHEYE_NATURAL_FOCAL_SCALE;
    cv::Mat  new_k_;          // CV_64F 3x3
    cv::Mat  map_x_, map_y_;  // CV_32FC1, built once
};

}  // namespace xense::taccap

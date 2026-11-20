// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace xense::taccap {

namespace {

std::string size_text(int w, int h) {
    return std::to_string(w) + "x" + std::to_string(h);
}

// The two capture resolutions the wrist camera is calibrated at. Shared with
// the PC tools' inference rule — see infer_fisheye_cal_size() in the header.
constexpr int kInferCandidates[][2] = {{640, 480}, {1920, 1080}};
constexpr double kInferCentreTolerance = 0.10;
// V2.11 firmware's write-sized plausibility window (fisheye_cal_slot_plausible).
constexpr double kSlotCentreTolerance = 0.15;

}  // namespace

bool fisheye_cal_size_plausible(const protocol::CameraFisheyeCal& cal,
                                int width, int height) {
    const float v[] = {cal.fx, cal.fy, cal.cx, cal.cy,
                       cal.k1, cal.k2, cal.k3, cal.k4};
    for (float x : v) {
        if (!std::isfinite(x)) return false;
    }
    if (!(cal.fx > 0.0f) || !(cal.fy > 0.0f)) return false;
    if (width < 1 || width > 4096 || height < 1 || height > 4096) return false;
    return cal.cx > 0.0f && cal.cx < static_cast<float>(width) &&
           cal.cy > 0.0f && cal.cy < static_cast<float>(height);
}

bool fisheye_cal_slot_plausible(const protocol::CameraFisheyeCal& cal,
                                int width, int height) {
    if (!is_fisheye_cal_slot_size(width, height)) return false;
    // Finite params and fx, fy > 0; the frame bounds it also checks are implied
    // by the centre window below.
    if (!fisheye_cal_size_plausible(cal, width, height)) return false;
    // Inclusive, in double so the boundary values (e.g. cx = 224 at 640) pass.
    return std::abs(static_cast<double>(cal.cx) - width / 2.0) <=
               kSlotCentreTolerance * width &&
           std::abs(static_cast<double>(cal.cy) - height / 2.0) <=
               kSlotCentreTolerance * height;
}

std::optional<cv::Size> infer_fisheye_cal_size(const protocol::CameraFisheyeCal& cal) {
    if (!is_usable_fisheye_cal(cal)) return std::nullopt;
    std::optional<cv::Size> found;
    int matches = 0;
    for (const auto& c : kInferCandidates) {
        const double w = c[0], h = c[1];
        // NaN cx/cy compare false here, so they never match.
        if (std::abs(cal.cx - w / 2.0) <= kInferCentreTolerance * w &&
            std::abs(cal.cy - h / 2.0) <= kInferCentreTolerance * h) {
            found = cv::Size(c[0], c[1]);
            ++matches;
        }
    }
    if (matches != 1) return std::nullopt;
    return found;
}

FisheyeUndistorter::FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                                       cv::Size size,
                                       float    balance)
    : FisheyeUndistorter(cal, cv::Size(FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT),
                         size, balance) {}

FisheyeUndistorter::FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                                       cv::Size calibrated_size,
                                       cv::Size frame_size,
                                       float    balance)
    : size_(frame_size) {
    if (calibrated_size.width <= 0 || calibrated_size.height <= 0) {
        throw Error(
            "FisheyeUndistorter: the calibration resolution is unknown (" +
            size_text(calibrated_size.width, calibrated_size.height) +
            "). Intrinsics only hold at the resolution they were measured at — "
            "recalibrate at the capture resolution, or resolve the record with "
            "Calibration::resolve_fisheye().");
    }
    if (frame_size != calibrated_size) {
        throw Error(
            "FisheyeUndistorter: the wrist fisheye intrinsics belong to " +
            size_text(calibrated_size.width, calibrated_size.height) +
            ", but the frames are " +
            size_text(frame_size.width, frame_size.height) +
            ". Intrinsics must not be rescaled to another resolution or aspect "
            "ratio (640x480 and 1920x1080 are different sensor crops) — "
            "configure the camera at " +
            size_text(calibrated_size.width, calibrated_size.height) +
            ", or recalibrate at the capture resolution.");
    }

    balance_ = std::clamp(balance, 0.0f, 1.0f);
    focal_scale_ = FISHEYE_NATURAL_FOCAL_SCALE +
                   static_cast<double>(balance_) *
                       (FISHEYE_WIDE_FOCAL_SCALE - FISHEYE_NATURAL_FOCAL_SCALE);

    cv::Mat k = (cv::Mat_<double>(3, 3) <<
                 cal.fx, 0.0,    cal.cx,
                 0.0,    cal.fy, cal.cy,
                 0.0,    0.0,    1.0);
    cv::Mat d = (cv::Mat_<double>(4, 1) << cal.k1, cal.k2, cal.k3, cal.k4);

    // Only fx/fy move; the principal point stays put so the image centre does
    // not drift as `balance` changes.
    new_k_ = k.clone();
    new_k_.at<double>(0, 0) *= focal_scale_;
    new_k_.at<double>(1, 1) *= focal_scale_;

    cv::fisheye::initUndistortRectifyMap(
        k, d, cv::Mat::eye(3, 3, CV_64F), new_k_, size_, CV_32FC1,
        map_x_, map_y_);
}

cv::Mat FisheyeUndistorter::apply(const cv::Mat& src) const {
    if (src.empty()) {
        throw Error("FisheyeUndistorter::apply: empty frame");
    }
    if (src.cols != size_.width || src.rows != size_.height) {
        throw Error(
            "FisheyeUndistorter::apply: frame is " + std::to_string(src.cols) +
            "x" + std::to_string(src.rows) + " but the remap tables were built "
            "for " + std::to_string(size_.width) + "x" +
            std::to_string(size_.height));
    }
    cv::Mat dst;
    // INTER_CUBIC, not INTER_LINEAR. Rectifying an equidistant fisheye into a
    // pinhole view upsamples the periphery — with this lens and f_new = f_src
    // the edge is sampled at about 3.3x — and bilinear visibly softens an image
    // being magnified that much. Measured on a structured test frame: cubic
    // carries 1.36x the Laplacian variance of linear. Matches the PC calibration
    // tool (camera-calibration, fisheye_calib/rectify.py RectifyMap.apply).
    //
    // Argument order is (src, dst, map1, map2) — swapping dst and map1 still
    // compiles and fails at runtime with a -215 assertion inside remap.
    cv::remap(src, dst, map_x_, map_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    return dst;
}

}  // namespace xense::taccap

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// FisheyeUndistorter tests: remap-table construction, the balance knob and the
// size contract. No hardware and no transport — the class takes a plain
// CameraFisheyeCal value.
//
// The reference behaviour is the PC tool's create_undistort_maps()
// (tc-gu-01-pc/camera_calibration/calibration/workflow.py): fisheye
// initUndistortRectifyMap with an identity rectification and a new camera
// matrix whose fx/fy are scaled by focal_scale, cx/cy untouched.

#include <gtest/gtest.h>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/payloads.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace tp = xense::taccap::protocol;
using xense::taccap::Error;
using xense::taccap::FisheyeUndistorter;
using xense::taccap::FISHEYE_CALIB_HEIGHT;
using xense::taccap::FISHEYE_CALIB_WIDTH;
using xense::taccap::FISHEYE_NATURAL_FOCAL_SCALE;
using xense::taccap::FISHEYE_WIDE_FOCAL_SCALE;

namespace {

// Not constexpr: cv::Size_ has no constexpr constructor, so it is not a
// literal type.
const cv::Size kCalibSize{FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT};

// Plausible 640x480 wrist-camera intrinsics with real barrel distortion.
tp::CameraFisheyeCal distorted_cal() {
    return tp::CameraFisheyeCal{
        /*fx=*/320.5f, /*fy=*/321.25f, /*cx=*/319.0f, /*cy=*/239.5f,
        /*k1=*/-0.031f, /*k2=*/0.0072f, /*k3=*/-0.0013f, /*k4=*/0.00021f};
}

// Same intrinsics, zero distortion — undistortion must then be a no-op.
tp::CameraFisheyeCal undistorted_cal() {
    auto c = distorted_cal();
    c.k1 = c.k2 = c.k3 = c.k4 = 0.0f;
    return c;
}

}  // namespace

// ---- Size contract --------------------------------------------------------

TEST(FisheyeUndistorter, RejectsAnySizeOtherThanTheCalibratedOne) {
    const auto cal = distorted_cal();
    // The legacy constructor takes the params as a 640x480 calibration, and a
    // differently-sized frame cannot be served without rescaling them, which
    // would be wrong. Refuse instead.
    EXPECT_THROW(FisheyeUndistorter(cal, cv::Size(1280, 720)), Error);
    EXPECT_THROW(FisheyeUndistorter(cal, cv::Size(320, 240)),  Error);
    EXPECT_THROW(FisheyeUndistorter(cal, cv::Size(640, 481)),  Error);
    EXPECT_NO_THROW(FisheyeUndistorter(cal, kCalibSize));
}

TEST(FisheyeUndistorter, ApplyRejectsAFrameThatDoesNotMatchTheTables) {
    FisheyeUndistorter u(distorted_cal(), kCalibSize);
    cv::Mat wrong(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    EXPECT_THROW(u.apply(wrong), Error);
    cv::Mat empty;
    EXPECT_THROW(u.apply(empty), Error);
}

TEST(FisheyeUndistorter, ApplyPreservesSizeAndChannels) {
    FisheyeUndistorter u(distorted_cal(), kCalibSize);
    cv::Mat src(kCalibSize, CV_8UC3, cv::Scalar(10, 20, 30));
    cv::Mat dst = u.apply(src);
    EXPECT_EQ(dst.cols, kCalibSize.width);
    EXPECT_EQ(dst.rows, kCalibSize.height);
    EXPECT_EQ(dst.channels(), 3);
    EXPECT_EQ(dst.type(), CV_8UC3);
}

// ---- Balance / focal scale ------------------------------------------------

TEST(FisheyeUndistorter, BalanceZeroKeepsTheCalibratedFocalLength) {
    const auto cal = distorted_cal();
    FisheyeUndistorter u(cal, kCalibSize, 0.0f);
    EXPECT_DOUBLE_EQ(u.focal_scale(), FISHEYE_NATURAL_FOCAL_SCALE);

    // new_K == K at balance 0, and cx/cy never move.
    const cv::Mat& k = u.new_camera_matrix();
    EXPECT_NEAR(k.at<double>(0, 0), cal.fx, 1e-9);
    EXPECT_NEAR(k.at<double>(1, 1), cal.fy, 1e-9);
    EXPECT_NEAR(k.at<double>(0, 2), cal.cx, 1e-9);
    EXPECT_NEAR(k.at<double>(1, 2), cal.cy, 1e-9);
}

TEST(FisheyeUndistorter, BalanceOneWidensToTheLegacyFocalScale) {
    const auto cal = distorted_cal();
    FisheyeUndistorter u(cal, kCalibSize, 1.0f);
    EXPECT_DOUBLE_EQ(u.focal_scale(), FISHEYE_WIDE_FOCAL_SCALE);

    const cv::Mat& k = u.new_camera_matrix();
    EXPECT_NEAR(k.at<double>(0, 0), cal.fx * FISHEYE_WIDE_FOCAL_SCALE, 1e-6);
    EXPECT_NEAR(k.at<double>(1, 1), cal.fy * FISHEYE_WIDE_FOCAL_SCALE, 1e-6);
    // Principal point must stay put so the view does not drift as balance moves.
    EXPECT_NEAR(k.at<double>(0, 2), cal.cx, 1e-9);
    EXPECT_NEAR(k.at<double>(1, 2), cal.cy, 1e-9);
}

TEST(FisheyeUndistorter, BalanceInterpolatesLinearly) {
    FisheyeUndistorter u(distorted_cal(), kCalibSize, 0.5f);
    const double want = FISHEYE_NATURAL_FOCAL_SCALE +
                        0.5 * (FISHEYE_WIDE_FOCAL_SCALE - FISHEYE_NATURAL_FOCAL_SCALE);
    EXPECT_DOUBLE_EQ(u.focal_scale(), want);  // 0.85, matching the PC tool
}

TEST(FisheyeUndistorter, BalanceIsClampedNotRejected) {
    // A taste knob, not a correctness one — out-of-range clamps silently.
    EXPECT_DOUBLE_EQ(FisheyeUndistorter(distorted_cal(), kCalibSize, -3.0f).focal_scale(),
                     FISHEYE_NATURAL_FOCAL_SCALE);
    EXPECT_DOUBLE_EQ(FisheyeUndistorter(distorted_cal(), kCalibSize, 9.0f).focal_scale(),
                     FISHEYE_WIDE_FOCAL_SCALE);
    EXPECT_FLOAT_EQ(FisheyeUndistorter(distorted_cal(), kCalibSize, 9.0f).balance(), 1.0f);
}

// ---- Geometry -------------------------------------------------------------

// NOTE: zero distortion is NOT the identity here. The fisheye model carries
// theta = atan(r) in its projection, so even with k1..k4 = 0 the transform
// still converts the equidistant fisheye projection to a rectilinear one. Any
// test asserting "D = 0 => output == input" is testing a false premise.
//
// What we can pin down exactly is the projection formula itself. Rebuilding
// the maps by hand and comparing catches a swapped map_x/map_y, a wrong
// rectification matrix, K and new_K mixed up, or distortion coefficients
// dropped — none of which a "does the image change" check would notice.
TEST(FisheyeUndistorter, MatchesTheFisheyeProjectionFormulaExactly) {
    const auto cal = distorted_cal();
    const float balance = 0.35f;
    FisheyeUndistorter u(cal, kCalibSize, balance);

    const double fs      = u.focal_scale();
    const double fx_new  = cal.fx * fs;
    const double fy_new  = cal.fy * fs;

    cv::Mat ref_x(kCalibSize, CV_32FC1), ref_y(kCalibSize, CV_32FC1);
    for (int v = 0; v < kCalibSize.height; ++v) {
        for (int uu = 0; uu < kCalibSize.width; ++uu) {
            // Output pixel -> normalized coords in the NEW camera frame
            // (rectification R is identity).
            const double a = (uu - cal.cx) / fx_new;
            const double b = (v  - cal.cy) / fy_new;
            const double r = std::hypot(a, b);
            const double th = std::atan(r);
            const double t2 = th * th;
            const double th_d = th * (1.0 + cal.k1 * t2 + cal.k2 * t2 * t2 +
                                      cal.k3 * t2 * t2 * t2 +
                                      cal.k4 * t2 * t2 * t2 * t2);
            const double scale = (r > 1e-12) ? (th_d / r) : 1.0;
            // ...then back to pixels through the ORIGINAL K.
            ref_x.at<float>(v, uu) = static_cast<float>(cal.fx * scale * a + cal.cx);
            ref_y.at<float>(v, uu) = static_cast<float>(cal.fy * scale * b + cal.cy);
        }
    }

    cv::Mat src(kCalibSize, CV_8UC3);
    cv::randu(src, cv::Scalar::all(0), cv::Scalar::all(255));
    cv::Mat want;
    // Same interpolation the implementation uses — this test is about the
    // mapping, so a mismatch here would show up as a projection error and send
    // the reader after the wrong thing.
    cv::remap(src, want, ref_x, ref_y, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

    cv::Mat got = u.apply(src);
    cv::Mat diff;
    cv::absdiff(want, got, diff);
    // OpenCV builds the same maps in float; allow a 1-LSB resampling wobble.
    EXPECT_LE(cv::norm(diff, cv::NORM_INF), 1.0);
}

TEST(FisheyeUndistorter, PrincipalPointIsAFixedPoint) {
    // r = 0 at the principal point, where theta/r -> 1 regardless of k1..k4,
    // so the centre samples itself no matter the distortion or the balance.
    for (float balance : {0.0f, 0.5f, 1.0f}) {
        FisheyeUndistorter u(distorted_cal(), kCalibSize, balance);
        cv::Mat src(kCalibSize, CV_8UC3, cv::Scalar(0, 0, 0));
        const int cx = static_cast<int>(distorted_cal().cx);
        const int cy = static_cast<int>(distorted_cal().cy);
        src.at<cv::Vec3b>(cy, cx) = cv::Vec3b(255, 255, 255);

        cv::Mat dst = u.apply(src);
        // Bilinear sampling spreads the impulse a little; the centre must stay
        // clearly the brightest neighbourhood rather than move away.
        EXPECT_GT(dst.at<cv::Vec3b>(cy, cx)[0], 100)
            << "principal point drifted at balance=" << balance;
    }
}

TEST(FisheyeUndistorter, DistortionActuallyChangesTheImage) {
    // Guards against a silently-inert undistorter (e.g. D dropped on the floor).
    cv::Mat src(kCalibSize, CV_8UC3);
    cv::randu(src, cv::Scalar::all(0), cv::Scalar::all(255));

    cv::Mat identity  = FisheyeUndistorter(undistorted_cal(), kCalibSize).apply(src);
    cv::Mat corrected = FisheyeUndistorter(distorted_cal(),   kCalibSize).apply(src);

    const cv::Rect interior(4, 4, kCalibSize.width - 8, kCalibSize.height - 8);
    cv::Mat diff;
    cv::absdiff(identity(interior), corrected(interior), diff);
    EXPECT_GT(cv::norm(diff, cv::NORM_INF), 1.0);
}

TEST(FisheyeUndistorter, ReportsWhatItWasBuiltFor) {
    FisheyeUndistorter u(distorted_cal(), kCalibSize, 0.25f);
    EXPECT_EQ(u.size().width,  kCalibSize.width);
    EXPECT_EQ(u.size().height, kCalibSize.height);
    EXPECT_FLOAT_EQ(u.balance(), 0.25f);
}

// ---- Calibration resolution (V2.10) ---------------------------------------
//
// 640x480 and 1920x1080 are different sensor crops, so the intrinsics belong to
// exactly one resolution and the undistorter must never rescale them.

namespace {

// Measured 1920x1080 wrist calibration (shape of the PC tool's output).
tp::CameraFisheyeCal hd_cal() {
    return tp::CameraFisheyeCal{
        /*fx=*/477.2f, /*fy=*/476.9f, /*cx=*/968.3f, /*cy=*/563.1f,
        /*k1=*/-0.021f, /*k2=*/0.0061f, /*k3=*/-0.0042f, /*k4=*/0.0011f};
}

}  // namespace

TEST(FisheyeCalSize, InferenceRecognisesTheVgaReference) {
    // FISHEYE_FALLBACK_CAL: cx = 321.4, cy = 239.95.
    const auto s = xense::taccap::infer_fisheye_cal_size(xense::taccap::FISHEYE_FALLBACK_CAL);
    ASSERT_TRUE(s.has_value());
    EXPECT_EQ(*s, cv::Size(640, 480));
}

TEST(FisheyeCalSize, InferenceRecognisesA1080pCalibration) {
    const auto s = xense::taccap::infer_fisheye_cal_size(hd_cal());
    ASSERT_TRUE(s.has_value());
    EXPECT_EQ(*s, cv::Size(1920, 1080));
}

TEST(FisheyeCalSize, InferenceRefusesWhenNoCandidateMatches) {
    using xense::taccap::infer_fisheye_cal_size;
    // Between the two centres: within 10% of neither.
    auto c = hd_cal();
    c.cx = 600.0f; c.cy = 400.0f;
    EXPECT_FALSE(infer_fisheye_cal_size(c).has_value());
    // x fits 640x480 but y fits only 1920x1080: one axis is not a match.
    c.cx = 321.0f; c.cy = 540.0f;
    EXPECT_FALSE(infer_fisheye_cal_size(c).has_value());
    // Exactly on the 10% boundary still matches; just past it does not.
    c.cx = 320.0f + 64.0f; c.cy = 240.0f;
    EXPECT_EQ(infer_fisheye_cal_size(c), std::optional<cv::Size>(cv::Size(640, 480)));
    c.cx = 320.0f + 64.5f;
    EXPECT_FALSE(infer_fisheye_cal_size(c).has_value());
}

TEST(FisheyeCalSize, InferenceRefusesAnUnusableCalibration) {
    using xense::taccap::infer_fisheye_cal_size;
    // All-zero record: not usable, and cx = cy = 0 fits nothing anyway.
    EXPECT_FALSE(infer_fisheye_cal_size(tp::CameraFisheyeCal{}).has_value());
    // A centred principal point does not rescue fx = 0.
    auto c = distorted_cal();
    c.fx = 0.0f;
    EXPECT_FALSE(infer_fisheye_cal_size(c).has_value());
    c = distorted_cal();
    c.cx = std::nanf("");
    EXPECT_FALSE(infer_fisheye_cal_size(c).has_value());
}

TEST(FisheyeCalSize, PlausibilityMatchesTheFirmwareRule) {
    using xense::taccap::fisheye_cal_size_plausible;
    EXPECT_TRUE(fisheye_cal_size_plausible(distorted_cal(), 640, 480));
    EXPECT_TRUE(fisheye_cal_size_plausible(hd_cal(), 1920, 1080));
    // The firmware's own example: a 1080p calibration (cx ~ 968) at 640x480.
    EXPECT_FALSE(fisheye_cal_size_plausible(hd_cal(), 640, 480));
    EXPECT_FALSE(fisheye_cal_size_plausible(distorted_cal(), 0, 480));
    EXPECT_FALSE(fisheye_cal_size_plausible(distorted_cal(), 640, 4097));
    auto c = distorted_cal();
    c.fy = -1.0f;
    EXPECT_FALSE(fisheye_cal_size_plausible(c, 640, 480));
    c = distorted_cal();
    c.k3 = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(fisheye_cal_size_plausible(c, 640, 480));
    c = distorted_cal();
    c.cx = 640.0f;   // on the edge is outside: 0 < cx < width
    EXPECT_FALSE(fisheye_cal_size_plausible(c, 640, 480));
}

TEST(FisheyeCalSize, RecordSizeIsKnownOnlyWhenBothDimensionsAreSet) {
    xense::taccap::FisheyeCalRecord r;
    EXPECT_FALSE(r.size_known());
    r.width = 1920;
    EXPECT_FALSE(r.size_known());
    r.height = 1080;
    EXPECT_TRUE(r.size_known());
}

TEST(FisheyeUndistorter, SizedConstructorAcceptsTheCalibratedResolution) {
    const cv::Size hd(1920, 1080);
    FisheyeUndistorter u(hd_cal(), hd, hd);
    EXPECT_EQ(u.size(), hd);
    cv::Mat src(hd, CV_8UC3, cv::Scalar(10, 20, 30));
    EXPECT_EQ(u.apply(src).size(), hd);
    // 640x480 through the sized form, too.
    EXPECT_NO_THROW(FisheyeUndistorter(distorted_cal(), kCalibSize, kCalibSize, 0.5f));
}

TEST(FisheyeUndistorter, SizedConstructorRejectsAnyOtherFrameSize) {
    const cv::Size hd(1920, 1080);
    // Neither direction is a rescale the SDK will do: different sensor crops.
    EXPECT_THROW(FisheyeUndistorter(hd_cal(), hd, kCalibSize), Error);
    EXPECT_THROW(FisheyeUndistorter(distorted_cal(), kCalibSize, hd), Error);
    // Same aspect ratio, half the size: still refused.
    EXPECT_THROW(FisheyeUndistorter(hd_cal(), hd, cv::Size(960, 540)), Error);
    // An unknown calibration size cannot be satisfied by any frame.
    EXPECT_THROW(FisheyeUndistorter(hd_cal(), cv::Size(0, 0), cv::Size(0, 0)), Error);

    try {
        FisheyeUndistorter(hd_cal(), hd, kCalibSize);
        FAIL() << "expected Error";
    } catch (const Error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("1920x1080"), std::string::npos) << msg;
        EXPECT_NE(msg.find("rescaled"), std::string::npos) << msg;
        EXPECT_NE(msg.find("recalibrate"), std::string::npos) << msg;
    }
}

TEST(FisheyeUndistorter, LegacyConstructorStillMeans640x480) {
    // Unchanged behaviour: (cal, size, balance) takes the params as 640x480.
    FisheyeUndistorter legacy(distorted_cal(), kCalibSize, 0.25f);
    FisheyeUndistorter sized(distorted_cal(), kCalibSize, kCalibSize, 0.25f);
    EXPECT_FLOAT_EQ(legacy.balance(), sized.balance());
    EXPECT_EQ(0, cv::norm(legacy.new_camera_matrix(), sized.new_camera_matrix(),
                          cv::NORM_INF));
    EXPECT_THROW(FisheyeUndistorter(hd_cal(), cv::Size(1920, 1080)), Error);
}

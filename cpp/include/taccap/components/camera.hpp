// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Camera component: thin wrapper around OpenCV's V4L2 capture for the
// wrist-mounted UVC camera (Sunplus XC...). Uses cv::VideoCapture under
// the hood — exposed as raw cv::Mat frames. The visuotactile (OG) sensors
// are not handled in C++ here; they are read at the Python level via the
// xensesdk wheel (on-sensor calibration / rectification path).
//
// We intentionally don't pull pybind11/numpy here — Python bindings live
// in python/bindings/components.cpp.

#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xense::taccap {

class FisheyeUndistorter;

struct CameraFrame {
    std::chrono::steady_clock::time_point host_time;
    uint64_t                              frame_index;   // monotonic per Camera instance
    cv::Mat                               image;         // see Config::color_mode; BGR8 by default
};

// Channel order of the frames Camera hands out.
//
// Bgr is the default because it is OpenCV's own convention and what this SDK has
// always returned. Flipping it silently would invert the colours of every
// existing caller's imshow/imwrite with nothing raised to notice.
//
// Rgb exists because consumers feeding a machine-learning pipeline generally
// want RGB — the LeRobot dataset format stores RGB, so an integration that keeps
// BGR either converts on the Python side or, worse, records swapped channels.
// Asking for it here converts once in the capture path instead of at every call
// site.
enum class ColorMode {
    Bgr,   // OpenCV native; the historical behaviour
    Rgb,
};

class Camera {
public:
    using Callback = std::function<void(const CameraFrame&)>;

    struct Config {
        std::string device     = "";     // e.g. /dev/video3 or /dev/serial/by-id/...
        int         width      = 640;
        int         height     = 480;
        double      fps        = 30.0;
        bool        use_mjpg   = true;   // request MJPEG fourcc
        ColorMode   color_mode = ColorMode::Bgr;
    };

    explicit Camera(const Config& cfg);
    ~Camera();
    Camera(const Camera&)            = delete;
    Camera& operator=(const Camera&) = delete;

    // Synchronous one-shot read. Returns false on read failure, and also while
    // a stream is running, which owns the device.
    //
    // No timeout parameter: it took one until 0.2.3 and never used it. The call
    // blocks for however long cv::VideoCapture::read blocks, which is its own
    // V4L2 timeout. A parameter the implementation ignores is worse than no
    // parameter, because a caller passing 50 still waits the V4L2 timeout and
    // has no way to tell. Add one back only together with poll/select wrapping
    // that actually honours it.
    bool read(CameraFrame& out);

    // Async streaming: spawn a background thread that calls `cb` on each
    // frame. Stop with stop() or destructor. Re-entrant safe (callback
    // invocation is serialised on the capture thread).
    void start(Callback cb);
    void stop();
    bool is_streaming() const noexcept { return running_.load(); }

    // ---- Optional fisheye undistortion -------------------------------------
    // When set, every frame handed out by read() and by the streaming callback
    // has already been rectified. Camera itself knows nothing about where the
    // intrinsics came from — the gripper reads them off the MCU and injects the
    // undistorter here (see Config::undistort_wrist), and callers that own the
    // UVC device themselves can build one directly.
    //
    // Safe to call while streaming: the capture thread takes a copy of the
    // shared_ptr per frame. Pass nullptr to go back to raw frames.
    //
    // If undistortion throws (e.g. a frame whose size does not match the remap
    // tables), the frame is passed through UNRECTIFIED and the error is logged
    // once per occurrence — a geometry problem must not kill the capture loop.
    void set_undistorter(std::shared_ptr<const FisheyeUndistorter> u);
    std::shared_ptr<const FisheyeUndistorter> undistorter() const;

    // Stats
    uint64_t total_frames() const noexcept { return total_; }
    uint64_t dropped_frames() const noexcept { return dropped_; }
    double   actual_fps() const noexcept { return last_fps_.load(); }

    const Config& config() const noexcept { return cfg_; }

private:
    void capture_loop_(Callback cb);
    // Rectify in place when an undistorter is set; never throws.
    void maybe_undistort_(cv::Mat& image) const;
    // Convert in place when Config::color_mode asks for other than the
    // capture-native BGR. Runs *after* undistortion so the undistorter always
    // sees BGR and its contract is untouched; remap is per-channel, so the order
    // of the two cannot change the result anyway.
    void maybe_convert_colour_(cv::Mat& image) const;

    Config            cfg_;
    void*             impl_ = nullptr;   // opaque cv::VideoCapture* (kept void* so
                                         // the public header doesn't drag in OpenCV
                                         // includes for users that don't need them)
    std::thread       worker_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> total_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<double>   last_fps_{0.0};
    std::mutex            cap_mu_;       // guards reads when start() inactive
    mutable std::mutex    undist_mu_;    // guards undist_ against the capture thread
    std::shared_ptr<const FisheyeUndistorter> undist_;
};

}  // namespace xense::taccap

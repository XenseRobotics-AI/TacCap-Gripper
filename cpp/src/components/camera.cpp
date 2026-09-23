// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/camera.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cerrno>
#include <utility>

namespace xense::taccap {

namespace {
inline cv::VideoCapture* as_cap(void* p) { return static_cast<cv::VideoCapture*>(p); }
}

Camera::Camera(const Config& cfg) : cfg_(cfg) {
    if (cfg_.device.empty()) {
        throw IoError("Camera: empty device path", EINVAL);
    }

    auto* cap = new cv::VideoCapture();
    impl_ = cap;
    try {
        if (!cap->open(cfg_.device, cv::CAP_V4L2)) {
            throw IoError("Camera: cv::VideoCapture::open(" + cfg_.device + ")", EIO);
        }
        if (cfg_.use_mjpg) {
            cap->set(cv::CAP_PROP_FOURCC,
                     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        }
        cap->set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
        cap->set(cv::CAP_PROP_FPS,          cfg_.fps);
    } catch (...) {
        delete cap;
        impl_ = nullptr;
        throw;
    }
}

Camera::~Camera() {
    stop();
    if (impl_) {
        as_cap(impl_)->release();
        delete as_cap(impl_);
        impl_ = nullptr;
    }
}

void Camera::set_undistorter(std::shared_ptr<const FisheyeUndistorter> u) {
    std::lock_guard<std::mutex> lk(undist_mu_);
    undist_ = std::move(u);
}

std::shared_ptr<const FisheyeUndistorter> Camera::undistorter() const {
    std::lock_guard<std::mutex> lk(undist_mu_);
    return undist_;
}

void Camera::maybe_undistort_(cv::Mat& image) const {
    std::shared_ptr<const FisheyeUndistorter> u;
    {
        // Copy the handle, then drop the lock: remap() is the expensive part
        // and must not block a concurrent set_undistorter().
        std::lock_guard<std::mutex> lk(undist_mu_);
        u = undist_;
    }
    if (!u) return;
    try {
        image = u->apply(image);
    } catch (const std::exception& e) {
        // Pass the raw frame through rather than dropping it or tearing down
        // the capture loop — a size/geometry mismatch is a config bug, and
        // losing the stream on top of it helps nobody.
        logger()->error("Camera({}): fisheye undistortion failed, passing the "
                        "raw frame through: {}", cfg_.device, e.what());
    }
}

void Camera::maybe_convert_colour_(cv::Mat& image) const {
    if (cfg_.color_mode == ColorMode::Bgr) return;   // capture-native, nothing to do
    if (image.empty() || image.channels() != 3) return;
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

bool Camera::read(CameraFrame& out) {
    if (!impl_) return false;
    if (running_.load()) {
        // While streaming, the worker thread owns the device — disallow
        // concurrent direct reads to keep the contract simple.
        return false;
    }
    std::lock_guard<std::mutex> lk(cap_mu_);
    cv::Mat frame;
    if (!as_cap(impl_)->read(frame) || frame.empty()) {
        ++dropped_;
        return false;
    }
    maybe_undistort_(frame);
    maybe_convert_colour_(frame);
    out.host_time   = std::chrono::steady_clock::now();
    out.frame_index = ++total_;
    out.image       = std::move(frame);
    return true;
}

void Camera::start(Callback cb) {
    if (running_.exchange(true)) return;   // idempotent
    worker_ = std::thread([this, cb = std::move(cb)]() { capture_loop_(cb); });
}

void Camera::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
}

void Camera::capture_loop_(Callback cb) {
    using clock = std::chrono::steady_clock;
    auto* cap = as_cap(impl_);

    auto fps_window_start = clock::now();
    uint64_t fps_window_frames = 0;

    while (running_.load(std::memory_order_acquire)) {
        cv::Mat frame;
        if (!cap->read(frame) || frame.empty()) {
            ++dropped_;
            continue;
        }
        maybe_undistort_(frame);
        maybe_convert_colour_(frame);
        const auto now = clock::now();
        CameraFrame cf{};
        cf.host_time   = now;
        cf.frame_index = ++total_;
        cf.image       = std::move(frame);

        try {
            cb(cf);
        } catch (...) { /* swallow — callbacks must not throw */ }

        ++fps_window_frames;
        const auto elapsed = std::chrono::duration<double>(now - fps_window_start).count();
        if (elapsed >= 1.0) {
            last_fps_.store(static_cast<double>(fps_window_frames) / elapsed,
                            std::memory_order_relaxed);
            fps_window_start  = now;
            fps_window_frames = 0;
        }
    }
}

}  // namespace xense::taccap

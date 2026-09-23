// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: Camera, FisheyeUndistorter, ColorMode
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_camera(py::module_& m) {
    using namespace xense::taccap;
    // ---- Camera ---------------------------------------------------------
    // ---- Fisheye fallback (used when the firmware has no calibration) ----
    m.attr("FISHEYE_FALLBACK_CAL") = py::cast(FISHEYE_FALLBACK_CAL);
    m.def("is_usable_fisheye_cal", &is_usable_fisheye_cal, py::arg("calibration"),
          "True when a CameraFisheyeCal carries usable intrinsics.\n\n"
          "An uncalibrated unit answers a flash read with an all-zero record "
          "rather than a NACK, and remapping with fx = fy = 0 yields a black "
          "frame. Use this to tell a real calibration from an empty one; "
          "FISHEYE_FALLBACK_CAL is the reference to fall back to, with a "
          "warning, when it returns False.");

    // ---- FisheyeUndistorter (V2.0+ wrist fisheye intrinsics) -------------
    // Usable standalone on frames this SDK never captured — the common case,
    // since the wrist UVC device is normally owned by an external service.
    py::class_<FisheyeUndistorter, std::shared_ptr<FisheyeUndistorter>>(
            m, "FisheyeUndistorter",
        "Rectifies wrist-camera frames using the fisheye intrinsics the firmware\n"
        "persists in flash (read them with Calibration.read_fisheye or, preferring\n"
        "a usable answer over the literal one, Calibration.resolve_fisheye).\n\n"
        "Protocol-free: it takes a plain CameraFisheyeCal and never touches a\n"
        "transport, so it works on frames this SDK never captured -- the common\n"
        "case, since the wrist UVC device is usually owned by another service. The\n"
        "remap tables are built once in the constructor, so build one and reuse it;\n"
        "rebuilding per frame would dominate the cost.")
        .def(py::init([](const protocol::CameraFisheyeCal& cal,
                         int width, int height, float balance) {
                 return std::make_shared<FisheyeUndistorter>(
                     cal, cv::Size(width, height), balance);
             }),
             py::arg("calibration"),
             // Defaults are the only calibrated resolution; anything else
             // raises, because the firmware record carries no image size.
             py::arg("width")   = FISHEYE_CALIB_WIDTH,
             py::arg("height")  = FISHEYE_CALIB_HEIGHT,
             // 0 = calibrated focal length (natural view, matches the PC tool);
             // 1 = 0.70x for the widest field of view. Clamped to [0,1].
             py::arg("balance") = 0.0f,
             "Build the remap tables for one calibration.\n\n"
             "width/height must be the calibrated 640x480 -- the firmware record\n"
             "carries no image size, so any other size raises RuntimeError rather\n"
             "than guessing a rescale. balance picks the output focal length: 0\n"
             "keeps the calibrated one, 1 shortens it to 0.70x for the widest field\n"
             "of view at the cost of more black border. It is clamped to [0, 1]\n"
             "rather than rejected.")
        .def("apply", [](const FisheyeUndistorter& self, const py::array& img) {
                 cv::Mat src = numpy_to_mat_bgr(img);
                 cv::Mat dst;
                 {
                     // remap() is pure CPU work on borrowed memory — let other
                     // threads run, but keep `img` alive via the caller's ref.
                     py::gil_scoped_release gil;
                     dst = self.apply(src);
                 }
                 return mat_to_numpy(dst);
             },
             py::arg("image"),
             "Rectify one (H, W, 3) uint8 BGR frame; returns a new array.\n\n"
             "Resampled with INTER_CUBIC: the periphery is magnified about "
             "3.3x by the fisheye-to-pinhole mapping, and bilinear visibly "
             "softens an image being enlarged that much.")
        .def_property_readonly("width",  [](const FisheyeUndistorter& s) { return s.size().width; },
                               "Frame width in pixels these tables were built for; apply() accepts no "
                               "other size, and returns that same size.")
        .def_property_readonly("height", [](const FisheyeUndistorter& s) { return s.size().height; },
                               "Frame height in pixels these tables were built for; apply() accepts no "
                               "other size, and returns that same size.")
        .def_property_readonly("balance", &FisheyeUndistorter::balance,
                               "The balance in force, after the constructor clamped it to [0, 1].")
        .def_property_readonly("focal_scale", &FisheyeUndistorter::focal_scale,
                               "Output focal length as a multiple of the calibrated one: 1.00 at "
                               "balance 0, 0.70 at balance 1, interpolated in between. Only fx/fy "
                               "scale; the principal point stays put.")
        .def_property_readonly("new_camera_matrix",
            [](const FisheyeUndistorter& s) {
                // The K rectified pixels live in — NOT the raw firmware K.
                const cv::Mat& k = s.new_camera_matrix();
                py::array_t<double> a(std::vector<py::ssize_t>{3, 3});
                std::memcpy(a.mutable_data(), k.ptr<double>(), 9 * sizeof(double));
                return a;
            },
            "3x3 camera matrix of the rectified image (calibrated K with fx/fy "
            "scaled by focal_scale).")
        .def("__repr__", [](const FisheyeUndistorter& s) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "FisheyeUndistorter(%dx%d, balance=%.2f, focal_scale=%.3f)",
                s.size().width, s.size().height, s.balance(), s.focal_scale());
            return std::string(buf);
        });

    py::enum_<ColorMode>(m, "ColorMode",
        "Channel order of the frames a Camera hands out.\n\n"
        "BGR is the default and is OpenCV's own convention, so imshow/imwrite on\n"
        "a frame straight out of read() looks right. Pick RGB when the frames\n"
        "feed a machine-learning pipeline — LeRobot datasets store RGB — so the\n"
        "conversion happens once in the capture path rather than at every call\n"
        "site, or worse, gets forgotten and records swapped channels.")
        .value("BGR", ColorMode::Bgr)
        .value("RGB", ColorMode::Rgb);

    py::class_<Camera>(m, "Camera",
        "The wrist UVC camera, captured through OpenCV's V4L2 backend.\n\n"
        "Poll it with read() or stream it with start(); the two are exclusive,\n"
        "because a running capture thread owns the device and read() then returns\n"
        "None. Install an undistorter to have every frame rectified on the way out.\n\n"
        "The visuotactile (OG) sensors are not handled here -- they are read at the\n"
        "Python level through the xensesdk wheel.")
        .def(py::init([](const std::string& dev, int w, int h, double fps, bool mjpg,
                         ColorMode color_mode) {
            return std::make_unique<Camera>(Camera::Config{dev, w, h, fps, mjpg, color_mode});
        }),
            py::arg("device"), py::arg("width") = 640, py::arg("height") = 480,
            py::arg("fps") = 30.0, py::arg("use_mjpg") = true,
            py::arg("color_mode") = ColorMode::Bgr,
            "Open a V4L2 device and ask it for this format.\n\n"
            "device is a /dev/video* path. Size, rate and MJPEG are requests, not\n"
            "guarantees -- V4L2 may settle on something else, so read actual_fps and\n"
            "the shape of the first frame instead of assuming. Raises IoError when\n"
            "the path is empty or the device will not open.")
        .def("read", [](Camera& self) -> py::object {
            CameraFrame f;
            bool ok;
            {
                py::gil_scoped_release gil;
                ok = self.read(f);
            }
            if (!ok) return py::none();
            return py::cast(std::move(f));
        }, "Grab one frame; returns a CameraFrame, or None when there is nothing to\n"
           "hand back.\n\n"
           "Blocks with the GIL released, for as long as the underlying\n"
           "VideoCapture.read blocks -- its own V4L2 timeout, which the caller cannot\n"
           "shorten. There is no timeout argument for that reason; one existed until\n"
           "0.2.3 and was ignored.\n\n"
           "None means either the capture failed, which also bumps dropped_frames, or\n"
           "a stream is running and owns the device.")
        .def("start", [](Camera& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            self.start([cb](const CameraFrame& f) {
                call_into_python("xense.taccap.Camera callback",
                                 [&] { (*cb)(f); });
            });
        }, py::arg("callback"),
           "Spawn a capture thread that calls callback(frame) for every frame.\n\n"
           "The callback runs on that capture thread, not the caller's, serialised\n"
           "with itself; an exception raised inside it is reported as unraisable and\n"
           "swallowed. While it runs the thread owns the device and read() returns\n"
           "None. Idempotent, and that cuts the other way: calling start() on a\n"
           "camera that is already streaming does nothing and drops the new callback\n"
           "on the floor.")
        .def("stop", [](Camera& self) {
            py::gil_scoped_release gil;
            self.stop();
        }, "Stop the capture thread and join it. Idempotent, and a no-op when not\n"
           "streaming.\n\n"
           "Blocks with the GIL released until the in-flight callback returns, so it\n"
           "must not be called from inside that callback -- joining the capture\n"
           "thread from itself fails.")
        // Pass None to go back to raw frames. Safe to call while streaming.
        .def("set_undistorter", [](Camera& self,
                                   std::shared_ptr<FisheyeUndistorter> u) {
            py::gil_scoped_release gil;
            self.set_undistorter(std::move(u));
        }, py::arg("undistorter").none(true),
           "Install a FisheyeUndistorter, or None to go back to raw frames. Every\n"
           "frame from read() and from the streaming callback is rectified before it\n"
           "is handed over.\n\n"
           "Safe to call while streaming. If rectification throws -- a frame whose\n"
           "size does not match the remap tables -- the raw frame is passed through\n"
           "and the error is logged; a geometry mistake must not kill the capture\n"
           "loop.")
        .def_property_readonly("is_streaming",   &Camera::is_streaming,
                               "True between start() and stop().")
        .def_property_readonly("total_frames",   &Camera::total_frames,
                               "Frames handed out since construction; also the frame_index of the "
                               "most recent one.")
        .def_property_readonly("dropped_frames", &Camera::dropped_frames,
                               "Captures that came back failed or empty since construction -- frames "
                               "V4L2 never delivered, not frames a slow callback missed.")
        .def_property_readonly("actual_fps",     &Camera::actual_fps,
                               "Rate measured over the last completed one-second window. Updated only "
                               "by the streaming path, and 0.0 until the first window closes.");
}

}  // namespace xense::taccap::python

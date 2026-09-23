// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Shared preamble for the bind_*.cpp translation units.
//
// These bindings used to live in one 1,700-line components.cpp. Splitting it
// per subsystem was a PURE MOVE: every registration kept its text and, more
// importantly, its ORDER — pybind11 evaluates default arguments at def() time,
// so a type used as a default must already be registered. bind_components()
// in components.cpp still calls the pieces in the original sequence, and that
// sequence is the contract.
//
// The helpers below were file-static in the old single file; they are inline
// here so every unit gets the same definition and the call sites stayed
// verbatim.

#pragma once

#include <pybind11/pybind11.h>

#include "gil_safe.hpp"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include <taccap/components/calibration.hpp>
#include <taccap/components/diagnostics.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/components/led.hpp>
#include <taccap/force_position_controller.hpp>
#include <taccap/impedance_controller.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/leader_gripper.hpp>
#include <taccap/discovery.hpp>
#include <taccap/ota.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace py = pybind11;

// GIL-safe callback ownership/invocation is shared with module.cpp.
using xense::taccap::python::gil::call_into_python;
using xense::taccap::python::gil::interpreter_gone;
using xense::taccap::python::gil::make_gil_safe_callback;

#include <array>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

// ---- helpers --------------------------------------------------------------

// Convert std::chrono::steady_clock::time_point to seconds-since-epoch float.
// We expose a monotonic-ish double; users that need wall-clock can compare
// against time.monotonic() in Python.
inline double tp_to_seconds(std::chrono::steady_clock::time_point tp) {
    using ns = std::chrono::nanoseconds;
    return std::chrono::duration_cast<ns>(tp.time_since_epoch()).count() * 1e-9;
}

// Wrap a std::array<float, 3> as a numpy float32 array of shape (3,).
//
// The shape MUST be spelled as a container. `py::array_t<float> arr(3)` picks
// a different overload on pybind11 2.9 and yields a shape-(3,) array with
// stride 0 — a broadcast view of element [0]. Every value written to p[1] and
// p[2] lands on the same address, so `accel_mps2` read back as [x, x, x].
inline py::array make_vec3(const std::array<float, 3>& v) {
    py::array_t<float> arr(std::vector<py::ssize_t>{3});
    auto* p = arr.mutable_data();
    p[0] = v[0]; p[1] = v[1]; p[2] = v[2];
    return arr;
}

// Wrap a cv::Mat (BGR8 expected) as a (H, W, 3) uint8 numpy array. This
// makes a copy so the array is safe across frame boundaries.
inline py::array mat_to_numpy(const cv::Mat& m) {
    if (m.empty()) {
        return py::array_t<uint8_t>({0, 0, 3});
    }
    if (m.type() != CV_8UC3) {
        // For now we only handle 8-bit 3-channel; advanced users can use
        // OpenCV directly. Fall back to empty.
        return py::array_t<uint8_t>({0, 0, 3});
    }
    py::array_t<uint8_t> arr({m.rows, m.cols, 3});
    auto* dst = arr.mutable_data();
    if (m.isContinuous()) {
        std::memcpy(dst, m.data, static_cast<size_t>(m.rows) * m.cols * 3);
    } else {
        for (int r = 0; r < m.rows; ++r) {
            std::memcpy(dst + r * m.cols * 3, m.ptr(r), m.cols * 3);
        }
    }
    return arr;
}

// Borrow a (H, W, 3) uint8 numpy array as a cv::Mat WITHOUT copying. The
// caller must keep `arr` alive for as long as the Mat is used — every use
// here is a synchronous read inside one binding call, so that holds.
//
// Deliberately strict rather than forcecast: silently casting a float image to
// uint8 would produce a plausible-looking but wrong result.
inline cv::Mat numpy_to_mat_bgr(const py::array& arr) {
    if (!py::dtype::of<uint8_t>().is(arr.dtype())) {
        throw std::invalid_argument(
            "expected a uint8 array (BGR8); got dtype " +
            py::str(arr.dtype()).cast<std::string>());
    }
    if (arr.ndim() != 3 || arr.shape(2) != 3) {
        throw std::invalid_argument(
            "expected shape (H, W, 3); got ndim=" + std::to_string(arr.ndim()));
    }
    if ((arr.flags() & py::array::c_style) == 0) {
        throw std::invalid_argument(
            "expected a C-contiguous array; pass numpy.ascontiguousarray(img)");
    }
    return cv::Mat(static_cast<int>(arr.shape(0)),
                   static_cast<int>(arr.shape(1)),
                   CV_8UC3,
                   const_cast<void*>(arr.data()));
}

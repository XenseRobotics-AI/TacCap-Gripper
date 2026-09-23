// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Registration order for the component bindings.
//
// This file used to hold all ~1,700 lines of them. It now holds only the
// order, because the order is the one thing the split could break: pybind11
// evaluates a def()'s default arguments immediately, so any type used as a
// default (an enum in `py::arg("color_mode") = ColorMode::Bgr`, say)
// must already be registered when that def() runs. Keep these calls in this
// sequence unless you have checked what moves.

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace xense::taccap::python {

void bind_sensors(py::module_& m);
void bind_ota(py::module_& m);
void bind_gripper_types(py::module_& m);
void bind_calibration(py::module_& m);
void bind_motor(py::module_& m);
void bind_camera(py::module_& m);
void bind_gripper(py::module_& m);
void bind_control(py::module_& m);

void bind_components(py::module_& m) {
    bind_sensors(m);
    bind_ota(m);
    bind_gripper_types(m);
    bind_calibration(m);
    bind_motor(m);
    bind_camera(m);
    bind_gripper(m);
    bind_control(m);
}

}  // namespace xense::taccap::python

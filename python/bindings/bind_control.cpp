// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: GripperObservation, ImpedanceController, ForcePositionController
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_control(py::module_& m) {
    using namespace xense::taccap;
    // ---- GripperObservation ---------------------------------------------
    py::class_<GripperObservation>(m, "GripperObservation")
        .def_readonly("valid",    &GripperObservation::valid)
        .def_readonly("position", &GripperObservation::position)   // [0,1]
        .def_readonly("velocity", &GripperObservation::velocity)
        .def_readonly("torque",   &GripperObservation::torque)
        .def_readonly("raw_pos",  &GripperObservation::raw_pos)
        .def_readonly("status",   &GripperObservation::status)
        .def_readonly("motor_temp_c", &GripperObservation::motor_temp_c)
        .def_readonly("seq",      &GripperObservation::seq)
        .def_readonly("age_ms",   &GripperObservation::age_ms)
        .def("__repr__", [](const GripperObservation& o) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperObservation(valid=%d, position=%.4f, vel=%.4f, "
                "torque=%.4f, temp=%.1fC, age=%.1fms, seq=%llu)",
                o.valid, o.position, o.velocity, o.torque, o.motor_temp_c, o.age_ms,
                (unsigned long long)o.seq);
            return std::string(buf);
        });

    // ---- ImpedanceController: supervised position tracking ---------------
    m.attr("MOTOR_RATED_TORQUE_NM") = MOTOR_RATED_TORQUE_NM;
    m.attr("MOTOR_PEAK_TORQUE_NM")  = MOTOR_PEAK_TORQUE_NM;

    py::enum_<ImpedanceState>(m, "ImpedanceState",
        "Ordered by precedence: FAULT beats TORQUE_CAPPED beats TRACKING。\n\n"
        "没有 STALLED —— 失速守卫已删除。被挡住的爪子会让误差钳位饱和在 "
        "max_position_torque_nm 并一直保持在那里,那是预期的稳态而不是一个状态:"
        "接触不需要检测,饱和本身就是接触。原来那个守卫会把有效目标压到爪子当前"
        "位置,误差归零、力矩随之塌掉 —— 实测是接触后 60 ms 在 0.35 Nm 松手。")
        .value("IDLE",          ImpedanceState::Idle)
        .value("TRACKING",      ImpedanceState::Tracking)
        .value("TORQUE_CAPPED", ImpedanceState::TorqueCapped)
        .value("FAULT",         ImpedanceState::Fault)
        .def("__str__", [](ImpedanceState s) { return to_string(s); });

    py::class_<ImpedanceConfig>(m, "ImpedanceConfig")
        .def(py::init<>())
        .def_readwrite("kp",                &ImpedanceConfig::kp)
        .def_readwrite("kd",                &ImpedanceConfig::kd)
        .def_readwrite("feedforward_torque", &ImpedanceConfig::feedforward_torque)
        .def_readwrite("max_position_torque_nm",
                       &ImpedanceConfig::max_position_torque_nm)
        .def_readwrite("rated_torque_nm",   &ImpedanceConfig::rated_torque_nm)
        .def_readwrite("status_timeout_ms", &ImpedanceConfig::status_timeout_ms)
        .def_readwrite("motor_stream_hz",   &ImpedanceConfig::motor_stream_hz);

    py::class_<ImpedanceSnapshot>(m, "ImpedanceSnapshot")
        .def_readonly("running",             &ImpedanceSnapshot::running)
        .def_readonly("state",               &ImpedanceSnapshot::state)
        .def_readonly("observation",         &ImpedanceSnapshot::observation)
        .def_readonly("target_position",     &ImpedanceSnapshot::target_position)
        .def_readonly("effective_position",  &ImpedanceSnapshot::effective_position)
        .def_readonly("commanded_torque_nm", &ImpedanceSnapshot::commanded_torque_nm)
        .def_readonly("torque_capped",       &ImpedanceSnapshot::torque_capped)
        .def_readonly("torque_caps",         &ImpedanceSnapshot::torque_caps)
        .def_readonly("device_limit_nm",     &ImpedanceSnapshot::device_limit_nm)
        .def_readonly("fault_reason",        &ImpedanceSnapshot::fault_reason)
        .def("__repr__", [](const ImpedanceSnapshot& s) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "ImpedanceSnapshot(state=%s, pos=%.4f, target=%.4f, "
                "effective=%.4f, torque=%.3fNm, command=%.3fNm, "
                "capped=%d, age=%.1fms)",
                to_string(s.state), s.observation.position, s.target_position,
                s.effective_position, s.observation.torque, s.commanded_torque_nm,
                s.torque_capped, s.observation.age_ms);
            return std::string(buf);
        });

    py::class_<ImpedanceController>(m, "ImpedanceController")
        .def(py::init([](FollowerGripper& g, const ImpedanceConfig& cfg) {
                return std::make_unique<ImpedanceController>(g, cfg);
            }), py::arg("gripper"), py::arg("config") = ImpedanceConfig{},
            py::keep_alive<1, 2>())
        .def("start", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.start();
        })
        .def("stop", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.stop();
        })
        .def_property_readonly("running", &ImpedanceController::running)
        .def("set_target", [](ImpedanceController& c, float p) {
            py::gil_scoped_release g; c.set_target(p);
        }, py::arg("position"))
        .def("set_gains", [](ImpedanceController& c, float kp, float kd, float ff) {
            py::gil_scoped_release g; c.set_gains(kp, kd, ff);
        }, py::arg("kp"), py::arg("kd"), py::arg("feedforward_torque") = 0.0f)
        .def("reset", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.reset();
        })
        .def_property_readonly("state", [](const ImpedanceController& c) {
            py::gil_scoped_release g; return c.state();
        })
        .def("snapshot", [](const ImpedanceController& c) {
            py::gil_scoped_release g; return c.snapshot();
        })
        .def_property_readonly("config", &ImpedanceController::config)
        .def("__enter__", [](ImpedanceController& c) -> ImpedanceController& {
            py::gil_scoped_release g; c.start(); return c;
        })
        .def("__exit__", [](ImpedanceController& c, py::object, py::object,
                            py::object) {
            py::gil_scoped_release g; c.stop();
        });

    // ---- ForcePositionController: contact -> bounded pure-torque hold ----
    m.attr("FORCE_POSITION_MAX_HOLD_TORQUE_NM") =
        FORCE_POSITION_MAX_HOLD_TORQUE_NM;
    m.attr("FORCE_POSITION_MAX_MOTION_TORQUE_NM") =
        FORCE_POSITION_MAX_MOTION_TORQUE_NM;

    py::enum_<ForcePositionState>(m, "ForcePositionState")
        .value("IDLE",             ForcePositionState::Idle)
        .value("HOLDING_POSITION", ForcePositionState::HoldingPosition)
        .value("CLOSING",          ForcePositionState::Closing)
        .value("HOLDING_FORCE",    ForcePositionState::HoldingForce)
        .value("OPENING",          ForcePositionState::Opening)
        .value("FAULT",            ForcePositionState::Fault)
        .def("__str__", [](ForcePositionState s) { return to_string(s); });

    // Seven fields, down from sixteen. The contact-detection constants and the
    // position gains are firmware mirrors / measured values with one right
    // answer for this gripper, and live in detail::ForcePositionTuning on the
    // C++ side where a caller cannot reach them.
    py::class_<ForcePositionConfig>(m, "ForcePositionConfig")
        .def(py::init<>())
        .def_readwrite("grasp_torque_nm",      &ForcePositionConfig::grasp_torque_nm)
        .def_readwrite("close_speed_radps",    &ForcePositionConfig::close_speed_radps)
        .def_readwrite("hold_torque_limit_nm", &ForcePositionConfig::hold_torque_limit_nm)
        .def_readwrite("motion_torque_limit_nm", &ForcePositionConfig::motion_torque_limit_nm)
        .def_readwrite("status_timeout_ms",    &ForcePositionConfig::status_timeout_ms)
        .def_readwrite("motor_stream_hz",      &ForcePositionConfig::motor_stream_hz)
        .def_readwrite("close_preload_nm",     &ForcePositionConfig::close_preload_nm);

    py::class_<ForcePositionSnapshot>(m, "ForcePositionSnapshot")
        .def_readonly("running",               &ForcePositionSnapshot::running)
        .def_readonly("state",                 &ForcePositionSnapshot::state)
        .def_readonly("observation",           &ForcePositionSnapshot::observation)
        .def_readonly("target_position",       &ForcePositionSnapshot::target_position)
        .def_readonly("hold_position",         &ForcePositionSnapshot::hold_position)
        .def_readonly("grasp_torque_nm",       &ForcePositionSnapshot::grasp_torque_nm)
        .def_readonly("commanded_torque_nm",   &ForcePositionSnapshot::commanded_torque_nm)
        .def_readonly("hold_torque_limit_nm",  &ForcePositionSnapshot::hold_torque_limit_nm)
        .def_readonly("motion_torque_limit_nm", &ForcePositionSnapshot::motion_torque_limit_nm)
        .def_readonly("device_limit_nm",       &ForcePositionSnapshot::device_limit_nm)
        // 观测量，非控制状态 —— 每帧从命令与误差导出
        .def_readonly("holding",               &ForcePositionSnapshot::holding)
        .def_readonly("arrived",               &ForcePositionSnapshot::arrived)
        .def_readonly("fault_reason",          &ForcePositionSnapshot::fault_reason)
        .def("__repr__", [](const ForcePositionSnapshot& s) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "ForcePositionSnapshot(state=%s, pos=%.4f, torque=%.3fNm, "
                "command=%.3fNm, hold_limit=%.3fNm, motion_limit=%.3fNm, "
                "device_limit=%.3fNm, age=%.1fms)",
                to_string(s.state), s.observation.position, s.observation.torque,
                s.commanded_torque_nm, s.hold_torque_limit_nm,
                s.motion_torque_limit_nm, s.device_limit_nm,
                s.observation.age_ms);
            return std::string(buf);
        });

    py::class_<ForcePositionController>(m, "ForcePositionController")
        .def(py::init([](FollowerGripper& g, const ForcePositionConfig& cfg) {
                return std::make_unique<ForcePositionController>(g, cfg);
            }), py::arg("gripper"), py::arg("config") = ForcePositionConfig{},
            py::keep_alive<1, 2>())
        .def("start", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.start();
        })
        .def("stop", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.stop();
        })
        .def("release", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.release();
        })
        .def("set_target", [](ForcePositionController& c, float position,
                               const std::optional<float>& grasp_torque_nm) {
            py::gil_scoped_release g;
            if (grasp_torque_nm) c.set_target(position, *grasp_torque_nm);
            else                 c.set_target(position);
        }, py::arg("position"), py::arg("grasp_torque_nm") = std::nullopt)
        .def("hold_position", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.hold_position();
        })
        .def("reset", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.reset();
        })
        .def_property_readonly("running", &ForcePositionController::running)
        .def_property_readonly("state", &ForcePositionController::state)
        .def_property_readonly("config", [](const ForcePositionController& c) {
            return c.config();
        })
        .def("snapshot", [](const ForcePositionController& c) {
            py::gil_scoped_release g; return c.snapshot();
        })
        .def("__enter__", [](ForcePositionController& c) -> ForcePositionController& {
            py::gil_scoped_release g; c.start(); return c;
        })
        .def("__exit__", [](ForcePositionController& c, py::object, py::object,
                            py::object) {
            py::gil_scoped_release g; c.stop();
        });
}

}  // namespace xense::taccap::python

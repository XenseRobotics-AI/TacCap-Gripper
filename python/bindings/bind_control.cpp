// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: GripperObservation, ControlLoop, ForcePositionController
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

    // ---- ControlLoop: fixed-rate send/recv for embodied control ----------
    py::enum_<ControlLoop::SubmitPhase>(m, "SubmitPhase",
        "When ControlLoop puts its MIT frame on the wire.\n\n"
        "STREAM_LOCKED (default) submits once per received motor-status frame, "
        "so the write lands while the MCU is not transmitting; `hz` is then "
        "ignored and the submit rate follows motor_stream_hz. FREE_RUNNING "
        "submits on its own clock at `hz` and will occasionally write into the "
        "MCU's transmission, which costs status frames.")
        .value("FREE_RUNNING",  ControlLoop::SubmitPhase::FreeRunning)
        .value("STREAM_LOCKED", ControlLoop::SubmitPhase::StreamLocked);

    py::enum_<ControlLoop::StallAction>(m, "StallAction",
        "What ControlLoop does when the jaw is blocked.\n\n"
        "HOLD_POSITION (default) clamps the effective target at the position "
        "the jaw actually reached, so kp*error -- and therefore torque -- stops "
        "growing; commanding a target back the other way releases it. NONE is "
        "the unguarded behaviour: on firmware 1.1.5 nothing below the host "
        "bounds a blocked impedance target except the motor's 6 Nm 0x700B "
        "ceiling.")
        .value("NONE",          ControlLoop::StallAction::None)
        .value("HOLD_POSITION", ControlLoop::StallAction::HoldPosition);

    py::class_<ControlLoop>(m, "ControlLoop")
        .def(py::init([](FollowerGripper& g, unsigned hz, float kp, float kd,
                         float feedforward_torque, unsigned motor_stream_hz,
                         ControlLoop::SubmitPhase phase,
                         float max_position_torque_nm,
                         float rated_torque_nm, unsigned rated_hold_ms,
                         float rated_release_rad,
                         float stall_torque_nm, float stall_vel_radps,
                         unsigned stall_hold_ms,
                         ControlLoop::StallAction stall_action) {
                ControlLoop::Config c;
                c.hz = hz; c.kp = kp; c.kd = kd;
                c.feedforward_torque = feedforward_torque;
                c.motor_stream_hz = motor_stream_hz;
                c.phase = phase;
                c.max_position_torque_nm = max_position_torque_nm;
                c.rated_torque_nm   = rated_torque_nm;
                c.rated_hold_ms     = rated_hold_ms;
                c.rated_release_rad = rated_release_rad;
                c.stall_torque_nm = stall_torque_nm;
                c.stall_vel_radps = stall_vel_radps;
                c.stall_hold_ms   = stall_hold_ms;
                c.stall_action    = stall_action;
                return std::make_unique<ControlLoop>(g, c);
            }),
            py::arg("gripper"), py::arg("hz") = 100u,
            py::arg("kp") = 20.0f, py::arg("kd") = 1.0f,
            py::arg("feedforward_torque") = 0.0f,
            py::arg("motor_stream_hz") = 100u,
            py::arg("phase") = ControlLoop::SubmitPhase::StreamLocked,
            py::arg("max_position_torque_nm") = 1.5f,
            py::arg("rated_torque_nm") = 2.0f,
            py::arg("rated_hold_ms") = 20u,
            py::arg("rated_release_rad") = 0.05f,
            py::arg("stall_torque_nm") = 1.2f,
            py::arg("stall_vel_radps") = 0.15f,
            py::arg("stall_hold_ms") = 60u,
            py::arg("stall_action") = ControlLoop::StallAction::HoldPosition,
            py::keep_alive<1, 2>())   // keep the gripper alive while the loop lives
        .def("start", [](ControlLoop& l) { py::gil_scoped_release g; l.start(); })
        .def("stop",  [](ControlLoop& l) { py::gil_scoped_release g; l.stop(); })
        .def_property_readonly("running", &ControlLoop::running)
        .def("set_target", [](ControlLoop& l, float p) {
            py::gil_scoped_release g; l.set_target(p);
        }, py::arg("position"))
        .def("set_gains", [](ControlLoop& l, float kp, float kd, float ff) {
            py::gil_scoped_release g; l.set_gains(kp, kd, ff);
        }, py::arg("kp"), py::arg("kd"), py::arg("feedforward_torque") = 0.0f)
        .def_property_readonly("target", &ControlLoop::target)
        .def("observation", [](const ControlLoop& l) {
            py::gil_scoped_release g; return l.observation();
        })
        .def_property_readonly("torque_capped", &ControlLoop::torque_capped)
        .def_property_readonly("torque_caps",   &ControlLoop::torque_caps)
        .def_property_readonly("stalled",      &ControlLoop::stalled)
        .def_property_readonly("stall_trips",  &ControlLoop::stall_trips)
        .def_property_readonly("submit_hz",    &ControlLoop::submit_hz)
        .def_property_readonly("submit_count", &ControlLoop::submit_count)
        .def("__enter__", [](ControlLoop& l) -> ControlLoop& {
            py::gil_scoped_release g; l.start(); return l;
        })
        .def("__exit__", [](ControlLoop& l, py::object, py::object, py::object) {
            py::gil_scoped_release g; l.stop();
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

    py::class_<ForcePositionConfig>(m, "ForcePositionConfig")
        .def(py::init<>())
        .def_readwrite("close_position",       &ForcePositionConfig::close_position)
        .def_readwrite("close_speed_radps",    &ForcePositionConfig::close_speed_radps)
        .def_readwrite("grasp_torque_nm",      &ForcePositionConfig::grasp_torque_nm)
        .def_readwrite("hold_torque_limit_nm", &ForcePositionConfig::hold_torque_limit_nm)
        .def_readwrite("motion_torque_limit_nm", &ForcePositionConfig::motion_torque_limit_nm)
        .def_readwrite("contact_torque_nm",    &ForcePositionConfig::contact_torque_nm)
        .def_readwrite("contact_vel_radps",    &ForcePositionConfig::contact_vel_radps)
        .def_readwrite("contact_vel_ratio",    &ForcePositionConfig::contact_vel_ratio)
        .def_readwrite("contact_moved_rad",    &ForcePositionConfig::contact_moved_rad)
        .def_readwrite("position_kp",          &ForcePositionConfig::position_kp)
        .def_readwrite("position_kd",          &ForcePositionConfig::position_kd)
        .def_readwrite("brake_distance_rad",   &ForcePositionConfig::brake_distance_rad)
        .def_readwrite("close_endpoint_tolerance_rad",
                       &ForcePositionConfig::close_endpoint_tolerance_rad)
        .def_readwrite("contact_samples",      &ForcePositionConfig::contact_samples)
        .def_readwrite("startup_guard_ms",     &ForcePositionConfig::startup_guard_ms)
        .def_readwrite("status_timeout_ms",    &ForcePositionConfig::status_timeout_ms)
        .def_readwrite("motor_stream_hz",      &ForcePositionConfig::motor_stream_hz);

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
        .def_readonly("contact_count",         &ForcePositionSnapshot::contact_count)
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

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
    py::class_<GripperObservation>(m, "GripperObservation",
        "One motor-status frame, as the active controller last saw it.\n\n"
        "Read it from a controller snapshot rather than polling the motor: it is\n"
        "refreshed from the status stream the controller already owns, so reading\n"
        "costs no bus traffic. `valid` is False before the first frame and after\n"
        "the stream goes stale, in which case every other field is meaningless.")
        .def_readonly("valid",    &GripperObservation::valid,
                      "False until the first frame arrives, and again once the stream goes stale.")
        .def_readonly("position", &GripperObservation::position,
                      "Normalized opening, 0 = closed, 1 = open. Needs a calibrated gripper.")
        .def_readonly("velocity", &GripperObservation::velocity, "rad/s at the motor.")
        .def_readonly("torque",   &GripperObservation::torque, "Measured torque, N*m.")
        .def_readonly("raw_pos",  &GripperObservation::raw_pos,
                      "Motor angle in radians, before the normalized position map.")
        .def_readonly("status",   &GripperObservation::status,
                      "Motor status word; see protocol.MotorStatusBit.")
        .def_readonly("motor_temp_c", &GripperObservation::motor_temp_c, "Motor temperature, C.")
        .def_readonly("seq",      &GripperObservation::seq,
                      "Frame counter; compare two reads to tell a fresh frame from a repeat.")
        .def_readonly("age_ms",   &GripperObservation::age_ms,
                      "Milliseconds since the frame arrived. Rising age means the stream stalled.")
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
    // The third rating, and the one the other two get mistaken for: what a
    // BLOCKED jaw may hold indefinitely. Both controller budgets default to it.
    m.attr("MOTOR_STALL_CONT_TORQUE_NM") = MOTOR_STALL_CONT_TORQUE_NM;
    m.attr("MOTOR_APPROACH_SPEED_RADPS") = MOTOR_APPROACH_SPEED_RADPS;
    m.attr("MOTOR_MAX_APPROACH_SPEED_RADPS") = MOTOR_MAX_APPROACH_SPEED_RADPS;
    m.attr("MOTOR_ABSOLUTE_TORQUE_CEILING_NM") = MOTOR_ABSOLUTE_TORQUE_CEILING_NM;

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

    py::class_<ImpedanceConfig> impedance_config(m, "ImpedanceConfig",
        "Tuning for ImpedanceController. Validated by the constructor, which\n"
        "raises ValueError rather than clamping.");
    impedance_config
        .def(py::init<>())
        .def_readwrite("kp",                &ImpedanceConfig::kp,
                       "Position stiffness, N*m/rad. Must be > 0.")
        .def_readwrite("kd",                &ImpedanceConfig::kd,
                       "Velocity damping, N*m*s/rad.")
        .def_readwrite("feedforward_torque", &ImpedanceConfig::feedforward_torque,
                       "Constant torque bias added to every frame, N*m. It counts\n"
                       "toward the backstop: max_position_torque_nm + |this| must\n"
                       "stay below rated_torque_nm.")
        .def_readwrite("max_position_torque_nm",
                       &ImpedanceConfig::max_position_torque_nm,
                       "Torque budget, N*m, applied as an error clamp on the commanded\n"
                       "position. This IS the protection and the grip: a blocked jaw\n"
                       "saturates here and holds indefinitely, so it must be a torque the\n"
                       "motor can sustain forever. It also sets the approach speed, about\n"
                       "this value / kd rad/s. 0 disables the clamp (not recommended).")
        .def_readwrite("rated_torque_nm",   &ImpedanceConfig::rated_torque_nm,
                       "Backstop on MEASURED torque, N*m -- not the grip. Above it the\n"
                       "controller commands kp=kd=0 and holds the budget. Must exceed\n"
                       "max_position_torque_nm; capped at MOTOR_RATED_TORQUE_NM because the\n"
                       "hold is indefinite. 0 disables it.")
        .def_readwrite("status_timeout_ms", &ImpedanceConfig::status_timeout_ms,
                       "A status stream older than this faults the controller and commands\n"
                       "zero torque.")
        .def_readwrite("motor_stream_hz",   &ImpedanceConfig::motor_stream_hz,
                       "Motor-status rate the controller requests, 1..100 Hz. One command\n"
                       "is submitted per status frame, so this is also the control rate.");

    impedance_config.def_static("for_spec", &ImpedanceConfig::for_spec, py::arg("spec"),
        "按设备实际装的电机给出默认值(传 motor.get_spec() 的结果)。\n\n"
        "类里的成员默认值是编译期烘进去的 EL05 数,**换了电机就不对**:RS00 上它把\n"
        "持续夹持限在 1.1 Nm,而那颗电机能无限期维持 3.6 —— 换它的理由有三分之二\n"
        "就这么没了。\n\n"
        "    cfg = ImpedanceConfig.for_spec(g.motor.get_spec())\n\n"
        "三个值必须一起动:预算取**连续堵转额定**(被挡住的爪子无限期坐在它上面),\n"
        "天花板取**旋转额定**(必须高于预算,否则正常夹持就跳闸、位置控制丢失),\n"
        "kd = 预算 / MOTOR_APPROACH_SPEED_RADPS —— 接近速度是 预算/kd,只提预算\n"
        "会把接近速度一起提上去,这正是「直接把力矩调大」不对的地方。"
    );

    py::class_<ImpedanceSnapshot>(m, "ImpedanceSnapshot",
        "A consistent view of ImpedanceController, taken under one lock.\n\n"
        "Every field below describes the same instant. Reading guards one property\n"
        "at a time can return a combination that never existed.")
        .def_readonly("running",             &ImpedanceSnapshot::running,
                      "True between start() and stop().")
        .def_readonly("state",               &ImpedanceSnapshot::state)
        .def_readonly("observation",         &ImpedanceSnapshot::observation,
                      "The motor-status frame this snapshot was computed from.")
        .def_readonly("target_position",     &ImpedanceSnapshot::target_position,
                      "Normalized target as commanded by set_target().")
        .def_readonly("effective_position",  &ImpedanceSnapshot::effective_position,
                      "Normalized target actually sent, after the error clamp. It trailing\n"
                      "target_position means the clamp is binding -- travel or contact.")
        .def_readonly("commanded_torque_nm", &ImpedanceSnapshot::commanded_torque_nm,
                      "Magnitude of the torque the frame asks for, N*m. Requested, not measured;\n"
                      "for the measured value read observation.torque.")
        .def_readonly("torque_capped",       &ImpedanceSnapshot::torque_capped,
                      "True while the rated_torque_nm backstop is holding the output down.")
        .def_readonly("torque_caps",         &ImpedanceSnapshot::torque_caps,
                      "How many times the backstop has engaged since start().")
        .def_readonly("device_limit_nm",     &ImpedanceSnapshot::device_limit_nm,
                      "The motor's own persisted torque limit (0x700B) read at start().")
        .def_readonly("fault_reason",        &ImpedanceSnapshot::fault_reason,
                      "Why the controller faulted; empty when it has not. The first cause is\n"
                      "kept, not the most recent.")
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

    py::class_<ImpedanceController>(m, "ImpedanceController",
        "Position tracking with a bounded torque budget, run on a background thread.\n\n"
        "Use it to follow a position -- teleop, a trajectory, a leader gripper. To\n"
        "grasp something, ForcePositionController is the better fit.\n\n"
        "It owns the motor-status stream and submits one command per status frame.\n"
        "While it runs, do not call Motor.set_position / set_velocity / set_torque /\n"
        "set_impedance or start a second controller on the same gripper: they write\n"
        "to the same bus and the last frame wins.\n\n"
        "Enabling the motor is the caller's job and is separate from start().\n\n"
        "    cfg = ImpedanceConfig()\n"
        "    with ImpedanceController(g, cfg) as c:   # start() / stop()\n"
        "        g.motor.enable()\n"
        "        c.set_target(0.0)\n"
        "        print(c.snapshot())")
        .def(py::init([](FollowerGripper& g, const ImpedanceConfig& cfg) {
                return std::make_unique<ImpedanceController>(g, cfg);
            }), py::arg("gripper"), py::arg("config") = ImpedanceConfig{},
            py::keep_alive<1, 2>(),
            "Bind a controller to a gripper. The config is validated here, so a bad\n"
            "combination raises ValueError before anything moves. The gripper is kept\n"
            "alive for as long as the controller is.")
        .def("start", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.start();
        }, "Start the status stream and the submit thread.\n\n"
           "The target is seeded at the jaw's current position, so starting against an\n"
           "already-enabled motor cannot step it.")
        .def("stop", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.stop();
        }, "Stop the thread, command zero torque, and DISABLE the motor.\n\n"
           "Disabling is deliberate: a zero-stiffness frame de-energizes nothing, and\n"
           "the firmware's host watchdog would then fire and switch the run mode out\n"
           "from under the next session. The observation goes invalid, since nothing\n"
           "maintains it any more. Returns even if the device vanished mid-run.")
        .def_property_readonly("running", &ImpedanceController::running,
                               "True between start() and stop().")
        .def("set_target", [](ImpedanceController& c, float p) {
            py::gil_scoped_release g; c.set_target(p);
        }, py::arg("position"),
           "Set the normalized target, 0 = closed, 1 = open.\n\n"
           "Non-blocking: it moves a setpoint the submit thread reads. Safe to stream\n"
           "every frame. Only valid while running.")
        .def("set_gains", [](ImpedanceController& c, float kp, float kd, float ff) {
            py::gil_scoped_release g; c.set_gains(kp, kd, ff);
        }, py::arg("kp"), py::arg("kd"), py::arg("feedforward_torque") = 0.0f,
           "Change kp (N*m/rad), kd (N*m*s/rad) and the feed-forward bias (N*m) while\n"
           "running. Rejected by the same rule the config is validated against.")
        .def("reset", [](ImpedanceController& c) {
            py::gil_scoped_release g; c.reset();
        }, "Leave FAULT and reseed the target at the current position.\n\n"
           "Clear the motor fault first (Motor.clear_fault); this only clears the\n"
           "controller's own state.")
        .def_property_readonly("state", [](const ImpedanceController& c) {
            py::gil_scoped_release g; return c.state();
        }, "Current ImpedanceState. Prefer snapshot() when reading more than one field.")
        .def("snapshot", [](const ImpedanceController& c) {
            py::gil_scoped_release g; return c.snapshot();
        }, "One consistent view of state, observation and command, taken under one lock.")
        .def_property_readonly("config", &ImpedanceController::config,
                               "The validated config in force. Gain changes made through\n"
                               "set_gains() are reflected here.")
        .def("__enter__", [](ImpedanceController& c) -> ImpedanceController& {
            py::gil_scoped_release g; c.start(); return c;
        }, "Calls start().")
        .def("__exit__", [](ImpedanceController& c, py::object, py::object,
                            py::object) {
            py::gil_scoped_release g; c.stop();
        }, "Calls stop(), including on an exception.");

    // ---- ForcePositionController: contact -> bounded pure-torque hold ----
    m.attr("FORCE_POSITION_MAX_HOLD_TORQUE_NM") =
        FORCE_POSITION_MAX_HOLD_TORQUE_NM;
    m.attr("FORCE_POSITION_MAX_MOTION_TORQUE_NM") =
        FORCE_POSITION_MAX_MOTION_TORQUE_NM;

    py::enum_<ForcePositionState>(m, "ForcePositionState",
        "Observed phase of the move. CLOSING/OPENING are travel, HOLDING_FORCE is a\n"
        "grip sitting at the budget, HOLDING_POSITION is a parked jaw.")
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
    py::class_<ForcePositionConfig> fp_config(m, "ForcePositionConfig",
        "Tuning for ForcePositionController. Validated by the constructor and again\n"
        "at start(), against the motor's own persisted limit.");
    fp_config
        .def(py::init<>())
        .def_readwrite("grasp_torque_nm",      &ForcePositionConfig::grasp_torque_nm,
                       "Torque budget for the grip, N*m. The default is the EL05's continuous\n"
                       "rating. Configuring more than the firmware's envelope allows does not\n"
                       "produce more: the firmware clamps it back and only the heating is real.")
        .def_readwrite("close_speed_radps",    &ForcePositionConfig::close_speed_radps,
                       "Advance rate of the SETPOINT RAMP during travel, rad/s at the motor --\n"
                       "not a velocity command. Not independent of grasp_torque_nm: the damping\n"
                       "gain is grasp/close_speed and saturates, so too low a speed stalls the\n"
                       "jaw below the torque you asked for. Rejected rather than softened.")
        .def_readwrite("hold_torque_limit_nm", &ForcePositionConfig::hold_torque_limit_nm,
                       "Upper bound accepted for grasp_torque_nm, N*m. Clamps nothing at runtime.")
        .def_readwrite("motion_torque_limit_nm", &ForcePositionConfig::motion_torque_limit_nm,
                       "Ceiling outside the grip budget, N*m; measured torque past it faults the\n"
                       "controller. Cross-checked against the motor's 0x700B at start(), device wins.")
        .def_readwrite("status_timeout_ms",    &ForcePositionConfig::status_timeout_ms,
                       "A status stream older than this faults the controller and commands\n"
                       "zero torque.")
        .def_readwrite("motor_stream_hz",      &ForcePositionConfig::motor_stream_hz,
                       "Motor-status rate the controller requests, 1..100 Hz. One command is\n"
                       "submitted per status frame.")
        .def_readwrite("close_preload_nm",     &ForcePositionConfig::close_preload_nm,
                       "Feed-forward applied only while holding the CLOSED endpoint, N*m, to seat\n"
                       "the jaw against its mechanical stop and take up backlash. Measured: 0.15\n"
                       "seats it fully and 0.50 moves it no further, so raising this buys nothing.");

    fp_config.def_static("for_spec", &ForcePositionConfig::for_spec, py::arg("spec"),
        "按设备实际装的电机给出默认值(传 motor.get_spec() 的结果)。\n\n"
        "    cfg = ForcePositionConfig.for_spec(g.motor.get_spec())\n\n"
        "grasp_torque_nm 取连续堵转额定、hold_torque_limit_nm 取旋转额定、\n"
        "motion_torque_limit_nm 取力矩量程、close_speed_radps 取\n"
        "MOTOR_APPROACH_SPEED_RADPS。\n\n"
        "**close_preload_nm 不派生**:0.25 Nm 是在某一个机构上扫出来的落座力矩\n"
        "(0.15 就能坐死,0.50 不再前进),它是爪子和减速箱的属性,不跟着电机走 ——\n"
        "按电机推它等于凭空编一个数。"
    );

    py::class_<ForcePositionSnapshot>(m, "ForcePositionSnapshot",
        "A consistent view of ForcePositionController, taken under one lock.")
        .def_readonly("running",               &ForcePositionSnapshot::running,
                      "True between start() and stop().")
        .def_readonly("state",                 &ForcePositionSnapshot::state)
        .def_readonly("observation",           &ForcePositionSnapshot::observation,
                      "The motor-status frame this snapshot was computed from.")
        .def_readonly("target_position",       &ForcePositionSnapshot::target_position,
                      "Normalized target as commanded by set_target().")
        .def_readonly("hold_position",         &ForcePositionSnapshot::hold_position,
                      "Normalized position being held; set by hold_position().")
        .def_readonly("grasp_torque_nm",       &ForcePositionSnapshot::grasp_torque_nm,
                      "Torque budget in force, N*m -- the per-target override when one was given.")
        .def_readonly("commanded_torque_nm",   &ForcePositionSnapshot::commanded_torque_nm,
                      "Torque this frame asks for, N*m. Requested, not measured.")
        .def_readonly("hold_torque_limit_nm",  &ForcePositionSnapshot::hold_torque_limit_nm)
        .def_readonly("motion_torque_limit_nm", &ForcePositionSnapshot::motion_torque_limit_nm)
        .def_readonly("device_limit_nm",       &ForcePositionSnapshot::device_limit_nm,
                      "The motor's own persisted torque limit (0x700B) read at start().")
        // 观测量，非控制状态 —— 每帧从命令与误差导出
        .def_readonly("holding",               &ForcePositionSnapshot::holding,
                      "Derived each frame from the command and the error, not a state: the jaw\n"
                      "is pressing rather than travelling.")
        .def_readonly("arrived",               &ForcePositionSnapshot::arrived,
                      "Derived each frame: the jaw has reached the commanded position.")
        .def_readonly("fault_reason",          &ForcePositionSnapshot::fault_reason,
                      "Why the controller faulted; empty when it has not.")
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

    py::class_<ForcePositionController>(m, "ForcePositionController",
        "Bounded-torque position control for grasping, run on a background thread.\n\n"
        "One control law for the whole move: a setpoint ramp carries the jaw, and the\n"
        "PD request is error-clamped against the grasp budget, so closing onto an\n"
        "object saturates at that budget and holds. It does NOT detect contact --\n"
        "saturation is contact, and the MCU already runs the same stall test at 500 Hz.\n\n"
        "Same interface and the same bus rules as ImpedanceController: it owns the\n"
        "status stream, submits one command per frame, and must not share a gripper\n"
        "with another controller or with the Motor.set_* primitives.\n\n"
        "    with ForcePositionController(g) as c:   # start() / stop()\n"
        "        g.motor.enable()                    # start BEFORE enable, see start()\n"
        "        c.set_target(0.0)                   # close onto the object\n"
        "        c.release()                         # open with bounded damping")
        .def(py::init([](FollowerGripper& g, const ForcePositionConfig& cfg) {
                return std::make_unique<ForcePositionController>(g, cfg);
            }), py::arg("gripper"), py::arg("config") = ForcePositionConfig{},
            py::keep_alive<1, 2>(),
            "Bind a controller to a gripper. The config is validated here, so a bad\n"
            "combination raises ValueError before anything moves. The gripper is kept\n"
            "alive for as long as the controller is.")
        .def("start", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.start();
        }, "Start the status stream and the submit thread.\n\n"
           "Call this BEFORE Motor.enable(): start() reads the motor's persisted torque\n"
           "limit (0x700B) and checks the config against it, which is a check worth\n"
           "making before the motor can move.")
        .def("stop", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.stop();
        }, "Stop the thread, command zero torque, and DISABLE the motor.\n\n"
           "Disabling is deliberate -- see ImpedanceController.stop for the measurement.\n"
           "Returns even if the device vanished mid-run.")
        .def("release", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.release();
        }, "Open toward 1.0 under bounded velocity damping.\n\n"
           "Preferred over set_target(1.0) for letting go: a position step would open\n"
           "as fast as the budget allows, this bounds the speed instead.")
        .def("set_target", [](ForcePositionController& c, float position,
                               const std::optional<float>& grasp_torque_nm) {
            py::gil_scoped_release g;
            if (grasp_torque_nm) c.set_target(position, *grasp_torque_nm);
            else                 c.set_target(position);
        }, py::arg("position"), py::arg("grasp_torque_nm") = std::nullopt,
           "Set the normalized target, 0 = closed, 1 = open, optionally overriding the\n"
           "grasp budget (N*m) for this move.\n\n"
           "Non-blocking, and it only moves the setpoint: there is no motion state to\n"
           "disturb, so streaming a target every frame is fine. Only valid while running.")
        .def("hold_position", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.hold_position();
        }, "Cancel travel and hold the latest measured position.")
        .def("reset", [](ForcePositionController& c) {
            py::gil_scoped_release g; c.reset();
        }, "Leave FAULT. Clear the motor fault first (Motor.clear_fault); this only\n"
           "clears the controller's own state.")
        .def_property_readonly("running", &ForcePositionController::running,
                               "True between start() and stop().")
        .def_property_readonly("state", &ForcePositionController::state,
                               "Current ForcePositionState. Prefer snapshot() for more than one field.")
        .def_property_readonly("config", [](const ForcePositionController& c) {
            return c.config();
        }, "The validated config in force.")
        .def("snapshot", [](const ForcePositionController& c) {
            py::gil_scoped_release g; return c.snapshot();
        }, "One consistent view of state, observation and command, taken under one lock.")
        .def("__enter__", [](ForcePositionController& c) -> ForcePositionController& {
            py::gil_scoped_release g; c.start(); return c;
        }, "Calls start().")
        .def("__exit__", [](ForcePositionController& c, py::object, py::object,
                            py::object) {
            py::gil_scoped_release g; c.stop();
        }, "Calls stop(), including on an exception.");
}

}  // namespace xense::taccap::python

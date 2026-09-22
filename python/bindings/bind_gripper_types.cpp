// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: GripperConfig, GripperEnvelope, power-on auto-cal records
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_gripper_types(py::module_& m) {
    using namespace xense::taccap;
    // ---- V1.7 follower (slave) types ------------------------------------
    py::enum_<protocol::MotorProtocol>(m, "MotorProtocol")
        .value("Private", protocol::MotorProtocol::Private)
        .value("Mit",     protocol::MotorProtocol::Mit);

    // V2.2 — MotorStatusExt.stop_reason / MotorFaultReport.stop_reason. The
    // fault/monitor *bit masks* stay C++-only, matching MotorStatusBit: Python
    // gets the raw integers and masks them itself.
    py::enum_<protocol::MotorStopReason>(m, "MotorStopReason")
        .value("None_",        protocol::MotorStopReason::None)
        .value("Disable",      protocol::MotorStopReason::Disable)
        .value("Emergency",    protocol::MotorStopReason::Emergency)
        .value("ClearFault",   protocol::MotorStopReason::ClearFault)
        .value("LimitStall",   protocol::MotorStopReason::LimitStall)
        .value("ControlError", protocol::MotorStopReason::ControlError)
        .value("HostTimeout",  protocol::MotorStopReason::HostTimeout);

    py::class_<protocol::GripperConfig>(m, "GripperConfig")
        .def(py::init([]() {
            protocol::GripperConfig c{};
            c.magic   = protocol::GRIPPER_CONFIG_MAGIC;
            c.version = protocol::GRIPPER_CONFIG_VERSION;
            c.flags   = protocol::GripperConfigFlag::Valid;
            return c;
        }))
        .def_readwrite("magic",        &protocol::GripperConfig::magic)
        .def_readwrite("version",      &protocol::GripperConfig::version)
        .def_readwrite("flags",        &protocol::GripperConfig::flags)
        .def_readwrite("max_open_rad", &protocol::GripperConfig::max_open_rad)
        .def_readwrite("min_open_rad", &protocol::GripperConfig::min_open_rad)
        .def("__repr__", [](const protocol::GripperConfig& c) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperConfig(flags=0x%04x, max_open=%.4frad, min_open=%.4frad)",
                c.flags, c.max_open_rad, c.min_open_rad);
            return std::string(buf);
        });

    // ---- GripperAutoCalConfig (V1.9 power-on auto-cal) -------------------
    py::module_ envmod = m;
    py::class_<protocol::GripperEnvelope>(envmod, "GripperEnvelope",
        "Motion safety envelope, carried inside the GripperConfig record "
        "(Cmd 0x66/0x67 - no new command, no payload size change).\n\n"
        "The firmware clamps every MIT frame against it: commanded position to "
        "within peak_torque_nm/kp of the measured position, feed-forward torque "
        "to cont_torque_nm, velocity feed-forward to max_velocity_rad_s. It "
        "lives in firmware because the host link is 100 Hz, phase-locked, and "
        "cannot be polled while controlling.")
        .def(py::init<>())
        .def_readwrite("cont_torque_nm",     &protocol::GripperEnvelope::cont_torque_nm)
        .def_readwrite("peak_torque_nm",     &protocol::GripperEnvelope::peak_torque_nm)
        .def_readwrite("temp_derate_start_c", &protocol::GripperEnvelope::temp_derate_start_c)
        .def_readwrite("temp_wall_c",        &protocol::GripperEnvelope::temp_wall_c)
        .def_readwrite("flags",              &protocol::GripperEnvelope::flags)
        .def("__repr__", [](const protocol::GripperEnvelope& e) {
            char buf[192];
            std::snprintf(buf, sizeof(buf),
                "GripperEnvelope(cont=%.3f Nm, peak=%.3f Nm, temp=%u/%uC, flags=0x%04x%s)",
                e.cont_torque_nm, e.peak_torque_nm,
                (unsigned)e.temp_derate_start_c, (unsigned)e.temp_wall_c, e.flags,
                (e.flags & 0x0002) ? " ENFORCE" : " inactive");
            return std::string(buf);
        });
    envmod.attr("GRIPPER_ENVELOPE_VALID")   = (uint16_t)0x0001;
    envmod.attr("GRIPPER_ENVELOPE_LAYOUT_VERSION") = protocol::GripperEnvelopeFlag::LayoutVersion;
    envmod.attr("GRIPPER_ENVELOPE_ENFORCE") = (uint16_t)0x0002;

    py::class_<protocol::GripperAutoCalConfig>(m, "GripperAutoCalConfig")
        .def(py::init([]() {
            protocol::GripperAutoCalConfig c{};
            c.magic   = protocol::GRIPPER_AUTO_CAL_MAGIC;
            c.version = protocol::GRIPPER_AUTO_CAL_VERSION;
            c.flags   = protocol::GripperAutoCalFlag::Valid;
            return c;
        }))
        .def_readwrite("magic",                &protocol::GripperAutoCalConfig::magic)
        .def_readwrite("version",              &protocol::GripperAutoCalConfig::version)
        .def_readwrite("flags",                &protocol::GripperAutoCalConfig::flags)
        .def_readwrite("close_stall_torque_nm", &protocol::GripperAutoCalConfig::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",  &protocol::GripperAutoCalConfig::open_stall_torque_nm)
        .def_readwrite("close_speed_rad_s",    &protocol::GripperAutoCalConfig::close_speed_rad_s)
        .def_readwrite("open_speed_rad_s",     &protocol::GripperAutoCalConfig::open_speed_rad_s)
        .def_readwrite("stall_hold_ms",        &protocol::GripperAutoCalConfig::stall_hold_ms)
        .def_readwrite("startup_delay_ms",     &protocol::GripperAutoCalConfig::startup_delay_ms)
        .def_readwrite("post_zero_delay_ms",   &protocol::GripperAutoCalConfig::post_zero_delay_ms)
        .def_readwrite("close_confirm_count",  &protocol::GripperAutoCalConfig::close_confirm_count)
        .def_readwrite("open_confirm_count",   &protocol::GripperAutoCalConfig::open_confirm_count)
        .def("__repr__", [](const protocol::GripperAutoCalConfig& c) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalConfig(flags=0x%04x, close_torque=%.3f, open_torque=%.3f)",
                c.flags, c.close_stall_torque_nm, c.open_stall_torque_nm);
            return std::string(buf);
        });

    // ---- GripperAutoCalStallParam / …Ex (V2.2 partial 0x68 write) ---------
    // Patch just the stall-detection fields; speeds, flags and the magic/version
    // header keep their stored values. No magic/version to fill in — the
    // firmware supplies them.
    py::class_<protocol::GripperAutoCalStallParam>(m, "GripperAutoCalStallParam")
        .def(py::init([]() { return protocol::GripperAutoCalStallParam{}; }))
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::open_stall_torque_nm)
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParam::stall_hold_ms)
        .def("__repr__", [](const protocol::GripperAutoCalStallParam& p) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalStallParam(close=%.3fNm, open=%.3fNm, hold=%ums)",
                p.close_stall_torque_nm, p.open_stall_torque_nm, p.stall_hold_ms);
            return std::string(buf);
        });

    py::class_<protocol::GripperAutoCalStallParamEx>(m, "GripperAutoCalStallParamEx")
        .def(py::init([]() { return protocol::GripperAutoCalStallParamEx{}; }))
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::open_stall_torque_nm)
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParamEx::stall_hold_ms)
        .def_readwrite("startup_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::startup_delay_ms)
        .def_readwrite("post_zero_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::post_zero_delay_ms)
        .def_readwrite("close_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::close_confirm_count)
        .def_readwrite("open_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::open_confirm_count)
        .def("__repr__", [](const protocol::GripperAutoCalStallParamEx& p) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalStallParamEx(close=%.3fNm, open=%.3fNm, hold=%ums, "
                "startup_delay=%ums)",
                p.close_stall_torque_nm, p.open_stall_torque_nm,
                p.stall_hold_ms, p.startup_delay_ms);
            return std::string(buf);
        });
}

}  // namespace xense::taccap::python

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: Motor, its status/fault records, Encoder, Diagnostics
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_motor(py::module_& m) {
    using namespace xense::taccap;
    // ---- MotorPrivateParam (V1.9+ private-protocol param GET response) ----
    py::class_<protocol::MotorPrivateParam>(m, "MotorPrivateParam")
        .def_readonly("index",     &protocol::MotorPrivateParam::index)
        .def_readonly("type",      &protocol::MotorPrivateParam::type)     // 1=u8, 2=f32
        .def_readonly("access",    &protocol::MotorPrivateParam::access)   // 0x01=R, 0x02=W
        .def_readonly("raw_value", &protocol::MotorPrivateParam::raw_value)
        .def_property_readonly("as_float", [](const protocol::MotorPrivateParam& p) {
            float f; std::memcpy(&f, &p.raw_value, 4); return f;
        })
        .def("__repr__", [](const protocol::MotorPrivateParam& p) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "MotorPrivateParam(index=0x%04x, type=%u, access=0x%02x, raw=0x%08x)",
                p.index, p.type, p.access, p.raw_value);
            return std::string(buf);
        });

    // ---- Diagnostics: UART counters + log control (fw 1.1.3 / 1.1.4) ----
    py::class_<protocol::UartStats>(m, "UartStats",
        "Free-running firmware UART counters since MCU boot (Cmd 0x54).\n\n"
        "Take a reading before and after a measurement window and subtract.\n"
        "tx_bytes_ok / tx_calls_ok count only what the firmware's transmit call\n"
        "accepted, i.e. what reached the MCU's transmit register -- compare them\n"
        "against what the host decoded to tell 'the MCU never sent it' apart from\n"
        "'it was lost after leaving the MCU'.\n\n"
        "log_dropped reads 0 on firmware 1.1.3, which had no such field; that is\n"
        "not distinguishable from a genuine zero.")
        .def_readonly("tx_bytes_ok",     &protocol::UartStats::tx_bytes_ok)
        .def_readonly("tx_calls_ok",     &protocol::UartStats::tx_calls_ok)
        .def_readonly("tx_fail_timeout", &protocol::UartStats::tx_fail_timeout)
        .def_readonly("tx_fail_other",   &protocol::UartStats::tx_fail_other)
        .def_readonly("rx_bytes",        &protocol::UartStats::rx_bytes)
        .def_readonly("rx_overflow",     &protocol::UartStats::rx_overflow)
        .def_readonly("debug_tx_bytes",  &protocol::UartStats::debug_tx_bytes)
        .def_readonly("rb_used",         &protocol::UartStats::rb_used)
        .def_readonly("rb_free",         &protocol::UartStats::rb_free)
        .def_readonly("log_dropped",     &protocol::UartStats::log_dropped)
        .def("__repr__", [](const protocol::UartStats& s) {
            char buf[220];
            std::snprintf(buf, sizeof(buf),
                "UartStats(tx_calls_ok=%u, tx_bytes_ok=%u, tx_fail_timeout=%u, "
                "rx_overflow=%u, debug_tx_bytes=%u, log_dropped=%u)",
                static_cast<unsigned>(s.tx_calls_ok),
                static_cast<unsigned>(s.tx_bytes_ok),
                static_cast<unsigned>(s.tx_fail_timeout),
                static_cast<unsigned>(s.rx_overflow),
                static_cast<unsigned>(s.debug_tx_bytes),
                static_cast<unsigned>(s.log_dropped));
            return std::string(buf);
        });

    py::class_<protocol::LogConfig>(m, "LogConfig")
        .def_readonly("level",       &protocol::LogConfig::level)
        .def_readonly("output_mask", &protocol::LogConfig::output_mask)
        .def("__repr__", [](const protocol::LogConfig& c) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "LogConfig(level=%u, output_mask=0x%02X)",
                          c.level, c.output_mask);
            return std::string(buf);
        });

    py::enum_<protocol::LogLevel>(m, "LogLevel")
        .value("NONE",    protocol::LogLevel::None)
        .value("ERROR",   protocol::LogLevel::Error)
        .value("WARN",    protocol::LogLevel::Warn)
        .value("INFO",    protocol::LogLevel::Info)
        .value("DEBUG",   protocol::LogLevel::Debug)
        .value("VERBOSE", protocol::LogLevel::Verbose);

    m.attr("LOG_OUTPUT_NONE") = py::int_(protocol::LogOutput::None);
    m.attr("LOG_OUTPUT_UART") = py::int_(protocol::LogOutput::Uart);

    py::class_<Diagnostics>(m, "Diagnostics",
        "Firmware UART counters and log control. Available on both gripper\n"
        "roles; needs firmware 1.1.3 (counters) / 1.1.4 (log control).")
        .def("uart_stats", [](Diagnostics& self, unsigned timeout_ms) {
                py::gil_scoped_release g;
                return self.uart_stats(std::chrono::milliseconds(timeout_ms));
            }, py::arg("timeout_ms") = 100u)
        .def("set_log_config", [](Diagnostics& self, protocol::LogLevel level,
                                  uint8_t output_mask, unsigned timeout_ms) {
                py::gil_scoped_release g;
                return self.set_log_config(level, output_mask,
                                           std::chrono::milliseconds(timeout_ms));
            },
            py::arg("level"), py::arg("output_mask") = protocol::LogOutput::None,
            py::arg("timeout_ms") = 100u,
            "Turn firmware logging on or off. DIAGNOSTIC LEVER, NOT A SETTING:\n"
            "the firmware log sink is a blocking polled UART write (~0.5 ms per\n"
            "line at 921600) that stalls whichever task emitted the line -- that\n"
            "is what livelocked the command channel before logging was switched\n"
            "off by default. Output also goes to the MCU's DEBUG UART, which is\n"
            "not routed over USB, so without a probe on that pin you pay the\n"
            "realtime cost and see nothing. Turn it on, look, turn it back off.")
        .def("disable_logging", [](Diagnostics& self, unsigned timeout_ms) {
                py::gil_scoped_release g;
                return self.disable_logging(std::chrono::milliseconds(timeout_ms));
            }, py::arg("timeout_ms") = 100u);

    py::class_<protocol::MotorExecutionStatus>(m, "MotorExecutionStatus")
        .def_readonly("owner", &protocol::MotorExecutionStatus::owner)
        .def_readonly("home_state", &protocol::MotorExecutionStatus::home_state)
        .def_readonly("envelope_state", &protocol::MotorExecutionStatus::envelope_state)
        .def_readonly("mode", &protocol::MotorExecutionStatus::mode)
        .def_readonly("timeout_ms", &protocol::MotorExecutionStatus::timeout_ms)
        .def_readonly("target_age_ms", &protocol::MotorExecutionStatus::target_age_ms)
        .def_readonly("target_seq", &protocol::MotorExecutionStatus::target_seq)
        .def_readonly("applied_seq", &protocol::MotorExecutionStatus::applied_seq)
        .def_readonly("last_error", &protocol::MotorExecutionStatus::last_error)
        .def_readonly("flags", &protocol::MotorExecutionStatus::flags)
        .def_readonly("torque_cap_nm", &protocol::MotorExecutionStatus::torque_cap_nm)
        .def_property_readonly("requested", [](const protocol::MotorExecutionStatus& s) {
            return py::make_tuple(s.requested[0], s.requested[1], s.requested[2], s.requested[3], s.requested[4]);
        })
        .def_property_readonly("applied", [](const protocol::MotorExecutionStatus& s) {
            return py::make_tuple(s.applied[0], s.applied[1], s.applied[2], s.applied[3], s.applied[4]);
        });

    py::class_<protocol::MotorControlStats>(m, "MotorControlStats")
        .def_readonly("running",             &protocol::MotorControlStats::running)
        .def_readonly("mode",                &protocol::MotorControlStats::mode)
        .def_readonly("target_hz",           &protocol::MotorControlStats::target_hz)
        .def_readonly("period_ms",           &protocol::MotorControlStats::period_ms)
        .def_readonly("sample_ms",           &protocol::MotorControlStats::sample_ms)
        .def_readonly("actual_hz",           &protocol::MotorControlStats::actual_hz)
        .def_readonly("target_seq",          &protocol::MotorControlStats::target_seq)
        .def_readonly("applied_seq",         &protocol::MotorControlStats::applied_seq)
        .def_readonly("loop_count",          &protocol::MotorControlStats::loop_count)
        .def_readonly("error_count",         &protocol::MotorControlStats::error_count)
        .def_readonly("deadline_miss_count", &protocol::MotorControlStats::deadline_miss_count)
        .def_readonly("timeout_count",       &protocol::MotorControlStats::timeout_count)
        .def_readonly("last_error",          &protocol::MotorControlStats::last_error)
        .def_readonly("target_age_ms",       &protocol::MotorControlStats::target_age_ms)
        .def_readonly("target_update_hz",    &protocol::MotorControlStats::target_update_hz)
        .def("__repr__", [](const protocol::MotorControlStats& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorControlStats(running=%d, mode=%d, actual_hz=%.1f, loops=%u, errors=%u)",
                s.running, s.mode, s.actual_hz,
                static_cast<unsigned>(s.loop_count),
                static_cast<unsigned>(s.error_count));
            return std::string(buf);
        });

    // ---- MotorStatusExt (V2.2 — Cmd 0x53, 72 bytes) ----------------------
    // Bytes 0..30 mirror MotorStatusSample; the rest is the fault / monitor
    // extension. `monitor_flags` gates the fault fields — check it first.
    py::class_<protocol::MotorStatusExt>(m, "MotorStatusExt")
        .def_readonly("actual_pos",          &protocol::MotorStatusExt::actual_pos)
        .def_readonly("actual_vel",          &protocol::MotorStatusExt::actual_vel)
        .def_readonly("actual_torque",       &protocol::MotorStatusExt::actual_torque)
        .def_readonly("motor_temp_c",        &protocol::MotorStatusExt::motor_temp)
        .def_readonly("status",              &protocol::MotorStatusExt::status)
        .def_readonly("target_pos",          &protocol::MotorStatusExt::target_pos)
        .def_readonly("target_vel",          &protocol::MotorStatusExt::target_vel)
        .def_readonly("target_torque",       &protocol::MotorStatusExt::target_torque)
        .def_readonly("control_mode",        &protocol::MotorStatusExt::control_mode)
        .def_readonly("monitor_version",     &protocol::MotorStatusExt::monitor_version)
        .def_readonly("monitor_flags",       &protocol::MotorStatusExt::monitor_flags)
        .def_readonly("stop_reason",         &protocol::MotorStatusExt::stop_reason)
        .def_readonly("monitor_reserved",    &protocol::MotorStatusExt::monitor_reserved)
        .def_readonly("fault_code",          &protocol::MotorStatusExt::fault_code)
        .def_readonly("latched_fault_code",  &protocol::MotorStatusExt::latched_fault_code)
        .def_readonly("fault_timestamp_ms",  &protocol::MotorStatusExt::fault_timestamp_ms)
        .def_readonly("status_timestamp_ms", &protocol::MotorStatusExt::status_timestamp_ms)
        .def_readonly("stop_fault_code",     &protocol::MotorStatusExt::stop_fault_code)
        .def_readonly("stop_timestamp_ms",   &protocol::MotorStatusExt::stop_timestamp_ms)
        .def_readonly("fault_can_id",        &protocol::MotorStatusExt::fault_can_id)
        .def_readonly("fault_can_dlc",       &protocol::MotorStatusExt::fault_can_dlc)
        // Raw C array -> bytes; def_readonly on uint8_t[8] would expose an
        // opaque proxy that does not survive the owning struct.
        .def_property_readonly("fault_can_data", [](const protocol::MotorStatusExt& s) {
            return py::bytes(reinterpret_cast<const char*>(s.fault_can_data),
                             sizeof(s.fault_can_data));
        })
        .def("__repr__", [](const protocol::MotorStatusExt& s) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "MotorStatusExt(pos=%.4frad, torque=%.3fNm, status=0x%04x, "
                "fault=0x%08x, latched=0x%08x, monitor=0x%02x, stop_reason=%u)",
                s.actual_pos, s.actual_torque, s.status,
                static_cast<unsigned>(s.fault_code),
                static_cast<unsigned>(s.latched_fault_code),
                s.monitor_flags, s.stop_reason);
            return std::string(buf);
        });

    // ---- MotorFaultReport (V2.2 — Cmd 0x52, 64 bytes) --------------------
    py::class_<protocol::MotorFaultReport>(m, "MotorFaultReport")
        .def_readonly("version",           &protocol::MotorFaultReport::version)
        .def_readonly("report_flags",      &protocol::MotorFaultReport::report_flags)
        .def_readonly("source_mask",       &protocol::MotorFaultReport::source_mask)
        .def_readonly("protocol_mode",     &protocol::MotorFaultReport::protocol_mode)
        .def_readonly("event_seq",         &protocol::MotorFaultReport::event_seq)
        .def_readonly("timestamp_ms",      &protocol::MotorFaultReport::timestamp_ms)
        .def_readonly("motor_fault_code",  &protocol::MotorFaultReport::motor_fault_code)
        .def_readonly("motor_latched_fault_code",
                      &protocol::MotorFaultReport::motor_latched_fault_code)
        .def_readonly("stop_fault_code",   &protocol::MotorFaultReport::stop_fault_code)
        .def_readonly("firmware_fault_code",
                      &protocol::MotorFaultReport::firmware_fault_code)
        .def_readonly("firmware_latched_fault_code",
                      &protocol::MotorFaultReport::firmware_latched_fault_code)
        .def_readonly("firmware_detail_code",
                      &protocol::MotorFaultReport::firmware_detail_code)
        .def_readonly("fault_can_id",      &protocol::MotorFaultReport::fault_can_id)
        .def_readonly("fault_can_dlc",     &protocol::MotorFaultReport::fault_can_dlc)
        .def_readonly("monitor_flags",     &protocol::MotorFaultReport::monitor_flags)
        .def_readonly("monitor_reserved",  &protocol::MotorFaultReport::monitor_reserved)
        .def_readonly("fault_timestamp_ms",
                      &protocol::MotorFaultReport::fault_timestamp_ms)
        .def_readonly("status_timestamp_ms",
                      &protocol::MotorFaultReport::status_timestamp_ms)
        .def_readonly("stop_reason",       &protocol::MotorFaultReport::stop_reason)
        .def_property_readonly("fault_can_data", [](const protocol::MotorFaultReport& r) {
            return py::bytes(reinterpret_cast<const char*>(r.fault_can_data),
                             sizeof(r.fault_can_data));
        })
        .def("__repr__", [](const protocol::MotorFaultReport& r) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "MotorFaultReport(flags=0x%02x, sources=0x%02x, motor=0x%08x, "
                "fw=0x%08x, detail=%d, stop_reason=%u)",
                r.report_flags, r.source_mask,
                static_cast<unsigned>(r.motor_fault_code),
                static_cast<unsigned>(r.firmware_fault_code),
                static_cast<int>(r.firmware_detail_code), r.stop_reason);
            return std::string(buf);
        });

    // ---- MotorStatusSample ----------------------------------------------
    py::class_<MotorStatusSample>(m, "MotorStatusSample")
        .def_property_readonly("host_time", [](const MotorStatusSample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("actual_pos",     &MotorStatusSample::actual_pos)
        .def_readonly("actual_vel",     &MotorStatusSample::actual_vel)
        .def_readonly("actual_torque",  &MotorStatusSample::actual_torque)
        .def_readonly("motor_temp_c",   &MotorStatusSample::motor_temp_c)
        .def_readonly("status",         &MotorStatusSample::status)
        // target_* / control_mode: correct on V1.9 firmware (31-byte status).
        .def_readonly("target_pos",     &MotorStatusSample::target_pos)
        .def_readonly("target_vel",     &MotorStatusSample::target_vel)
        .def_readonly("target_torque",  &MotorStatusSample::target_torque)
        .def_readonly("control_mode",   &MotorStatusSample::control_mode)
        .def("__repr__", [](const MotorStatusSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorStatusSample(pos=%.4frad, vel=%.4frad/s, torque=%.3fNm, temp=%.1fC, status=0x%04x)",
                s.actual_pos, s.actual_vel, s.actual_torque, s.motor_temp_c, s.status);
            return std::string(buf);
        });

    // ---- Encoder --------------------------------------------------------
    py::class_<Encoder>(m, "Encoder")
        .def("read_once", [](Encoder& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_once(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        .def("on_data", [](Encoder& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_data([cb](const EncoderSample& s) {
                call_into_python("xense.taccap.Encoder callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &Encoder::off, py::arg("subscription_id"))
        .def("set_zero", [](Encoder& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            self.set_zero(std::chrono::milliseconds(timeout_ms));
        },
            py::arg("timeout_ms") = 500u,
            "Latch the current encoder reading as the new zero position. "
            "The gripper must already be held at the desired zero pose "
            "(e.g. fully closed) before calling. Raises on NACK / timeout.")
        .def("set_position_map", &Encoder::set_position_map, py::arg("position_map"),
             "Install the raw-rad -> [0,1] converter that fills "
             "EncoderSample.position. Applies to read_once() and to every "
             "already-registered on_data() subscriber. Raises ProtocolError if "
             "the map is not valid.")
        .def("clear_position_map", &Encoder::clear_position_map,
             "Remove the converter; EncoderSample.position goes back to nan.")
        .def_property_readonly("has_position_map", &Encoder::has_position_map)
        .def_property_readonly("position_map", &Encoder::position_map,
             "Copy of the installed converter; .valid is False when none is "
             "installed.");

    // ---- Motor ----------------------------------------------------------
    py::class_<Motor>(m, "Motor")
        .def("enable",      [](Motor& self) { py::gil_scoped_release g; self.enable();      })
        .def("disable",     [](Motor& self) { py::gil_scoped_release g; self.disable();     })
        .def("clear_fault", [](Motor& self) { py::gil_scoped_release g; self.clear_fault(); })
        // Python retains controller-based motion APIs. C++ raw targets now
        // validate parameters/ownership, and firmware 1.1.7 enforces its own
        // external envelope. Raw Python motion remains intentionally hidden.
        .def("read_status", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        .def("on_status", [](Motor& self, py::function pycb) {
            // make_gil_safe_callback, not a bare make_shared: the last
            // reference can die on the transport reader thread, which holds
            // no GIL.
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_status([cb](const MotorStatusSample& s) {
                call_into_python("xense.taccap.Motor callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &Motor::off, py::arg("subscription_id"))
        // ---- Follower motor admin (follower-only; validated against hw_v1.1.0)
        .def("set_zero", [](Motor& self) { py::gil_scoped_release g; self.set_zero(); })
        .def("get_can_id", [](Motor& self) { py::gil_scoped_release g; return self.get_can_id(); })
        .def("set_can_id", [](Motor& self, uint8_t id) {
            py::gil_scoped_release g; self.set_can_id(id);
        }, py::arg("can_id"))
        .def("switch_protocol", [](Motor& self, protocol::MotorProtocol p) {
            py::gil_scoped_release g; self.switch_protocol(p);
        }, py::arg("protocol"))
        .def("get_protocol", [](Motor& self) {
            py::gil_scoped_release g; return self.get_protocol();
        })
        // Private-protocol single-parameter access (Cmd 0x38/0x39). NACKs
        // InvalidParam under MIT (the whole SDK assumes MIT).
        .def("get_private_param", [](Motor& self, uint16_t index) {
            py::gil_scoped_release g; return self.get_private_param(index);
        }, py::arg("index"))
        .def("set_private_param", [](Motor& self, uint16_t index, uint32_t raw_value) {
            py::gil_scoped_release g; self.set_private_param(index, raw_value);
        }, py::arg("index"), py::arg("raw_value"))
        .def("execution_status", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release release;
            return self.execution_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 300, "Firmware 1.1.7+: requested and applied targets, limits and owner.")
        .def("control_stats", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.control_stats(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        // ---- V2.2 diagnostics (follower >= 1.1.2; older firmware NACKs) ----
        .def("read_status_ext", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.read_status_ext(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        // force=True costs a CAN round trip and can disturb a running control
        // loop — leave it False when polling.
        .def("fault_report", [](Motor& self, bool force, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.fault_report(force, std::chrono::milliseconds(timeout_ms));
        }, py::arg("force") = false, py::arg("timeout_ms") = 200)
        .def("set_startup_limit_torque", [](Motor& self, float torque_nm) {
            py::gil_scoped_release g; self.set_startup_limit_torque(torque_nm);
        }, py::arg("torque_nm"))
        .def("get_startup_limit_torque", [](Motor& self) {
            py::gil_scoped_release g; return self.get_startup_limit_torque();
        });
}

}  // namespace xense::taccap::python

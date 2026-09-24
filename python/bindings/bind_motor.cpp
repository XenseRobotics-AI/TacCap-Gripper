// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: Motor, its status/fault records, Encoder, Diagnostics
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

#include <cstring>

namespace xense::taccap::python {

void bind_motor(py::module_& m) {
    using namespace xense::taccap;
    // ---- MotorPrivateParam (V1.9+ private-protocol param GET response) ----
    py::class_<protocol::MotorPrivateParam>(m, "MotorPrivateParam",
        "One motor parameter as returned by Motor.get_private_param (Cmd 0x38).\n\n"
        "The value comes back as four raw bytes; `type` says how to read them.")
        .def_readonly("index",     &protocol::MotorPrivateParam::index,
                      "Motor register index, e.g. 0x700B (limit_torque).")
        .def_readonly("type",      &protocol::MotorPrivateParam::type,     // 1=u8, 2=f32
                      "Encoding of raw_value: 1 = u8, 2 = f32.")
        .def_readonly("access",    &protocol::MotorPrivateParam::access,   // 0x01=R, 0x02=W
                      "Access bits from the firmware's whitelist: 0x01 read, 0x02 write.")
        .def_readonly("raw_value", &protocol::MotorPrivateParam::raw_value,
                      "The four bytes as received, uninterpreted. Read them per `type`.")
        .def_property_readonly("as_float", [](const protocol::MotorPrivateParam& p) {
            float f; std::memcpy(&f, &p.raw_value, 4); return f;
        }, "raw_value reinterpreted as f32. Meaningless unless type == 2.")
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
        .def_readonly("tx_calls_ok",     &protocol::UartStats::tx_calls_ok,
                      "Successful transmit calls on the control port -- roughly frames out.")
        .def_readonly("tx_fail_timeout", &protocol::UartStats::tx_fail_timeout,
                      "Polled-send timeouts on the control port. One of these mid-frame\n"
                      "leaves a short frame on the wire, indistinguishable from loss in transit.")
        .def_readonly("tx_fail_other",   &protocol::UartStats::tx_fail_other,
                      "Control-port send failures that were not timeouts.")
        .def_readonly("rx_bytes",        &protocol::UartStats::rx_bytes)
        .def_readonly("rx_overflow",     &protocol::UartStats::rx_overflow,
                      "Bytes dropped because the control-port ring buffer was full -- the\n"
                      "firmware's command task could not keep up with the host.")
        .def_readonly("debug_tx_bytes",  &protocol::UartStats::debug_tx_bytes,
                      "Bytes out of the DEBUG port; quantifies what firmware logging costs.")
        .def_readonly("rb_used",         &protocol::UartStats::rb_used,
                      "Control-port ring buffer occupancy, bytes.")
        .def_readonly("rb_free",         &protocol::UartStats::rb_free,
                      "Control-port ring buffer headroom, bytes.")
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

    py::class_<protocol::LogConfig>(m, "LogConfig",
        "The firmware log configuration reported back by Diagnostics.set_log_config\n"
        "and disable_logging (Cmd 0x55).")
        .def_readonly("level",       &protocol::LogConfig::level,
                      "LogLevel as a raw integer, 0 (NONE) .. 5 (VERBOSE).")
        .def_readonly("output_mask", &protocol::LogConfig::output_mask,
                      "Bitmask of sinks: LOG_OUTPUT_NONE / LOG_OUTPUT_UART. The UART sink is\n"
                      "the MCU's DEBUG pin, which is not routed over USB.")
        .def("__repr__", [](const protocol::LogConfig& c) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "LogConfig(level=%u, output_mask=0x%02X)",
                          c.level, c.output_mask);
            return std::string(buf);
        });

    py::enum_<protocol::LogLevel>(m, "LogLevel",
        "Firmware log verbosity. NONE emits nothing whatever output_mask says.")
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
            }, py::arg("timeout_ms") = 100u,
            "Read the free-running UART counters (Cmd 0x54). Blocks for the ACK.\n\n"
            "Counters are cumulative since MCU boot, so take one reading before a\n"
            "measurement window and one after, then subtract. Needs firmware 1.1.3;\n"
            "older firmware NACKs InvalidCmd -> ProtocolError.")
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
            }, py::arg("timeout_ms") = 100u,
            "Turn firmware logging off again -- level NONE, no output. Blocks for the\n"
            "ACK and returns the configuration the firmware reports as active.");

    py::class_<protocol::MotorControlStats>(m, "MotorControlStats",
        "Health of the firmware's own motor control task (Cmd 0x51).\n\n"
        "This is the out-of-band feedback for the no-ACK submit_* path: nothing\n"
        "else says whether the frames are being applied. Poll it off the realtime\n"
        "thread, never from inside a submit loop.")
        .def_readonly("running",             &protocol::MotorControlStats::running,
                      "Non-zero while the firmware's control task is running.")
        .def_readonly("mode",                &protocol::MotorControlStats::mode,
                      "MotorMode as a raw integer: 0 idle, 1 position, 2 velocity,\n"
                      "3 torque, 4 impedance.")
        .def_readonly("target_hz",           &protocol::MotorControlStats::target_hz)
        .def_readonly("period_ms",           &protocol::MotorControlStats::period_ms)
        .def_readonly("sample_ms",           &protocol::MotorControlStats::sample_ms,
                      "Width of the window actual_hz is measured over, ms.")
        .def_readonly("actual_hz",           &protocol::MotorControlStats::actual_hz)
        .def_readonly("target_seq",          &protocol::MotorControlStats::target_seq,
                      "Counts targets the host has submitted.")
        .def_readonly("applied_seq",         &protocol::MotorControlStats::applied_seq,
                      "Counts targets the control task has applied. A gap against\n"
                      "target_seq that stops closing means submissions are not landing --\n"
                      "e.g. while the firmware owns the motor during auto-calibration.")
        .def_readonly("loop_count",          &protocol::MotorControlStats::loop_count)
        .def_readonly("error_count",         &protocol::MotorControlStats::error_count)
        .def_readonly("deadline_miss_count", &protocol::MotorControlStats::deadline_miss_count)
        .def_readonly("timeout_count",       &protocol::MotorControlStats::timeout_count,
                      "Reserved in firmware; reads 0.")
        .def_readonly("last_error",          &protocol::MotorControlStats::last_error,
                      "Last control-loop error code, signed. 0 = none.")
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
    py::class_<protocol::MotorStatusExt>(m, "MotorStatusExt",
        "The 72-byte extended motor status (Cmd 0x53), from Motor.read_status_ext.\n\n"
        "A superset of MotorStatusSample: the same pose/torque/status prefix plus\n"
        "the fault words, their latched and stop-time snapshots, the firmware's\n"
        "collection-health flags and the raw CAN evidence. Check monitor_flags\n"
        "before trusting any fault field.")
        .def_readonly("actual_pos",          &protocol::MotorStatusExt::actual_pos,
                      "Motor angle, rad.")
        .def_readonly("actual_vel",          &protocol::MotorStatusExt::actual_vel,
                      "rad/s at the motor.")
        .def_readonly("actual_torque",       &protocol::MotorStatusExt::actual_torque,
                      "Measured torque, N*m.")
        .def_readonly("motor_temp_c",        &protocol::MotorStatusExt::motor_temp)
        .def_readonly("status",              &protocol::MotorStatusExt::status,
                      "Motor status word: MotorStatusBit flags (Enabled 0x0001, Fault 0x0002,\n"
                      "Stalled 0x0004, ...). The masks stay C++-side; test bits on the integer.")
        .def_readonly("target_pos",          &protocol::MotorStatusExt::target_pos,
                      "Last applied target angle, rad.")
        .def_readonly("target_vel",          &protocol::MotorStatusExt::target_vel,
                      "Last applied target velocity, rad/s.")
        .def_readonly("target_torque",       &protocol::MotorStatusExt::target_torque,
                      "Last applied target / feed-forward / clamp torque, N*m.")
        .def_readonly("control_mode",        &protocol::MotorStatusExt::control_mode,
                      "MotorMode of the last applied command, raw: 0 idle, 1 position,\n"
                      "2 velocity, 3 torque, 4 impedance.")
        .def_readonly("monitor_version",     &protocol::MotorStatusExt::monitor_version,
                      "Layout version of the monitor extension the firmware filled in.")
        .def_readonly("monitor_flags",       &protocol::MotorStatusExt::monitor_flags,
                      "Health of the firmware's fault COLLECTION, not motor faults:\n"
                      "StatusValid 0x01 / FaultValid 0x02 say whether the corresponding\n"
                      "fields were ever filled. Read this before trusting fault_code.")
        .def_readonly("stop_reason",         &protocol::MotorStatusExt::stop_reason,
                      "MotorStopReason as a raw integer.")
        .def_readonly("monitor_reserved",    &protocol::MotorStatusExt::monitor_reserved,
                      "Provenance bits for the fault data -- RxSeen 0x01, TimeoutSeen 0x02,\n"
                      "CanFrameSeen 0x04 -- which separate 'no fault' from 'never read one'.")
        .def_readonly("fault_code",          &protocol::MotorStatusExt::fault_code,
                      "Live motor fault word, a bit mask raised by the motor itself.")
        .def_readonly("latched_fault_code",  &protocol::MotorStatusExt::latched_fault_code,
                      "OR of every motor fault bit seen since power-on.")
        .def_readonly("fault_timestamp_ms",  &protocol::MotorStatusExt::fault_timestamp_ms,
                      "MCU tick of the last full fault reply, ms since MCU boot.")
        .def_readonly("status_timestamp_ms", &protocol::MotorStatusExt::status_timestamp_ms,
                      "MCU tick of the last motion-status reply, ms since MCU boot.")
        .def_readonly("stop_fault_code",     &protocol::MotorStatusExt::stop_fault_code,
                      "Motor fault word latched at the moment the gripper stopped.")
        .def_readonly("stop_timestamp_ms",   &protocol::MotorStatusExt::stop_timestamp_ms,
                      "MCU tick of the stop snapshot, ms since MCU boot.")
        .def_readonly("fault_can_id",        &protocol::MotorStatusExt::fault_can_id,
                      "Standard CAN id of the raw fault reply; normally 0xFD.")
        .def_readonly("fault_can_dlc",       &protocol::MotorStatusExt::fault_can_dlc,
                      "Raw DLC of that reply. A valid one is >= 5.")
        // Raw C array -> bytes; def_readonly on uint8_t[8] would expose an
        // opaque proxy that does not survive the owning struct.
        .def_property_readonly("fault_can_data", [](const protocol::MotorStatusExt& s) {
            return py::bytes(reinterpret_cast<const char*>(s.fault_can_data),
                             sizeof(s.fault_can_data));
        }, "The 8 raw bytes of the CAN fault reply: byte 0 is the motor CAN id,\n"
           "bytes 1..4 the little-endian fault word.")
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

    // ---- MotorVersion (Cmd 0x58, 8 bytes, follower firmware >= 1.2.6) ----
    py::class_<protocol::MotorVersion>(m, "MotorVersion",
        "The version of the program inside the MOTOR (Cmd 0x58), from\n"
        "Motor.motor_version. Not the gripper's firmware version.\n\n"
        "A failed read is reported here rather than raised, because 'the motor did\n"
        "not answer' and 'this firmware has no such command' are different\n"
        "diagnoses -- so check `valid` before reading `version`.")
        .def_property_readonly("version", [](const protocol::MotorVersion& v) {
            return py::make_tuple(v.version[0], v.version[1],
                                  v.version[2], v.version[3]);
        }, "版本号四段,**高位在前**(手册 p.31 应答 Byte3~6)")
        .def_readonly("valid",         &protocol::MotorVersion::valid,
                      "1 when the read succeeded. `version` means nothing otherwise.")
        .def_readonly("motor_stopped", &protocol::MotorVersion::motor_stopped,
                      "1 when the request probably stopped the motor: it reuses RobStride\n"
                      "communication type 4, which is also 'motor stop', and a motor that\n"
                      "does not know the 00 C4 magic executes it. Re-enable before\n"
                      "commanding motion again.")
        .def_readonly("protocol_mode", &protocol::MotorVersion::protocol_mode,
                      "The motor's CAN protocol when the read was made: 0 = private, 2 = MIT.")
        .def("__str__", [](const protocol::MotorVersion& v) {
            if (!v.valid) {
                return std::string("MotorVersion(invalid, protocol=") +
                       (v.protocol_mode == 2 ? "MIT — 电机在 MIT 下不理扩展帧"
                                             : "private") + ")";
            }
            char buf[64];
            std::snprintf(buf, sizeof(buf), "MotorVersion(%u.%u.%u.%u)",
                          v.version[0], v.version[1], v.version[2], v.version[3]);
            return std::string(buf);
        });

    // ---- MotorFaultReport (V2.2 — Cmd 0x52, 64 bytes) --------------------
    py::class_<protocol::MotorFaultReport>(m, "MotorFaultReport",
        "One diagnostic snapshot merging the motor's own fault word, the MCU's\n"
        "firmware-level fault state and the raw CAN reply (Cmd 0x52).\n\n"
        "The firmware ACKs OK even when there is nothing to report, so check\n"
        "report_flags (MotorValid 0x04 / FwValid 0x08) before trusting any fault code.")
        .def_readonly("version",           &protocol::MotorFaultReport::version,
                      "Report layout version; 1 for the layout this SDK decodes.")
        .def_readonly("report_flags",      &protocol::MotorFaultReport::report_flags,
                      "Bit flags: Valid 0x01, ForceRead 0x02, MotorValid 0x04, FwValid 0x08,\n"
                      "CanEvidence 0x10, ReadTimeout 0x20, PrivatePartial 0x40.")
        .def_readonly("source_mask",       &protocol::MotorFaultReport::source_mask,
                      "Which layer raised the fault: Motor 0x01, MCU CAN 0x02,\n"
                      "MCU control 0x04, MCU system 0x08.")
        .def_readonly("protocol_mode",     &protocol::MotorFaultReport::protocol_mode,
                      "MotorProtocol in effect, raw: 0 = private, 2 = MIT.")
        .def_readonly("event_seq",         &protocol::MotorFaultReport::event_seq,
                      "Increments once per new fault event.")
        .def_readonly("timestamp_ms",      &protocol::MotorFaultReport::timestamp_ms,
                      "MCU tick when this report was built, ms since MCU boot.")
        .def_readonly("motor_fault_code",  &protocol::MotorFaultReport::motor_fault_code,
                      "The motor's own fault word, a bit mask.")
        .def_readonly("motor_latched_fault_code",
                      &protocol::MotorFaultReport::motor_latched_fault_code,
                      "OR of every motor fault bit seen since power-on.")
        .def_readonly("stop_fault_code",   &protocol::MotorFaultReport::stop_fault_code,
                      "Motor fault word latched at the moment the gripper stopped.")
        .def_readonly("firmware_fault_code",
                      &protocol::MotorFaultReport::firmware_fault_code,
                      "MCU-side fault. Unlike the motor word this is an ENUMERATED value,\n"
                      "not a bit mask -- compare for equality, do not mask it.")
        .def_readonly("firmware_latched_fault_code",
                      &protocol::MotorFaultReport::firmware_latched_fault_code,
                      "Latched MCU-side fault; enumerated like firmware_fault_code.")
        .def_readonly("firmware_detail_code",
                      &protocol::MotorFaultReport::firmware_detail_code,
                      "Driver-level errno behind the firmware fault, signed.")
        .def_readonly("fault_can_id",      &protocol::MotorFaultReport::fault_can_id,
                      "Standard CAN id of the raw fault reply.")
        .def_readonly("fault_can_dlc",     &protocol::MotorFaultReport::fault_can_dlc,
                      "Raw DLC of that reply.")
        .def_readonly("monitor_flags",     &protocol::MotorFaultReport::monitor_flags,
                      "Health of the firmware's fault collection; same bits as\n"
                      "MotorStatusExt.monitor_flags.")
        .def_readonly("monitor_reserved",  &protocol::MotorFaultReport::monitor_reserved,
                      "Provenance bits for the fault data; same bits as\n"
                      "MotorStatusExt.monitor_reserved.")
        .def_readonly("fault_timestamp_ms",
                      &protocol::MotorFaultReport::fault_timestamp_ms,
                      "MCU tick of the last full fault reply, ms since MCU boot.")
        .def_readonly("status_timestamp_ms",
                      &protocol::MotorFaultReport::status_timestamp_ms,
                      "MCU tick of the last motion-status reply, ms since MCU boot.")
        .def_readonly("stop_reason",       &protocol::MotorFaultReport::stop_reason,
                      "MotorStopReason as a raw integer.")
        .def_property_readonly("fault_can_data", [](const protocol::MotorFaultReport& r) {
            return py::bytes(reinterpret_cast<const char*>(r.fault_can_data),
                             sizeof(r.fault_can_data));
        }, "The 8 raw bytes of the CAN fault reply: byte 0 is the motor CAN id,\n"
           "bytes 1..4 the little-endian fault word.")
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
    py::class_<MotorStatusSample>(m, "MotorStatusSample",
        "One decoded motor-status frame, from Motor.read_status or the status stream.\n\n"
        "The decoder accepts the 31-, 59- and 72-byte firmware layouts and zero-fills\n"
        "the tail, so the diagnostic fields below read 0 against firmware that still\n"
        "sends the 31-byte prefix -- which is not distinguishable from a real zero.")
        .def_property_readonly("host_time", [](const MotorStatusSample& s) {
            return tp_to_seconds(s.host_time);
        }, "Host steady-clock seconds when the frame was decoded; comparable with\n"
           "time.monotonic(), not with wall-clock time.")
        .def_readonly("actual_pos",     &MotorStatusSample::actual_pos,
                      "Motor angle, rad.")
        .def_readonly("actual_vel",     &MotorStatusSample::actual_vel,
                      "rad/s at the motor.")
        .def_readonly("actual_torque",  &MotorStatusSample::actual_torque,
                      "Measured torque, N*m.")
        .def_readonly("motor_temp_c",   &MotorStatusSample::motor_temp_c)
        .def_readonly("status",         &MotorStatusSample::status,
                      "Motor status word: MotorStatusBit flags (Enabled 0x0001, Fault 0x0002,\n"
                      "Stalled 0x0004, ...). The masks stay C++-side; test bits on the integer.")
        // target_* / control_mode: correct on V1.9 firmware (31-byte status).
        .def_readonly("target_pos",     &MotorStatusSample::target_pos,
                      "Last applied target angle, rad.")
        .def_readonly("target_vel",     &MotorStatusSample::target_vel,
                      "Last applied target velocity, rad/s.")
        .def_readonly("target_torque",  &MotorStatusSample::target_torque,
                      "Last applied target / feed-forward / clamp torque, N*m.")
        .def_readonly("control_mode",   &MotorStatusSample::control_mode,
                      "MotorMode of the last applied command, raw: 0 idle, 1 position,\n"
                      "2 velocity, 3 torque, 4 impedance.")
        // 诊断字段:从爪改发 59 字节 V2 状态后随流到达,不再需要轮询 0x53
        // (轮询会和相位锁里的控制帧对撞)。仍发 31 字节前缀的固件上这些为 0。
        .def_readonly("monitor_version",    &MotorStatusSample::monitor_version,
                      "Layout version of the monitor extension. 0 means the frame carried\n"
                      "only the 31-byte prefix, so every diagnostic field here is zero-fill\n"
                      "rather than a reading -- check this one first.")
        .def_readonly("stop_reason",        &MotorStatusSample::stop_reason,
                      "MotorStopReason as a raw integer.")
        .def_readonly("monitor_reserved",   &MotorStatusSample::monitor_reserved,
                      "Provenance bits for the fault data -- RxSeen 0x01, TimeoutSeen 0x02,\n"
                      "CanFrameSeen 0x04.")
        .def_readonly("fault_code",         &MotorStatusSample::fault_code,
                      "Live motor fault word, a bit mask raised by the motor itself.")
        .def_readonly("latched_fault_code", &MotorStatusSample::latched_fault_code,
                      "OR of every motor fault bit seen since power-on.")
        .def("__repr__", [](const MotorStatusSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorStatusSample(pos=%.4frad, vel=%.4frad/s, torque=%.3fNm, temp=%.1fC, status=0x%04x)",
                s.actual_pos, s.actual_vel, s.actual_torque, s.motor_temp_c, s.status);
            return std::string(buf);
        });

    // ---- Encoder --------------------------------------------------------
    py::class_<Encoder>(m, "Encoder",
        "The gripper's opening encoder: one-shot reads plus the streamed frames.\n\n"
        "On a leader this is the dedicated encoder hardware; on a follower the same\n"
        "telemetry also rides on the motor status. Install a position map and every\n"
        "sample additionally carries the opening normalized to [0, 1].")
        .def("read_once", [](Encoder& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_once(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100,
           "Read one encoder sample. Blocks for the ACK; raises ProtocolError on NACK\n"
           "and TimeoutError when no reply arrives.")
        .def("on_data", [](Encoder& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_data([cb](const EncoderSample& s) {
                call_into_python("xense.taccap.Encoder callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"),
           "Subscribe to streamed encoder frames; returns the id to pass to off().\n\n"
           "Frames arrive only while encoder streaming is running -- see\n"
           "LeaderGripper.start_streaming. The callback runs on the transport's\n"
           "dispatcher thread and should be quick: the dispatch queue is bounded and\n"
           "drops the OLDEST frame when it fills.")
        .def("off", &Encoder::off, py::arg("subscription_id"),
             "Cancel a subscription returned by on_data().")
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
        .def_property_readonly("has_position_map", &Encoder::has_position_map,
             "True while a converter is installed.")
        .def_property_readonly("position_map", &Encoder::position_map,
             "Copy of the installed converter; .valid is False when none is "
             "installed.");

    // ---- MotorModel (0x59 / 0x5A) ----------------------------------------
    py::class_<protocol::MotorModel>(m, "MotorModel",
        "Which actuator this gripper is built around, as the MCU records it.\n\n"
        "**This is not a question the motor can answer.** Its version frame\n"
        "carries a version and nothing else, neither the EL05 nor the RS00\n"
        "manual has a model field anywhere in the readable parameter tables,\n"
        "and the closest proxy (0x302d rated_i) is an inference rather than an\n"
        "identity AND is a private-protocol parameter -- unreadable while the\n"
        "motor speaks MIT. So the MCU keeps the answer in flash and this reads\n"
        "that record back.\n\n"
        "Getting it wrong does not raise: MIT frames quantise torque and\n"
        "velocity into 16 bits against the model's ranges, so an EL05 record on\n"
        "an RS00 turns a commanded 1.1 N*m into 2.57 N*m and reads feedback\n"
        "back at 0.43x -- which keeps the host's torque ceiling from tripping.\n"
        "t_max_nm and v_max_rad_s are the ranges actually in force, so they are\n"
        "the evidence when a gripper behaves as if its torques were scaled.")
        .def_readonly("id", &protocol::MotorModel::id,
                      "Stable model id as persisted. Not a table index -- ids are\n"
                      "never renumbered or reused, because a stored byte cannot\n"
                      "follow a table that got reordered.")
        .def_property_readonly("name", [](const protocol::MotorModel& s) {
            return std::string(s.name, ::strnlen(s.name, sizeof(s.name)));
        }, "Model name, e.g. 'EL05' or 'RS00'.")
        .def_readonly("from_flash", &protocol::MotorModel::from_flash,
                      "True when this device has a model written to flash. False\n"
                      "means the MCU fell back to its compile-time default -- a\n"
                      "legitimate state, but one where the ranges are an\n"
                      "assumption rather than a record.")
        .def_readonly("t_max_nm", &protocol::MotorModel::t_max_nm,
                      "MIT torque range currently in force, N*m. EL05 6.0, RS00 14.0.")
        .def_readonly("v_max_rad_s", &protocol::MotorModel::v_max_rad_s,
                      "MIT velocity range currently in force, rad/s. EL05 50.0, RS00 33.0.")
        .def("__repr__", [](const protocol::MotorModel& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorModel(%s, id=%u, %s, t_max=%.1fNm, v_max=%.1frad/s)",
                std::string(s.name, ::strnlen(s.name, sizeof(s.name))).c_str(),
                (unsigned)s.id, s.from_flash ? "from flash" : "COMPILE-TIME DEFAULT",
                (double)s.t_max_nm, (double)s.v_max_rad_s);
            return std::string(buf);
        });

    // ---- MotorSpec (0x56) ------------------------------------------------
    py::class_<protocol::MotorSpec>(m, "MotorSpec",
        "The device's own ratings for the actuator it has (Cmd 0x56).\n\n"
        "The firmware is the authority here: MOTOR_RATED_TORQUE_NM and\n"
        "MOTOR_PEAK_TORQUE_NM in this module are EL05 numbers compiled in as a\n"
        "fallback, and nothing on the host knows which motor is plugged in.\n"
        "A zero field means UNKNOWN -- the firmware's table does not carry that\n"
        "number for this model -- never the number zero.")
        .def_property_readonly("name", [](const protocol::MotorSpec& s) {
            return std::string(s.name, ::strnlen(s.name, sizeof(s.name)));
        }, "Motor model as the firmware names it, e.g. 'EL05'.")
        .def_readonly("p_max_rad",            &protocol::MotorSpec::p_max_rad,
                      "Position range, symmetric +/-.")
        .def_readonly("v_max_rad_s",          &protocol::MotorSpec::v_max_rad_s,
                      "Velocity range, symmetric +/-.")
        .def_readonly("t_max_nm",             &protocol::MotorSpec::t_max_nm,
                      "Torque range, i.e. the peak load -- transient, not a hold.")
        .def_readonly("kp_max",               &protocol::MotorSpec::kp_max,
                      "Upper end of the stiffness range, N*m/rad.")
        .def_readonly("kd_max",               &protocol::MotorSpec::kd_max,
                      "Upper end of the damping range, N*m*s/rad.")
        .def_readonly("rated_torque_nm",      &protocol::MotorSpec::rated_torque_nm,
                      "Rated torque, N*m -- what the motor may hold indefinitely.")
        .def_readonly("stall_cont_torque_nm", &protocol::MotorSpec::stall_cont_torque_nm,
                      "Indefinite stall rating, N*m. 0 = unknown.")
        .def_readonly("winding_limit_c",      &protocol::MotorSpec::winding_limit_c,
                      "0 = unknown.")
        .def_readonly("board_limit_c",        &protocol::MotorSpec::board_limit_c,
                      "0 = unknown.")
        .def("__repr__", [](const protocol::MotorSpec& s) {
            return "MotorSpec(" + std::string(s.name, ::strnlen(s.name, sizeof(s.name)))
                 + ", rated=" + std::to_string(s.rated_torque_nm)
                 + ", peak=" + std::to_string(s.t_max_nm) + ")";
        });

    // ---- Motor ----------------------------------------------------------
    py::class_<Motor>(m, "Motor",
        "The follower gripper's motor: lifecycle, raw control frames, status\n"
        "telemetry and the follower-only admin commands.\n\n"
        "Follower-only. On a leader the firmware reports the motor as absent, so\n"
        "every command here NACKs SensorOffline and raises ProtocolError.\n\n"
        "To move a jaw, prefer ImpedanceController or ForcePositionController. The\n"
        "submit_* frames below carry no host-side error clamp and no torque ceiling\n"
        "-- those live in the controllers -- and a free-running submit cadence can\n"
        "land while the MCU is transmitting, which costs the frame it was sending.\n"
        "The controllers submit once per received status frame and never overlap one.")
        .def("enable",      [](Motor& self) { py::gil_scoped_release g; self.enable();      },
             "Energize the motor. Blocks for the ACK; raises ProtocolError on NACK.\n\n"
             "NACKs SysBusy while the firmware owns the motor -- notably during\n"
             "power-on auto-calibration.")
        .def("disable",     [](Motor& self) { py::gil_scoped_release g; self.disable();     },
             "De-energize the motor. Blocks for the ACK; raises ProtocolError on NACK.\n\n"
             "This is the host-side safe state and it needs a live link: with the\n"
             "device gone it fails with IoError while the motor goes on holding\n"
             "whatever it was last commanded.")
        .def("clear_fault", [](Motor& self) { py::gil_scoped_release g; self.clear_fault(); },
             "Clear the motor's fault state. Blocks for the ACK; raises ProtocolError\n"
             "on NACK. Clears the motor only -- a controller's own FAULT state needs\n"
             "its reset().")
        // ---- 裸电机控制 --------------------------------------------------
        // 这些是**无 ACK 的直投命令**:帧丢上总线就返回,没有主机侧的误差钳位、
        // 力矩天花板 —— 那些长在 ImpedanceController / ForcePositionController
        // 里,越过控制器就一个都拿不到。
        //
        // 它们曾经刻意不暴露给 Python。原因是真实事故:客户控制台用
        // submit_impedance() 在 kp=20 下顶住刚性物体,kp*误差 一路涨到电机的
        // 0x700B 上限,24V 下的电流需求把整块板子拉垮 —— 夹爪松手掉件、USB
        // 链路消失。**那是固件 1.1.5、运动安全包络还不存在的时候。**
        //
        // 1.1.6 的包络改变了这个结论。同一份代码、同一台设备、同一物体、
        // kp=20,只切包络标志位(docs/CONTROL_LAYERING.md §1 [实测]):
        //     包络关闭 -> 整机掉电重启,力矩失控索要 ~12 Nm
        //     包络启用 -> 25 秒稳定夹持,恒定 0.549 Nm
        // 包络长在固件 MIT 分支上,是所有阻抗/位置/力/力位混合命令的唯一必经
        // 点 —— 裸命令绕不过它。CONTROL_LAYERING §3 的分层本来就是
        // 「SDK 给意图,固件 500Hz 执行安全包络」,暴露命令层与之一致。
        //
        // 仍然要知道的两件事:
        //  1. **热方向没有底层兜底**(同文档 §2 [实测]):电机自身的过温保护到
        //     100 °C 壳温都未动作,297 秒恒 1.65 Nm 故障位全零。热保护只有固件
        //     的温度墙,而温度墙是包络的一部分 —— 包络被关掉就什么都没有了。
        //  2. 主机看门狗照常计时:停止下发 300ms 后固件 T1 接管零速保持,30s 后
        //     T2 失能。裸命令不豁免这条。
        //
        // 需要误差钳位、力矩天花板和堵转处理的调用方,仍应该用控制器。
        .def("submit_impedance", [](Motor& self, float target_pos_rad, float kp,
                                    float kd, float feedforward_torque_nm,
                                    float feedforward_vel_radps) {
            py::gil_scoped_release g;
            self.submit_impedance(target_pos_rad, kp, kd, feedforward_torque_nm,
                                  feedforward_vel_radps);
        }, py::arg("target_pos_rad"), py::arg("kp_nm_per_rad"),
           py::arg("kd_nm_s_per_rad"), py::arg("feedforward_torque_nm"),
           py::arg("feedforward_vel_radps") = 0.0f,
           "MIT hybrid frame, no ACK. tau = kp*(target-pos) + kd*(vel_ff-vel) "
           "+ tau_ff, computed by the motor. Bounded only by the firmware "
           "envelope and the motor's own 0x700B.")
        .def("submit_position", [](Motor& self, float target_pos_rad,
                                   float max_vel_radps, float max_torque_nm) {
            py::gil_scoped_release g;
            self.submit_position(target_pos_rad, max_vel_radps, max_torque_nm);
        }, py::arg("target_pos_rad"), py::arg("max_vel_radps"),
           py::arg("max_torque_nm"), "Position command, no ACK.")
        .def("submit_velocity", [](Motor& self, float target_vel_radps,
                                   float max_torque_nm, float profile_acc_radps2) {
            py::gil_scoped_release g;
            self.submit_velocity(target_vel_radps, max_torque_nm, profile_acc_radps2);
        }, py::arg("target_vel_radps"), py::arg("max_torque_nm"),
           py::arg("profile_acc_radps2"), "Velocity command, no ACK.")
        .def("submit_torque", [](Motor& self, float target_torque_nm,
                                 float max_vel_radps) {
            py::gil_scoped_release g;
            self.submit_torque(target_torque_nm, max_vel_radps);
        }, py::arg("target_torque_nm"), py::arg("max_vel_radps"),
           "Torque command, no ACK.")
        .def("read_status", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100,
           "Read one motor-status frame. Blocks for the ACK; raises ProtocolError on\n"
           "NACK and TimeoutError when no reply arrives.\n\n"
           "This is the cheap read and the one the DATA stream carries. When something\n"
           "has already gone wrong, read_status_ext() says what.")
        .def("on_status", [](Motor& self, py::function pycb) {
            // make_gil_safe_callback, not a bare make_shared: the last
            // reference can die on the transport reader thread, which holds
            // no GIL.
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_status([cb](const MotorStatusSample& s) {
                call_into_python("xense.taccap.Motor callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"),
           "Subscribe to streamed motor-status frames; returns the id to pass to off().\n\n"
           "Frames arrive only while motor streaming is running -- see\n"
           "FollowerGripper.start_streaming. The callback runs on the transport's\n"
           "dispatcher thread and should be quick: the dispatch queue is bounded and\n"
           "drops the OLDEST frame when it fills.")
        .def("off", &Motor::off, py::arg("subscription_id"),
             "Cancel a subscription returned by on_status().")
        // ---- Follower motor admin (follower-only; validated against hw_v1.1.0)
        .def("set_zero", [](Motor& self) { py::gil_scoped_release g; self.set_zero(); },
             "Latch the motor's current shaft angle as mechanical zero.\n\n"
             "The jaw must already be held at the pose that should read zero. The\n"
             "firmware stops its control task before zeroing. Blocks for the ACK;\n"
             "raises ProtocolError on NACK.")
        .def("get_can_id", [](Motor& self) { py::gil_scoped_release g; return self.get_can_id(); },
             "Read the motor's CAN id. Blocks for the ACK; raises ProtocolError on\n"
             "NACK or an empty reply.")
        .def("set_can_id", [](Motor& self, uint8_t id) {
            py::gil_scoped_release g; self.set_can_id(id);
        }, py::arg("can_id"),
           "Write the motor's CAN id. Blocks for the ACK; raises ProtocolError on\n"
           "NACK -- SysBusy while the firmware's control task is running, since the\n"
           "write triggers a discovery scan on the bus that task is using.")
        .def("switch_protocol", [](Motor& self, protocol::MotorProtocol p) {
            py::gil_scoped_release g; self.switch_protocol(p);
        }, py::arg("protocol"),
           "Switch the motor's CAN protocol and persist the choice to flash.\n\n"
           "The rest of the SDK assumes MIT. Blocks for the ACK; raises ProtocolError\n"
           "on NACK -- SysBusy while the firmware's control task is running. The new\n"
           "protocol does not take effect until the 24 V supply is cycled; rebooting\n"
           "the MCU alone is not enough.")
        .def("get_protocol", [](Motor& self) {
            py::gil_scoped_release g; return self.get_protocol();
        }, "Read the motor's current CAN protocol. Blocks for the ACK; raises\n"
           "ProtocolError on NACK or an empty reply.")
        // Private-protocol single-parameter access (Cmd 0x38/0x39). NACKs
        // InvalidParam under MIT (the whole SDK assumes MIT).
        .def("get_private_param", [](Motor& self, uint16_t index) {
            py::gil_scoped_release g; return self.get_private_param(index);
        }, py::arg("index"),
           "Read one private-protocol motor parameter. Blocks for the ACK.\n\n"
           "Measured on firmware 1.1.5: under MIT the firmware rejects every index up\n"
           "front, so this raises ProtocolError(InvalidParam) there whatever the index\n"
           "-- and MIT is the normal case for a gripper. A non-whitelisted index is\n"
           "rejected the same way.")
        .def("set_private_param", [](Motor& self, uint16_t index, uint32_t raw_value) {
            py::gil_scoped_release g; self.set_private_param(index, raw_value);
        }, py::arg("index"), py::arg("raw_value"),
           "Write one private-protocol motor parameter. raw_value is 4 raw bytes,\n"
           "encoded per the parameter's own type. Same MIT restriction as\n"
           "get_private_param().\n\n"
           "Writing 0x700B also rewrites the persisted startup limit torque, so a\n"
           "one-off tweak silently becomes the new boot default.")
        .def("control_stats", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.control_stats(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100,
           "Read the firmware control task's statistics. Blocks for the ACK; raises\n"
           "ProtocolError on NACK.\n\n"
           "The health channel for the no-ACK submit_* path. Poll it off the realtime\n"
           "thread, never from inside a submit loop.")
        // ---- V2.2 diagnostics (follower >= 1.1.2; older firmware NACKs) ----
        .def("read_status_ext", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.read_status_ext(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100,
           "Read the 72-byte extended motor status: read_status() plus the fault\n"
           "words, their latched and stop-time snapshots, the collection-health flags\n"
           "and the raw CAN evidence. Blocks for the ACK.\n\n"
           "Reach for it once something has gone wrong; read_status() stays the right\n"
           "call in a control loop. Needs follower firmware >= 1.1.2 -- older firmware\n"
           "NACKs InvalidCmd -> ProtocolError, and there is no capability query.")
        // force=True costs a CAN round trip and can disturb a running control
        // loop — leave it False when polling.
        .def("fault_report", [](Motor& self, bool force, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.fault_report(force, std::chrono::milliseconds(timeout_ms));
        }, py::arg("force") = false, py::arg("timeout_ms") = 200,
           "Read the diagnostic fault report, merging the motor's fault word, the\n"
           "MCU's firmware-level fault state and the raw CAN reply. Blocks for the ACK.\n\n"
           "force=False answers from the firmware's cache and is cheap enough to poll.\n"
           "force=True makes the MCU issue a fresh CAN read first, which costs a bus\n"
           "round trip and can perturb a running control loop.\n\n"
           "The firmware ACKs OK even with nothing to report, so check report_flags\n"
           "before trusting the fault codes. Same 1.1.2 firmware floor as\n"
           "read_status_ext().")
        .def("get_model", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.get_model(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 1000,
           "本机记录的电机型号(0x59)。**问的是 MCU 的 flash,不是电机** ——\n"
           "电机答不出自己的型号,见 MotorModel。需要从爪固件 >= 1.2.7。\n\n"
           "当一台夹爪表现得像是力矩被整体缩放时先读它:回包里的 t_max_nm /\n"
           "v_max_rad_s 就是当前真正在用的 MIT 量程。")
        .def("set_model", [](Motor& self, uint8_t model_id, unsigned timeout_ms) {
            py::gil_scoped_release g;
            self.set_model(model_id, std::chrono::milliseconds(timeout_ms));
        }, py::arg("model_id"), py::arg("timeout_ms") = 1000,
           "记录本机装的是哪一款电机(0x5A)。**写 MCU flash,掉电保持。**\n\n"
           "**下次上电才生效**,不是立刻:MIT 量程在启动时定下来,控制环已经按它在\n"
           "跑,中途换刻度会让在途的命令和反馈用两套标准解释。和改电机 CAN ID 一样,\n"
           "写完要断电重启(USB 线和电源线同时拔)。\n\n"
           "写错型号不会报错,只会让之后每一帧力矩差一个固定倍率 —— 先用\n"
           "get_model() 确认写进去的是对的。")
        .def("motor_version", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.motor_version(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 3000,
           "**电机自身**的固件版本(不是夹爪固件,那是 gripper.firmware_version)。\n"
           "需要从爪固件 >= 1.2.6;更旧的固件没有 0x58 这条命令,会抛 InvalidCmd。\n\n"
           "先看 valid 再读 version:读不到时在 payload 里如实上报而不是抛异常,\n"
           "因为『电机没应答』和『这里不支持这条命令』是两种不同的诊断,一个 NACK\n"
           "说不清是哪种。\n\n"
           "**已实测**:私有协议下可读(0074s / 0018s 都答了 1.0.5.0.x)。MIT 下能否\n"
           "读到**尚未验证** —— 请求是扩展帧,理论上 MIT 会忽略,但从没在跑 1.2.6 的\n"
           "MIT 机器上试过。别基于任一假设去写开机检查,切协议来回要断两次 24V。\n\n"
           "默认超时 3000 ms:实测往返可达约 1371 ms,原来的 500 ms 会在应答可能到达\n"
           "之前就超时,把一次正常读取显示成电机没反应。\n\n"
           "**可能把电机停掉**:请求帧复用了通信类型 4(电机停止),不认 00 C4 魔数\n"
           "的电机会把它当停止执行。motor_stopped=1 就是这种情况,再发运动命令前\n"
           "要重新 enable()。")
        .def("set_startup_limit_torque", [](Motor& self, float torque_nm) {
            py::gil_scoped_release g; self.set_startup_limit_torque(torque_nm);
        }, py::arg("torque_nm"),
           "Persist the power-on limit torque, N*m, in MCU flash.\n\n"
           "It changes nothing now: the firmware writes this value to the motor's\n"
           "0x700B limit_torque on every boot, so it takes effect at the next\n"
           "power-on. The firmware's own default and maximum are both 6.0 N*m and a\n"
           "value outside 0..6.0 NACKs InvalidParam.\n\n"
           "Unlike set_private_param(0x700B, ...) this works under MIT as well as\n"
           "Private. Blocks for the ACK; raises ProtocolError on NACK -- SysBusy while\n"
           "the firmware's control task is running.")
        .def("get_spec", [](Motor& self) {
            py::gil_scoped_release g; return self.get_spec();
        }, "当前电机的型号规格(额定/峰值/量程)。设备是自身额定值的权威 —— "
           "SDK 里的 MOTOR_*_TORQUE_NM 只是 EL05 的 fallback。字段为 0 表示未知。")
        .def("get_startup_limit_torque", [](Motor& self) {
            py::gil_scoped_release g; return self.get_startup_limit_torque();
        }, "Read back the persisted power-on limit torque, N*m. Blocks for the ACK;\n"
           "raises ProtocolError on NACK or a short reply.");
}

}  // namespace xense::taccap::python

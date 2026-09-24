// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: GripperConfig, GripperEnvelope, power-on auto-cal records
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

#include <taccap/follower_gripper.hpp>

namespace xense::taccap::python {

void bind_gripper_types(py::module_& m) {
    using namespace xense::taccap;
    // ---- V1.7 follower (slave) types ------------------------------------
    py::enum_<protocol::MotorProtocol>(m, "MotorProtocol",
        "CAN protocol the follower's motor speaks (Cmd 0x36 switches, 0x37 reads).\n\n"
        "Mit is what this SDK controls with. Private is the vendor's own protocol\n"
        "and the only one that answers the single-parameter accesses (Cmd 0x38/0x39)\n"
        "-- under Mit those NACK. The choice is persisted in the motor and switching\n"
        "it costs a power cycle each way.")
        .value("Private", protocol::MotorProtocol::Private)
        .value("Mit",     protocol::MotorProtocol::Mit);

    // V2.2 — MotorStatusExt.stop_reason / MotorFaultReport.stop_reason. The
    // fault/monitor *bit masks* stay C++-only, matching MotorStatusBit: Python
    // gets the raw integers and masks them itself.
    py::enum_<protocol::MotorStopReason>(m, "MotorStopReason",
        "Why the follower's motor last stopped; carried as a raw integer in\n"
        "MotorStatusExt.stop_reason and MotorFaultReport.stop_reason.\n\n"
        "HostTimeout means the slave control task found its cached target stale --\n"
        "the host stopped sending. It is NOT the motor's own 0x7028 CAN timeout,\n"
        "which cannot fire on host loss: the MCU keeps re-sending the cached target\n"
        "at 500 Hz, so CAN frames never stop arriving.\n\n"
        "None_ carries the trailing underscore only because None is a keyword.")
        .value("None_",        protocol::MotorStopReason::None)
        .value("Disable",      protocol::MotorStopReason::Disable)
        .value("Emergency",    protocol::MotorStopReason::Emergency)
        .value("ClearFault",   protocol::MotorStopReason::ClearFault)
        .value("LimitStall",   protocol::MotorStopReason::LimitStall)
        .value("ControlError", protocol::MotorStopReason::ControlError)
        .value("HostTimeout",  protocol::MotorStopReason::HostTimeout);

    py::class_<protocol::GripperConfig>(m, "GripperConfig",
        "Follower travel calibration as the firmware persists it (Cmd 0x66/0x67),\n"
        "and the record GripperPosition builds its normalized [0,1] map from.\n\n"
        "The 32-byte record also carries the GripperEnvelope in a trailing block\n"
        "this class does not expose. A config read back from the device still holds\n"
        "those bytes and can be written straight back; one constructed here has them\n"
        "zeroed, so writing that drops a stored envelope -- read, modify, write.")
        .def(py::init([]() {
            protocol::GripperConfig c{};
            c.magic   = protocol::GRIPPER_CONFIG_MAGIC;
            c.version = protocol::GRIPPER_CONFIG_VERSION;
            c.flags   = protocol::GripperConfigFlag::Valid;
            return c;
        }), "Zero record with the magic, the version and the Valid flag filled in.")
        .def_readwrite("magic",        &protocol::GripperConfig::magic,
                       "Record tag, 0x47525052 ('GRPR'); the constructor fills it in.")
        .def_readwrite("version",      &protocol::GripperConfig::version,
                       "Record layout version; the constructor fills in the one this SDK writes.")
        .def_readwrite("flags",        &protocol::GripperConfig::flags,
                       "Bit 0 (0x0001) the record is valid, bit 1 (0x0002) reverse -- 'open'\n"
                       "is the motor's NEGATIVE direction. GripperPosition treats the gripper\n"
                       "as calibrated only when bit 0 is set and the travel span is positive.")
        .def_readwrite("max_open_rad", &protocol::GripperConfig::max_open_rad)
        .def_readwrite("min_open_rad", &protocol::GripperConfig::min_open_rad,
                       "Raw shaft angle normalized 0.0 maps to. NOT always zero: the power-on\n"
                       "auto-calibration zeroes the motor while the jaw is held loaded, so the\n"
                       "firmware stores a small inset here to keep normalized 0.0 a point\n"
                       "control can actually reach.")
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
        "within peak_torque_nm/kp of the measured position, and feed-forward "
        "torque to cont_torque_nm. It lives in firmware because the host link is "
        "100 Hz, phase-locked, and cannot be polled while controlling.\n\n"
        "There is deliberately NO speed ceiling. The only speed an MIT frame can "
        "clamp is the velocity FEED-FORWARD, which impedance control leaves at "
        "zero, so such a clamp would never fire; the motor's own limit_spd "
        "(0x7017) is the right layer but is unreachable while the motor speaks "
        "MIT. Approach speed comes from peak_torque_nm/kd instead.")
        .def(py::init<>(), "All-zero envelope: flags 0, i.e. neither valid nor enforced.")
        .def_readwrite("cont_torque_nm",     &protocol::GripperEnvelope::cont_torque_nm,
                       "Indefinitely holdable torque, N*m; the ceiling for the feed-forward\n"
                       "term. 0 = unlimited.")
        .def_readwrite("peak_torque_nm",     &protocol::GripperEnvelope::peak_torque_nm,
                       "Motion transient ceiling, N*m, applied by holding the commanded\n"
                       "position within peak_torque_nm/kp of the measured one. 0 = unlimited.\n"
                       "It also sets the approach speed, about peak_torque_nm/kd: measured on\n"
                       "firmware 1.1.5 with peak = 1.5 N*m and kd = 1.0, 1.70 rad/s.")
        .def_readwrite("temp_derate_start_c", &protocol::GripperEnvelope::temp_derate_start_c,
                       "Board/case temperature (C) where the linear torque derate begins.\n"
                       "0 = firmware default, 90 C.")
        .def_readwrite("temp_wall_c",        &protocol::GripperEnvelope::temp_wall_c,
                       "Board/case temperature (C) above which only the minimum hold torque is\n"
                       "allowed. 0 = firmware default, 100 C. This is case temperature, not the\n"
                       "winding, which runs hotter by an unmeasured margin.")
        .def_readwrite("flags",              &protocol::GripperEnvelope::flags,
                       "GRIPPER_ENVELOPE_VALID (0x0001) marks the record written,\n"
                       "GRIPPER_ENVELOPE_ENFORCE (0x0002) makes the firmware apply it every\n"
                       "cycle -- without it the record is stored and ignored. The top nibble is\n"
                       "the layout version, stamped by FollowerGripper.set_envelope() so\n"
                       "firmware built against another field order refuses the record instead\n"
                       "of misreading it.")
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

    // ---- HomeDiagReport (0x57) ---------------------------------------------
    py::enum_<protocol::HomeState>(m, "HomeState",
        "Phase of the power-on auto-calibration. The sequence ends with\n"
        "OpenFinalBackoff, a deliberate 0.12 rad retreat from the open limit --\n"
        "so a jaw resting short of max_open_rad is expected, not a fault.")
        .value("Idle", protocol::HomeState::Idle)
        .value("Init", protocol::HomeState::Init)
        .value("ClearFault", protocol::HomeState::ClearFault)
        .value("Close", protocol::HomeState::Close)
        .value("CloseBackoff", protocol::HomeState::CloseBackoff)
        .value("CloseAverageHold", protocol::HomeState::CloseAverageHold)
        .value("SetZero", protocol::HomeState::SetZero)
        .value("PostZero", protocol::HomeState::PostZero)
        .value("Open", protocol::HomeState::Open)
        .value("OpenBackoff", protocol::HomeState::OpenBackoff)
        .value("SaveMax", protocol::HomeState::SaveMax)
        .value("OpenFinalBackoff", protocol::HomeState::OpenFinalBackoff)
        .value("Done", protocol::HomeState::Done)
        .value("Error", protocol::HomeState::Error);

    py::enum_<protocol::HomeFail>(m, "HomeFail", "Why auto-calibration stopped.")
        .value("None", protocol::HomeFail::None)
        .value("Timeout", protocol::HomeFail::Timeout)
        .value("ClearFault", protocol::HomeFail::ClearFault)
        .value("SetZero", protocol::HomeFail::SetZero)
        .value("MaxOpenTooSmall", protocol::HomeFail::MaxOpenTooSmall)
        .value("SaveConfig", protocol::HomeFail::SaveConfig);

    py::class_<protocol::HomeDiagReport>(m, "HomeDiagReport",
        "What the power-on auto-calibration actually did (Cmd 0x57).\n\n"
        "It exists because the sequence's failure reasons only ever went to the\n"
        "firmware's UART7, which is not wired to USB -- from the host, a\n"
        "calibration that ended early and one that ended right look identical,\n"
        "since both leave a plausible max_open_rad behind.\n\n"
        "To judge whether a stall call was real, compare `last_abs_torque`\n"
        "against `stall_threshold_nm`: the threshold is the commanded limit\n"
        "times the model's feedback saturation ratio times a margin, and a\n"
        "measurement barely above it means the call was marginal. On a model\n"
        "whose saturation ratio is not measured yet (0 in the firmware's table)\n"
        "the threshold falls back to a conservative ratio, which errs towards\n"
        "calling the stall EARLY.")
        .def_readonly("version", &protocol::HomeDiagReport::version)
        .def_readonly("state", &protocol::HomeDiagReport::state,
                      "HomeState value of the last/current phase.")
        .def_readonly("fail_reason", &protocol::HomeDiagReport::fail_reason,
                      "HomeFail value; 0 when nothing failed.")
        .def_readonly("flags", &protocol::HomeDiagReport::flags,
                      "0x01 active, 0x02 done, 0x04 attempted, 0x08 saw real motion.")
        .def_readonly("fail_ret", &protocol::HomeDiagReport::fail_ret)
        .def_readonly("elapsed_ms", &protocol::HomeDiagReport::elapsed_ms)
        .def_readonly("state_elapsed_ms", &protocol::HomeDiagReport::state_elapsed_ms,
                      "Time in the current phase -- a hang shows up here first.")
        .def_readonly("vel_cmd", &protocol::HomeDiagReport::vel_cmd)
        .def_readonly("torque_limit_nm", &protocol::HomeDiagReport::torque_limit_nm,
                      "Last commanded current limit, N*m.")
        .def_readonly("stall_threshold_nm", &protocol::HomeDiagReport::stall_threshold_nm,
                      "limit * the model's feedback saturation ratio * margin.")
        .def_readonly("last_abs_torque", &protocol::HomeDiagReport::last_abs_torque,
                      "|torque| at the last stall evaluation. Compare with\n"
                      "stall_threshold_nm to see how marginal the call was.")
        .def_readonly("last_abs_vel", &protocol::HomeDiagReport::last_abs_vel)
        .def_readonly("peak_vel", &protocol::HomeDiagReport::peak_vel)
        .def_readonly("progress_rad", &protocol::HomeDiagReport::progress_rad)
        .def_readonly("last_pos", &protocol::HomeDiagReport::last_pos)
        .def_readonly("close_pos", &protocol::HomeDiagReport::close_pos,
                      "Measured closed limit, raw rad.")
        .def_readonly("open_pos", &protocol::HomeDiagReport::open_pos,
                      "Measured open limit, raw rad.")
        .def_readonly("open_sign", &protocol::HomeDiagReport::open_sign)
        .def_readonly("reenable_count", &protocol::HomeDiagReport::reenable_count,
                      "Times the motor was found in Reset while the MCU believed\n"
                      "it enabled.")
        .def_readonly("report_miss_count", &protocol::HomeDiagReport::report_miss_count,
                      "Times active reporting was asked for and never delivered, so\n"
                      "the firmware fell back to polling. Non-zero means this motor\n"
                      "does not honour MIT instruction 13.")
        .def("__repr__", [](const protocol::HomeDiagReport& d) {
            char buf[224];
            std::snprintf(buf, sizeof(buf),
                "HomeDiagReport(state=%u fail=%u close=%.4f open=%.4f "
                "thr=%.3f last_tq=%.3f %ums)",
                (unsigned)d.state, (unsigned)d.fail_reason, d.close_pos,
                d.open_pos, d.stall_threshold_nm, d.last_abs_torque,
                (unsigned)d.elapsed_ms);
            return std::string(buf);
        });

    // ---- Envelope audit -----------------------------------------------------
    namespace Issue = xense::taccap::GripperEnvelopeIssue;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_NOT_WRITTEN")   = Issue::NotWritten;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_NOT_ENFORCED")  = Issue::NotEnforced;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_LAYOUT_MISMATCH") = Issue::LayoutMismatch;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_PEAK_UNLIMITED")  = Issue::PeakUnlimited;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_CONT_UNLIMITED")  = Issue::ContUnlimited;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_CONT_ABOVE_STALL_RATING") =
        Issue::ContAboveStallRating;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_PEAK_NOT_ABOVE_CONT") =
        Issue::PeakNotAboveCont;
    envmod.attr("GRIPPER_ENVELOPE_ISSUE_REPAIR_MASK") = Issue::RepairMask;

    py::class_<xense::taccap::EnvelopeAudit>(m, "EnvelopeAudit",
        "What a follower stores, what it would be told to store, and what it is\n"
        "actually enforcing -- three different things.\n\n"
        "`stored` is the record in flash. `effective` is what the firmware applies.\n"
        "They differ in a way nothing on the host could see before: the firmware\n"
        "clamps cont_torque_nm down to the installed motor's continuous STALL\n"
        "rating (1.1 N*m on an EL05 -- the 1.8 on the datasheet's front page is the\n"
        "ROTATING rating) and reports that only on a UART that is not wired to USB.\n"
        "So a gripper can print cont=1.800 forever while running at 1.100.\n\n"
        "`effective` is None when the firmware applies NOTHING: the record was never\n"
        "written, Enforce is clear, or the layout version is one this firmware\n"
        "refuses. None is not 'an empty envelope' -- in this record 0 means\n"
        "unlimited, so a zeroed struct would state the opposite. On such a device\n"
        "there is no position-error clamp, no I2t derate and no temperature wall.\n\n"
        "    a = g.audit_envelope()\n"
        "    if not a.ok():\n"
        "        print(a.detail)")
        .def_readonly("stored", &xense::taccap::EnvelopeAudit::stored,
                      "The record as read from flash.")
        .def_readonly("recommended", &xense::taccap::EnvelopeAudit::recommended,
                      "Derived from this device's own motor spec (0x56): cont = the\n"
                      "continuous stall rating, peak = the rated (rotating) torque.\n"
                      "peak is deliberately NOT the torque range -- that is the motor's\n"
                      "absolute ceiling and the firmware's own 0x700B default, so using\n"
                      "it would make the envelope a no-op.")
        .def_readonly("effective", &xense::taccap::EnvelopeAudit::effective,
                      "What the firmware enforces right now, or None when it enforces\n"
                      "nothing at all. Compare against THIS, not against `stored`.")
        .def_readonly("issues", &xense::taccap::EnvelopeAudit::issues,
                      "Bitmask of GRIPPER_ENVELOPE_ISSUE_* constants.")
        .def_readonly("peak_from_device",
                      &xense::taccap::EnvelopeAudit::peak_from_device,
                      "True when peak came from the device's spec rather than a\n"
                      "compiled-in fallback.")
        .def_readonly("cont_from_device",
                      &xense::taccap::EnvelopeAudit::cont_from_device,
                      "Separate from peak_from_device on purpose: the firmware's table\n"
                      "gives every RS0x a real rated torque but no stall rating, so one\n"
                      "combined flag would read 'from device' while cont silently came\n"
                      "from the EL05 fallback.")
        .def_readonly("motor_model", &xense::taccap::EnvelopeAudit::motor_model,
                      "'EL05'; empty when the spec could not be read.")
        .def_readonly("detail", &xense::taccap::EnvelopeAudit::detail,
                      "One human-readable line per issue, ready to print.")
        .def_property_readonly("ok", &xense::taccap::EnvelopeAudit::ok,
                               "No issues at all.")
        .def_property_readonly("needs_write",
                               &xense::taccap::EnvelopeAudit::needs_write,
                               "ensure_envelope() would write. Narrower than `not ok`:\n"
                               "an envelope TIGHTER than recommended is reported and\n"
                               "never repaired, because tighter is not a hole.")
        .def("__repr__", [](const xense::taccap::EnvelopeAudit& a) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "EnvelopeAudit(%s, stored cont=%.3f peak=%.3f, effective %s, "
                "issues=0x%02x)",
                a.ok() ? "ok" : (a.needs_write() ? "needs write" : "advisory"),
                a.stored.cont_torque_nm, a.stored.peak_torque_nm,
                a.effective ? "present" : "NONE", a.issues);
            return std::string(buf);
        });

    py::class_<xense::taccap::EnvelopeWrite>(m, "EnvelopeWrite",
        "What ensure_envelope() did.\n\n"
        "`wrote` is False when the stored record was already correct -- that is the\n"
        "idempotent case, not a failure.")
        .def_readonly("before", &xense::taccap::EnvelopeWrite::before,
                      "The audit that drove the decision.")
        .def_readonly("wrote", &xense::taccap::EnvelopeWrite::wrote,
                      "True if MCU flash was written.")
        .def_readonly("written", &xense::taccap::EnvelopeWrite::written,
                      "The record written. Meaningful only when `wrote`.")
        .def("__repr__", [](const xense::taccap::EnvelopeWrite& w) {
            char buf[160];
            if (w.wrote) {
                std::snprintf(buf, sizeof(buf),
                    "EnvelopeWrite(wrote cont=%.3f peak=%.3f)",
                    w.written.cont_torque_nm, w.written.peak_torque_nm);
            } else {
                std::snprintf(buf, sizeof(buf),
                    "EnvelopeWrite(nothing to do)");
            }
            return std::string(buf);
        });

    py::class_<protocol::GripperAutoCalConfig>(m, "GripperAutoCalConfig",
        "Power-on auto-calibration config, persisted in the follower (Cmd 0x68/0x69).\n\n"
        "With the Enable flag set, the firmware calibrates itself on every power-up:\n"
        "it closes until the motor stalls at close_stall_torque_nm -- that pose\n"
        "becomes zero -- then opens until it stalls at open_stall_torque_nm, and\n"
        "that span becomes max_open.\n\n"
        "V2.2 changed the procedure without changing the wire layout: a stall is\n"
        "confirmed from one sample held for stall_hold_ms rather than averaged, the\n"
        "open stall records the frame before the trigger, and 0.013 rad is subtracted\n"
        "from the saved max_open as margin. Expect a slightly smaller max_open than\n"
        "the same hardware reported on 1.1.1.")
        .def(py::init([]() {
            protocol::GripperAutoCalConfig c{};
            c.magic   = protocol::GRIPPER_AUTO_CAL_MAGIC;
            c.version = protocol::GRIPPER_AUTO_CAL_VERSION;
            c.flags   = protocol::GripperAutoCalFlag::Valid;
            return c;
        }), "Zero record with the magic, the version and the Valid flag filled in.\n"
            "Auto-cal still needs the Enable bit set in flags.")
        .def_readwrite("magic",                &protocol::GripperAutoCalConfig::magic,
                       "Record tag, 0x4743414C ('GCAL'); the constructor fills it in.")
        .def_readwrite("version",              &protocol::GripperAutoCalConfig::version,
                       "Record layout version; the constructor fills in the one this SDK writes.")
        .def_readwrite("flags",                &protocol::GripperAutoCalConfig::flags,
                       "Bit 0 (0x0001) the record is valid, bit 1 (0x0002) run auto-cal on\n"
                       "power-up. Clearing bit 1 keeps the tuning without arming it.")
        .def_readwrite("close_stall_torque_nm", &protocol::GripperAutoCalConfig::close_stall_torque_nm,
                       "Torque, N*m, the closing stroke stalls at; that pose becomes zero.")
        .def_readwrite("open_stall_torque_nm",  &protocol::GripperAutoCalConfig::open_stall_torque_nm,
                       "Torque, N*m, the opening stroke stalls at; that span becomes max_open.")
        .def_readwrite("close_speed_rad_s",    &protocol::GripperAutoCalConfig::close_speed_rad_s,
                       "Auto-cal close-to-stall sweep speed, rad/s. A REAL velocity command to\n"
                       "the motor, paired with close_stall_torque_nm -- not a setpoint ramp like\n"
                       "ForcePositionConfig.close_speed_radps, and not used by any runtime\n"
                       "control path.\n\n"
                       "It is the SLOW/final-approach speed: while the jaw is still more than\n"
                       "0.32 rad out AND a previous calibration survives in flash, the firmware\n"
                       "substitutes a fixed 2.0 rad/s instead. With no calibration history it\n"
                       "applies for the whole sweep. Firmware clamps it to 0.02..3.0 rad/s on\n"
                       "both write and load; default 0.25.")
        .def_readwrite("open_speed_rad_s",     &protocol::GripperAutoCalConfig::open_speed_rad_s,
                       "Auto-cal open-to-stall sweep speed, rad/s, paired with\n"
                       "open_stall_torque_nm. Same nature and the same 0.02..3.0 clamp as\n"
                       "close_speed_rad_s; default 0.35.")
        .def_readwrite("stall_hold_ms",        &protocol::GripperAutoCalConfig::stall_hold_ms,
                       "How long a stall must hold before it counts, ms.")
        .def_readwrite("startup_delay_ms",     &protocol::GripperAutoCalConfig::startup_delay_ms,
                       "Delay after power-on before auto-cal starts, ms.")
        .def_readwrite("post_zero_delay_ms",   &protocol::GripperAutoCalConfig::post_zero_delay_ms,
                       "Delay between setting the zero and starting the open stroke, ms.")
        .def_readwrite("close_confirm_count",  &protocol::GripperAutoCalConfig::close_confirm_count,
                       "Compat only -- V2.2 firmware confirms a stall once and ignores this.")
        .def_readwrite("open_confirm_count",   &protocol::GripperAutoCalConfig::open_confirm_count,
                       "Compat only -- V2.2 firmware confirms a stall once and ignores this.")
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
    py::class_<protocol::GripperAutoCalStallParam>(m, "GripperAutoCalStallParam",
        "Short 0x68 write that patches only the stall-detection fields, leaving the\n"
        "speeds, the flags and the magic/version header at their stored values.\n\n"
        "Saves a read-modify-write when tuning stall torque, and cannot clobber the\n"
        "rest of the config from a stale read. Firmware older than follower 1.1.2\n"
        "NACKs LengthMismatch -- write a full GripperAutoCalConfig there.")
        .def(py::init([]() { return protocol::GripperAutoCalStallParam{}; }),
             "All-zero patch; fill in the fields to write.")
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::close_stall_torque_nm,
                       "Torque, N*m, the closing stroke stalls at; that pose becomes zero.")
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::open_stall_torque_nm,
                       "Torque, N*m, the opening stroke stalls at; that span becomes max_open.")
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParam::stall_hold_ms,
                       "How long a stall must hold before it counts, ms.")
        .def("__repr__", [](const protocol::GripperAutoCalStallParam& p) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalStallParam(close=%.3fNm, open=%.3fNm, hold=%ums)",
                p.close_stall_torque_nm, p.open_stall_torque_nm, p.stall_hold_ms);
            return std::string(buf);
        });

    py::class_<protocol::GripperAutoCalStallParamEx>(m, "GripperAutoCalStallParamEx",
        "GripperAutoCalStallParam plus the two delays and the compat-only confirm\n"
        "counts -- the 16-byte form of the partial 0x68 write. Same rules: speeds,\n"
        "flags and header keep their stored values, and follower firmware older than\n"
        "1.1.2 NACKs LengthMismatch.")
        .def(py::init([]() { return protocol::GripperAutoCalStallParamEx{}; }),
             "All-zero patch; fill in the fields to write.")
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::close_stall_torque_nm,
                       "Torque, N*m, the closing stroke stalls at; that pose becomes zero.")
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::open_stall_torque_nm,
                       "Torque, N*m, the opening stroke stalls at; that span becomes max_open.")
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParamEx::stall_hold_ms,
                       "How long a stall must hold before it counts, ms.")
        .def_readwrite("startup_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::startup_delay_ms,
                       "Delay after power-on before auto-cal starts, ms.")
        .def_readwrite("post_zero_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::post_zero_delay_ms,
                       "Delay between setting the zero and starting the open stroke, ms.")
        .def_readwrite("close_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::close_confirm_count,
                       "Compat only -- V2.2 firmware confirms a stall once and ignores this.")
        .def_readwrite("open_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::open_confirm_count,
                       "Compat only -- V2.2 firmware confirms a stall once and ignores this.")
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

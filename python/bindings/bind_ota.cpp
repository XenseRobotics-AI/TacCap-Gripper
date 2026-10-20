// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: OtaStatus / OtaTargetVersion / OtaSession
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_ota(py::module_& m) {
    using namespace xense::taccap;
    // ---- OtaSession (V1.3) ---------------------------------------------
    // OtaStatus is the wire payload returned by Cmd::OtaGetStatus; bound
    // here so `OtaSession.get_status()` can hand it back to Python.
    py::class_<xense::taccap::protocol::OtaStatus>(m, "OtaStatus",
        "The MCU's OTA state machine as of the last OtaSession.get_status().")
        .def_readonly("state",         &xense::taccap::protocol::OtaStatus::state,
                      "0 idle, 1 started (awaiting blocks), 2 receiving, 3 verified (CRC32\n"
                      "passed), 4 applying (bank swap in flight), 5 error. State 5 sticks\n"
                      "until OtaSession.abort() clears it.")
        .def_readonly("error_code",    &xense::taccap::protocol::OtaStatus::error_code,
                      "Last ErrorCode the OTA handler reported -- OtaBusy, OtaNotStarted,\n"
                      "OtaOffsetErr, OtaFlashErr, OtaVerifyFail, OtaSizeExceed and so on.\n"
                      "0 means none.")
        .def_readonly("bytes_written", &xense::taccap::protocol::OtaStatus::bytes_written,
                      "Bytes of the image the firmware has accepted so far.")
        .def_readonly("progress_ppt",  &xense::taccap::protocol::OtaStatus::progress_ppt,
                      "Progress in per-mille, 0-1000. Not percent.")
        .def("__repr__", [](const xense::taccap::protocol::OtaStatus& s) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "OtaStatus(state=%u, err=0x%02X, bytes=%u, ppt=%u)",
                s.state, s.error_code, s.bytes_written, s.progress_ppt);
            return std::string(buf);
        });

    py::class_<OtaSession::TargetVersion>(m, "OtaTargetVersion",
        "The version stamped into OtaStart.\n\n"
        "Firmware keeps it as the upgrade record and checks it against the bank\n"
        "metadata at apply time, so it has to describe the image actually being\n"
        "flashed. Each component is a single byte, 0-255.")
        .def(py::init<>(),
             "All four components default-constructed; set them before flashing.")
        .def(py::init<uint8_t, uint8_t, uint8_t, uint8_t>(),
             py::arg("major"), py::arg("minor"), py::arg("patch"), py::arg("build"),
             "Version to stamp in, e.g. OtaTargetVersion(1, 2, 6, 0). Releases are\n"
             "named in three parts; the fourth byte is the build and firmware has\n"
             "always set it to 0.")
        .def_readwrite("major", &OtaSession::TargetVersion::major)
        .def_readwrite("minor", &OtaSession::TargetVersion::minor)
        .def_readwrite("patch", &OtaSession::TargetVersion::patch)
        .def_readwrite("build", &OtaSession::TargetVersion::build);

    py::class_<OtaSession>(m, "OtaSession",
        "Firmware update over the same serial link the sensors use, reached as\n"
        "gripper.ota. No SWD probe involved.\n\n"
        "FLASHING IS DESTRUCTIVE: update_from_file() and update_from_bytes()\n"
        "overwrite the MCU's inactive flash bank and swap it in. The wrong image\n"
        "for the unit leaves it unbootable, and there is no undo once apply has\n"
        "gone through. POWER-CYCLE the gripper afterwards. Follower: unplug the 24 V\n"
        "cable, wait ~2 s, plug it back (USB may stay in) -- the MCU and motor run\n"
        "on 24 V, so pulling only USB resets nothing.\n"
        "Leader: unplug and replug its USB.\n\n"
        "get_status() and abort() are the recovery half: they tell you why an\n"
        "earlier attempt left the state machine stuck, and clear it.")
        .def("update_from_file", [](OtaSession& self,
                                    const std::string& path,
                                    const OtaSession::TargetVersion& v,
                                    py::object on_progress) {
            OtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t wr, uint32_t tot) {
                    call_into_python("OtaSession progress",
                                     [&] { (*pycb)(wr, tot); });
                };
            }
            py::gil_scoped_release gil;
            self.update_from_file(path, v, std::move(cb));
        },
            py::arg("firmware_path"),
            py::arg("target_version"),
            py::arg("on_progress") = py::none(),
            "Flash a firmware image from disk: read it, CRC32 it, then run\n"
            "start -> write blocks -> verify -> apply.\n\n"
            "DESTRUCTIVE -- see the class docstring. Blocks for the whole transfer\n"
            "with the GIL released. on_progress, when given, is called after every\n"
            "accepted block with (bytes_written, total_bytes); it runs on the calling\n"
            "thread, and an exception raised inside it is reported as unraisable and\n"
            "swallowed.\n\n"
            "Any failure before apply sends OtaAbort and re-raises, leaving the\n"
            "device idle for another attempt. After the apply ACK the MCU reboots and\n"
            "every further command on this transport times out -- that is the\n"
            "expected end, not a failure; re-open the gripper once USB has\n"
            "re-enumerated.\n\n"
            "POWER-CYCLE before trusting any measurement afterwards (follower: cut\n"
            "24 V for ~2 s; leader: replug USB). The bank swap is\n"
            "a soft reset: the unit comes back with the right version, a running\n"
            "stream and clean counters while quietly dropping status frames.\n\n"
            "Raises IoError when the file cannot be read, ProtocolError on a NACK, on\n"
            "an empty image, or on one past the 456 KiB single-bank maximum.")
        .def("update_from_bytes", [](OtaSession& self,
                                     py::bytes blob,
                                     const OtaSession::TargetVersion& v,
                                     py::object on_progress) {
            // Materialise the bytes view; copy into std::vector once.
            std::string buf = blob;
            std::vector<uint8_t> fw(buf.begin(), buf.end());
            OtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t wr, uint32_t tot) {
                    call_into_python("OtaSession progress",
                                     [&] { (*pycb)(wr, tot); });
                };
            }
            py::gil_scoped_release gil;
            self.update_from_bytes(fw, v, std::move(cb));
        },
            py::arg("firmware_bytes"),
            py::arg("target_version"),
            py::arg("on_progress") = py::none(),
            "Same sequence as update_from_file(), from an image already in memory;\n"
            "the bytes are copied once into the session's own buffer.\n\n"
            "DESTRUCTIVE, blocking, and under the same power-cycle rule -- read\n"
            "update_from_file() before using this.")
        .def("get_status", [](OtaSession& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.get_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 500u,
           "Read the MCU's OTA state machine. Changes nothing.\n\n"
           "Blocks up to timeout_ms with the GIL released. Safe at any point,\n"
           "including when no update is running -- this is how to find out whether an\n"
           "earlier attempt left the device in state 5 (error), which only abort()\n"
           "clears. Raises ProtocolError on NACK, TimeoutError when the device does\n"
           "not answer.")
        .def("abort", [](OtaSession& self) {
            py::gil_scoped_release gil;
            self.abort();
        }, "Clear an in-flight or stuck OTA session so a fresh update can start.\n\n"
           "Best-effort and never raises: anything that fails here is recoverable by\n"
           "another update or by resetting the MCU. Blocks briefly with the GIL\n"
           "released.");

    m.def("crc32_iso_hdlc", [](py::buffer b) {
        py::buffer_info info = b.request();
        if (info.itemsize != 1) {
            throw py::value_error("crc32_iso_hdlc: needs a bytes-like buffer");
        }
        return xense::taccap::crc32_iso_hdlc(
            static_cast<const uint8_t*>(info.ptr),
            static_cast<size_t>(info.size));
    }, py::arg("data"),
       "Compute CRC32 with the same parameters as zlib.crc32 / firmware.");

    // ---- MotorOtaSession: reflash the RobStride motor itself ------------
    py::class_<MotorOtaSession>(m, "MotorOtaSession",
        "Motor firmware upgrade -- this flashes the RobStride motor module's own\n"
        "program, not the gripper MCU (that is gripper.ota). Each frame goes through\n"
        "Motor.can_ext_xfer (0x5B, follower firmware >= 1.2.8), forwarded by the MCU,\n"
        "which waits for the motor's reply; the flow runs on the host.\n\n"
        "Precondition (measured): the motor must be in the **private protocol** --\n"
        "under MIT the motor does not reply to OTA frames. Switching protocol needs a\n"
        "24V power cycle; and gripper OTA switches the motor back to MIT, so motor\n"
        "OTA must come after gripper OTA.\n\n"
        "**The protocol does not check the model**: the upgrade package has no model\n"
        "field, and an RS00 package flashed into an EL05 motor is accepted too. Call\n"
        "preflight(expected_model) before update.\n\n"
        "On a failure midway, RobStride's advice is to power-cycle the motor and\n"
        "flash again from the start.")
        .def(py::init([](Motor& motor, uint8_t can_id, uint16_t handshake_timeout_ms,
                         uint16_t data_timeout_ms, int max_attempts, int start_attempts,
                         int max_resumes) {
                 MotorOtaSession::Options o;
                 o.start_attempts = start_attempts;
                 o.handshake_timeout_ms = handshake_timeout_ms;
                 o.data_timeout_ms = data_timeout_ms;
                 o.max_attempts = max_attempts;
                 o.max_resumes = max_resumes;
                 return std::make_unique<MotorOtaSession>(motor, can_id, o);
             }),
             py::arg("motor"), py::arg("can_id"),
             py::arg("handshake_timeout_ms") = 2000, py::arg("data_timeout_ms") = 500,
             py::arg("max_attempts") = 3, py::arg("start_attempts") = 6,
             py::arg("max_resumes") = 64,
             py::keep_alive<1, 2>(),
             "Bind to one follower's motor. Read can_id with motor.get_can_id().\n"
             "The timeouts are per attempt; max_attempts is the number of resends when\n"
             "a single frame gets no reply, and start_attempts is given separately for\n"
             "the start frame: the motor first reboots into the bootloader before it\n"
             "replies, measured at about 4 s.\n"
             "max_resumes is how many times over the whole upgrade the motor is allowed\n"
             "to ask to rewind and resume.")
        .def("preflight", [](MotorOtaSession& self, const std::string& expected_model) {
            py::gil_scoped_release g;
            self.preflight(expected_model);
        }, py::arg("expected_model"),
           "The gate before flashing: the motor must be in the private protocol, and\n"
           "the motor model recorded on this unit must be expected_model (\"RS00\" /\n"
           "\"EL05\"). Raises ProtocolError otherwise.\n"
           "When the model is only the compile-time default (not a record written to\n"
           "flash) it only warns -- check the nameplate.")
        .def("read_uid", [](MotorOtaSession& self) {
            std::array<uint8_t, 8> uid;
            {
                py::gil_scoped_release g;
                uid = self.read_uid();
            }
            return py::bytes(reinterpret_cast<const char*>(uid.data()), uid.size());
        }, "Communication type 0: read the motor's 64-bit MCU UID. Read-only; does not\n"
           "change the motor's state.")
        .def("update_from_file", [](MotorOtaSession& self, const std::string& path,
                                    py::object on_progress) {
            MotorOtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t done, uint32_t total) {
                    call_into_python("MotorOtaSession progress",
                                     [&] { (*pycb)(done, total); });
                };
            }
            py::gil_scoped_release g;
            self.update_from_file(path, std::move(cb));
        }, py::arg("path"), py::arg("on_progress") = py::none(),
           "**Rewrites motor flash.** Full flow: read UID -> start -> info -> data pack\n"
           "by pack -> end.\n"
           "on_progress(packs_done, packs_total) is called every 256 packs, and once\n"
           "more at the end.\n"
           "Raises TimeoutError when the motor stops replying, ProtocolError when the\n"
           "motor reports an unrecoverable failure.\n"
           "After the end frame is acknowledged the motor reboots and returns to the MIT\n"
           "protocol (measured on 0086s), so the version cannot be read right then.")
        .def("update_from_bytes", [](MotorOtaSession& self, py::bytes blob,
                                     py::object on_progress) {
            const std::string buf = blob;
            std::vector<uint8_t> image(buf.begin(), buf.end());
            MotorOtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t done, uint32_t total) {
                    call_into_python("MotorOtaSession progress",
                                     [&] { (*pycb)(done, total); });
                };
            }
            py::gil_scoped_release g;
            self.update_from_bytes(image, std::move(cb));
        }, py::arg("image"), py::arg("on_progress") = py::none(),
           "Same as update_from_file, with the image already in memory. **Rewrites\n"
           "motor flash.**\n"
           "Afterwards the motor likewise reboots back into the MIT protocol, so the\n"
           "version cannot be read right then.");
}

}  // namespace xense::taccap::python

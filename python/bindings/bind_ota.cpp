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
        "gone through. POWER-CYCLE the gripper afterwards -- on a follower that\n"
        "means cutting the 24 V supply, not just unplugging USB, because 24 V is a\n"
        "separate rail that a USB replug never interrupts.\n\n"
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
            "POWER-CYCLE before trusting any measurement afterwards. The bank swap is\n"
            "a soft reset: the unit comes back with the right version, a running\n"
            "stream and clean counters while quietly dropping status frames. Cut the\n"
            "24 V supply, not just USB.\n\n"
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
}

}  // namespace xense::taccap::python

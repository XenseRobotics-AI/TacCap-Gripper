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
        "电机固件升级 —— 刷的是 RobStride 电机模组自己的程序,不是夹爪 MCU\n"
        "(那是 gripper.ota)。每一帧经 Motor.can_ext_xfer(0x5B,从爪固件 >= 1.2.8)\n"
        "由 MCU 转发并等电机应答,流程在主机侧。\n\n"
        "前提(实测):电机必须在**私有协议**下 —— MIT 下电机不回 OTA 帧。切协议要\n"
        "断 24V;而夹爪 OTA 会把电机切回 MIT,所以电机 OTA 要放在夹爪 OTA 之后。\n\n"
        "**协议不校验型号**:升级包里没有型号字段,RS00 的包刷进 EL05 电机也会被\n"
        "接受。update 之前先调 preflight(expected_model)。\n\n"
        "中途失败时,RobStride 的建议是给电机断电重上,从头再刷。")
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
             "绑定到一台从爪的 motor。can_id 用 motor.get_can_id() 读。\n"
             "超时是每次尝试的时长;max_attempts 是单帧无应答时的重发次数,\n"
             "start_attempts 单独给启动帧:电机先重启进 bootloader 才应答,实测约 4 秒。\n"
             "max_resumes 是整次升级里允许电机要求回退续传的次数。")
        .def("preflight", [](MotorOtaSession& self, const std::string& expected_model) {
            py::gil_scoped_release g;
            self.preflight(expected_model);
        }, py::arg("expected_model"),
           "刷之前的把关:电机必须在私有协议下,且本机记录的电机型号必须是\n"
           "expected_model(\"RS00\" / \"EL05\")。不满足就抛 ProtocolError。\n"
           "型号只是编译期默认值(不是写进 flash 的记录)时只告警 —— 请核对铭牌。")
        .def("read_uid", [](MotorOtaSession& self) {
            std::array<uint8_t, 8> uid;
            {
                py::gil_scoped_release g;
                uid = self.read_uid();
            }
            return py::bytes(reinterpret_cast<const char*>(uid.data()), uid.size());
        }, "通信类型 0,读电机的 64 位 MCU UID。只读,不改变电机状态。")
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
           "**会改写电机 flash。**完整流程:读 UID -> 启动 -> 信息 -> 逐包数据 -> 结束。\n"
           "on_progress(packs_done, packs_total) 每 256 包调用一次,结束时再调一次。\n"
           "电机不再应答抛 TimeoutError,电机报告无法恢复的失败抛 ProtocolError。\n"
           "结束帧确认后电机会重启,并回到 MIT 协议(实测 0086s),当场读不到版本号。")
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
           "同 update_from_file,镜像已在内存里。**会改写电机 flash。**\n"
           "结束后电机同样重启回 MIT 协议,当场读不到版本号。");
}

}  // namespace xense::taccap::python

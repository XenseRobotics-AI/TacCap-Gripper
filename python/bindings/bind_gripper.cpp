// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: discovery, LeaderGripper, FollowerGripper
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_gripper(py::module_& m) {
    using namespace xense::taccap;
    // ---- discovery ------------------------------------------------------
    py::enum_<discovery::Side>(m, "Side")
        .value("Left",    discovery::Side::Left)
        .value("Right",   discovery::Side::Right)
        .value("Unknown", discovery::Side::Unknown);

    py::enum_<discovery::Role>(m, "Role")
        .value("Leader",   discovery::Role::Leader)
        .value("Follower", discovery::Role::Follower)
        .value("Unknown",  discovery::Role::Unknown);

    py::class_<discovery::ParsedSerial>(m, "ParsedSerial")
        .def_readonly("raw",      &discovery::ParsedSerial::raw)
        .def_readonly("product",  &discovery::ParsedSerial::product)
        .def_readonly("batch",    &discovery::ParsedSerial::batch)
        .def_property_readonly("line", [](const discovery::ParsedSerial& p) {
            return std::string(1, p.line);
        })
        .def_readonly("sequence", &discovery::ParsedSerial::sequence)
        .def_readonly("side",     &discovery::ParsedSerial::side)
        .def_readonly("role",     &discovery::ParsedSerial::role)
        .def_readonly("valid",    &discovery::ParsedSerial::valid)
        .def("__repr__", [](const discovery::ParsedSerial& p) {
            return std::string("ParsedSerial(raw=") + p.raw +
                   ", product=" + p.product +
                   ", line=" + std::string(1, p.line) +
                   ", seq=" + p.sequence +
                   ", side=" + (p.side ? discovery::to_string(*p.side) : "None") +
                   ", role=" + discovery::to_string(p.role) +
                   ", valid=" + (p.valid ? "True" : "False") + ")";
        });
    m.def("parse_serial", &discovery::parse_serial, py::arg("serial"));

    py::class_<discovery::GripperEndpoints>(m, "GripperEndpoints")
        .def_readonly("side",                 &discovery::GripperEndpoints::side)
        .def_readonly("role",                 &discovery::GripperEndpoints::role)
        .def_readonly("mcu_device",           &discovery::GripperEndpoints::mcu_device)
        .def_readonly("mcu_serial",           &discovery::GripperEndpoints::mcu_serial)
        .def_readonly("firmware_sn",          &discovery::GripperEndpoints::firmware_sn)
        .def("__repr__", [](const discovery::GripperEndpoints& e) {
            return std::string("GripperEndpoints(side=") +
                   discovery::to_string(e.side) +
                   ", role=" + discovery::to_string(e.role) +
                   ", mcu=" + e.mcu_device +
                   " ch343_sn=" + e.mcu_serial +
                   " fw_sn=" + e.firmware_sn + ")";
        });
    m.def("scan_grippers",  &discovery::scan_all);
    m.def("find_one",       &discovery::find_one);
    m.def("find_left",      &discovery::find_left);
    m.def("find_right",     &discovery::find_right);
    m.def("find_leader",    &discovery::find_leader);
    m.def("find_follower",  &discovery::find_follower);

    // ---- LeaderGripper --------------------------------------------------
    py::class_<LeaderGripper>(m, "LeaderGripper")
        .def(py::init([](const std::string& mcu, const std::string& wrist,
                         uint32_t baud, unsigned ack_ms, unsigned retries,
                         bool open_cameras, bool normalize_position,
                         float encoder_max_rad,
                         bool undistort_wrist, float fisheye_balance,
                         ColorMode wrist_color_mode) {
                LeaderGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                cfg.normalize_position   = normalize_position;
                cfg.encoder_max_rad      = encoder_max_rad;
                cfg.undistort_wrist      = undistort_wrist;
                cfg.fisheye_balance      = fisheye_balance;
                cfg.wrist_color_mode     = wrist_color_mode;
                py::gil_scoped_release gil;
                return std::make_unique<LeaderGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 200u,
             py::arg("max_retries")         = 1u,
             py::arg("open_cameras")        = false,
             // Normalized encoder position: fills EncoderSample.position with
             // the opening in [0,1] (0=closed, 1=open) on read_once() and on
             // every streamed sample. position_rad keeps reporting radians.
             // Raises at construction if the firmware has no encoder-max
             // calibration and encoder_max_rad isn't supplied.
             py::arg("normalize_position")  = false,
             py::arg("encoder_max_rad")     = 0.0f,
             // Wrist fisheye undistortion (needs open_cameras=True and a V2.0+
             // firmware that holds a calibration). Degrades to raw frames with
             // a warning when it does not; raises if the camera is not at the
             // calibrated 640x480.
             py::arg("undistort_wrist")     = false,
             py::arg("fisheye_balance")     = 0.0f,
             // Channel order of the wrist frames. RGB by default, unlike the
             // bare Camera which keeps OpenCV's BGR: this stream feeds vision
             // pipelines, which all want RGB. Pass ColorMode.BGR for code that
             // hands frames straight to cv2.imshow/imwrite.
             py::arg("wrist_color_mode")    = ColorMode::Rgb)
        .def_static("open", [](bool normalize_position, float encoder_max_rad) {
            py::gil_scoped_release gil;
            if (!normalize_position && encoder_max_rad <= 0.0f) {
                return LeaderGripper::open();  // returns unique_ptr<LeaderGripper>
            }
            auto eps = discovery::find_one();
            LeaderGripper::Config cfg{};
            cfg.mcu_device         = eps.mcu_device;
            cfg.normalize_position = normalize_position;
            cfg.encoder_max_rad    = encoder_max_rad;
            return std::make_unique<LeaderGripper>(cfg);
        }, py::arg("normalize_position") = false, py::arg("encoder_max_rad") = 0.0f)
        .def("start_streaming", [](LeaderGripper& self, unsigned imu_hz, unsigned enc_hz) {
            py::gil_scoped_release gil;
            self.start_streaming(imu_hz, enc_hz);
        }, py::arg("imu_hz") = 100u, py::arg("encoder_hz") = 100u,
           "Start the MCU sensor stream.\n\nA rate of 0 turns that source OFF (its source_mask bit is cleared). Passing 0 used to stream the source at the firmware's 100 Hz default instead; raises IoError(EINVAL) if every rate is 0.\n\nThe firmware divides a 1 kHz tick by an integer, so only divisors of 1000 arrive at the requested rate -- 300 Hz becomes 333 Hz, 150 Hz becomes 167 Hz, and anything above 1000 Hz collapses to 100 Hz. Motor status is additionally capped at 100 Hz. None of this is NACKed by the firmware, so the SDK logs a warning when it applies.")
        .def("stop_streaming", [](LeaderGripper& self) {
            py::gil_scoped_release gil;
            self.stop_streaming();
        })
        .def_property_readonly("imu",           [](LeaderGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("encoder",       [](LeaderGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal)
        .def_property_readonly("wrist_camera",  [](LeaderGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal)
        .def_property_readonly("key",           [](LeaderGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("led",           [](LeaderGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("sensor_errors", [](LeaderGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("diagnostics", [](LeaderGripper& g) -> Diagnostics&    { return g.diagnostics(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("firmware_version",
            [](const LeaderGripper& g) { return g.firmware_version(); },
            "Firmware version read at open(), or None if the MCU did not answer.")
        .def_property_readonly("calibration",   [](LeaderGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal)
        .def_property_readonly("ota",           [](LeaderGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("transport",     [](LeaderGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal)
        .def_property_readonly("is_streaming",  &LeaderGripper::is_streaming)

        // ---- Normalized position (0 = closed, 1 = open) ------------------
        // Available regardless of the normalize_position flag — that flag only
        // controls whether EncoderSample.position gets filled in. All of these
        // raise ProtocolError when the gripper has no encoder-max calibration.
        .def("position", [](LeaderGripper& g, unsigned timeout_ms) {
                py::gil_scoped_release gil;
                return g.position(std::chrono::milliseconds(timeout_ms));
            }, py::arg("timeout_ms") = 100u,
            "Read the encoder and return the opening normalized to [0,1].")
        .def("pos_to_rad", [](LeaderGripper& g, float p) {
                py::gil_scoped_release gil;
                return g.pos_to_rad(p);
            }, py::arg("position"))
        .def("rad_to_pos", [](LeaderGripper& g, float r) {
                py::gil_scoped_release gil;
                return g.rad_to_pos(r);
            }, py::arg("raw_rad"))
        .def_property_readonly("position_map", [](LeaderGripper& g) {
                py::gil_scoped_release gil;
                return g.position_map();
            }, "Cached raw-rad <-> [0,1] converter (loads on first access).")
        .def("reload_position_map", [](LeaderGripper& g) {
                py::gil_scoped_release gil;
                g.reload_position_map();
            },
            "Re-read the encoder-max calibration and rebuild the converter. "
            "Call after write_encoder_max_rad() or a re-zero.")

        .def("__enter__", [](LeaderGripper& g) -> LeaderGripper& { return g; })
        .def("__exit__",  [](LeaderGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // Also tear the link down, so callbacks and the reader thread are
            // gone by the end of the `with` block rather than lingering until
            // interpreter shutdown. Leaving them alive is what let a late DATA
            // frame call into a finalized interpreter.
            g.transport().stop();
        });

    // ---- FollowerGripper ------------------------------------------------
    // ---- FirmwareVersion -------------------------------------------------
    py::class_<protocol::FirmwareVersion>(m, "FirmwareVersion")
        .def_readonly("major", &protocol::FirmwareVersion::major)
        .def_readonly("minor", &protocol::FirmwareVersion::minor)
        .def_readonly("patch", &protocol::FirmwareVersion::patch)
        .def_readonly("build", &protocol::FirmwareVersion::build)
        .def_property_readonly("tuple",
            [](const protocol::FirmwareVersion& v) {
                return py::make_tuple(v.major, v.minor, v.patch);
            },
            "(major, minor, patch) — comparable, so a feature gate reads as "
            "`gripper.firmware_version.tuple >= (2, 0, 0)`.")
        .def("__repr__", [](const protocol::FirmwareVersion& v) {
            return "FirmwareVersion(" + std::to_string(v.major) + "." +
                   std::to_string(v.minor) + "." + std::to_string(v.patch) +
                   "." + std::to_string(v.build) + ")";
        });

    py::class_<FollowerGripper>(m, "FollowerGripper")
        .def(py::init([](const std::string& mcu, const std::string& wrist,
                         uint32_t baud, unsigned ack_ms, unsigned retries,
                         bool open_cameras,
                         bool undistort_wrist, float fisheye_balance,
                         ColorMode wrist_color_mode,
                         bool allow_outdated_firmware) {
                FollowerGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                cfg.undistort_wrist      = undistort_wrist;
                cfg.fisheye_balance      = fisheye_balance;
                cfg.wrist_color_mode     = wrist_color_mode;
                cfg.allow_outdated_firmware = allow_outdated_firmware;
                py::gil_scoped_release gil;
                return std::make_unique<FollowerGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 1000u,
             py::arg("max_retries")         = 2u,
             py::arg("open_cameras")        = false,
             // See LeaderGripper above — same semantics and same failure policy.
             py::arg("undistort_wrist")     = false,
             py::arg("fisheye_balance")     = 0.0f,
             // Channel order of the wrist frames. RGB by default, unlike the
             // bare Camera which keeps OpenCV's BGR: this stream feeds vision
             // pipelines, which all want RGB. Pass ColorMode.BGR for code that
             // hands frames straight to cv2.imshow/imwrite.
             py::arg("wrist_color_mode")    = ColorMode::Rgb,
             // 固件版本门:低于 **1.2.5** 直接拒绝打开。两条理由叠加 —— 1.1.6
             // 之前 MIT 路径上没有任何堵转保护(安全底线);1.2.5 之前标定写入的
             // 闭合零位带内缩量,而本 SDK 的闭合端预压按「归一化 0.0 就是止点」
             // 设计,在更旧固件上会压过位置映射所说的完全闭合点。
             // 置 true 只为了在升级前读一台旧设备的配置;OTA 本身走
             // LeaderGripper(不设门),升级通道始终可用。
             // 门槛的唯一真值源是 FollowerGripper::kMinFirmware*,别在这里写死。
             py::arg("allow_outdated_firmware") = false)
        .def_property_readonly("firmware_version",
            [](const FollowerGripper& g) -> py::object {
                auto v = g.firmware_version();
                if (!v) return py::none();
                return py::cast(*v);
            },
            "The firmware version reported at open(), or None when the MCU did "
            "not answer. Gate features on it: the wrist fisheye intrinsics need "
            "command set V2.0+, and skipping the check turns 'firmware too old' "
            "into an opaque protocol error.")
        .def_static("open", []() {
            py::gil_scoped_release gil;
            return FollowerGripper::open();
        })
        .def("start_streaming", [](FollowerGripper& self, unsigned motor_hz) {
            py::gil_scoped_release gil;
            self.start_streaming(motor_hz);
        }, py::arg("motor_hz") = 100u,
           "Start the motor-status stream.\n\n"
           "Motor status is the ONLY source a follower streams -- the firmware\n"
           "compiles IMU, encoder and eskin streaming out on this role. This used\n"
           "to take imu_hz and encoder_hz for parity with the leader; they set\n"
           "their mask bits and produced nothing, and the old defaults\n"
           "(imu=100, encoder=100, motor=0) meant a bare start_streaming() call\n"
           "started a stream carrying NOTHING and reported success.\n\n"
           "motor_hz=0 raises IoError(EINVAL) rather than starting an empty\n"
           "stream. The firmware caps motor status at 100 Hz and divides a 1 kHz\n"
           "tick by an integer, so only divisors of 1000 arrive at the requested\n"
           "rate. It never NACKs a rate it had to adjust, so the SDK logs a\n"
           "warning when that happens.")
        .def("stop_streaming", [](FollowerGripper& self) {
            py::gil_scoped_release gil;
            self.stop_streaming();
        })
        .def_property_readonly("imu",           [](FollowerGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("encoder",       [](FollowerGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal)
        .def_property_readonly("motor",         [](FollowerGripper& g) -> Motor&          { return g.motor(); },         py::return_value_policy::reference_internal)
        .def_property_readonly("wrist_camera",  [](FollowerGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal)
        .def_property_readonly("key",           [](FollowerGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("led",           [](FollowerGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("sensor_errors", [](FollowerGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("diagnostics", [](FollowerGripper& g) -> Diagnostics&    { return g.diagnostics(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("calibration",   [](FollowerGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal)
        .def_property_readonly("ota",           [](FollowerGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("transport",     [](FollowerGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal)
        .def_property_readonly("is_streaming",  &FollowerGripper::is_streaming)
        // Follower gripper open/close limit config (Cmd 0x66/0x67).
        .def("get_gripper_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_gripper_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        .def("set_gripper_config", [](FollowerGripper& g,
                                      const protocol::GripperConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_gripper_config(cfg);
        }, py::arg("config"))
        // Power-on auto-calibration config (Cmd 0x68/0x69).
        .def("get_envelope", [](FollowerGripper& g) {
            py::gil_scoped_release r; return g.get_envelope();
        })
        .def("set_envelope", [](FollowerGripper& g, const protocol::GripperEnvelope& e) {
            py::gil_scoped_release r; g.set_envelope(e);
        }, py::arg("envelope"))
        .def("get_auto_cal_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_auto_cal_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        .def("set_auto_cal_config", [](FollowerGripper& g,
                                       const protocol::GripperAutoCalConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_auto_cal_config(cfg);
        }, py::arg("config"))
        // V2.2 — patch only the stall-detection fields. Overloaded on the
        // param type; follower firmware older than 1.1.2 NACKs LengthMismatch.
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParam& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"))
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParamEx& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"))
        // ---- Normalized position (0 = closed, 1 = open) -------------------
        // NOTE: normalized [0,1] — distinct from motor.set_position() (raw rad).
        // set_position() is fire-and-forget (no ACK); poll motor.control_stats()
        // for health. Throws if the gripper isn't calibrated.
        .def("position", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.position(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        // set_position() 同样不暴露:follower_gripper.cpp 里它就是
        // motor_.submit_impedance() 外面包了一层归一化,绕开控制器的程度和裸
        // 原语完全一样,只是名字看起来更像正经 API。用 ControlLoop 或
        // ForcePositionController 的 set_target()。
        .def("pos_to_rad", [](FollowerGripper& g, float position) {
            py::gil_scoped_release gil;
            return g.pos_to_rad(position);
        }, py::arg("position"))
        .def("rad_to_pos", [](FollowerGripper& g, float raw_rad) {
            py::gil_scoped_release gil;
            return g.rad_to_pos(raw_rad);
        }, py::arg("raw_rad"))
        .def("position_map", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            return g.position_map();          // copy of the cached converter
        })
        .def("reload_config", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            g.reload_config();
        })
        .def("__enter__", [](FollowerGripper& g) -> FollowerGripper& { return g; })
        .def("__exit__",  [](FollowerGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // See LeaderGripper::__exit__ — drop the reader thread and its
            // callbacks here rather than at interpreter shutdown.
            g.transport().stop();
        });
}

}  // namespace xense::taccap::python

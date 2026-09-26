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
    py::enum_<discovery::Side>(m, "Side",
        "Which end of the robot a gripper sits on.\n\n"
        "Read from the firmware-burned SN via Cmd::GetSn -- the sequence's last\n"
        "digit, odd = Left, even = Right -- with the flash-burned DEV_TYPE\n"
        "(Cmd::GetDevType) as a secondary firmware source. The CH343 USB-chip SN\n"
        "is never consulted: it is burned independently of the board's left/right\n"
        "identity. Unknown when neither firmware source answered; discovery does\n"
        "not guess.")
        .value("Left",    discovery::Side::Left)
        .value("Right",   discovery::Side::Right)
        .value("Unknown", discovery::Side::Unknown);

    py::enum_<discovery::Role>(m, "Role",
        "Leader (teleop master) or Follower (robot-side executor).\n\n"
        "Taken from the SN patch suffix alone: 'm' = Leader, 's' = Follower.\n"
        "Unknown for sensor SNs, unburned SNs and anything that does not parse --\n"
        "USB enumeration cannot tell the two board builds apart.")
        .value("Leader",   discovery::Role::Leader)
        .value("Follower", discovery::Role::Follower)
        .value("Unknown",  discovery::Role::Unknown);

    py::class_<discovery::ParsedSerial>(m, "ParsedSerial",
        "Structured view of a TacCap SN, e.g. 'TCGU01A24Z0001m'.\n\n"
        "Fields are filled best-effort even when `valid` is False, so a legacy or\n"
        "partially-conforming SN still yields a side and a role where it can.")
        .def_readonly("raw",      &discovery::ParsedSerial::raw,
                      "The input string, verbatim.")
        .def_readonly("product",  &discovery::ParsedSerial::product,
                      "'TCGU01' for a gripper, 'GSPS01' for a sensor.")
        .def_readonly("batch",    &discovery::ParsedSerial::batch,
                      "Batch code: one letter plus two digits, e.g. 'A24'.")
        .def_property_readonly("line", [](const discovery::ParsedSerial& p) {
            return std::string(1, p.line);
        }, "Product line: 'Z' = R&D/test, 'A' = production. '?' when unparsed.")
        .def_readonly("sequence", &discovery::ParsedSerial::sequence,
                      "Four-digit unit number. Its last digit decides the side.")
        .def_readonly("side",     &discovery::ParsedSerial::side,
                      "None when the string carries no digit at all.")
        .def_readonly("role",     &discovery::ParsedSerial::role)
        .def_readonly("valid",    &discovery::ParsedSerial::valid,
                      "True only when the full SN grammar matched. side and role are\n"
                      "still filled best-effort when it is False.")
        .def("__repr__", [](const discovery::ParsedSerial& p) {
            return std::string("ParsedSerial(raw=") + p.raw +
                   ", product=" + p.product +
                   ", line=" + std::string(1, p.line) +
                   ", seq=" + p.sequence +
                   ", side=" + (p.side ? discovery::to_string(*p.side) : "None") +
                   ", role=" + discovery::to_string(p.role) +
                   ", valid=" + (p.valid ? "True" : "False") + ")";
        });
    m.def("parse_serial", &discovery::parse_serial, py::arg("serial"),
          "Parse a TacCap SN into its fields.\n\n"
          "Never raises: a string that does not match the grammar comes back with\n"
          "valid=False and whatever side/role could be recovered from it.");

    py::class_<discovery::GripperEndpoints>(m, "GripperEndpoints",
        "One discovered gripper unit, identified by its MCU board.\n\n"
        "One MCU board = one gripper. The wrist camera and the visuotactile\n"
        "sensors are owned by an external camera service and are not enumerated.")
        .def_readonly("side",                 &discovery::GripperEndpoints::side,
                      "From the firmware SN, or DEV_TYPE when the SN did not answer;\n"
                      "Unknown when neither did.")
        .def_readonly("role",                 &discovery::GripperEndpoints::role,
                      "From the SN patch suffix; Unknown for a legacy, unburned or\n"
                      "unparsable SN.")
        .def_readonly("mcu_device",           &discovery::GripperEndpoints::mcu_device,
                      "Control-link device path, always the CH343's if02 interface under\n"
                      "/dev/serial/by-id/. Pass this to a gripper constructor rather than\n"
                      "a /dev/ttyACM* path, whose minor number changes on re-enumeration.")
        .def_readonly("mcu_serial",           &discovery::GripperEndpoints::mcu_serial,
                      "CH343 USB-chip SN. Identification only -- it never decides `side`.")
        .def_readonly("firmware_sn",          &discovery::GripperEndpoints::firmware_sn,
                      "SN burned into STM32 flash, read via Cmd::GetSn, e.g.\n"
                      "'TCGU01A24Z0002m'. Empty when the firmware did not answer -- too\n"
                      "old for GetSn, or the SN was never burned. `side` and `role` come\n"
                      "from this string.")
        .def("__repr__", [](const discovery::GripperEndpoints& e) {
            return std::string("GripperEndpoints(side=") +
                   discovery::to_string(e.side) +
                   ", role=" + discovery::to_string(e.role) +
                   ", mcu=" + e.mcu_device +
                   " ch343_sn=" + e.mcu_serial +
                   " fw_sn=" + e.firmware_sn + ")";
        });
    m.def("scan_grippers",  &discovery::scan_all,
          "Enumerate every gripper currently plugged in.\n\n"
          "Opens a transient link to each MCU and probes it for DEV_TYPE and the\n"
          "SN before closing it again, so a scan is not free and cannot run while\n"
          "another process holds the port. Returns an empty list rather than\n"
          "raising when nothing is attached.");
    m.def("find_one",       &discovery::find_one,
          "The single attached gripper, whatever its side or role.\n\n"
          "Raises IoError when none is attached, and again when more than one is --\n"
          "pick with find_left()/find_right() or find_leader()/find_follower().");
    m.def("find_left",      &discovery::find_left,
          "The attached gripper whose firmware reports side Left.\n\n"
          "Raises IoError when none matches; a unit whose firmware answered neither\n"
          "GetSn nor GetDevType has side Unknown and never matches.");
    m.def("find_right",     &discovery::find_right,
          "The attached gripper whose firmware reports side Right.\n\n"
          "Raises IoError when none matches; a unit whose firmware answered neither\n"
          "GetSn nor GetDevType has side Unknown and never matches.");
    m.def("find_leader",    &discovery::find_leader,
          "The attached gripper whose SN patch suffix is 'm' (Master).\n\n"
          "Raises IoError when none matches; a legacy or unburned SN reports role\n"
          "Unknown and never matches.");
    m.def("find_follower",  &discovery::find_follower,
          "The attached gripper whose SN patch suffix is 's' (Slave).\n\n"
          "Raises IoError when none matches; a legacy or unburned SN reports role\n"
          "Unknown and never matches.");

    // ---- LeaderGripper --------------------------------------------------
    py::class_<LeaderGripper>(m, "LeaderGripper",
        "One TacCap-Gripper leader (teleop master end).\n\n"
        "Owns the MCU control link plus IMU, encoder, button, LED and -- only when\n"
        "asked for -- the wrist camera. There is no motor on leader hardware; that\n"
        "is FollowerGripper.\n\n"
        "The constructor opens the MCU serial device and holds it for the object's\n"
        "lifetime. Use it as a context manager: __exit__ drops the reader thread\n"
        "and its callbacks, which otherwise live until interpreter shutdown.\n\n"
        "    with LeaderGripper.open() as g:\n"
        "        g.start_streaming(imu_hz=100, encoder_hz=100)\n"
        "        print(g.encoder.read_once())")
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
             py::arg("wrist_color_mode")    = ColorMode::Rgb,
             "Open a leader by explicit device path -- mcu_device is the\n"
             "/dev/serial/by-id/ path from discovery.\n\n"
             "Blocks: the firmware version and SN are read once here, and a device\n"
             "that answers neither is logged, not raised on. normalize_position=True\n"
             "additionally resolves the travel span now, raising ProtocolError when\n"
             "the firmware holds no encoder-max calibration and encoder_max_rad was\n"
             "not supplied -- better here than as silently-absent .position fields\n"
             "once a 100 Hz stream is already running.")
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
        }, py::arg("normalize_position") = false, py::arg("encoder_max_rad") = 0.0f,
           "Auto-discover the one attached gripper and open it as a leader.\n\n"
           "Raises IoError when none is attached or when more than one is; pass an\n"
           "explicit mcu_device from find_leader() in that case. Cameras stay\n"
           "closed -- an external service owns the wrist V4L2 device.")
        .def("start_streaming", [](LeaderGripper& self, unsigned imu_hz, unsigned enc_hz) {
            py::gil_scoped_release gil;
            self.start_streaming(imu_hz, enc_hz);
        }, py::arg("imu_hz") = 100u, py::arg("encoder_hz") = 100u,
           "Start the MCU sensor stream.\n\nA rate of 0 turns that source OFF (its source_mask bit is cleared). Passing 0 used to stream the source at the firmware's 100 Hz default instead; raises IoError(EINVAL) if every rate is 0.\n\nThe firmware divides a 1 kHz tick by an integer, so only divisors of 1000 arrive at the requested rate -- 300 Hz becomes 333 Hz, 150 Hz becomes 167 Hz, and anything above 1000 Hz collapses to 100 Hz. Motor status is additionally capped at 100 Hz. None of this is NACKed by the firmware, so the SDK logs a warning when it applies.")
        .def("stop_streaming", [](LeaderGripper& self) {
            py::gil_scoped_release gil;
            self.stop_streaming();
        }, "Stop the MCU sensor stream.\n\n"
           "Best-effort: a no-op when no stream was started, and it returns even\n"
           "when the MCU never ACKs the StopStream.")
        .def_property_readonly("imu",           [](LeaderGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal,
                               "IMU component: one-shot reads and the streamed DATA frames.")
        .def_property_readonly("encoder",       [](LeaderGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal,
                               "Encoder component. Its samples carry .position in [0,1] only when the\n"
                               "gripper was built with normalize_position=True; .position_rad is always\n"
                               "in radians.")
        .def_property_readonly("wrist_camera",  [](LeaderGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal,
                               "Wrist UVC camera. Raises IoError(ENODEV) unless the gripper was built\n"
                               "with open_cameras=True and a wrist_video path.")
        .def_property_readonly("key",           [](LeaderGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal,
                               "Physical button K1 (command set V1.4+). Event-driven: the firmware\n"
                               "pushes one frame per state change whether or not a stream is running.")
        .def_property_readonly("led",           [](LeaderGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal,
                               "WS2812 LED: base color plus compositing effects (command set V1.9+).\n"
                               "Both commands are ACKed.")
        .def_property_readonly("sensor_errors", [](LeaderGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal,
                               "Per-sensor error reports (command set V1.6+). Event-driven like key:\n"
                               "pushed when a sensor's error state changes, independent of streaming.")
        .def_property_readonly("diagnostics", [](LeaderGripper& g) -> Diagnostics&    { return g.diagnostics(); }, py::return_value_policy::reference_internal,
                               "Firmware UART counters and log control. Counters need firmware 1.1.3,\n"
                               "log control 1.1.4; older firmware NACKs InvalidCmd as a ProtocolError.")
        .def_property_readonly("firmware_version",
            [](const LeaderGripper& g) { return g.firmware_version(); },
            "Firmware version read at open(), or None if the MCU did not answer.")
        .def_property_readonly("calibration",   [](LeaderGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal,
                               "Calibration records the firmware persists in flash: fisheye intrinsics\n"
                               "(command set V2.0+) and the encoder max travel angle (V2.1+, i.e.\n"
                               "leader firmware 1.2.0+). Reading one never written NACKs CalNotSet,\n"
                               "surfaced as None rather than an exception.")
        .def_property_readonly("ota",           [](LeaderGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal,
                               "Firmware update session (command set V1.3+). Role-agnostic, so a\n"
                               "follower is also flashed through a LeaderGripper. After flashing,\n"
                               "unplug USB and power together, then reconnect both.")
        .def_property_readonly("transport",     [](LeaderGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal,
                               "The raw MCU link underneath, for commands this SDK does not wrap.")
        .def_property_readonly("is_streaming",  &LeaderGripper::is_streaming,
                               "True between start_streaming() and stop_streaming().")

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
            }, py::arg("position"),
            "Convert a normalized position to the encoder angle in radians.\n\n"
            "The input is clamped to [0,1] first. Loads the position map on first\n"
            "use, which is one request/ACK round trip; a local computation after\n"
            "that. Raises ProtocolError when there is no encoder-max calibration.")
        .def("rad_to_pos", [](LeaderGripper& g, float r) {
                py::gil_scoped_release gil;
                return g.rad_to_pos(r);
            }, py::arg("raw_rad"),
            "Convert an encoder angle in radians to a normalized position, clamped\n"
            "to [0,1]. Loads the position map on first use; local after that.\n"
            "Raises ProtocolError when there is no encoder-max calibration.")
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

        .def("__enter__", [](LeaderGripper& g) -> LeaderGripper& { return g; },
             "Returns the gripper unchanged -- the link is already open by the time\n"
             "you hold the object.")
        .def("__exit__",  [](LeaderGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // Also tear the link down, so callbacks and the reader thread are
            // gone by the end of the `with` block rather than lingering until
            // interpreter shutdown. Leaving them alive is what let a late DATA
            // frame call into a finalized interpreter.
            g.transport().stop();
        }, "Stop the stream and tear the link down, including on an exception.");

    // ---- FollowerGripper ------------------------------------------------
    // ---- FirmwareVersion -------------------------------------------------
    py::class_<protocol::FirmwareVersion>(m, "FirmwareVersion",
        "A firmware version as major.minor.patch.build, as reported at open().")
        .def_readonly("major", &protocol::FirmwareVersion::major)
        .def_readonly("minor", &protocol::FirmwareVersion::minor)
        .def_readonly("patch", &protocol::FirmwareVersion::patch)
        .def_readonly("build", &protocol::FirmwareVersion::build,
                      "Build number. Not part of `tuple`, so it never takes part in a\n"
                      "version comparison.")
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

    py::class_<FollowerGripper>(m, "FollowerGripper",
        "One TacCap-Gripper follower (robot-side executor).\n\n"
        "The leader's sensor surface plus a Motor driving the FDCAN-attached\n"
        "actuator. The constructor holds the MCU serial device for the object's\n"
        "lifetime; use it as a context manager so the reader thread goes away at\n"
        "the end of the block rather than at interpreter shutdown.\n\n"
        "Follower firmware older than 1.2.5 is REFUSED with ProtocolError. That\n"
        "is a follower-line version: the two roles number independently, so a\n"
        "leader's version is not comparable with it. Below 1.1.6 the firmware\n"
        "has no stall protection on the MIT command path at all; below\n"
        "1.2.5 the calibrated closed zero sits short of the mechanical stop, so\n"
        "normalized 0.0 does not mean what this SDK's closed-endpoint preload\n"
        "assumes. A version that could not be read is a warning, not a refusal.\n"
        "allow_outdated_firmware=True is the escape hatch, meant for reading an\n"
        "old device's config before upgrading it; OTA itself goes through\n"
        "LeaderGripper and is never gated.\n\n"
        "Enumeration cannot tell leader hardware from follower hardware, so opening\n"
        "this class on a leader succeeds and the motor commands fail later with\n"
        "ProtocolError(SensorOffline) -- which is the right moment to notice.\n\n"
        "    with FollowerGripper.open() as g:\n"
        "        g.start_streaming(motor_hz=100)\n"
        "        print(g.position())")
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
             py::arg("allow_outdated_firmware") = false,
             "Open a follower by explicit device path -- mcu_device is the\n"
             "/dev/serial/by-id/ path from discovery.\n\n"
             "Blocks: the firmware version and SN are read once here, then the\n"
             ">= 1.2.5 gate is applied. A version below the floor raises\n"
             "ProtocolError with the upgrade instructions; a version that could not\n"
             "be read only logs a warning. allow_outdated_firmware=True skips the\n"
             "check entirely.")
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
        }, "Auto-discover the one attached gripper and open it as a follower.\n\n"
           "Raises IoError when none is attached or when more than one is; pass an\n"
           "explicit mcu_device from find_follower() in that case. It does not\n"
           "check that the hardware really is a follower -- see the class docstring.\n"
           "Cameras stay closed.")
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
        }, "Stop the motor-status stream.\n\n"
           "Best-effort: a no-op when no stream was started, and it returns even\n"
           "when the MCU never ACKs the StopStream.")
        .def_property_readonly("imu",           [](FollowerGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal,
                               "IMU component. Present on both roles, but a follower does not stream\n"
                               "it -- see start_streaming().")
        .def_property_readonly("encoder",       [](FollowerGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal,
                               "Encoder component. Present on both roles, but a follower does not\n"
                               "stream it -- see start_streaming().")
        .def_property_readonly("motor",         [](FollowerGripper& g) -> Motor&          { return g.motor(); },         py::return_value_policy::reference_internal,
                               "Motor: enable/disable, the control primitives and the status reads.\n"
                               "Its set_* / submit_* primitives take RAW shaft radians, not the\n"
                               "normalized [0,1] the gripper-level calls use. Do not drive it while a\n"
                               "controller is running -- both write the same bus and the last frame wins.")
        .def_property_readonly("wrist_camera",  [](FollowerGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal,
                               "Wrist UVC camera. Raises IoError(ENODEV) unless the gripper was built\n"
                               "with open_cameras=True and a wrist_video path.")
        .def_property_readonly("key",           [](FollowerGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal,
                               "Physical button K1 (command set V1.4+). Event-driven: the firmware\n"
                               "pushes one frame per state change whether or not a stream is running.")
        .def_property_readonly("led",           [](FollowerGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal,
                               "WS2812 LED: base color plus compositing effects (command set V1.9+).\n"
                               "Both commands are ACKed.")
        .def_property_readonly("sensor_errors", [](FollowerGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal,
                               "Per-sensor error reports (command set V1.6+). Event-driven: pushed\n"
                               "when a sensor's error state changes, independent of streaming.")
        .def_property_readonly("diagnostics", [](FollowerGripper& g) -> Diagnostics&    { return g.diagnostics(); }, py::return_value_policy::reference_internal,
                               "Firmware UART counters and log control. Counters need firmware 1.1.3,\n"
                               "log control 1.1.4; older firmware NACKs InvalidCmd as a ProtocolError.")
        .def_property_readonly("calibration",   [](FollowerGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal,
                               "Calibration records the firmware persists in flash. The fisheye\n"
                               "intrinsics (command set V2.0+) work here; the encoder-max methods are\n"
                               "leader-only and NACK InvalidCmd on a follower.")
        .def_property_readonly("ota",           [](FollowerGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal,
                               "Firmware update session (command set V1.3+). ota_update.py flashes a\n"
                               "follower through a LeaderGripper instead, so the upgrade path is never\n"
                               "blocked by this class's firmware gate. After flashing, unplug USB\n"
                               "and power together, then reconnect both.")
        .def_property_readonly("transport",     [](FollowerGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal,
                               "The raw MCU link underneath, for commands this SDK does not wrap.")
        .def_property_readonly("is_streaming",  &FollowerGripper::is_streaming,
                               "True between start_streaming() and stop_streaming().")
        // Follower gripper open/close limit config (Cmd 0x66/0x67).
        .def("get_gripper_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_gripper_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u,
           "Read the travel calibration and open/close limits (Cmd 0x67).\n\n"
           "Blocks for the ACK; timeout_ms bounds that wait. Raises ProtocolError on\n"
           "a NACK -- which is what a leader answers, having no follower config.")
        .def("set_gripper_config", [](FollowerGripper& g,
                                      const protocol::GripperConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_gripper_config(cfg);
        }, py::arg("config"),
           "Write the travel calibration and open/close limits (Cmd 0x66).\n\n"
           "Blocks for the ACK; raises ProtocolError on a NACK. Drops the cached\n"
           "position map, so the next normalized call rebuilds it from the new\n"
           "limits.\n\n"
           "The same record carries the motion safety envelope in its reserved\n"
           "bytes, so a config assembled from scratch here wipes the envelope --\n"
           "modify one returned by get_gripper_config(), or use set_envelope().")
        // Power-on auto-calibration config (Cmd 0x68/0x69).
        .def("get_envelope", [](FollowerGripper& g) {
            py::gil_scoped_release r; return g.get_envelope();
        }, "Read the motion safety envelope.\n\n"
           "It rides inside the gripper config record, so this is the same blocking\n"
           "0x67 transaction as get_gripper_config(). Returns flags == 0 on a device\n"
           "that never had an envelope written.")
        .def("set_envelope", [](FollowerGripper& g, const protocol::GripperEnvelope& e) {
            py::gil_scoped_release r; g.set_envelope(e);
        }, py::arg("envelope"),
           "Write the motion safety envelope.\n\n"
           "Read-modify-write over the gripper config record -- two blocking\n"
           "transactions -- because the envelope shares that record with the travel\n"
           "calibration and neither may clobber the other. The layout version is\n"
           "stamped here, so firmware built against a different field order refuses\n"
           "the record instead of misreading it.\n\n"
           "Setting the Enforce flag changes real motion behaviour: the firmware\n"
           "then clamps every MIT frame against this envelope. Write it deliberately.")
        .def("home_diag", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release r;
            return g.home_diag(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 200u,
           "上电自动标定到底做了什么(0x57)。\n\n"
           "它存在的理由:标定的五种失败原因此前只走固件的 UART7,而 UART7 没接到\n"
           "USB —— 从主机看,一次提前结束的标定和一次正常的标定长得一模一样,两者\n"
           "都会留下一个看起来合理的 max_open_rad。\n\n"
           "判断一次堵转判定实不实:拿 last_abs_torque 和 stall_threshold_nm 比。\n"
           "阈值 = 命令限流 × 型号的反馈饱和比 × 余量;实测只比阈值高一点点,说明\n"
           "那次判定很勉强。型号表里饱和比还是 0(未测)时,阈值会落到一个保守的\n"
           "回退值上,而保守意味着**偏向提前判出堵转**。\n\n"
           "注意标定最后一步是从开限位**故意后退 0.12 rad**,所以爪子停在比\n"
           "max_open_rad 小的位置是设计如此,不是故障。"
        )
        .def("audit_envelope", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release r;
            return g.audit_envelope(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 1000u,
           "Read the envelope and judge it. Never writes.\n\n"
           "Returns an EnvelopeAudit carrying `stored` (what is in flash),\n"
           "`effective` (what the firmware applies, or None when it applies\n"
           "nothing) and `recommended` (derived from this device's motor spec).\n"
           "Those are three different things -- see EnvelopeAudit.\n\n"
           "Two blocking transactions, 0x67 then 0x56. Do NOT call it while a\n"
           "controller is running: the ACK collides with the phase-locked 100 Hz\n"
           "control frames. Before start(), or after stop().")
        .def("ensure_envelope", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release r;
            return g.ensure_envelope(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 1000u,
           "Make sure this gripper has a usable motion safety envelope.\n\n"
           "**Writes MCU flash** -- persistent, survives power loss -- but only when\n"
           "the stored record is ineffective, misreports what is enforced, or does\n"
           "not match the device's spec. Calling it twice writes once; `wrote` says\n"
           "which happened. Not for a loop, a constructor, or a controller start().\n\n"
           "When the device reports both ratings, cont/peak are written to exactly\n"
           "the stall / rotating rating -- a lower stored value is raised too\n"
           "(GRIPPER_ENVELOPE_ISSUE_NOT_AT_SPEC). Only when the spec is unknown is\n"
           "a stricter stored record (say cont=0.6) kept and just its flags\n"
           "repaired.\n\n"
           "Which numbers belong here is not the caller's question. The device\n"
           "knows its motor's ratings and the firmware clamps against them whatever\n"
           "was written, so the values come from the device (0x56) rather than from\n"
           "a flag someone has to pick.\n\n"
           "Same threading rule as audit_envelope(): not while a controller runs.")
        .def("get_auto_cal_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_auto_cal_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u,
           "Read the power-on auto-calibration config (Cmd 0x69, command set V1.9+).\n\n"
           "Blocks for the ACK; raises ProtocolError on a NACK.")
        .def("set_auto_cal_config", [](FollowerGripper& g,
                                       const protocol::GripperAutoCalConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_auto_cal_config(cfg);
        }, py::arg("config"),
           "Write the power-on auto-calibration config (Cmd 0x68).\n\n"
           "When enabled, the firmware calibrates on power-up: close to stall for\n"
           "zero, open to stall for max travel. Blocks for the ACK; raises\n"
           "ProtocolError on a NACK.")
        // V2.2 — patch only the stall-detection fields. Overloaded on the
        // param type; follower firmware older than 1.1.2 NACKs LengthMismatch.
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParam& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"),
           "Patch only the stall torques and hold time, leaving the speeds and the\n"
           "Enable flag as stored (command set V2.2).\n\n"
           "Saves a read-modify-write when tuning stall torque and cannot clobber\n"
           "the rest of the config from a stale read. Blocks for the ACK. Follower\n"
           "firmware older than 1.1.2 NACKs LengthMismatch -- fall back to\n"
           "set_auto_cal_config() there.")
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParamEx& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"),
           "Same partial write, additionally patching the two delays and the\n"
           "compat-only confirm counts. Same 1.1.2 firmware floor.")
        // ---- Normalized position (0 = closed, 1 = open) -------------------
        // NOTE: normalized [0,1] — distinct from motor.set_position() (raw rad).
        // set_position() is fire-and-forget (no ACK); poll motor.control_stats()
        // for health. Throws if the gripper isn't calibrated.
        .def("position", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.position(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u,
           "Read the motor status and return the opening normalized to [0,1],\n"
           "0 = closed, 1 = open.\n\n"
           "Blocks for the ACK; timeout_ms bounds that wait. Raises ProtocolError\n"
           "when the gripper is not calibrated. While a controller is running, read\n"
           "its snapshot instead -- that comes from the status stream the controller\n"
           "already owns and costs no bus traffic.")
        // set_position() 同样不暴露:follower_gripper.cpp 里它就是
        // motor_.submit_impedance() 外面包了一层归一化,绕开控制器的程度和裸
        // 原语完全一样,只是名字看起来更像正经 API。用 ImpedanceController 或
        // ForcePositionController 的 set_target()。
        .def("pos_to_rad", [](FollowerGripper& g, float position) {
            py::gil_scoped_release gil;
            return g.pos_to_rad(position);
        }, py::arg("position"),
           "Convert a normalized position to the motor's raw shaft angle in radians\n"
           "-- the unit Motor's own primitives take.\n\n"
           "The input is clamped to [0,1] first. Loads the position map on first\n"
           "use, which is one request/ACK round trip; a local computation after\n"
           "that. Raises ProtocolError when the gripper is not calibrated.")
        .def("rad_to_pos", [](FollowerGripper& g, float raw_rad) {
            py::gil_scoped_release gil;
            return g.rad_to_pos(raw_rad);
        }, py::arg("raw_rad"),
           "Convert the motor's raw shaft angle in radians to a normalized position,\n"
           "clamped to [0,1]. Loads the position map on first use; local after that.\n"
           "Raises ProtocolError when the gripper is not calibrated.")
        .def("position_map", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            return g.position_map();          // copy of the cached converter
        }, "A copy of the cached raw-rad <-> [0,1] converter, loaded from the\n"
           "firmware config on first access.\n\n"
           "Hardware-free once you hold it, so a realtime loop can convert without\n"
           "touching the bus. Raises ProtocolError when the gripper is not\n"
           "calibrated.")
        .def("reload_config", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            g.reload_config();
        }, "Re-read the gripper config and rebuild the position converter.\n\n"
           "Blocks. Call it after a re-calibration or a write from elsewhere;\n"
           "set_gripper_config() already invalidates the cache by itself.")
        .def("__enter__", [](FollowerGripper& g) -> FollowerGripper& { return g; },
             "Returns the gripper unchanged -- the link is already open by the time\n"
             "you hold the object.")
        .def("__exit__",  [](FollowerGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // See LeaderGripper::__exit__ — drop the reader thread and its
            // callbacks here rather than at interpreter shutdown.
            g.transport().stop();
        }, "Stop the stream and tear the link down, including on an exception.\n\n"
           "It does not disable the motor -- a controller's stop() does that.");
}

}  // namespace xense::taccap::python

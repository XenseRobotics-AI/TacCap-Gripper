// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: IMU / Encoder samples, Key, Led, SensorErrors
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_sensors(py::module_& m) {
    using namespace xense::taccap;

    // ---- ImuSample / EncoderSample -------------------------------------
    py::class_<ImuSample>(m, "ImuSample",
        "One IMU reading, with the fixed-point wire values already converted to\n"
        "m/s^2, rad/s, uT and C.\n\n"
        "Handed out by IMU.read_once() and to IMU.on_data() subscribers.\n"
        "valid_flag says which groups this frame actually carries; check it before\n"
        "trusting one.")
        .def_property_readonly("host_time", [](const ImuSample& s) {
            return tp_to_seconds(s.host_time);
        }, "Host arrival time in seconds, on the host's monotonic clock -- directly\n"
           "comparable with time.monotonic(). The MCU timestamp counts from the\n"
           "MCU's own boot instead.")
        .def_readonly("mcu_timestamp_us", &ImuSample::mcu_timestamp_us,
                      "MCU-side capture time, microseconds since the MCU booted.")
        .def_readonly("valid_flag",       &ImuSample::valid_flag,
                      "Bitmask of the groups this frame carries: 0x01 accel, 0x02 gyro,\n"
                      "0x04 mag, 0x08 temperature.")
        .def_readonly("seq",              &ImuSample::seq,
                      "MCU packet counter, 16-bit and wrapping. A gap means frames were lost.")
        .def_readonly("temperature_c",    &ImuSample::temperature_c)
        .def_property_readonly("accel_mps2", [](const ImuSample& s) { return make_vec3(s.accel_mps2); },
                               "Acceleration as a fresh (3,) float32 array, [x, y, z] in m/s^2.")
        .def_property_readonly("gyro_radps", [](const ImuSample& s) { return make_vec3(s.gyro_radps); },
                               "Angular rate as a fresh (3,) float32 array, [x, y, z] in rad/s.")
        .def_property_readonly("mag_uT",     [](const ImuSample& s) { return make_vec3(s.mag_uT); },
                               "Magnetic field as a fresh (3,) float32 array, [x, y, z] in uT.")
        .def("__repr__", [](const ImuSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "ImuSample(seq=%u, accel=[%.2f,%.2f,%.2f], gyro=[%.3f,%.3f,%.3f], temp=%.2fC)",
                s.seq, s.accel_mps2[0], s.accel_mps2[1], s.accel_mps2[2],
                s.gyro_radps[0], s.gyro_radps[1], s.gyro_radps[2], s.temperature_c);
            return std::string(buf);
        });

    py::class_<EncoderSample>(m, "EncoderSample",
        "One encoder reading, from Encoder.read_once() or the encoder stream. On\n"
        "the leader this is the dedicated encoder hardware; on the follower the\n"
        "same telemetry also reaches you through the Motor component.\n\n"
        "position_rad is the user-facing angle and is clamped to >= 0 so a little\n"
        "post-zero drift does not read as motion past the stop; raw_position_rad\n"
        "keeps the unclamped firmware value for calibration and diagnostics.")
        .def_property_readonly("host_time", [](const EncoderSample& s) {
            return tp_to_seconds(s.host_time);
        }, "Host arrival time in seconds, on the host's monotonic clock -- directly\n"
           "comparable with time.monotonic(). The MCU timestamp counts from the\n"
           "MCU's own boot instead.")
        .def_readonly("mcu_timestamp_us", &EncoderSample::mcu_timestamp_us,
                      "MCU-side capture time, microseconds since the MCU booted.")
        // position_rad is the user-facing reading after SDK-side
        // normalisation: clamped to >= 0 to absorb small post-zero
        // drift. raw_position_rad exposes the unclamped firmware value
        // so calibration / diagnostic tooling can still see the drift.
        .def_readonly("position_rad",     &EncoderSample::position_rad,
                      "Jaw angle in radians, clamped to >= 0. raw_position_rad has the value\n"
                      "before the clamp.")
        .def_readonly("velocity_rad_s",   &EncoderSample::velocity_rad_s)
        // Normalized opening in [0, 1] (0 = fully closed, 1 = fully open).
        // float('nan') unless the Encoder has a position map installed —
        // LeaderGripper(normalize_position=True) does that automatically.
        // position_rad stays in radians either way.
        .def_readonly("position",         &EncoderSample::position,
                      "Normalized opening, 0 = closed, 1 = open. NaN unless a position map is\n"
                      "installed; LeaderGripper(normalize_position=True) installs one.")
        .def_property_readonly("raw_position_rad",
                               [](const EncoderSample& s) { return s.raw.position_rad; },
                               "Firmware's angle in radians, before the >= 0 clamp. A negative value\n"
                               "here is the drift that position_rad absorbs.")
        .def_property_readonly("raw_velocity_rad_s",
                               [](const EncoderSample& s) { return s.raw.velocity_rad_s; },
                               "Firmware's velocity in rad/s, exactly as received.")
        .def_readonly("status",           &EncoderSample::status,
                      "Encoder status bits: 0 = ok, 0x0001 error, 0x0002 overflow.")
        .def_readonly("seq",              &EncoderSample::seq,
                      "MCU packet counter, 16-bit and wrapping.")
        .def("__repr__", [](const EncoderSample& s) {
            char buf[192];
            std::snprintf(buf, sizeof(buf),
                "EncoderSample(seq=%u, pos=%.4frad (raw=%.4f), vel=%.4frad/s, "
                "position=%.4f)",
                s.seq, s.position_rad, s.raw.position_rad, s.velocity_rad_s,
                s.position);
            return std::string(buf);
        });

    // ---- CameraFrame ----------------------------------------------------
    py::class_<CameraFrame>(m, "CameraFrame",
        "One captured frame and the host time it arrived.")
        .def_property_readonly("host_time", [](const CameraFrame& f) {
            return tp_to_seconds(f.host_time);
        }, "Time the frame came back from V4L2, in seconds on the host's monotonic\n"
           "clock -- comparable with time.monotonic().")
        .def_readonly("frame_index", &CameraFrame::frame_index,
                      "1-based counter of the frames this Camera has handed out. It never\n"
                      "skips, so it cannot be used to spot losses -- read\n"
                      "Camera.dropped_frames for those.")
        .def_property_readonly("image", [](const CameraFrame& f) { return mat_to_numpy(f.image); },
                               "The frame as a fresh (H, W, 3) uint8 array. It is a copy, so it stays\n"
                               "valid once the next frame arrives. Channel order follows the Camera's\n"
                               "color_mode, BGR by default. Shape (0, 0, 3) when the frame was empty\n"
                               "or not 8-bit 3-channel.");

    // ---- IMU ------------------------------------------------------------
    py::class_<IMU>(m, "IMU",
        "Inertial unit: accelerometer, gyroscope, magnetometer and temperature.\n\n"
        "read_once() polls a single sample over the command channel; on_data()\n"
        "subscribes to the stream that start_streaming() drives.")
        .def("read_once", [](IMU& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_once(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100,
           "Poll one sample and wait for the answer. Works without start_streaming().\n\n"
           "Blocks up to timeout_ms with the GIL released. Raises ProtocolError when\n"
           "the firmware NACKs or answers with the wrong payload size, TimeoutError\n"
           "when it does not answer at all.")
        .def("on_data", [](IMU& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_data([cb](const ImuSample& s) {
                call_into_python("xense.taccap.IMU callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"),
           "Subscribe to streamed IMU frames; returns a subscription id for off().\n\n"
           "The callback runs on the transport's dispatcher thread, not the caller's,\n"
           "and every subscriber's callback is serialised there -- keep it short, or\n"
           "the reader-to-dispatcher queue fills and the oldest frames are dropped.\n"
           "An exception raised inside it is reported as unraisable and swallowed,\n"
           "never propagated. Frames arrive only while the gripper is streaming.")
        .def("off", &IMU::off, py::arg("subscription_id"),
             "Cancel a subscription taken out by on_data(). Takes effect even for\n"
             "frames already queued for dispatch.")
        .def("set_mag_calibration", [](IMU& self,
                                       std::array<float, 3> hard,
                                       std::array<float, 9> soft,
                                       unsigned timeout_ms) {
            py::gil_scoped_release gil;
            self.set_mag_calibration(hard, soft,
                                     std::chrono::milliseconds(timeout_ms));
        },
            py::arg("hard_iron"),
            py::arg("soft_iron_row_major"),
            py::arg("timeout_ms") = 500u,
            "Write the magnetometer hard-iron bias and soft-iron correction to the\n"
            "firmware (V1.4 command set).\n\n"
            "hard_iron is 3 floats of bias in uT; soft_iron_row_major is the 3x3\n"
            "correction matrix flattened ROW-major into 9 floats, the layout the\n"
            "firmware stores after MotionCal. Blocks for the ACK with the GIL\n"
            "released; raises ProtocolError on NACK.");

    // ---- KeySample + Key (V1.4) ----------------------------------------
    py::class_<KeySample>(m, "KeySample",
        "One button event: which key, and what happened to it.")
        .def_property_readonly("host_time", [](const KeySample& s) {
            return tp_to_seconds(s.host_time);
        }, "Host arrival time in seconds on the monotonic clock -- comparable with\n"
           "time.monotonic().")
        .def_readonly("key_id",    &KeySample::key_id,
                      "0 = K1, the only button on the TC-GU-01 board today.")
        .def_readonly("key_state", &KeySample::key_state,
                      "What happened; compare against the KeyState submodule constants\n"
                      "(SingleClickDown/Up, DoubleClick, LongPressDown/Up).")
        .def("__repr__", [](const KeySample& s) {
            char buf[64];
            std::snprintf(buf, sizeof(buf),
                "KeySample(key_id=%u, key_state=%u)", s.key_id, s.key_state);
            return std::string(buf);
        });
    py::class_<Key>(m, "Key",
        "The board's physical button (K1).\n\n"
        "Event-driven: the firmware emits one frame per state change whatever the\n"
        "stream configuration, so on_event() works without start_streaming().")
        .def("on_event", [](Key& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_event([cb](const KeySample& s) {
                call_into_python("xense.taccap.Key callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"),
           "Subscribe to button events; returns a subscription id for off().\n\n"
           "The callback runs on the transport's dispatcher thread, not the caller's,\n"
           "serialised with every other subscriber. An exception raised inside it is\n"
           "reported as unraisable and swallowed, never propagated.")
        .def("off", &Key::off, py::arg("subscription_id"),
             "Cancel a subscription taken out by on_event().");
    // KeyState constants for ergonomic comparison from Python.
    py::module_ key_state_mod = m.def_submodule("KeyState",
        "TC-GU-01 button state constants (V1.4).");
    key_state_mod.attr("SingleClickDown") = py::int_(xense::taccap::protocol::KeyState::SingleClickDown);
    key_state_mod.attr("SingleClickUp")   = py::int_(xense::taccap::protocol::KeyState::SingleClickUp);
    key_state_mod.attr("DoubleClick")     = py::int_(xense::taccap::protocol::KeyState::DoubleClick);
    key_state_mod.attr("LongPressDown")   = py::int_(xense::taccap::protocol::KeyState::LongPressDown);
    key_state_mod.attr("LongPressUp")     = py::int_(xense::taccap::protocol::KeyState::LongPressUp);

    // ---- Led (WS2812, V1.9) --------------------------------------------
    py::enum_<protocol::Ws2812Mode>(m, "Ws2812Mode",
        "Which layer Led.set() writes: Off darkens the LED, EffectSet sets the base\n"
        "colour the effect layer composites over, Override drives the LED directly.")
        .value("Off",       protocol::Ws2812Mode::Off)
        .value("EffectSet", protocol::Ws2812Mode::EffectSet)
        .value("Override",  protocol::Ws2812Mode::Override);
    py::enum_<protocol::Ws2812EffectType>(m, "Ws2812EffectType",
        "Effects Led.effect() can start. None_ stops the effect layer.\n\n"
        "NormalSolid / NormalBlink / OtaBlink / FaultBlink are firmware presets:\n"
        "their colours and rates live on the MCU and have changed between releases\n"
        "(leader 1.2.1 recoloured NormalSolid and halved the FaultBlink period), so\n"
        "only the wire values are pinned here. ColorBlink, ColorBreathe, HsvCycle\n"
        "and ColorLerp take their colours and period from effect()'s arguments.")
        .value("None_",       protocol::Ws2812EffectType::None)
        .value("NormalSolid", protocol::Ws2812EffectType::NormalSolid)
        .value("NormalBlink", protocol::Ws2812EffectType::NormalBlink)
        .value("OtaBlink",    protocol::Ws2812EffectType::OtaBlink)
        .value("FaultBlink",  protocol::Ws2812EffectType::FaultBlink)
        .value("Demo",        protocol::Ws2812EffectType::Demo)
        .value("ColorBlink",  protocol::Ws2812EffectType::ColorBlink)
        .value("ColorBreathe",protocol::Ws2812EffectType::ColorBreathe)
        .value("HsvCycle",    protocol::Ws2812EffectType::HsvCycle)
        .value("ColorLerp",   protocol::Ws2812EffectType::ColorLerp);
    py::class_<Led>(m, "Led",
        "The board's WS2812 addressable LED, in two layers: a base colour written\n"
        "by set(), and an effect written by effect() that composites additively\n"
        "over it. Both commands are ACKed, so every call here blocks.")
        .def("set", [](Led& self, protocol::Ws2812Mode mode, uint8_t r, uint8_t g,
                       uint8_t b, uint8_t brightness, uint16_t blink_ms) {
            py::gil_scoped_release gil;
            self.set(mode, r, g, b, brightness, blink_ms);
        }, py::arg("mode"), py::arg("r"), py::arg("g"), py::arg("b"),
           py::arg("brightness") = 0, py::arg("blink_ms") = 0,
           "Write the base colour layer. r/g/b are 0-255.\n\n"
           "brightness is the global level, 0-255, where 0 means leave it unchanged\n"
           "rather than go dark. blink_ms is the blink half-period in milliseconds;\n"
           "0 is steady. Blocks for the ACK with the GIL released; raises\n"
           "ProtocolError on NACK.")
        .def("off", [](Led& self) { py::gil_scoped_release gil; self.off(); },
             "Darken the LED: set(Ws2812Mode.Off, 0, 0, 0). The effect layer has its\n"
             "own stop, effect_off().")
        .def("effect", [](Led& self, protocol::Ws2812EffectType eff, uint8_t r1,
                          uint8_t g1, uint8_t b1, uint16_t period_ms, uint8_t r2,
                          uint8_t g2, uint8_t b2, uint8_t param1, uint8_t param2) {
            py::gil_scoped_release gil;
            self.effect(eff, r1, g1, b1, period_ms, r2, g2, b2, param1, param2);
        }, py::arg("effect"), py::arg("r1"), py::arg("g1"), py::arg("b1"),
           py::arg("period_ms") = 1000, py::arg("r2") = 0, py::arg("g2") = 0,
           py::arg("b2") = 0, py::arg("param1") = 0, py::arg("param2") = 0,
           "Start an effect on the layer that composites over the base colour.\n\n"
           "r1/g1/b1 is the effect's colour (its start colour for ColorLerp);\n"
           "r2/g2/b2 is used only by ColorLerp. period_ms is the effect period.\n"
           "param1/param2 are effect-specific -- breathe reads them as its minimum\n"
           "and maximum brightness. Blocks for the ACK with the GIL released; raises\n"
           "ProtocolError on NACK.")
        .def("effect_off", [](Led& self) { py::gil_scoped_release gil; self.effect_off(); },
             "Stop the effect layer: effect(Ws2812EffectType.None_, 0, 0, 0). It sends\n"
             "no base-colour command, so whatever set() last wrote is left alone.");

    // ---- SensorErrorSample + SensorErrors (V1.6) -----------------------
    py::class_<SensorErrorSample>(m, "SensorErrorSample",
        "One sensor-error report, pushed when a sensor's error state changes --\n"
        "including the change back to healthy, which arrives as error_code 0.")
        .def_property_readonly("host_time", [](const SensorErrorSample& s) {
            return tp_to_seconds(s.host_time);
        }, "Host arrival time in seconds on the monotonic clock -- comparable with\n"
           "time.monotonic().")
        .def_readonly("sensor_id",        &SensorErrorSample::sensor_id,
                      "Which sensor: 0 imu, 1 imu magnetometer, 2 encoder, 3 eskin1,\n"
                      "4 eskin2, 5 motor.")
        .def_readonly("error_code",       &SensorErrorSample::error_code,
                      "0x00 none (the sensor recovered), 0x01 init failed, 0x02 comm timeout,\n"
                      "0x03 data invalid, 0x04 offline, 0x05 overflow, 0x06 out of range.")
        .def_readonly("error_count",      &SensorErrorSample::error_count,
                      "Cumulative for this sensor since the MCU booted, not a count for this\n"
                      "report.")
        .def_readonly("mcu_timestamp_ms", &SensorErrorSample::mcu_timestamp_ms,
                      "MCU tick in MILLISECONDS since boot -- the IMU and encoder samples\n"
                      "timestamp in microseconds instead.")
        .def("__repr__", [](const SensorErrorSample& s) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "SensorErrorSample(sensor=%u, code=0x%02x, count=%u, ts_ms=%u)",
                s.sensor_id, s.error_code, s.error_count, s.mcu_timestamp_ms);
            return std::string(buf);
        });
    py::class_<SensorErrors>(m, "SensorErrors",
        "Asynchronous sensor health reports (V1.6 command set).\n\n"
        "The firmware pushes a frame the moment it detects a sensor fault and again\n"
        "when the sensor recovers, instead of the older pattern of learning about\n"
        "it from a NACK on the next command. Event-driven like Key: it needs no\n"
        "start_streaming().")
        .def("on_report", [](SensorErrors& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_report([cb](const SensorErrorSample& s) {
                call_into_python("xense.taccap.SensorErrors callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"),
           "Subscribe to sensor-error reports; returns a subscription id for off().\n\n"
           "The callback runs on the transport's dispatcher thread, not the caller's.\n"
           "An exception raised inside it is reported as unraisable and swallowed.")
        .def("off", &SensorErrors::off, py::arg("subscription_id"),
             "Cancel a subscription taken out by on_report().");
}

}  // namespace xense::taccap::python

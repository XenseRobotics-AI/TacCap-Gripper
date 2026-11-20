// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: CameraFisheyeCal, Calibration, GripperPosition
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_calibration(py::module_& m) {
    using namespace xense::taccap;
    // ---- CameraFisheyeCal (V2.0 — Cmd::CameraFisheyeCal 0x2B) -------------
    py::class_<protocol::CameraFisheyeCal>(m, "CameraFisheyeCal",
        "Wrist-camera intrinsics and fisheye distortion, as the firmware persists\n"
        "them (Cmd 0x2B, command set >= V2.0). k1..k4 are the OpenCV fisheye\n"
        "(equidistant) coefficients, in D order.\n\n"
        "Eight floats in an order fixed by the firmware. They hold only at the\n"
        "capture resolution they were measured at -- 640x480 and 1920x1080 are\n"
        "different sensor crops, so they cannot be rescaled between the two. That\n"
        "resolution travels separately: see FisheyeCalRecord and\n"
        "Calibration.read_fisheye_record.")
        .def(py::init([]() { return protocol::CameraFisheyeCal{}; }),
             "All-zero record. Not usable for rectification -- fx = fy = 0 maps every\n"
             "pixel outside the source image, so the frame comes back black.")
        .def(py::init([](float fx, float fy, float cx, float cy,
                         float k1, float k2, float k3, float k4) {
                 return protocol::CameraFisheyeCal{fx, fy, cx, cy, k1, k2, k3, k4};
             }),
             py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
             py::arg("k1") = 0.0f, py::arg("k2") = 0.0f,
             py::arg("k3") = 0.0f, py::arg("k4") = 0.0f,
             "Build from focal length and principal point in px, plus the four\n"
             "distortion coefficients (default 0, i.e. no distortion model).")
        .def_readwrite("fx", &protocol::CameraFisheyeCal::fx, "Focal length x, px.")
        .def_readwrite("fy", &protocol::CameraFisheyeCal::fy, "Focal length y, px.")
        .def_readwrite("cx", &protocol::CameraFisheyeCal::cx, "Principal point x, px.")
        .def_readwrite("cy", &protocol::CameraFisheyeCal::cy, "Principal point y, px.")
        .def_readwrite("k1", &protocol::CameraFisheyeCal::k1)
        .def_readwrite("k2", &protocol::CameraFisheyeCal::k2)
        .def_readwrite("k3", &protocol::CameraFisheyeCal::k3)
        .def_readwrite("k4", &protocol::CameraFisheyeCal::k4)
        // OpenCV-shaped views: cv2.fisheye.undistortImage(img, K, D).
        .def_property_readonly("K", [](const protocol::CameraFisheyeCal& c) {
            py::array_t<double> a(std::vector<py::ssize_t>{3, 3});
            auto* p = a.mutable_data();
            p[0] = c.fx; p[1] = 0.0;  p[2] = c.cx;
            p[3] = 0.0;  p[4] = c.fy; p[5] = c.cy;
            p[6] = 0.0;  p[7] = 0.0;  p[8] = 1.0;
            return a;
        }, "3x3 camera matrix as float64 numpy array (OpenCV K).")
        .def_property_readonly("D", [](const protocol::CameraFisheyeCal& c) {
            // Shape must be spelled as a container: array_t<double>(4) picks
            // the wrong overload on pybind11 2.9 and yields a stride-0 view.
            py::array_t<double> a(std::vector<py::ssize_t>{4});
            auto* p = a.mutable_data();
            p[0] = c.k1; p[1] = c.k2; p[2] = c.k3; p[3] = c.k4;
            return a;
        }, "4x1 fisheye distortion vector as float64 numpy array (OpenCV D).")
        .def("__repr__", [](const protocol::CameraFisheyeCal& c) {
            char buf[224];
            std::snprintf(buf, sizeof(buf),
                "CameraFisheyeCal(fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f, "
                "k=[%.5f, %.5f, %.5f, %.5f])",
                c.fx, c.fy, c.cx, c.cy, c.k1, c.k2, c.k3, c.k4);
            return std::string(buf);
        });

    // ---- FisheyeCalRecord (V2.10 — params + calibration resolution) --------
    py::class_<FisheyeCalRecord>(m, "FisheyeCalRecord",
        "A fisheye record as the firmware holds it: the params plus the capture\n"
        "resolution they were measured at (Calibration.read_fisheye_record).\n\n"
        "width/height 0/0 means the size is UNKNOWN -- the params were written\n"
        "without one (write_fisheye, or firmware older than 1.3.0-rc1).\n"
        "Do not assume 640x480 then; infer_fisheye_cal_size()\n"
        "applies the shared inference rule, and resolve_fisheye() uses it.")
        .def(py::init([](const protocol::CameraFisheyeCal& cal,
                         uint16_t width, uint16_t height) {
                 return FisheyeCalRecord{cal, width, height};
             }),
             py::arg("calibration") = protocol::CameraFisheyeCal{},
             py::arg("width") = 0, py::arg("height") = 0,
             "Build a record; width/height default to 0/0 (size unknown).")
        .def_readwrite("calibration", &FisheyeCalRecord::calibration,
                       "The 8 params, as a CameraFisheyeCal.")
        .def_readwrite("width", &FisheyeCalRecord::width,
                       "Capture width in px the params belong to; 0 = unknown.")
        .def_readwrite("height", &FisheyeCalRecord::height,
                       "Capture height in px the params belong to; 0 = unknown.")
        .def("size_known", &FisheyeCalRecord::size_known,
             "True when width and height are both non-zero, i.e. the firmware "
             "recorded the calibration resolution.")
        .def("__repr__", [](const FisheyeCalRecord& r) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "FisheyeCalRecord(fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f, size=%s)",
                r.calibration.fx, r.calibration.fy, r.calibration.cx,
                r.calibration.cy,
                r.size_known()
                    ? (std::to_string(r.width) + "x" + std::to_string(r.height)).c_str()
                    : "unknown");
            return std::string(buf);
        });

    // ---- ResolvedFisheyeCal (Calibration.resolve_fisheye's answer) ---------
    // Read-only, and still unpacks as the (calibration, is_reference, reason)
    // tuple resolve_fisheye() returned before the size fields existed, so
    // `cal, is_reference, reason = g.calibration.resolve_fisheye()` keeps working.
    py::class_<ResolvedFisheyeCal>(m, "ResolvedFisheyeCal",
        "What to rectify the wrist camera with, from Calibration.resolve_fisheye().\n\n"
        "Unpacks as (calibration, is_reference, reason) for compatibility with\n"
        "code written before calibrated_size / size_inferred existed.")
        .def_property_readonly("calibration",
            [](const ResolvedFisheyeCal& r) { return r.calibration; },
            "The CameraFisheyeCal to rectify with.")
        .def_property_readonly("is_reference",
            [](const ResolvedFisheyeCal& r) { return r.is_reference; },
            "True when the SDK's shared reference values stood in.")
        .def_property_readonly("reason",
            [](const ResolvedFisheyeCal& r) { return r.reason; },
            "Why the reference stood in; empty when is_reference is False.")
        .def_property_readonly("calibrated_size",
            [](const ResolvedFisheyeCal& r) {
                return py::make_tuple(r.calibrated_size.width, r.calibrated_size.height);
            },
            "(width, height) the calibration belongs to: recorded by the firmware,\n"
            "inferred for a record written without one (size_inferred), or\n"
            "(640, 480) for the reference. Frames must be exactly this size.")
        .def_property_readonly("size_inferred",
            [](const ResolvedFisheyeCal& r) { return r.size_inferred; },
            "True when calibrated_size was inferred from the principal point\n"
            "because the record carries no size (see infer_fisheye_cal_size).")
        .def_property_readonly("mismatched_size",
            [](const ResolvedFisheyeCal& r) -> py::object {
                if (r.mismatched_size.empty()) return py::none();
                return py::make_tuple(r.mismatched_size.width, r.mismatched_size.height);
            },
            "Set only by resolve_fisheye_for(): when the reference stands in solely\n"
            "because this unit's own calibration belongs to another resolution, that\n"
            "(width, height); None otherwise. The SDK's wrist undistortion raises in\n"
            "that case rather than use the reference.")
        .def_property_readonly("matches_reference",
            [](const ResolvedFisheyeCal& r) { return r.matches_reference; },
            "True when this unit's own record was used (is_reference False) but it\n"
            "IS the reference values (matches_fisheye_fallback_cal): the factory\n"
            "flashed the reference instead of calibrating the unit. A warning is\n"
            "logged too. Recalibrate the unit.")
        .def("__len__", [](const ResolvedFisheyeCal&) { return 3; })
        .def("__getitem__", [](const ResolvedFisheyeCal& r, py::ssize_t i) {
            py::tuple t = py::make_tuple(r.calibration, r.is_reference, r.reason);
            return py::object(t[py::int_(i < 0 ? i + 3 : i)]);
        })
        .def("__iter__", [](const ResolvedFisheyeCal& r) {
            return py::iter(py::make_tuple(r.calibration, r.is_reference, r.reason));
        })
        .def("__repr__", [](const ResolvedFisheyeCal& r) {
            return "ResolvedFisheyeCal(is_reference=" +
                   std::string(r.is_reference ? "True" : "False") +
                   ", calibrated_size=" + std::to_string(r.calibrated_size.width) +
                   "x" + std::to_string(r.calibrated_size.height) +
                   ", size_inferred=" + (r.size_inferred ? "True" : "False") +
                   (r.matches_reference ? std::string(", matches_reference=True") : std::string()) +
                   (r.reason.empty() ? std::string() : ", reason='" + r.reason + "'") +
                   ")";
        });

    // ---- Calibration (V2.0/V2.1 firmware-persisted calibration records) ----
    py::class_<Calibration>(m, "Calibration",
        "The calibration records the firmware keeps in MCU flash; reach it as\n"
        "gripper.calibration.\n\n"
        "The firmware is a dumb store for both records: it persists the bytes, does\n"
        "no unit conversion and no range clamping, and rejects only NaN/Inf (plus\n"
        "max_rad <= 0). Every call here is a command round trip that blocks for the\n"
        "ACK, so none of them belong in a control loop.\n\n"
        "A record that was never written NACKs ErrorCode.CalNotSet, which the read\n"
        "methods surface as None -- an expected state, not an error. Every other\n"
        "NACK raises ProtocolError.")
        .def("read_fisheye",
             [](Calibration& self, unsigned timeout_ms) -> py::object {
                 std::optional<protocol::CameraFisheyeCal> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_fisheye(std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::cast(*v);
             },
             py::arg("timeout_ms") = 200,
             "Read the fisheye intrinsics + distortion persisted in MCU flash. "
             "Returns None when the firmware has never been calibrated "
             "(ErrorCode.CalNotSet); raises ProtocolError on any other NACK.")
        .def("resolve_fisheye",
             [](Calibration& self, unsigned timeout_ms) {
                 ResolvedFisheyeCal r;
                 {
                     py::gil_scoped_release nogil;
                     r = self.resolve_fisheye(std::chrono::milliseconds(timeout_ms));
                 }
                 return r;
             },
             py::arg("timeout_ms") = 200,
             "Intrinsics to rectify with, as a ResolvedFisheyeCal -- which still "
             "unpacks as (calibration, is_reference, reason).\n\n"
             "Prefer this over read_fisheye() unless you need to know what the "
             "flash actually holds. An uncalibrated unit answers a read with an "
             "all-zero record rather than a NACK, and handing that to "
             "FisheyeUndistorter remaps every frame to black; this applies the "
             "fallback policy once, so a caller owning the wrist UVC device "
             "itself makes the same decision the SDK makes internally.\n\n"
             "is_reference is True when the SDK's shared reference values stood "
             "in, and reason says why — empty otherwise. Warn when it is True: "
             "the reference intrinsics are approximate, since lens placement "
             "varies per assembly.\n\n"
             "calibrated_size is the (width, height) the intrinsics belong to: "
             "the size the firmware recorded, or -- for a record written without "
             "one -- infer_fisheye_cal_size()'s answer, with size_inferred True "
             "and a warning logged. A record whose size cannot be inferred is "
             "treated as unusable and the reference (640x480) stands in.")
        .def("resolve_fisheye_for",
             [](Calibration& self, int width, int height, unsigned timeout_ms) {
                 ResolvedFisheyeCal r;
                 {
                     py::gil_scoped_release nogil;
                     r = self.resolve_fisheye_for(width, height,
                                                  std::chrono::milliseconds(timeout_ms));
                 }
                 return r;
             },
             py::arg("width"), py::arg("height"), py::arg("timeout_ms") = 200,
             "resolve_fisheye() for a camera running at width x height: this "
             "unit's calibration for THAT resolution, as a ResolvedFisheyeCal.\n\n"
             "On firmware with dual calibration slots (1.3.0-rc4+ / 1.3.1, "
             "command set V2.11) this reads the 640x480 or 1920x1080 slot. On "
             "older firmware it reads the single record and uses it only when "
             "its size -- recorded, or inferred as resolve_fisheye() does -- "
             "equals width x height.\n\n"
             "Otherwise the reference values stand in (is_reference True, reason "
             "says why). The reference exists at 640x480 only, so for any other "
             "size its calibrated_size still reads (640, 480) and a "
             "FisheyeUndistorter built for the camera refuses it.")
        .def("write_fisheye",
             [](Calibration& self, const protocol::CameraFisheyeCal& cal,
                unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.write_fisheye(cal, std::chrono::milliseconds(timeout_ms));
             },
             py::arg("cal"), py::arg("timeout_ms") = 500,
             "Persist fisheye parameters to MCU flash (survives power cycles). "
             "NaN/Inf values are rejected by the firmware.\n\n"
             "Writes the params only: on firmware that records the calibration "
             "resolution (firmware 1.3.0-rc1+ / 1.3.1) this marks it unknown. "
             "Prefer write_fisheye_sized().")
        .def("supports_fisheye_size",
             [](Calibration& self, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 return self.supports_fisheye_size(std::chrono::milliseconds(timeout_ms));
             },
             py::arg("timeout_ms") = 200,
             "True when the firmware records the fisheye calibration resolution "
             "(command set V2.10: firmware 1.3.1 or later, or 1.3.0-rc1+).\n\n"
             "Probes with the sized read: older firmware rejects it as "
             "InvalidParam. The answer is cached on this Calibration; a transport "
             "failure raises and is not cached.")
        .def("read_fisheye_record",
             [](Calibration& self, unsigned timeout_ms) -> py::object {
                 std::optional<FisheyeCalRecord> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_fisheye_record(std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::cast(*v);
             },
             py::arg("timeout_ms") = 200,
             "Read the fisheye params together with the resolution they belong to, "
             "as a FisheyeCalRecord. On firmware that cannot record a size the "
             "params come back with width/height 0/0 (unknown). Returns None when "
             "never calibrated (ErrorCode.CalNotSet).")
        .def("write_fisheye_sized",
             [](Calibration& self, const protocol::CameraFisheyeCal& cal,
                int width, int height, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.write_fisheye_sized(cal, width, height,
                                          std::chrono::milliseconds(timeout_ms));
             },
             py::arg("cal"), py::arg("width"), py::arg("height"),
             py::arg("timeout_ms") = 500,
             "Persist fisheye params together with the capture resolution they "
             "were measured at.\n\n"
             "Raises RuntimeError before sending anything unless "
             "fisheye_cal_size_plausible(cal, width, height) and, for 640x480 "
             "and 1920x1080, fisheye_cal_slot_plausible(cal, width, height) "
             "(principal point within 15% of the frame centre). Raises "
             "ProtocolError when the firmware rejects the write.\n\n"
             "Dual-slot firmware (V2.11) stores it in the slot for its size and "
             "leaves the other slot alone; any other size raises RuntimeError "
             "there. Firmware that "
             "cannot record a size (older than firmware 1.3.0-rc1) "
             "accepts only a 640x480 calibration, through the legacy write; any "
             "other resolution raises RuntimeError asking for an upgrade.")
        .def("supports_fisheye_slots",
             [](Calibration& self, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 return self.supports_fisheye_slots(std::chrono::milliseconds(timeout_ms));
             },
             py::arg("timeout_ms") = 200,
             "True when the firmware keeps a separate fisheye calibration for "
             "640x480 and for 1920x1080 (command set V2.11: firmware 1.3.1, or "
             "1.3.0-rc4+).\n\n"
             "Probes with the slot read at 640x480: older firmware rejects it as "
             "InvalidParam. Firmware already known not to record a size answers "
             "False without a round trip. Cached on this Calibration; a "
             "transport failure raises and is not cached.")
        .def("read_fisheye_slot",
             [](Calibration& self, int width, int height,
                unsigned timeout_ms) -> py::object {
                 std::optional<FisheyeCalRecord> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_fisheye_slot(width, height,
                                                std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::cast(*v);
             },
             py::arg("width"), py::arg("height"), py::arg("timeout_ms") = 200,
             "The calibration for one capture resolution, as a FisheyeCalRecord, "
             "or None when there is none. width x height must be 640x480 or "
             "1920x1080 (RuntimeError otherwise, before anything is sent).\n\n"
             "Dual-slot firmware (V2.11) reads that slot; the 640x480 slot comes "
             "back with size (0, 0) when its params were written without a size. "
             "Single-record firmware returns its record when the recorded size "
             "matches; a record with no size counts as the 640x480 one, as it "
             "does on V2.11.")
        .def("clear_fisheye_slot",
             [](Calibration& self, int width, int height, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.clear_fisheye_slot(width, height,
                                         std::chrono::milliseconds(timeout_ms));
             },
             py::arg("width"), py::arg("height"), py::arg("timeout_ms") = 500,
             "Erase the fisheye calibration for 640x480 or 1920x1080 (dual-slot "
             "firmware, V2.11, only); that slot then reads None. Clearing "
             "640x480 also drops its size record; clearing an empty slot "
             "succeeds; the other slot is never touched.\n\n"
             "Raises RuntimeError before sending anything for any other size, or "
             "on firmware without slots. Raises ProtocolError when the firmware "
             "rejects the clear.")
        .def("read_encoder_max_rad",
             [](Calibration& self, unsigned timeout_ms) -> py::object {
                 std::optional<float> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_encoder_max_rad(std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::float_(*v);
             },
             py::arg("timeout_ms") = 200,
             "Leader only. Read the encoder shaft angle (rad) at full open, "
             "measured from the encoder zero (fully closed). Returns None when "
             "never calibrated. Raises ProtocolError(InvalidCmd) on a follower "
             "or on firmware older than V2.1.")
        .def("write_encoder_max_rad",
             [](Calibration& self, float max_rad, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.write_encoder_max_rad(max_rad, std::chrono::milliseconds(timeout_ms));
             },
             py::arg("max_rad"), py::arg("timeout_ms") = 500,
             "Leader only. Persist the full-open shaft angle (rad) to MCU "
             "flash. Firmware rejects NaN/Inf and max_rad <= 0.");

    // ---- GripperPosition: pure raw-rad <-> normalized [0,1] converter ------
    py::class_<GripperPosition>(m, "GripperPosition",
        "Converter between a normalized position in [0, 1] (0 = fully closed,\n"
        "1 = fully open) and the follower motor's raw shaft angle in rad.\n\n"
        "Pure and hardware-free: usable without opening a gripper. The motor zero\n"
        "is the closed pose and travel runs to max_open_rad, in the motor's negative\n"
        "direction when the config carries the reverse flag. Both conversions clamp,\n"
        "so a caller cannot command past the calibrated travel -- the firmware clamps\n"
        "to the same range anyway.")
        .def(py::init<>(),
             "Unconfigured converter: valid is False and both conversions return 0.0.")
        .def(py::init<const protocol::GripperConfig&>(), py::arg("config"),
             "Build from a firmware GripperConfig, e.g.\n"
             "FollowerGripper.get_gripper_config().")
        .def_static("from_travel", &GripperPosition::from_travel,
                    py::arg("max_rad"), py::arg("min_rad") = 0.0f,
                    py::arg("reverse") = false,
                    "Build from an explicit travel span instead of a "
                    "GripperConfig — this is how the leader gripper's map is "
                    "built from its EncoderMaxCal value.")
        .def_property_readonly("valid",        &GripperPosition::valid,
                               "True when the travel span is positive and, for a converter built\n"
                               "from a GripperConfig, the config's valid bit is set. False means the\n"
                               "gripper is not calibrated and a normalized position is meaningless.")
        .def_property_readonly("max_open_rad", &GripperPosition::max_open_rad)
        .def_property_readonly("min_open_rad", &GripperPosition::min_open_rad,
                               "Raw shaft angle normalized 0.0 maps to -- not always zero; see\n"
                               "GripperConfig.min_open_rad.")
        .def_property_readonly("reverse",      &GripperPosition::reverse,
                               "True when open travel advances in the motor's negative direction.")
        .def("to_position", &GripperPosition::to_position, py::arg("raw_rad"),
             "Raw shaft angle (rad) -> normalized position, clamped to [0, 1].\n"
             "Returns 0.0 when the travel span is not positive.")
        .def("to_rad",      &GripperPosition::to_rad,      py::arg("position"),
             "Normalized position -> raw shaft angle (rad). The input is clamped to\n"
             "[0, 1] first, so the result never leaves the calibrated travel.")
        .def("__repr__", [](const GripperPosition& gp) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "GripperPosition(valid=%d, max_open=%.4frad, reverse=%d)",
                gp.valid(), gp.max_open_rad(), gp.reverse());
            return std::string(buf);
        });
}

}  // namespace xense::taccap::python

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 entry point for the xense.taccap Python package.
//
// This file binds the wire layer -- version, error types, the protocol enums
// and payload structs, framing, SerialBus and Transport. Everything above it
// is bound elsewhere and pulled in at the bottom of PYBIND11_MODULE:
// bind_log() and bind_components(), the latter fanning out to the bind_*.cpp
// files (see bindings_common.hpp for why their call order matters).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>

#include <taccap/version.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/bus/frame.hpp>
#include <taccap/bus/serial_bus.hpp>
#include <taccap/bus/transport.hpp>

#include "gil_safe.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

namespace py = pybind11;

// GIL-safe callback ownership/invocation is shared with components.cpp.
using xense::taccap::python::gil::call_into_python;
using xense::taccap::python::gil::make_gil_safe_callback;

namespace {

// Helper: copy a py::bytes into a fresh std::vector<uint8_t>.
std::vector<uint8_t> bytes_to_vec(py::bytes b) {
    char* buf = nullptr;
    py::ssize_t len = 0;
    PYBIND11_BYTES_AS_STRING_AND_SIZE(b.ptr(), &buf, &len);
    return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(buf),
                                reinterpret_cast<uint8_t*>(buf) + len);
}

py::bytes vec_to_bytes(const std::vector<uint8_t>& v) {
    return py::bytes(reinterpret_cast<const char*>(v.data()), v.size());
}

}  // namespace

namespace xense::taccap::python {
void bind_components(py::module_& m);  // defined in components.cpp
void bind_log(py::module_& m);         // defined in log.cpp
}

PYBIND11_MODULE(_taccap_native, m) {
    m.doc() = "xense.taccap native extension: TC-GU-01 wire protocol, gripper\n"
              "components and the background controllers.\n\n"
              "Import xense.taccap rather than this module directly -- the package\n"
              "is the documented surface and this name is an implementation detail.";

    m.attr("__version__") = TACCAP_VERSION_STRING;

    m.def("hello", []() {
        return std::string("taccap-gripper OK; version ") + TACCAP_VERSION_STRING;
    });

    using namespace xense::taccap;
    namespace tp = protocol;
    namespace tb = bus;

    // ---- Exception translation ------------------------------------------
    py::register_exception<ProtocolError>(m, "ProtocolError");
    py::register_exception<CrcError>(m, "CrcError");
    py::register_exception<IoError>(m, "IoError");
    py::register_exception<TimeoutError>(m, "TimeoutError");

    // ---- Enums -----------------------------------------------------------
    py::enum_<tp::Address>(m, "Address")
        .value("PC",        tp::Address::PC)
        .value("MCU",       tp::Address::MCU)
        .value("MCU_RIGHT", tp::Address::MCU_RIGHT);

    py::enum_<tp::FrameType>(m, "FrameType")
        .value("CMD_NEED_ACK", tp::FrameType::CMD_NEED_ACK)
        .value("CMD_NO_ACK",   tp::FrameType::CMD_NO_ACK)
        .value("ACK",          tp::FrameType::ACK)
        .value("DATA",         tp::FrameType::DATA);

    py::enum_<tp::Cmd>(m, "Cmd")
        .value("Heartbeat",          tp::Cmd::Heartbeat)
        .value("GetVersion",         tp::Cmd::GetVersion)
        .value("ResetDevice",        tp::Cmd::ResetDevice)
        .value("GetSn",              tp::Cmd::GetSn)
        .value("SetSn",              tp::Cmd::SetSn)
        .value("GetDevType",         tp::Cmd::GetDevType)
        .value("SetDevType",         tp::Cmd::SetDevType)
        .value("GetImu",             tp::Cmd::GetImu)
        .value("GetEncoder",         tp::Cmd::GetEncoder)
        .value("GetEskin1",          tp::Cmd::GetEskin1)
        .value("GetEskin2",          tp::Cmd::GetEskin2)
        .value("GetAllSensors",      tp::Cmd::GetAllSensors)
        .value("KeyStatus",          tp::Cmd::KeyStatus)
        .value("StartStream",        tp::Cmd::StartStream)
        .value("StopStream",         tp::Cmd::StopStream)
        .value("SetStreamRate",      tp::Cmd::SetStreamRate)
        .value("SetStreamMode",      tp::Cmd::SetStreamMode)
        .value("SetEncoderZero",     tp::Cmd::SetEncoderZero)
        .value("SetImuCal",          tp::Cmd::SetImuCal)
        .value("SetImuMagCal",       tp::Cmd::SetImuMagCal)
        .value("SetCalResult",       tp::Cmd::SetCalResult)
        .value("SetAllCalResult",    tp::Cmd::SetAllCalResult)
        .value("GetCalResult",       tp::Cmd::GetCalResult)
        .value("SensorErrorReport",  tp::Cmd::SensorErrorReport)
        .value("CameraFisheyeCal",   tp::Cmd::CameraFisheyeCal)
        .value("EncoderMaxCal",      tp::Cmd::EncoderMaxCal)
        .value("MotorEnable",        tp::Cmd::MotorEnable)
        .value("MotorDisable",       tp::Cmd::MotorDisable)
        .value("MotorClearFault",    tp::Cmd::MotorClearFault)
        .value("MotorSetZero",       tp::Cmd::MotorSetZero)
        .value("MotorGetCanId",      tp::Cmd::MotorGetCanId)
        .value("MotorSetCanId",      tp::Cmd::MotorSetCanId)
        .value("MotorSwitchProtocol", tp::Cmd::MotorSwitchProtocol)
        .value("MotorGetProtocol",   tp::Cmd::MotorGetProtocol)
        .value("MotorSetStartupLimitTorque", tp::Cmd::MotorSetStartupLimitTorque)
        .value("MotorGetStartupLimitTorque", tp::Cmd::MotorGetStartupLimitTorque)
        .value("MotorPosCtrl",       tp::Cmd::MotorPosCtrl)
        .value("MotorVelCtrl",       tp::Cmd::MotorVelCtrl)
        .value("MotorTorqueCtrl",    tp::Cmd::MotorTorqueCtrl)
        .value("MotorImpedanceCtrl", tp::Cmd::MotorImpedanceCtrl)
        .value("GetMotorStatus",     tp::Cmd::GetMotorStatus)
        .value("GetMotorControlStats", tp::Cmd::GetMotorControlStats)
        .value("GetMotorFault",      tp::Cmd::GetMotorFault)
        .value("GetMotorStatusExt",  tp::Cmd::GetMotorStatusExt)
        // 这三项此前漏在枚举外(GetMotorSpec / GetHomeDiag 从来没补过)。枚举的
        // 用处就是镜像命令集,缺项会让 transport.send_cmd() 的调用方只能写裸值。
        .value("GetMotorSpec",       tp::Cmd::GetMotorSpec)
        .value("GetHomeDiag",        tp::Cmd::GetHomeDiag)
        .value("GetMotorVersion",    tp::Cmd::GetMotorVersion)
        .value("SetImuConfig",       tp::Cmd::SetImuConfig)
        .value("GetImuConfig",       tp::Cmd::GetImuConfig)
        .value("SetEncoderConfig",   tp::Cmd::SetEncoderConfig)
        .value("GetEncoderConfig",   tp::Cmd::GetEncoderConfig)
        .value("SetEskinConfig",     tp::Cmd::SetEskinConfig)
        .value("GetEskinConfig",     tp::Cmd::GetEskinConfig)
        .value("SetGripperConfig",   tp::Cmd::SetGripperConfig)
        .value("GetGripperConfig",   tp::Cmd::GetGripperConfig)
        .value("OtaStart",           tp::Cmd::OtaStart)
        .value("OtaWriteBlock",      tp::Cmd::OtaWriteBlock)
        .value("OtaVerify",          tp::Cmd::OtaVerify)
        .value("OtaApply",           tp::Cmd::OtaApply)
        .value("OtaAbort",           tp::Cmd::OtaAbort)
        .value("OtaGetStatus",       tp::Cmd::OtaGetStatus);

    py::enum_<tp::ErrorCode>(m, "ErrorCode")
        .value("Ok",             tp::ErrorCode::Ok)
        .value("InvalidCmd",     tp::ErrorCode::InvalidCmd)
        .value("InvalidParam",   tp::ErrorCode::InvalidParam)
        .value("LengthMismatch", tp::ErrorCode::LengthMismatch)
        .value("CrcError",       tp::ErrorCode::CrcError)
        .value("Timeout",        tp::ErrorCode::Timeout)
        .value("MotorFault",     tp::ErrorCode::MotorFault)
        .value("SensorOffline",  tp::ErrorCode::SensorOffline)
        .value("SysBusy",        tp::ErrorCode::SysBusy)
        .value("SeqMismatch",    tp::ErrorCode::SeqMismatch)
        .value("CalNotSet",      tp::ErrorCode::CalNotSet)
        .value("OtaBusy",        tp::ErrorCode::OtaBusy)
        .value("OtaNotStarted",  tp::ErrorCode::OtaNotStarted)
        .value("OtaOffsetErr",   tp::ErrorCode::OtaOffsetErr)
        .value("OtaFlashErr",    tp::ErrorCode::OtaFlashErr)
        .value("OtaVerifyFail",  tp::ErrorCode::OtaVerifyFail)
        .value("OtaSizeExceed",  tp::ErrorCode::OtaSizeExceed);

    // Module-level frame constants
    m.attr("FRAME_HEAD")      = py::int_(tb::FRAME_HEAD);
    m.attr("FRAME_TAIL")      = py::int_(tb::FRAME_TAIL);
    m.attr("FRAME_ESCAPE")    = py::int_(tb::FRAME_ESCAPE);
    m.attr("MIN_FRAME_LEN")   = py::int_(tb::MIN_FRAME_LEN);
    m.attr("MAX_PAYLOAD_LEN") = py::int_(tb::MAX_PAYLOAD_LEN);
    m.attr("MAX_FRAME_LEN")   = py::int_(tb::MAX_FRAME_LEN);

    // ---- Frame -----------------------------------------------------------
    py::class_<tb::Frame>(m, "Frame")
        .def(py::init<>())
        .def_readwrite("addr", &tb::Frame::addr)
        .def_readwrite("seq",  &tb::Frame::seq)
        .def_readwrite("type", &tb::Frame::type)
        .def_readwrite("cmd",  &tb::Frame::cmd)
        .def_property(
            "payload",
            [](const tb::Frame& f) { return vec_to_bytes(f.payload); },
            [](tb::Frame& f, py::bytes b) { f.payload = bytes_to_vec(b); })
        .def("__repr__", [](const tb::Frame& f) {
            return std::string("Frame(addr=") + tp::to_string(f.addr) +
                   ", seq=" + std::to_string(f.seq) +
                   ", type=" + tp::to_string(f.type) +
                   ", cmd="  + tp::to_string(f.cmd) +
                   ", payload=" + std::to_string(f.payload.size()) + "B)";
        });

    // ---- Pure functions --------------------------------------------------
    m.def("crc16_modbus", [](py::bytes b) {
        char* buf = nullptr;
        py::ssize_t len = 0;
        PYBIND11_BYTES_AS_STRING_AND_SIZE(b.ptr(), &buf, &len);
        return tb::crc16_modbus(reinterpret_cast<const uint8_t*>(buf), len);
    }, py::arg("data"));

    m.def("pack_frame",
          [](tp::Address addr, uint8_t seq, tp::FrameType type, tp::Cmd cmd,
             py::bytes payload) {
              auto v = bytes_to_vec(payload);
              auto wire = tb::pack_frame(addr, seq, type, cmd, v);
              return vec_to_bytes(wire);
          },
          py::arg("addr"), py::arg("seq"), py::arg("type"), py::arg("cmd"),
          py::arg("payload") = py::bytes(""));

    m.def("stuff_data", [](py::bytes b) {
        auto v = bytes_to_vec(b);
        return vec_to_bytes(tb::stuff_data(v.data(), v.size()));
    }, py::arg("data"));

    m.def("unstuff_data", [](py::bytes b) {
        auto v = bytes_to_vec(b);
        return vec_to_bytes(tb::unstuff_data(v.data(), v.size()));
    }, py::arg("data"));

    // ---- FrameParser -----------------------------------------------------
    py::class_<tb::FrameParser>(m, "FrameParser")
        .def(py::init<std::size_t>(), py::arg("max_buffered") = 64u * 1024u)
        .def("feed", [](tb::FrameParser& p, py::bytes b) {
            auto v = bytes_to_vec(b);
            p.feed(v.data(), v.size());
        }, py::arg("data"))
        .def("pop", [](tb::FrameParser& p) -> py::object {
            tb::Frame f;
            if (!p.try_pop(f)) return py::none();
            return py::cast(std::move(f));
        })
        .def("pending",        &tb::FrameParser::pending)
        .def("buffered_bytes", &tb::FrameParser::buffered_bytes)
        .def("reset",          &tb::FrameParser::reset);

    // ---- SerialBus -------------------------------------------------------
    py::class_<tb::SerialBus>(m, "SerialBus")
        .def(py::init([](const std::string& dev, uint32_t br,
                         unsigned rt_ms, unsigned wt_ms) {
                return std::make_unique<tb::SerialBus>(
                    tb::SerialBus::Config{dev, br, rt_ms, wt_ms});
             }),
             py::arg("device"),
             py::arg("baudrate")         = 3'000'000u,
             py::arg("read_timeout_ms")  = 1u,
             py::arg("write_timeout_ms") = 1000u)
        .def("read", [](tb::SerialBus& s, std::size_t max) {
            auto v = s.read(max);
            return vec_to_bytes(v);
        }, py::arg("max"))
        .def("write", [](tb::SerialBus& s, py::bytes b) {
            auto v = bytes_to_vec(b);
            s.write(v);
        }, py::arg("data"))
        .def("flush_input",  &tb::SerialBus::flush_input)
        .def("flush_output", &tb::SerialBus::flush_output)
        .def_property_readonly("is_open", &tb::SerialBus::is_open)
        .def_property_readonly("device", [](const tb::SerialBus& s) {
            return s.config().device;
        })
        .def_property_readonly("baudrate", [](const tb::SerialBus& s) {
            return s.config().baudrate;
        })
        .def_static("list_ports", &tb::SerialBus::list_ports);

    // ---- AckResponse + Transport ----------------------------------------
    py::class_<tb::AckResponse>(m, "AckResponse")
        .def_readonly("seq",        &tb::AckResponse::seq)
        .def_readonly("cmd",        &tb::AckResponse::cmd)
        .def_readonly("is_nack",    &tb::AckResponse::is_nack)
        .def_readonly("error_code", &tb::AckResponse::error_code)
        .def_property_readonly(
            "data",
            [](const tb::AckResponse& a) { return vec_to_bytes(a.data); })
        // Backwards-compat alias: `payload` used to exist.
        .def_property_readonly(
            "payload",
            [](const tb::AckResponse& a) { return vec_to_bytes(a.data); })
        .def("__repr__", [](const tb::AckResponse& a) {
            return std::string("AckResponse(seq=") + std::to_string(a.seq) +
                   ", cmd=" + (a.is_nack ? "NACK" : tp::to_string(a.cmd)) +
                   ", error=" + tp::to_string(a.error_code) +
                   ", data=" + std::to_string(a.data.size()) + "B)";
        });

    py::class_<tb::Transport::Stats>(m, "TransportStats")
        .def_readonly("bytes_read",          &tb::Transport::Stats::bytes_read)
        .def_readonly("bytes_written",       &tb::Transport::Stats::bytes_written)
        .def_readonly("frames_received",     &tb::Transport::Stats::frames_received)
        .def_readonly("frames_sent",         &tb::Transport::Stats::frames_sent)
        .def_readonly("ack_timeouts",        &tb::Transport::Stats::ack_timeouts)
        .def_readonly("retries",             &tb::Transport::Stats::retries)
        .def_readonly("unexpected_frames",   &tb::Transport::Stats::unexpected_frames)
        .def_readonly("callback_exceptions", &tb::Transport::Stats::callback_exceptions)
        // ---- Loss accounting ------------------------------------------
        // Read together these localise a rate drop:
        //   crc_errors / resync_bytes rising -> bytes lost before the parser
        //       (the host stopped draining the tty long enough to overflow
        //       the kernel/USB buffers). Check callback_max_us for the cause.
        //   queue_dropped rising             -> bytes were fine, the
        //       subscriber could not keep up and stale frames were evicted.
        //   both flat, rate still low        -> the firmware really is
        //       sending slower; nothing host-side to fix.
        .def_readonly("crc_errors",            &tb::Transport::Stats::crc_errors)
        .def_readonly("resync_bytes",          &tb::Transport::Stats::resync_bytes)
        .def_readonly("parser_overflow_bytes", &tb::Transport::Stats::parser_overflow_bytes)
        .def_readonly("queue_dropped",         &tb::Transport::Stats::queue_dropped)
        .def_readonly("queue_high_water",      &tb::Transport::Stats::queue_high_water)
        .def_readonly("callback_max_us",       &tb::Transport::Stats::callback_max_us)
        .def("__repr__", [](const tb::Transport::Stats& s) {
            char buf[320];
            std::snprintf(buf, sizeof(buf),
                "TransportStats(frames_rx=%llu, crc_err=%llu, resync_B=%llu, "
                "queue_dropped=%llu, queue_hw=%llu, cb_max=%.1fms, "
                "ack_timeouts=%llu, retries=%llu)",
                (unsigned long long)s.frames_received,
                (unsigned long long)s.crc_errors,
                (unsigned long long)s.resync_bytes,
                (unsigned long long)s.queue_dropped,
                (unsigned long long)s.queue_high_water,
                s.callback_max_us / 1000.0,
                (unsigned long long)s.ack_timeouts,
                (unsigned long long)s.retries);
            return std::string(buf);
        });

    py::class_<tb::Transport>(m, "Transport")
        .def(py::init([](const std::string& dev,
                         uint32_t baud,
                         tp::Address peer,
                         unsigned ack_timeout_ms,
                         unsigned max_retries,
                         unsigned retry_interval_ms,
                         unsigned read_timeout_ms,
                         unsigned write_timeout_ms,
                         std::size_t rx_chunk_bytes,
                         std::size_t parser_max_buf,
                         std::size_t dispatch_queue_frames) {
                tb::Transport::Config cfg;
                cfg.serial.device           = dev;
                cfg.serial.baudrate         = baud;
                cfg.serial.read_timeout_ms  = read_timeout_ms;
                cfg.serial.write_timeout_ms = write_timeout_ms;
                cfg.peer                    = peer;
                cfg.ack_timeout             = std::chrono::milliseconds(ack_timeout_ms);
                cfg.max_retries             = max_retries;
                cfg.retry_interval          = std::chrono::milliseconds(retry_interval_ms);
                cfg.rx_chunk_bytes          = rx_chunk_bytes;
                cfg.parser_max_buf          = parser_max_buf;
                cfg.dispatch_queue_frames   = dispatch_queue_frames;
                // Release the GIL while opening the serial port (can be slow).
                py::gil_scoped_release gil;
                return std::make_unique<tb::Transport>(cfg);
             }),
             py::arg("device"),
             py::arg("baudrate")          = 3'000'000u,
             py::arg("peer")              = tp::Address::MCU,
             py::arg("ack_timeout_ms")    = 10u,
             py::arg("max_retries")       = 3u,
             py::arg("retry_interval_ms") = 10u,
             py::arg("read_timeout_ms")   = 1u,
             py::arg("write_timeout_ms")  = 1000u,
             py::arg("rx_chunk_bytes")    = std::size_t{4096},
             py::arg("parser_max_buf")    = std::size_t{64u * 1024u},
             // Depth of the reader -> callback hand-off queue. It absorbs
             // bursts; it is not a backlog. Lower it when staleness under a
             // persistently slow subscriber matters more than surviving a
             // scheduler hiccup.
             py::arg("dispatch_queue_frames") = std::size_t{256})
        .def("send_cmd",
             [](tb::Transport& t, tp::Cmd cmd, py::bytes payload,
                unsigned timeout_ms) {
                 auto v = bytes_to_vec(payload);
                 // Block on serial without holding the GIL.
                 py::gil_scoped_release gil;
                 return t.send_cmd(cmd, v, std::chrono::milliseconds(timeout_ms));
             },
             py::arg("cmd"),
             py::arg("payload")    = py::bytes(""),
             py::arg("timeout_ms") = 0u)
        .def("send_cmd_no_ack",
             [](tb::Transport& t, tp::Cmd cmd, py::bytes payload) {
                 auto v = bytes_to_vec(payload);
                 py::gil_scoped_release gil;
                 t.send_cmd_no_ack(cmd, v);
             },
             py::arg("cmd"), py::arg("payload") = py::bytes(""))
        .def("subscribe",
             [](tb::Transport& t, tp::Cmd cmd, py::function pycb) {
                 // make_gil_safe_callback, not a bare make_shared: the last
                 // reference dies wherever the owning lambda is destroyed —
                 // the dispatcher thread, or stop()/clear_subs_() on a caller
                 // that has released the GIL. Decref'ing a Python object
                 // there segfaults, which is exactly what a bare make_shared
                 // did here on every stop() with a live subscription.
                 auto cb = make_gil_safe_callback(std::move(pycb));
                 return t.subscribe(cmd, [cb](const tb::Frame& f) {
                     call_into_python("xense.taccap.Transport callback",
                                      [&] { (*cb)(f); });
                 });
             },
             py::arg("cmd"), py::arg("callback"))
        .def("unsubscribe", &tb::Transport::unsubscribe, py::arg("subscription_id"))
        .def("stop",
             [](tb::Transport& t) {
                 py::gil_scoped_release gil;
                 t.stop();
             })
        .def_property_readonly("is_running", &tb::Transport::is_running)
        .def_property_readonly("stats",      &tb::Transport::stats)
        .def_property_readonly("device", [](const tb::Transport& t) {
            return t.config().serial.device;
        })
        .def_property_readonly("baudrate", [](const tb::Transport& t) {
            return t.config().serial.baudrate;
        })
        .def("__enter__", [](tb::Transport& t) -> tb::Transport& { return t; })
        .def("__exit__",
             [](tb::Transport& t, py::object, py::object, py::object) {
                 py::gil_scoped_release gil;
                 t.stop();
             });

    // spdlog-backed logger shared with the C++ core.
    xense::taccap::python::bind_log(m);

    // Component classes (IMU/Encoder/Camera/LeaderGripper/FollowerGripper).
    xense::taccap::python::bind_components(m);
}

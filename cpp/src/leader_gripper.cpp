// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/leader_gripper.hpp>
#include "wrist_fisheye.hpp"
#include "stream_rate.hpp"
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <optional>

namespace xense::taccap {

namespace {

bus::Transport::Config make_transport_config(const LeaderGripper::Config& cfg) {
    bus::Transport::Config out;
    out.serial.device           = cfg.mcu_device;
    out.serial.baudrate         = cfg.baudrate;
    out.serial.read_timeout_ms  = 1;
    out.serial.write_timeout_ms = 1000;
    out.peer                    = protocol::Address::MCU;
    out.ack_timeout             = std::chrono::milliseconds(cfg.ack_timeout_ms);
    out.max_retries             = cfg.max_retries;
    return out;
}

Camera::Config make_wrist_config(const LeaderGripper::Config& cfg) {
    Camera::Config c = cfg.wrist_cam_extra;
    c.device     = cfg.wrist_video;
    // Config::wrist_color_mode is the wrist-specific default (RGB); it wins over
    // wrist_cam_extra's generic Camera default (BGR), which callers reaching for
    // wrist_cam_extra are not setting deliberately.
    c.color_mode = cfg.wrist_color_mode;
    if (c.width  <= 0) c.width  = 640;
    if (c.height <= 0) c.height = 480;
    if (c.fps    <= 0) c.fps    = 30.0;
    return c;
}

}  // namespace

LeaderGripper::LeaderGripper(const Config& cfg)
    : cfg_(cfg),
      t_(make_transport_config(cfg)),
      imu_(t_),
      encoder_(t_),
      key_(t_),
      led_(t_),
      errors_(t_),
      diag_(t_),
      cal_(t_),
      dev_(t_),
      ota_(t_) {
    // Read firmware version + SN once at construction time so the log
    // shows what the host is actually talking to. A best-effort
    // StopStream first drains any leftover DATA backlog from a prior
    // host process so the GetVersion ACK doesn't get buried — same
    // pattern as discovery::scan_all() and start_streaming().
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    std::string fw_version_str = "<unknown>";
    std::string fw_sn_str      = "<unknown>";
    try {
        auto ack = t_.send_cmd(protocol::Cmd::GetVersion, {},
                               std::chrono::milliseconds(500));
        if (!ack.is_nack &&
            ack.data.size() == sizeof(protocol::FirmwareVersion)) {
            auto v = protocol::decode_version(ack.data.data(),
                                              ack.data.size());
            fw_version_    = v;
            fw_version_str = protocol::version_string(v);
        }
    } catch (...) {
        // Leave fw_version_str as "<unknown>"; we still want construction
        // to succeed even if version probe times out.
    }
    try {
        auto ack = t_.send_cmd(protocol::Cmd::GetSn, {},
                               std::chrono::milliseconds(500));
        if (!ack.is_nack && !ack.data.empty()) {
            fw_sn_    = protocol::decode_sn(ack.data.data(),
                                            ack.data.size());
            fw_sn_str = fw_sn_;
        }
    } catch (...) { /* fall through with "<unknown>" */ }

    logger()->info(
        "LeaderGripper opened: device={} firmware={} sn={} open_cameras={}",
        cfg_.mcu_device, fw_version_str, fw_sn_str, cfg_.open_cameras);

    // The wrist camera is off by default — an external camera service owns the
    // wrist UVC V4L2 device. Only open it when explicitly asked AND a device
    // path is provided.
    if (cfg_.open_cameras) {
        if (!cfg_.wrist_video.empty()) {
            wrist_ = std::make_unique<Camera>(make_wrist_config(cfg_));
            if (cfg_.undistort_wrist) {
                detail::install_wrist_undistorter(cal_, *wrist_,
                                                  cfg_.fisheye_balance,
                                                  "LeaderGripper");
            }
        } else if (cfg_.undistort_wrist) {
            logger()->warn("LeaderGripper: undistort_wrist=true but "
                           "wrist_video is empty, so no camera was opened");
        }
    }

    // Opt-in normalized encoder position. Resolve the travel span now so a
    // misconfiguration surfaces at open() time rather than as silently-absent
    // .position fields once a 100 Hz stream is already running.
    if (cfg_.normalize_position) {
        ensure_position_map_();
        encoder_.set_position_map(pos_map_);
        logger()->info(
            "LeaderGripper: encoder position normalization on, "
            "max_open={:.4f} rad (source: {})",
            pos_map_.max_open_rad(),
            cfg_.encoder_max_rad > 0.0f ? "config override" : "firmware EncoderMaxCal");
    }
}

LeaderGripper::~LeaderGripper() {
    try { stop_streaming(); } catch (...) {}
}

std::unique_ptr<LeaderGripper> LeaderGripper::open() {
    // Discovery is MCU-only; cameras are owned externally and stay off
    // (open_cameras defaults to false). A caller that still wants this
    // gripper to drive the cameras must construct it explicitly with
    // open_cameras=true and the device paths/serials.
    auto eps = discovery::find_one();
    Config cfg{};
    cfg.mcu_device = eps.mcu_device;
    return std::make_unique<LeaderGripper>(cfg);
}

void LeaderGripper::start_streaming(unsigned imu_hz, unsigned encoder_hz) {
    if (streaming_) return;

    // Best-effort reset of the firmware streaming state. If a previous host
    // process exited without calling stop_streaming() the MCU keeps
    // pushing DATA frames; sending StopStream first drains that backlog
    // and gets the firmware into a quiescent state before we configure
    // the new run. We swallow timeouts/NACKs here because "already
    // stopped" is the desired outcome either way.
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    // A rate of 0 means "off", and the only way to say that to the firmware
    // is to leave the source's mask bit clear — the rate field alone would
    // fall through to the firmware's 100 Hz default. See stream_rate.hpp.
    const uint16_t mask = detail::stream_source_mask(imu_hz, encoder_hz, 0);
    if (mask == 0) {
        throw IoError("LeaderGripper::start_streaming: both rates are 0, so "
                      "nothing would be streamed — pass a non-zero rate for at "
                      "least one source, or just don't start the stream",
                      EINVAL);
    }
    detail::warn_if_rate_adjusted("LeaderGripper", "IMU", imu_hz, 0);
    detail::warn_if_rate_adjusted("LeaderGripper", "encoder", encoder_hz, 0);

    // Build StreamConfig (12 bytes — see protocol::StreamConfig).
    protocol::StreamConfig sc{};
    sc.source_mask  = mask;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.imu_rate     = static_cast<uint16_t>(imu_hz);
    sc.encoder_rate = static_cast<uint16_t>(encoder_hz);
    sc.eskin_rate   = 0;
    sc.motor_rate   = 0;
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    auto wire = protocol::encode(sc);
    auto ack = t_.send_cmd(protocol::Cmd::StartStream, wire);
    if (ack.is_nack) {
        throw ProtocolError(std::string("LeaderGripper::start_streaming NACK: ") +
                            protocol::to_string(ack.error_code));
    }

    // The MCU now begins emitting DATA frames; subscribers on imu_/encoder_
    // start receiving immediately.

    streaming_ = true;
}

void LeaderGripper::stop_streaming() {
    if (!streaming_) return;
    streaming_ = false;
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {}, std::chrono::milliseconds{500});
    } catch (...) {
        // Best-effort: even if the MCU doesn't ACK we proceed; tearing down
        // the host-side resources is more important than a clean fw stop.
    }
}

// ---- Normalized gripper position (0 = closed, 1 = open) --------------------

void LeaderGripper::ensure_position_map_() {
    if (pos_map_loaded_) return;

    // Explicit override wins and skips the firmware round-trip entirely — it
    // is the escape hatch for firmware older than V2.1, which doesn't know
    // Cmd::EncoderMaxCal at all.
    if (cfg_.encoder_max_rad > 0.0f) {
        pos_map_ = GripperPosition::from_travel(cfg_.encoder_max_rad);
        pos_map_loaded_ = true;
        return;
    }

    std::optional<float> max_rad;
    try {
        max_rad = cal_.read_encoder_max_rad();
    } catch (const ProtocolError& e) {
        // InvalidCmd here means pre-V2.1 firmware (or follower hardware);
        // either way the caller needs Config::encoder_max_rad.
        throw ProtocolError(
            std::string("LeaderGripper: cannot read the encoder max travel "
                        "angle (Cmd::EncoderMaxCal 0x2C, firmware >= V2.1) — ") +
            e.what() +
            ". Set Config::encoder_max_rad to supply the span from the host "
            "instead.");
    }
    if (!max_rad) {
        throw ProtocolError(
            "LeaderGripper: the encoder max travel angle has never been "
            "calibrated (firmware returned CalNotSet) — normalized position is "
            "unavailable. Hold the gripper fully closed and call "
            "encoder().set_zero(), then open it fully and write the observed "
            "position_rad via calibration().write_encoder_max_rad(). Or set "
            "Config::encoder_max_rad to supply the span from the host.");
    }

    auto m = GripperPosition::from_travel(*max_rad);
    if (!m.valid()) {
        throw ProtocolError(
            "LeaderGripper: firmware reported a non-positive encoder max "
            "travel angle (" + std::to_string(*max_rad) + " rad)");
    }
    pos_map_ = m;
    pos_map_loaded_ = true;
}

void LeaderGripper::reload_position_map() {
    pos_map_loaded_ = false;
    ensure_position_map_();
    // Keep a normalization-enabled Encoder in sync with the reloaded span,
    // otherwise streamed .position values would keep using the stale one.
    if (cfg_.normalize_position) {
        encoder_.set_position_map(pos_map_);
    }
}

const GripperPosition& LeaderGripper::position_map() {
    ensure_position_map_();
    return pos_map_;
}

float LeaderGripper::pos_to_rad(float position) {
    ensure_position_map_();
    return pos_map_.to_rad(position);
}

float LeaderGripper::rad_to_pos(float raw_rad) {
    ensure_position_map_();
    return pos_map_.to_position(raw_rad);
}

float LeaderGripper::position(std::chrono::milliseconds timeout) {
    ensure_position_map_();
    return pos_map_.to_position(encoder_.read_once(timeout).position_rad);
}

}  // namespace xense::taccap

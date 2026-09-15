// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/follower_gripper.hpp>
#include "wrist_fisheye.hpp"
#include "stream_rate.hpp"
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace xense::taccap {

namespace {

bus::Transport::Config make_transport_config(const FollowerGripper::Config& cfg) {
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

Camera::Config make_wrist_config(const FollowerGripper::Config& cfg) {
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

FollowerGripper::FollowerGripper(const Config& cfg)
    : cfg_(cfg),
      t_(make_transport_config(cfg)),
      imu_(t_),
      encoder_(t_),
      motor_(t_),
      key_(t_),
      led_(t_),
      errors_(t_),
      diag_(t_),
      cal_(t_),
      ota_(t_) {
    // Mirror LeaderGripper: drain leftover DATA, then probe firmware
    // version + SN once at construction time so the log shows what the
    // host is talking to.
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* fw already idle */ }

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
    } catch (...) {}
    try {
        auto ack = t_.send_cmd(protocol::Cmd::GetSn, {},
                               std::chrono::milliseconds(500));
        if (!ack.is_nack && !ack.data.empty()) {
            fw_sn_str = protocol::decode_sn(ack.data.data(),
                                            ack.data.size());
        }
    } catch (...) {}

    logger()->info(
        "FollowerGripper opened: device={} firmware={} sn={} open_cameras={}",
        cfg_.mcu_device, fw_version_str, fw_sn_str, cfg_.open_cameras);

    // ---- Firmware gate -----------------------------------------------------
    // 1.1.6 is a mandatory upgrade, not a recommendation: everything older has
    // no stall protection whatsoever on the MIT command path, so a blocked jaw
    // is bounded only by the motor's 0x700B ceiling. Refusing here is the point
    // -- a caller that silently ran on 1.1.5 would be one blocked grasp away
    // from browning out the board and dropping its payload.
    if (!cfg_.allow_outdated_firmware) {
        constexpr uint32_t need = (uint32_t(kMinFirmwareMajor) << 16) |
                                  (uint32_t(kMinFirmwareMinor) << 8) |
                                  kMinFirmwarePatch;
        const bool known = fw_version_.has_value();
        const uint32_t have =
            known ? ((uint32_t(fw_version_->major) << 16) |
                     (uint32_t(fw_version_->minor) << 8) | fw_version_->patch)
                  : 0u;
        if (known && have < need) {
            const std::string msg =
                "\n"
                "==========================================================\n"
                "  从爪固件版本过低,必须升级后才能使用本 SDK\n"
                "  Follower firmware too old -- upgrade required\n"
                "----------------------------------------------------------\n"
                "  当前 / current : " + fw_version_str + "\n"
                "  需要 / required: 1.1.6 或更高 / or newer\n"
                "  设备 / device  : " + cfg_.mcu_device + "\n"
                "  SN             : " + fw_sn_str + "\n"
                "----------------------------------------------------------\n"
                "  1.1.6 引入运动安全包络。在此之前的固件,MIT 命令路径上\n"
                "  没有任何堵转保护 —— 夹爪被挡住时力矩只受电机 0x700B 限制,\n"
                "  24V 供电下实测会拉垮母线、松手掉件、整机掉电重启。\n"
                "\n"
                "  Versions before 1.1.6 have no stall protection at all on\n"
                "  the MIT path. A blocked jaw browns out the board on 24 V.\n"
                "----------------------------------------------------------\n"
                "  升级 / upgrade:\n"
                "    python python/examples/ota_update.py slave\n"
                "\n"
                "  (role 选择器,自动找从爪并挑对应镜像;插了多台时也可用\n"
                "   SN 精确指定: ota_update.py firmware/tc-gu-01-slave.bin "
                + fw_sn_str + ")\n"
                "  刷完后必须断电重启 / power-cycle after flashing\n"
                "==========================================================";
            logger()->error(msg);
            throw ProtocolError(msg);
        }
        if (!known) {
            logger()->warn(
                "FollowerGripper: 读不到固件版本,无法确认是否 >= 1.1.6。"
                "若设备固件低于 1.1.6,MIT 路径上没有任何堵转保护。"
                " (could not read firmware version; 1.1.6 or newer is required)");
        }
    }

    // The wrist camera is off by default — an external camera service owns the
    // wrist UVC V4L2 device. Only open it when explicitly asked AND a device
    // path is provided.
    if (cfg_.open_cameras) {
        if (!cfg_.wrist_video.empty()) {
            wrist_ = std::make_unique<Camera>(make_wrist_config(cfg_));
            if (cfg_.undistort_wrist) {
                detail::install_wrist_undistorter(cal_, *wrist_,
                                                  cfg_.fisheye_balance,
                                                  "FollowerGripper");
            }
        } else if (cfg_.undistort_wrist) {
            logger()->warn("FollowerGripper: undistort_wrist=true but "
                           "wrist_video is empty, so no camera was opened");
        }
    }
}

FollowerGripper::~FollowerGripper() {
    try { stop_streaming(); } catch (...) {}
}

std::unique_ptr<FollowerGripper> FollowerGripper::open() {
    // Discovery is MCU-only; cameras are owned externally and stay off
    // (open_cameras defaults to false). A caller that still wants this
    // gripper to drive the cameras must construct it explicitly with
    // open_cameras=true and the device paths/serials.
    auto eps = discovery::find_one();
    Config cfg{};
    cfg.mcu_device = eps.mcu_device;
    return std::make_unique<FollowerGripper>(cfg);
}

void FollowerGripper::start_streaming(unsigned motor_hz) {
    if (streaming_) return;

    // Motor status is the only source the follower firmware emits; everything
    // else in its stream task is compiled out on this role. So there is exactly
    // one rate to honour, and a rate of 0 has nowhere useful to go.
    if (motor_hz == 0) {
        throw IoError("FollowerGripper::start_streaming: motor_hz is 0, and "
                      "motor status is the only source a follower streams — "
                      "pass a non-zero rate, or just don't start the stream",
                      EINVAL);
    }

    // Drain the firmware queue from any previous host process — same
    // rationale as LeaderGripper::start_streaming.
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    detail::warn_if_rate_adjusted("FollowerGripper", "motor status", motor_hz,
                                  detail::kMotorMaxRateHz);

    protocol::StreamConfig sc{};
    // The MotorStatus bit enables the source; a rate alone does not. A
    // set bit with rate 0 would stream at the firmware's 100Hz default rather
    // than staying off. See stream_rate.hpp.
    sc.source_mask  = protocol::StreamSrc::MotorStatus;
    const auto version = firmware_version();
    if (version && (version->major > 1 || (version->major == 1 &&
        (version->minor > 1 || (version->minor == 1 && version->patch >= 8)))))
        sc.source_mask |= protocol::StreamSrc::MotorExecution;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.imu_rate     = 0;
    sc.encoder_rate = 0;
    sc.eskin_rate   = 0;
    sc.motor_rate   = static_cast<uint16_t>(motor_hz);
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    auto wire = protocol::encode(sc);
    auto ack = t_.send_cmd(protocol::Cmd::StartStream, wire);
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::start_streaming NACK: ") +
                            protocol::to_string(ack.error_code));
    }

    streaming_ = true;
}

void FollowerGripper::stop_streaming() {
    if (!streaming_) return;
    streaming_ = false;
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {}, std::chrono::milliseconds{500});
    } catch (...) {
        // Best-effort.
    }
}

// ---- Follower gripper open/close limit config (Cmd 0x66/0x67) --------------

protocol::GripperConfig FollowerGripper::get_gripper_config(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetGripperConfig, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::get_gripper_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_gripper_config(ack.data.data(), ack.data.size());
}

protocol::GripperEnvelope FollowerGripper::get_envelope(
        std::chrono::milliseconds timeout) {
    const protocol::GripperConfig cfg = get_gripper_config(timeout);
    protocol::GripperEnvelope env{};
    std::memcpy(&env, cfg.reserved, sizeof(env));
    return env;
}

void FollowerGripper::set_envelope(const protocol::GripperEnvelope& env) {
    const auto active = protocol::GripperEnvelopeFlag::Valid | protocol::GripperEnvelopeFlag::Enforce;
    if ((env.flags & active) == active &&
        (!std::isfinite(env.cont_torque_nm) || env.cont_torque_nm <= 0 || env.cont_torque_nm > 1.2f ||
         !std::isfinite(env.peak_torque_nm) || env.peak_torque_nm < env.cont_torque_nm || env.peak_torque_nm > 5.5f))
        throw std::invalid_argument("RS05 envelope requires 0 < continuous <= 1.2 and continuous <= peak <= 5.5 Nm");
    // Read-modify-write: the envelope shares its record with the travel
    // calibration, and the firmware's own sanitiser rebuilds the record from a
    // zeroed default. Writing a config assembled from scratch here would drop
    // max_open_rad / min_open_rad on the floor.
    protocol::GripperConfig cfg = get_gripper_config();
    // Stamp the layout version so firmware built against a different field
    // order refuses the record instead of misreading it.
    protocol::GripperEnvelope stamped = env;
    stamped.flags = static_cast<uint16_t>(
        (stamped.flags & ~protocol::GripperEnvelopeFlag::LayoutMask) |
        protocol::GripperEnvelopeFlag::LayoutBits);
    std::memcpy(cfg.reserved, &stamped, sizeof(stamped));
    set_gripper_config(cfg);
    logger()->info(
        "FollowerGripper envelope written: cont={:.3f}Nm peak={:.3f}Nm "
        "flags=0x{:04x}",
        stamped.cont_torque_nm, stamped.peak_torque_nm, stamped.flags);
}

void FollowerGripper::set_gripper_config(const protocol::GripperConfig& cfg) {
    auto ack = t_.send_cmd(protocol::Cmd::SetGripperConfig, protocol::encode(cfg));
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::set_gripper_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    // Config just changed on the firmware — drop the cache so the next
    // position call rebuilds the converter from the new limits.
    pos_map_loaded_ = false;
}

protocol::GripperAutoCalConfig FollowerGripper::get_auto_cal_config(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetGripperAutoCalConfig, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::get_auto_cal_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_gripper_auto_cal_config(ack.data.data(), ack.data.size());
}

void FollowerGripper::set_auto_cal_config(const protocol::GripperAutoCalConfig& cfg) {
    auto ack = t_.send_cmd(protocol::Cmd::SetGripperAutoCalConfig, protocol::encode(cfg));
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::set_auto_cal_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void FollowerGripper::set_auto_cal_stall_param(
        const protocol::GripperAutoCalStallParam& p) {
    auto ack = t_.send_cmd(protocol::Cmd::SetGripperAutoCalConfig, protocol::encode(p));
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::set_auto_cal_stall_param NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void FollowerGripper::set_auto_cal_stall_param(
        const protocol::GripperAutoCalStallParamEx& p) {
    auto ack = t_.send_cmd(protocol::Cmd::SetGripperAutoCalConfig, protocol::encode(p));
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::set_auto_cal_stall_param NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

// ---- Normalized gripper position (0 = closed, 1 = open) --------------------

void FollowerGripper::ensure_position_map_() {
    if (pos_map_loaded_) return;
    GripperPosition m(get_gripper_config());
    if (!m.valid()) {
        throw ProtocolError(
            "FollowerGripper: gripper config is not calibrated "
            "(GripperConfig flags lack Valid, or max_open_rad <= min_open_rad) "
            "— normalized position is unavailable until the gripper is "
            "calibrated (zero at full close, then write max_open via "
            "set_gripper_config)");
    }
    pos_map_ = m;
    pos_map_loaded_ = true;
}

void FollowerGripper::reload_config() {
    pos_map_loaded_ = false;
    ensure_position_map_();
}

const GripperPosition& FollowerGripper::position_map() {
    ensure_position_map_();
    return pos_map_;
}

float FollowerGripper::pos_to_rad(float position) {
    ensure_position_map_();
    return pos_map_.to_rad(position);
}

float FollowerGripper::rad_to_pos(float raw_rad) {
    ensure_position_map_();
    return pos_map_.to_position(raw_rad);
}

float FollowerGripper::position(std::chrono::milliseconds timeout) {
    ensure_position_map_();
    return pos_map_.to_position(motor_.read_status(timeout).actual_pos);
}

void FollowerGripper::set_position(float position, float kp, float kd,
                                   float feedforward_torque_nm) {
    ensure_position_map_();
    motor_.submit_impedance(pos_map_.to_rad(position), kp, kd,
                            feedforward_torque_nm);
}

}  // namespace xense::taccap

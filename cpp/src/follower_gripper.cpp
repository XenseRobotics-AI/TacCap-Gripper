// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/follower_gripper.hpp>
#include "wrist_fisheye.hpp"
#include "stream_rate.hpp"
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
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
      dev_(t_),
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
            fw_sn_    = protocol::decode_sn(ack.data.data(),
                                            ack.data.size());
            fw_sn_str = fw_sn_;
        }
    } catch (...) {}

    logger()->info(
        "FollowerGripper opened: device={} firmware={} sn={} open_cameras={}",
        cfg_.mcu_device, fw_version_str, fw_sn_str, cfg_.open_cameras);

    // ---- Firmware gate -----------------------------------------------------
    // Two separate reasons stack up to 1.2.5, and both are about the host and
    // the firmware disagreeing rather than about the firmware alone:
    //
    //   < 1.1.6  no stall protection at all on the MIT command path -- a
    //            blocked jaw is bounded only by the motor's 0x700B ceiling,
    //            which on 24 V browns out the board and drops the payload.
    //            This one is a safety floor and always was.
    //
    //   < 1.2.5  auto-calibration wrote the closed zero with an INSET (20 mrad
    //            up to 1.2.3, 5 mrad in 1.2.4), so normalized 0.0 sat short of
    //            the mechanical stop. This SDK's closed-endpoint preload
    //            (ForcePositionConfig::close_preload_nm) is built on 1.2.5's
    //            semantics -- that normalized 0.0 IS the stop -- and on older
    //            firmware it presses the jaw past where the position map says
    //            fully closed is. The force stays bounded (the firmware clamps
    //            the target to the calibrated range and the torque to the
    //            envelope's cont), so this is not the brown-out class of
    //            problem; what breaks is the normalized scale's meaning at the
    //            closed end, and a position that reads 0.0 while the jaw is
    //            20 mrad tighter than 0.0 is supposed to be.
    //
    // Refusing rather than warning is deliberate: a scale that silently means
    // something different is worse to debug than a device that will not open.
    //
    // NOTE the leader has no such gate, ON PURPOSE. ota_update.py opens every
    // gripper -- follower included -- through LeaderGripper, so gating there
    // would block the upgrade path for exactly the devices that need it.
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
                "  需要 / required: 1.2.5 或更高 / or newer\n"
                "  设备 / device  : " + cfg_.mcu_device + "\n"
                "  SN             : " + fw_sn_str + "\n"
                "----------------------------------------------------------\n"
                "  1.1.6 引入运动安全包络:在此之前 MIT 命令路径上没有任何\n"
                "  堵转保护,夹爪被挡住时力矩只受电机 0x700B 限制,24V 下实测\n"
                "  会拉垮母线、松手掉件、整机掉电重启。\n"
                "\n"
                "  1.2.5 把标定写入的闭合零位移到机械止点本身(此前 1.2.3 及\n"
                "  更早内缩 20mrad、1.2.4 内缩 5mrad)。本 SDK 的闭合端预压\n"
                "  (close_preload_nm)是按「归一化 0.0 就是止点」设计的;在更旧\n"
                "  的固件上它会把爪子压过位置映射所说的完全闭合点 —— 力是有界的,\n"
                "  但归一化刻度在闭合端不再表示它字面的意思。\n"
                "\n"
                "  Before 1.1.6: no stall protection on the MIT path at all.\n"
                "  Before 1.2.5: the calibrated closed zero sat short of the\n"
                "  mechanical stop, so this SDK's closed-endpoint preload\n"
                "  presses past what normalized 0.0 claims to mean.\n"
                "----------------------------------------------------------\n"
                "  升级 / upgrade:\n"
                "    python python/examples/ota_update.py slave\n"
                "\n"
                "  (role 选择器,自动找从爪并挑对应镜像;插了多台时也可用\n"
                "   SN 精确指定: ota_update.py slave "
                + fw_sn_str + ")\n"
                "  刷完后 USB 线与电源线必须同时拔下再插回 /\n"
                "  after flashing, unplug USB and power together, then reconnect\n"
                "==========================================================";
            logger()->error(msg);
            throw ProtocolError(msg);
        }
        if (!known) {
            logger()->warn(
                "FollowerGripper: 读不到固件版本,无法确认是否 >= 1.2.5。"
                "低于 1.1.6 时 MIT 路径上没有堵转保护;低于 1.2.5 时闭合端"
                "预压会压过归一化 0.0 所表示的位置。"
                " (could not read firmware version; 1.2.5 or newer is required)");
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
    // Only the MotorStatus bit: a rate alone would not turn a source on, and a
    // set bit with rate 0 would stream at the firmware's 100Hz default rather
    // than staying off. See stream_rate.hpp.
    sc.source_mask  = protocol::StreamSrc::MotorStatus;
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

namespace detail {

namespace {

void note(std::string& detail, const std::string& line) {
    if (!detail.empty()) detail += "; ";
    detail += line;
}

std::string fmt(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(v));
    return buf;
}

}  // namespace

protocol::GripperEnvelope recommend_envelope(
        const std::optional<protocol::MotorSpec>& spec) {
    protocol::GripperEnvelope out{};

    // peak is the ROTATING rating, deliberately not t_max_nm. t_max_nm is the
    // motor's absolute ceiling and is also the firmware's own default for the
    // persisted 0x700B startup limit -- writing it here would make the envelope
    // a no-op relative to a protection that already exists.
    out.peak_torque_nm =
        (spec && spec->rated_torque_nm > 0.0f) ? spec->rated_torque_nm
                                               : MOTOR_RATED_TORQUE_NM;

    // cont is the STALL rating: what a blocked jaw may hold indefinitely, which
    // is the gripper's main duty cycle. min() rather than the bare constant so
    // a small actuator whose rated torque is below it cannot end up with
    // cont >= peak, which switches the firmware's I2t derate off entirely.
    const float fallback_cont =
        std::min(MOTOR_STALL_CONT_TORQUE_NM, out.peak_torque_nm);
    out.cont_torque_nm =
        (spec && spec->stall_cont_torque_nm > 0.0f) ? spec->stall_cont_torque_nm
                                                    : fallback_cont;

    // Left at 0 on purpose: 0 means "firmware default" (90/100), and that pair
    // is a product decision that lives in the firmware. Writing the numbers
    // here would pin them and diverge silently if the firmware ever moves them.
    out.temp_derate_start_c = 0;
    out.temp_wall_c         = 0;

    out.flags = static_cast<uint16_t>(protocol::GripperEnvelopeFlag::Valid |
                                      protocol::GripperEnvelopeFlag::Enforce |
                                      protocol::GripperEnvelopeFlag::LayoutBits);
    return out;
}

EnvelopeAudit audit_envelope(const protocol::GripperEnvelope& stored,
                             const std::optional<protocol::MotorSpec>& spec) {
    EnvelopeAudit a;
    a.stored      = stored;
    a.recommended = recommend_envelope(spec);

    if (spec) {
        a.motor_model      = std::string(
            spec->name, ::strnlen(spec->name, sizeof(spec->name)));
        a.peak_from_device = spec->rated_torque_nm > 0.0f;
        a.cont_from_device = spec->stall_cont_torque_nm > 0.0f;
        if (!a.cont_from_device && a.peak_from_device) {
            note(a.detail,
                 "cont " + fmt(a.recommended.cont_torque_nm) +
                     " Nm is this SDK's fallback -- the firmware's table has no "
                     "stall rating for " + a.motor_model);
        }
    } else {
        note(a.detail,
             "motor spec unavailable; both values are compiled-in fallbacks");
    }

    // ---- Gating, first match wins ----
    const bool valid =
        (stored.flags & protocol::GripperEnvelopeFlag::Valid) != 0;
    const bool enforce =
        (stored.flags & protocol::GripperEnvelopeFlag::Enforce) != 0;
    const bool layout_ok =
        protocol::GripperEnvelopeFlag::layout_of(stored.flags) ==
        protocol::GripperEnvelopeFlag::LayoutVersion;

    if (!valid) {
        a.issues |= GripperEnvelopeIssue::NotWritten;
        note(a.detail,
             "no envelope has ever been written -- the firmware clamps nothing "
             "on the MIT path: no position-error clamp, no I2t derate, no "
             "temperature wall");
        return a;
    }
    if (!enforce) {
        a.issues |= GripperEnvelopeIssue::NotEnforced;
        note(a.detail, "envelope stored but Enforce is clear, so it is ignored");
        return a;
    }
    if (!layout_ok) {
        a.issues |= GripperEnvelopeIssue::LayoutMismatch;
        note(a.detail,
             "record layout v" +
                 std::to_string(
                     protocol::GripperEnvelopeFlag::layout_of(stored.flags)) +
                 " but this SDK speaks v" +
                 std::to_string(protocol::GripperEnvelopeFlag::LayoutVersion) +
                 "; the firmware refuses it and the field bytes cannot be "
                 "trusted either way");
        return a;
    }

    // ---- Values. The layout is current, so the fields mean what they say. ----
    protocol::GripperEnvelope eff = stored;

    if (stored.peak_torque_nm <= 0.0f) {
        a.issues |= GripperEnvelopeIssue::PeakUnlimited;
        note(a.detail,
             "peak is 0 (unlimited): kp*error is NOT bounded even though the "
             "record reads as enforced");
    }
    if (stored.cont_torque_nm <= 0.0f) {
        a.issues |= GripperEnvelopeIssue::ContUnlimited;
        note(a.detail, "cont is 0 (unlimited): no sustained-torque ceiling");
    }
    if (spec && spec->stall_cont_torque_nm > 0.0f &&
        stored.cont_torque_nm > spec->stall_cont_torque_nm) {
        a.issues |= GripperEnvelopeIssue::ContAboveStallRating;
        eff.cont_torque_nm = spec->stall_cont_torque_nm;
        note(a.detail,
             "stored cont " + fmt(stored.cont_torque_nm) +
                 " Nm is above the " + a.motor_model + " stall rating; the "
                 "firmware enforces " + fmt(spec->stall_cont_torque_nm) +
                 " Nm and logs that on a UART not wired to USB");
    }
    if (a.cont_from_device && a.peak_from_device) {
        constexpr float kTol = 1e-3f;
        const bool cont_off =
            std::abs(stored.cont_torque_nm - a.recommended.cont_torque_nm) > kTol;
        const bool peak_off =
            std::abs(stored.peak_torque_nm - a.recommended.peak_torque_nm) > kTol;
        if (cont_off || peak_off) {
            a.issues |= GripperEnvelopeIssue::NotAtSpec;
            note(a.detail,
                 "stored cont/peak " + fmt(stored.cont_torque_nm) + "/" +
                     fmt(stored.peak_torque_nm) + " Nm, the " + a.motor_model +
                     " is rated " + fmt(a.recommended.cont_torque_nm) + "/" +
                     fmt(a.recommended.peak_torque_nm) +
                     " Nm (stall/rotating) -- a lower cont holds the grip "
                     "below what the motor sustains indefinitely");
        }
    }
    if (stored.cont_torque_nm > 0.0f &&
        stored.peak_torque_nm <= stored.cont_torque_nm) {
        a.issues |= GripperEnvelopeIssue::PeakNotAboveCont;
        note(a.detail,
             "peak is not above cont, so the I2t derate never engages; only "
             "peak and the temperature wall apply (tighter, not a hole)");
    }

    a.effective = eff;
    return a;
}

protocol::GripperEnvelope repair_envelope(const EnvelopeAudit& audit) {
    const auto& rec    = audit.recommended;
    const auto& stored = audit.stored;

    protocol::GripperEnvelope out = rec;

    const bool untrustworthy =
        (audit.issues & (GripperEnvelopeIssue::NotWritten |
                         GripperEnvelopeIssue::LayoutMismatch)) != 0;
    // With both ratings read from the device the recommendation IS the spec,
    // and cont/peak follow it exactly -- see GripperEnvelopeIssue::NotAtSpec.
    const bool spec_known = audit.cont_from_device && audit.peak_from_device;
    if (!untrustworthy) {
        // Temperatures carry over verbatim, 0 included -- 0 is "firmware
        // default", a valid choice, not a missing value.
        out.temp_derate_start_c = stored.temp_derate_start_c;
        out.temp_wall_c         = stored.temp_wall_c;
    }
    if (!untrustworthy && !spec_known) {
        // The recommendation is this SDK's compiled-in guess, so a stored value
        // that is already STRICTER than it is kept rather than raised on the
        // strength of a number nobody measured on this motor.
        if (stored.cont_torque_nm > 0.0f &&
            stored.cont_torque_nm <= rec.cont_torque_nm) {
            out.cont_torque_nm = stored.cont_torque_nm;
        }
        if (stored.peak_torque_nm > 0.0f &&
            stored.peak_torque_nm <= rec.peak_torque_nm) {
            out.peak_torque_nm = stored.peak_torque_nm;
        }

        // A carried-over peak must not collapse the I2t band.
        if (out.peak_torque_nm <= out.cont_torque_nm) {
            out.peak_torque_nm = rec.peak_torque_nm;
        }
    }

    out.flags = static_cast<uint16_t>(protocol::GripperEnvelopeFlag::Valid |
                                      protocol::GripperEnvelopeFlag::Enforce |
                                      protocol::GripperEnvelopeFlag::LayoutBits);
    return out;
}

}  // namespace detail

protocol::HomeDiagReport FollowerGripper::home_diag(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetHomeDiag, {}, timeout);
    if (ack.data.size() < sizeof(protocol::HomeDiagReport)) {
        throw ProtocolError("FollowerGripper::home_diag: short payload (" +
                            std::to_string(ack.data.size()) + " bytes) -- "
                            "follower firmware older than 1.2.0 has no 0x57");
    }
    protocol::HomeDiagReport out{};
    std::memcpy(&out, ack.data.data(), sizeof(out));
    return out;
}

protocol::GripperEnvelope FollowerGripper::get_envelope(
        std::chrono::milliseconds timeout) {
    const protocol::GripperConfig cfg = get_gripper_config(timeout);
    protocol::GripperEnvelope env{};
    std::memcpy(&env, cfg.reserved, sizeof(env));
    return env;
}

void FollowerGripper::set_envelope(const protocol::GripperEnvelope& env) {
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

EnvelopeAudit FollowerGripper::audit_envelope(std::chrono::milliseconds timeout) {
    const protocol::GripperEnvelope stored = get_envelope(timeout);

    std::optional<protocol::MotorSpec> spec;
    std::string spec_error;
    try {
        spec = motor().get_spec(timeout);
    } catch (const std::exception& e) {
        spec_error = e.what();
        // warn, not debug: this fallback decides a value that ensure_envelope()
        // may persist to flash. 0x56 landed in firmware 1.1.6.26 and this class
        // already refuses followers below 1.2.5, so a failure here is a
        // transport problem, not an old device.
        logger()->warn(
            "FollowerGripper::audit_envelope: motor spec unavailable ({}), "
            "falling back to the compiled-in EL05 ratings", spec_error);
    }

    EnvelopeAudit a = detail::audit_envelope(stored, spec);
    if (!spec_error.empty()) {
        a.detail += a.detail.empty() ? "" : "; ";
        a.detail += "spec read failed: " + spec_error;
    }
    return a;
}

EnvelopeWrite FollowerGripper::ensure_envelope(std::chrono::milliseconds timeout) {
    EnvelopeWrite out;
    out.before = audit_envelope(timeout);
    if (!out.before.needs_write()) {
        logger()->info("FollowerGripper::ensure_envelope: already correct ({})",
                       out.before.detail.empty() ? "no issues" : out.before.detail);
        return out;
    }

    // set_envelope() republishes the whole GripperConfig record, so a config
    // that is already unrecognisable is about to be rewritten with whatever
    // travel calibration came back. Say so before doing it rather than after.
    try {
        const protocol::GripperConfig cfg = get_gripper_config(timeout);
        const bool cfg_valid =
            (cfg.flags & protocol::GripperConfigFlag::Valid) != 0;
        if (cfg.magic != protocol::GRIPPER_CONFIG_MAGIC || !cfg_valid) {
            logger()->warn(
                "FollowerGripper::ensure_envelope: gripper config looks "
                "uninitialised (magic=0x{:08x} flags=0x{:04x}); the envelope "
                "write republishes that record, travel calibration included",
                cfg.magic, cfg.flags);
        }
    } catch (const std::exception& e) {
        logger()->debug("ensure_envelope: config pre-check skipped ({})", e.what());
    }

    out.written = detail::repair_envelope(out.before);
    set_envelope(out.written);
    out.wrote = true;
    logger()->info(
        "FollowerGripper::ensure_envelope: repaired [{}] -> cont={:.3f}Nm "
        "peak={:.3f}Nm", out.before.detail, out.written.cont_torque_nm,
        out.written.peak_torque_nm);
    return out;
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

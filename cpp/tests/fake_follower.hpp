// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// A minimal fake follower firmware over a pty, shared by the controller tests.
// It answers the handful of commands FollowerGripper's constructor and the
// controllers' start() issue, pushes motor-status DATA frames while
// "streaming", and records the MIT impedance frames the host submits -- which
// is what lets a test assert on kp/kd/tau_ff rather than only on frame counts.

#pragma once

#include "pty_helper.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace taccap_test {

namespace tx = xense::taccap;

inline std::vector<uint8_t> pod_bytes(const void* p, std::size_t n) {
    std::vector<uint8_t> out(n);
    std::memcpy(out.data(), p, n);
    return out;
}

// Minimal follower firmware: answers the handful of commands
// FollowerGripper's constructor and ForcePositionController::start() issue,
// pushes motor-status DATA frames while "streaming", and records the MIT
// impedance frames the controller submits.
using xense::taccap::FollowerGripper;

class FakeFollower {
public:
    // The default version is **derived from the SDK's floor constants**; do not hard-code it. It
    // was hard-coded once: when the floor was raised from 1.1.6 to 1.2.5, this was still at
    // 1.1.6, so all 47 tests using the default constructor were rejected by their own version
    // gate, and the failure messages pointed at the controller instead of at the version. Once
    // derived, raising the floor no longer breaks these tests — and the cases that really should
    // check the floor pass a version explicitly anyway.
    FakeFollower(Pty& pty,
                 uint8_t major = FollowerGripper::kMinFirmwareMajor,
                 uint8_t minor = FollowerGripper::kMinFirmwareMinor,
                 uint8_t patch = FollowerGripper::kMinFirmwarePatch)
        : pty_(pty), fw_major_(major), fw_minor_(minor), fw_patch_(patch) {
        status_.actual_pos = 0.60f;
        status_.control_mode = 0;
        thread_ = std::thread([this] { run_(); });
    }
    ~FakeFollower() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

    // Stop answering the status stream without stopping the responder: this is
    // what a wedged firmware looks like to the host.
    void freeze_stream(bool frozen) { frozen_.store(frozen); }

    void set_status(float pos, float vel, float torque) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.actual_pos = pos;
        status_.actual_vel = vel;
        status_.actual_torque = torque;
    }

    void set_temperature(float c) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.motor_temp = c;
    }

    // protocol::MotorStatusBit::* on the streamed frames.
    void set_status_word(uint16_t bits) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.status = bits;
    }

    // Next polling ACK can carry a fault while subsequent DATA is healthy.
    void set_next_polled_status_word(uint16_t bits) {
        std::lock_guard<std::mutex> lk(mu_);
        next_polled_status_word_ = bits;
    }

    std::optional<tp::MotorImpedanceCtrl> last_submit() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_submit_;
    }
    unsigned submit_count() const { return submits_.load(); }
    unsigned disable_count() const { return disables_.load(); }
    void set_imu_supported(bool on) { imu_supported_.store(on); }
    void set_imu_v2_supported(bool on) { imu_v2_supported_.store(on); imu_supported_.store(on); }
    unsigned imu_queries() const { return imu_queries_.load(); }
    unsigned stream_stops() const { return stream_stops_.load(); }
    tp::StreamConfig stream_config() const {
        std::lock_guard<std::mutex> lk(mu_);
        return stream_config_;
    }
    // ---- Half-duplex exchange (StreamMode::Polled + Cmd::SlaveExchange) ----
    // Firmware older than the polled mode NACKs StartStream mode 2.
    void set_exchange_supported(bool on) { exchange_supported_.store(on); }
    bool polled() const { return polled_.load(); }
    unsigned exchanges() const { return exchanges_.load(); }
    unsigned telemetry_exchanges() const { return telemetry_exchanges_.load(); }
    unsigned exchange_targets() const { return exchange_targets_.load(); }
    // Hold each exchange reply this long, and count any host frame that
    // arrives meanwhile: a half-duplex host never writes while waiting.
    void set_exchange_reply_delay_ms(int ms) { exchange_delay_ms_.store(ms); }
    unsigned overlaps() const { return overlaps_.load(); }
    // Swallow the next n exchange requests (the target is still applied),
    // so the host sees timeouts.
    void drop_exchange_replies(unsigned n) { drop_replies_.store(n); }
    // Make the next exchange reply's target_result this ErrorCode byte.
    void reject_next_exchange_target(uint8_t err) { reject_next_.store(err); }
    // Non-exchange frames seen while polled (heartbeats, queries, etc.).
    unsigned other_frames_while_polled() const { return other_while_polled_.load(); }

    // Make 0x5C store something other than what was sent.
    void set_corrupt_fw_ver_writes(bool on) { corrupt_fw_ver_writes_.store(on); }

    // ---- Persisted gripper config / motor spec -----------------------------
    // config_writes() is what pins "the second ensure_envelope() does not
    // write": a claim no amount of reading the returned struct can establish.
    unsigned config_writes() const {
        std::lock_guard<std::mutex> lk(mu_);
        return config_writes_;
    }
    tp::GripperConfig stored_config() const {
        std::lock_guard<std::mutex> lk(mu_);
        return cfg_;
    }
    tp::GripperEnvelope stored_envelope() const {
        std::lock_guard<std::mutex> lk(mu_);
        tp::GripperEnvelope e{};
        std::memcpy(&e, cfg_.reserved, sizeof(e));
        return e;
    }
    // Flip the mounting direction the config advertises. The whole point of
    // GripperPosition::to_grip_frame() is that observation signs must not depend
    // on this, so a test has to be able to set it.
    void set_reverse(bool on) {
        std::lock_guard<std::mutex> lk(mu_);
        if (on) {
            cfg_.flags = static_cast<uint16_t>(cfg_.flags | tp::GripperConfigFlag::Reverse);
        } else {
            cfg_.flags = static_cast<uint16_t>(cfg_.flags & ~tp::GripperConfigFlag::Reverse);
        }
    }

    void set_stored_envelope(const tp::GripperEnvelope& e) {
        std::lock_guard<std::mutex> lk(mu_);
        std::memcpy(cfg_.reserved, &e, sizeof(e));
    }
    void set_motor_spec(const tp::MotorSpec& s) {
        std::lock_guard<std::mutex> lk(mu_);
        spec_ = s;
    }
    // Firmware older than 1.1.6.26 has no 0x56.
    void set_spec_supported(bool on) { spec_supported_.store(on); }

    // Make SetSn store something other than what was sent, so the SDK's
    // read-back check has something to catch.
    void set_corrupt_sn_writes(bool on) { corrupt_sn_writes_.store(on); }
    // What 0x59 reports: a model recorded in flash, or the compile-time default.
    void set_model_from_flash(bool on) { model_from_flash_.store(on); }
    // What 0x59 names: id 0 "EL05" (default) or id 1 "RS00".
    void set_model_rs00(bool on) { model_rs00_.store(on); }
    void set_can_id(uint8_t id) { can_id_.store(id); }
    void set_sn(const std::string& sn) {
        std::lock_guard<std::mutex> lk(mu_);
        sn_ = sn;
    }
    std::string stored_sn() const {
        std::lock_guard<std::mutex> lk(mu_);
        return sn_;
    }

private:
    void run_() {
        auto next_status = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            if (auto f = pty_.expect_frame(5)) handle_(*f);

            const auto now = std::chrono::steady_clock::now();
            if (streaming_.load() && !frozen_.load() && now >= next_status) {
                next_status = now + std::chrono::milliseconds(10);
                std::lock_guard<std::mutex> lk(mu_);
                if (stream_config_.source_mask & tp::StreamSrc::MotorStatus)
                    pty_.send_data(0, tp::Cmd::GetMotorStatus,
                               pod_bytes(&status_, sizeof(status_)));
                if (stream_config_.source_mask & tp::StreamSrc::Imu) {
                    tp::ImuData imu{};
                    imu.accel_z = 1000;
                    pty_.send_data(0, tp::Cmd::GetImu, pod_bytes(&imu, sizeof(imu)));
                }
                if (stream_config_.source_mask & tp::StreamSrc::ImuV2)
                    pty_.send_data(0,tp::Cmd::GetImuV2,imu_v2_packet_());
            }
        }
    }

    void handle_(const xense::taccap::bus::Frame& f) {
        if (polled_.load() && f.cmd != tp::Cmd::SlaveExchange) ++other_while_polled_;
        switch (f.cmd) {
            case tp::Cmd::SlaveExchange: {
                handle_exchange_(f);
                return;
            }
            case tp::Cmd::GetVersion: {
                const tp::FirmwareVersion v{fw_major_, fw_minor_, fw_patch_, 0};
                pty_.send_response(f.seq, f.cmd, pod_bytes(&v, sizeof(v)));
                return;
            }
            case tp::Cmd::GetSn: {
                std::vector<uint8_t> out(17, 0);   // sn_info_t: 16 chars + NUL
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    std::memcpy(out.data(), sn_.data(), std::min<size_t>(sn_.size(), 16));
                }
                pty_.send_response(f.seq, f.cmd, out);
                return;
            }
            case tp::Cmd::SetSn: {
                std::string sn(f.payload.begin(), f.payload.end());
                sn = sn.substr(0, sn.find('\0'));
                if (corrupt_sn_writes_.load() && !sn.empty()) sn.back() = '?';
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    sn_ = sn;
                }
                pty_.send_response(f.seq, f.cmd, {});
                return;
            }
            case tp::Cmd::GetDevType: {
                pty_.send_response(f.seq, f.cmd, {dev_type_.load()});
                return;
            }
            case tp::Cmd::SetDevType: {
                // Firmware: LEFT(0) / RIGHT(1) only, anything else InvalidParam.
                if (f.payload.size() != 1 || f.payload[0] > 1) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidParam);
                    return;
                }
                dev_type_.store(f.payload[0]);
                pty_.send_response(f.seq, f.cmd, {});
                return;
            }
            case tp::Cmd::Heartbeat: {
                // uptime_ms (u32 LE) + the four version bytes, as the firmware sends.
                std::vector<uint8_t> out(8, 0);
                const uint32_t up = 123456u;
                std::memcpy(out.data(), &up, 4);
                out[4] = fw_major_; out[5] = fw_minor_; out[6] = fw_patch_; out[7] = 0;
                pty_.send_response(f.seq, f.cmd, out);
                return;
            }
            case tp::Cmd::GetGripperConfig: {
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_response(f.seq, f.cmd,
                                   pod_bytes(&cfg_, sizeof(cfg_)));
                return;
            }
            case tp::Cmd::SetGripperConfig: {
                // Round-trips the record, so a write is observable rather than
                // a NACK. Without this every path that persists anything --
                // the envelope included -- is untestable.
                if (f.payload.size() >= sizeof(tp::GripperConfig)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    std::memcpy(&cfg_, f.payload.data(), sizeof(cfg_));
                    ++config_writes_;
                }
                pty_.send_response(f.seq, f.cmd, {});
                return;
            }
            case tp::Cmd::GetMotorSpec: {
                if (!spec_supported_.load()) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                    return;
                }
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_response(f.seq, f.cmd, pod_bytes(&spec_, sizeof(spec_)));
                return;
            }
            case tp::Cmd::GetGripperAutoCalConfig: {
                tp::GripperAutoCalConfig c{};
                c.magic = 0x4743414CUL;
                c.version = 1;
                c.flags = tp::GripperAutoCalFlag::Valid;
                // The firmware's own tuned numbers, so the controller's
                // advisory cross-check sees a realistic device.
                c.close_stall_torque_nm = 0.35f;
                c.open_stall_torque_nm = 0.35f;
                c.close_speed_rad_s = 0.25f;
                c.open_speed_rad_s = 0.35f;
                c.stall_hold_ms = 30;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&c, sizeof(c)));
                return;
            }
            case tp::Cmd::MotorDisable:
                ++disables_;
                pty_.send_response(f.seq, f.cmd, {});
                return;
            case tp::Cmd::MotorSetStartupLimitTorque:
            case tp::Cmd::MotorGetCanId:
                pty_.send_response(f.seq, f.cmd, {can_id_.load()});
                return;
            case tp::Cmd::MotorSetCanId:
                if (!f.payload.empty()) can_id_.store(f.payload[0]);
                pty_.send_response(f.seq, f.cmd, {});
                return;
            case tp::Cmd::MotorSwitchProtocol:
            case tp::Cmd::SetMotorModel:
                pty_.send_response(f.seq, f.cmd, {});
                return;
            case tp::Cmd::SetMotorFwVersion: {
                std::lock_guard<std::mutex> lk(mu_);
                if (f.payload.size() != 4) {
                    pty_.send_nack(f.seq, tp::ErrorCode::LengthMismatch);
                    return;
                }
                std::memcpy(fw_ver_, f.payload.data(), 4);
                if (corrupt_fw_ver_writes_.load()) fw_ver_[3] ^= 0x01;
                fw_ver_valid_ = true;
                fw_ver_source_ = tp::MotorVersionSource::FlashHost;
                tp::MotorFwVersionRecord r{};
                std::memcpy(r.version, fw_ver_, 4);
                r.valid = 1; r.source = fw_ver_source_;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&r, sizeof(r)));
                return;
            }
            case tp::Cmd::GetMotorVersion: {
                // MIT: the motor never answers; fall back to the flash record.
                std::lock_guard<std::mutex> lk(mu_);
                tp::MotorVersion v{};
                v.protocol_mode = 2;
                if (fw_ver_valid_) {
                    std::memcpy(v.version, fw_ver_, 4);
                    v.valid = 1; v.source = fw_ver_source_;
                }
                pty_.send_response(f.seq, f.cmd, pod_bytes(&v, sizeof(v)));
                return;
            }
            case tp::Cmd::GetMotorModel: {
                // motor_model_t, 20 B. from_flash is the only field the SDK's
                // open-time check reads.
                std::vector<uint8_t> out(20, 0);
                out[0] = model_rs00_.load() ? 1 : 0;
                std::memcpy(out.data() + 1, model_rs00_.load() ? "RS00" : "EL05", 4);
                out[9] = model_from_flash_.load() ? 1 : 0;
                pty_.send_response(f.seq, f.cmd, out);
                return;
            }
            case tp::Cmd::MotorGetStartupLimitTorque: {
                const float limit = 6.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&limit, sizeof(limit)));
                return;
            }
            case tp::Cmd::GetMotorStatus: {
                std::lock_guard<std::mutex> lk(mu_);
                auto reply = status_;
                if (next_polled_status_word_) {
                    reply.status = *next_polled_status_word_;
                    next_polled_status_word_.reset();
                }
                pty_.send_response(f.seq, f.cmd,
                                   pod_bytes(&reply, sizeof(reply)));
                return;
            }
            case tp::Cmd::TimeSession: {
                if(!imu_v2_supported_.load()) {pty_.send_nack(f.seq,tp::ErrorCode::InvalidCmd);return;}
                if(f.payload.size()!=8) {pty_.send_nack(f.seq,tp::ErrorCode::LengthMismatch);return;}
                uint64_t proposed=0; std::memcpy(&proposed,f.payload.data(),8);
                if(!imu_session_) imu_session_=proposed;
                uint64_t reply[2]={imu_time_,imu_session_};
                pty_.send_response(f.seq,f.cmd,pod_bytes(reply,sizeof(reply)));return;
            }
            case tp::Cmd::GetImuV2: {
                if(!imu_v2_supported_.load()) {pty_.send_nack(f.seq,tp::ErrorCode::InvalidCmd);return;}
                pty_.send_response(f.seq,f.cmd,imu_v2_packet_());return;
            }
            case tp::Cmd::GetImu: {
                ++imu_queries_;
                if (!imu_supported_.load()) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                } else {
                    tp::ImuData imu{};
                    imu.accel_z = 1000;
                    pty_.send_response(f.seq, f.cmd, pod_bytes(&imu, sizeof(imu)));
                }
                return;
            }
            case tp::Cmd::StartStream: {
                std::lock_guard<std::mutex> lk(mu_);
                if (f.payload.size() != sizeof(stream_config_)) {
                    pty_.send_nack(f.seq, tp::ErrorCode::LengthMismatch);
                    return;
                }
                tp::StreamConfig requested{};
                std::memcpy(&requested, f.payload.data(), sizeof(requested));
                const bool want_polled = requested.mode == static_cast<uint8_t>(tp::StreamMode::Polled);
                if (want_polled && !exchange_supported_.load()) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidParam);
                    return;
                }
                stream_config_ = requested;
                polled_.store(want_polled);
                streaming_.store(!want_polled);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            }
            case tp::Cmd::StopStream:
                ++stream_stops_;
                polled_.store(false);
                streaming_.store(false);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::MotorImpedanceCtrl: {
                // Fire-and-forget: no ACK, exactly like the real firmware.
                if (f.payload.size() >= sizeof(tp::MotorImpedanceCtrl)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    tp::MotorImpedanceCtrl c{};
                    std::memcpy(&c, f.payload.data(), sizeof(c));
                    last_submit_ = c;
                }
                submits_.fetch_add(1);
                return;
            }
            default:
                pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                return;
        }
    }

    void handle_exchange_(const xense::taccap::bus::Frame& f) {
        tp::ExchangeRequestHeader h{};
        if (f.payload.size() < sizeof(h)) { pty_.send_nack(f.seq, tp::ErrorCode::LengthMismatch); return; }
        std::memcpy(&h, f.payload.data(), sizeof(h));
        ++exchanges_;
        const bool telemetry = h.flags & tp::ExchangeFlag::Telemetry;
        if (telemetry) ++telemetry_exchanges_;
        uint8_t result = tp::EXCHANGE_TARGET_RESULT_NONE;
        if (h.target_kind != 0) {
            ++exchange_targets_;
            result = reject_next_.exchange(0);
            if (h.target_kind == static_cast<uint8_t>(tp::ExchangeTarget::Impedance) &&
                f.payload.size() == sizeof(h) + sizeof(tp::MotorImpedanceCtrl) && result == 0) {
                std::lock_guard<std::mutex> lk(mu_);
                tp::MotorImpedanceCtrl c{};
                std::memcpy(&c, f.payload.data() + sizeof(h), sizeof(c));
                last_submit_ = c;
            }
        }
        if (const int delay = exchange_delay_ms_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            if (auto extra = pty_.expect_frame(1)) {
                ++overlaps_;
                deferred_.push_back(*extra);
            }
        }
        unsigned drop = drop_replies_.load();
        while (drop && !drop_replies_.compare_exchange_weak(drop, drop - 1)) {}
        if (drop) return;
        tp::ExchangeResponseHeader r{};
        r.version = tp::EXCHANGE_VERSION;
        r.target_result = result;
        std::vector<uint8_t> out(sizeof(r));
        if (telemetry) {
            std::lock_guard<std::mutex> lk(mu_);
            if (stream_config_.source_mask & tp::StreamSrc::MotorStatus) {
                r.flags |= tp::ExchangeRespFlag::Motor;
                auto s = pod_bytes(&status_, sizeof(status_));
                s.resize(tp::MOTOR_STATUS_V2_SIZE, 0);
                out.insert(out.end(), s.begin(), s.end());
            }
            if (stream_config_.source_mask & tp::StreamSrc::ImuV2) {
                r.imu_cmd = static_cast<uint8_t>(tp::Cmd::GetImuV2);
                r.imu_count = 1;
                auto imu = imu_v2_packet_();
                out.insert(out.end(), imu.begin(), imu.end());
            } else if (stream_config_.source_mask & tp::StreamSrc::Imu) {
                r.imu_cmd = static_cast<uint8_t>(tp::Cmd::GetImu);
                r.imu_count = 1;
                tp::ImuData imu{};
                imu.accel_z = 1000;
                imu.seq = imu_seq_++;
                auto b = pod_bytes(&imu, sizeof(imu));
                out.insert(out.end(), b.begin(), b.end());
            }
        }
        std::memcpy(out.data(), &r, sizeof(r));
        pty_.send_response(f.seq, f.cmd, out);
        auto later = std::move(deferred_);
        deferred_.clear();
        for (auto& g : later) handle_(g);
    }

    std::atomic<bool> exchange_supported_{true}, polled_{false};
    std::atomic<unsigned> exchanges_{0}, telemetry_exchanges_{0}, exchange_targets_{0};
    std::atomic<unsigned> overlaps_{0}, drop_replies_{0}, other_while_polled_{0};
    std::atomic<int> exchange_delay_ms_{0};
    std::atomic<uint8_t> reject_next_{0};
    std::vector<xense::taccap::bus::Frame> deferred_;

    Pty& pty_;
    uint8_t fw_major_, fw_minor_, fw_patch_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> imu_supported_{false};
    std::atomic<bool> imu_v2_supported_{false};
    uint64_t imu_session_{0}, imu_time_{UINT64_C(0x200000000)};
    uint16_t imu_seq_{0};
    std::vector<uint8_t> imu_v2_packet_() {
        imu_time_+=10000;
        tp::ImuData raw{};raw.accel_z=1000;raw.timestamp_us=uint32_t(imu_time_);raw.seq=imu_seq_++;
        auto p=pod_bytes(&raw,sizeof(raw));
        for(auto value:{imu_time_,imu_session_})
            for(unsigned i=0;i<8;i++)p.push_back(uint8_t(value>>(i*8)));
        return p;
    }
    std::atomic<unsigned> imu_queries_{0}, stream_stops_{0};
    tp::StreamConfig stream_config_{};
    std::atomic<bool> frozen_{false};
    std::atomic<unsigned> submits_{0};
    std::atomic<unsigned> disables_{0};
    std::atomic<bool> corrupt_fw_ver_writes_{false};
    uint8_t fw_ver_[4] = {0, 0, 0, 0};
    bool fw_ver_valid_ = false;
    uint8_t fw_ver_source_ = 0;
    std::atomic<bool> spec_supported_{true};
    std::atomic<bool> corrupt_sn_writes_{false};
    std::atomic<bool> model_from_flash_{true};
    std::atomic<bool> model_rs00_{false};
    std::atomic<uint8_t> can_id_{0x7F};
    std::atomic<uint8_t> dev_type_{0xFF};
    std::string sn_ = "TCGU01A28Z0001s";
    mutable std::mutex mu_;
    tp::MotorStatus status_{};
    std::optional<uint16_t> next_polled_status_word_;
    std::optional<tp::MotorImpedanceCtrl> last_submit_;

    // Factory state: a valid travel calibration and a ZEROED envelope, which
    // is what a device that has never had one written looks like.
    tp::GripperConfig cfg_ = [] {
        tp::GripperConfig c{};
        c.magic        = tp::GRIPPER_CONFIG_MAGIC;
        c.version      = 1;
        c.flags        = tp::GripperConfigFlag::Valid;
        c.max_open_rad = 1.30f;
        c.min_open_rad = 0.0f;
        return c;
    }();
    unsigned config_writes_ = 0;

    // The firmware's EL05 row. Only the fields the SDK reads are filled.
    tp::MotorSpec spec_ = [] {
        tp::MotorSpec s{};
        std::strncpy(s.name, "EL05", sizeof(s.name));
        s.p_max_rad            = 12.57f;
        s.v_max_rad_s          = 50.0f;
        s.t_max_nm             = 6.0f;
        s.kp_max               = 500.0f;
        s.kd_max               = 5.0f;
        s.rated_torque_nm      = 1.8f;
        s.stall_cont_torque_nm = 1.1f;
        s.winding_limit_c      = 135;
        s.board_limit_c        = 103;
        return s;
    }();
};

inline std::unique_ptr<tx::FollowerGripper> open_follower(const Pty& pty) {
    tx::FollowerGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 1;
    return std::make_unique<tx::FollowerGripper>(cfg);
}

// Spin until `pred` or the deadline. Returns whether it held.
template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

}  // namespace taccap_test

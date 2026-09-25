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
    // 默认版本**从 SDK 的门槛常量派生**,不要写死。写死过一次:门槛从 1.1.6 抬到
    // 1.2.5 时,这里还停在 1.1.6,于是 47 个用默认构造的测试全被自己的版本门拒掉,
    // 失败信息指向控制器而不是指向版本。派生之后,抬门槛不会再打断这些测试 ——
    // 而真正该检查门槛的用例本来就显式传版本。
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

    // protocol::MotorStatusBit::* on the streamed frames.
    void set_status_word(uint16_t bits) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.status = bits;
    }

    std::optional<tp::MotorImpedanceCtrl> last_submit() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_submit_;
    }
    unsigned submit_count() const { return submits_.load(); }

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

private:
    void run_() {
        auto next_status = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            if (auto f = pty_.expect_frame(5)) handle_(*f);

            const auto now = std::chrono::steady_clock::now();
            if (streaming_.load() && !frozen_.load() && now >= next_status) {
                next_status = now + std::chrono::milliseconds(10);
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_data(0, tp::Cmd::GetMotorStatus,
                               pod_bytes(&status_, sizeof(status_)));
            }
        }
    }

    void handle_(const xense::taccap::bus::Frame& f) {
        switch (f.cmd) {
            case tp::Cmd::GetVersion: {
                const tp::FirmwareVersion v{fw_major_, fw_minor_, fw_patch_, 0};
                pty_.send_response(f.seq, f.cmd, pod_bytes(&v, sizeof(v)));
                return;
            }
            case tp::Cmd::GetSn: {
                const std::string sn = "TCGU01A28Z0001s";
                pty_.send_response(f.seq, f.cmd,
                                   std::vector<uint8_t>(sn.begin(), sn.end()));
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
            case tp::Cmd::MotorGetStartupLimitTorque: {
                const float limit = 6.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&limit, sizeof(limit)));
                return;
            }
            case tp::Cmd::GetMotorStatus: {
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_response(f.seq, f.cmd,
                                   pod_bytes(&status_, sizeof(status_)));
                return;
            }
            case tp::Cmd::StartStream:
                streaming_.store(true);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::StopStream:
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

    Pty& pty_;
    uint8_t fw_major_, fw_minor_, fw_patch_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> frozen_{false};
    std::atomic<unsigned> submits_{0};
    std::atomic<bool> spec_supported_{true};
    mutable std::mutex mu_;
    tp::MotorStatus status_{};
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

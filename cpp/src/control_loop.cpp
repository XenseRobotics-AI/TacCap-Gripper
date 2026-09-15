// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/control_loop.hpp>
#include <taccap/control_runtime.hpp>

#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>
#include <cmath>

namespace xense::taccap {

ControlLoop::ControlLoop(FollowerGripper& gripper)
    : ControlLoop(gripper, Config{}) {}

ControlLoop::ControlLoop(FollowerGripper& gripper, Config cfg)
    : g_(gripper), cfg_(cfg), runtime_(std::make_unique<detail::ControlRuntime>(gripper)) {
    if (!cfg_.hz || !cfg_.status_timeout_ms || !cfg_.motor_stream_hz)
        throw std::invalid_argument("control rates and timeout must be positive");
    set_gains(cfg_.kp, cfg_.kd, cfg_.feedforward_torque);
}

ControlLoop::~ControlLoop() {
    try { stop(); } catch (...) {}
}

void ControlLoop::start() {
    if (running_.load(std::memory_order_acquire)) return;

    // Loads + validates the calibration (throws if not calibrated) and gives us
    // the raw<->position converter. Copied so the threads read it lock-free.
    pos_map_ = g_.position_map();

    // Seed the target with the current position so enabling + starting the loop
    // holds in place rather than jumping to some stale target.
    const float here = g_.position();
    {
        std::lock_guard<std::mutex> lk(mu_);
        target_ = here;
        obs_ = GripperObservation{};   // reset; first stream frame marks it valid
        stall_clamped_ = false;
        stall_since_   = {};
        torque_capped_ = false;
        cap_since_     = {};
        torque_capped_pub_.store(false, std::memory_order_relaxed);
        stalled_.store(false, std::memory_order_relaxed);
    }

    submit_count_.store(0);
    submit_hz_.store(0.0f);
    rate_count_ = 0;
    rate_start_ = std::chrono::steady_clock::now();
    obs_time_ = rate_start_;
    running_.store(true);
    try {
        runtime_->start(cfg_.motor_stream_hz,
            cfg_.phase == SubmitPhase::FreeRunning ? cfg_.hz : 0,
            [this](const MotorStatusSample& sample) { on_status_(sample); },
            [this](bool event) {
                if (!running()) return;
                bool stale;
                {
                    std::lock_guard<std::mutex> lock(mu_);
                    stale = std::chrono::steady_clock::now() - obs_time_ >=
                        std::chrono::milliseconds(cfg_.status_timeout_ms);
                }
                if (stale) {
                    running_.store(false);
                    g_.motor().disable();
                    logger()->error("ControlLoop: stale feedback, motor disabled");
                    return;
                }
                if (event && submit_once_()) note_rate_(rate_count_, rate_start_);
            });
    } catch (...) { running_.store(false); throw; }

    logger()->info("ControlLoop started: hz={} kp={:.2f} kd={:.2f} stream_hz={}",
                   cfg_.hz, cfg_.kp, cfg_.kd, cfg_.motor_stream_hz);
}

void ControlLoop::stop() {
    running_.store(false);
    runtime_->stop();
}

void ControlLoop::set_target(float position01) {
    if (!std::isfinite(position01)) throw std::invalid_argument("non-finite target");
    const float p = std::clamp(position01, 0.0f, 1.0f);
    std::lock_guard<std::mutex> lk(mu_);
    target_ = p;
}

void ControlLoop::set_gains(float kp, float kd, float feedforward_torque) {
    if (!std::isfinite(kp) || kp < 0 || kp > 500 ||
        !std::isfinite(kd) || kd < 0 || kd > 5 ||
        !std::isfinite(feedforward_torque) || std::abs(feedforward_torque) > 5.5f)
        throw std::invalid_argument("invalid RS05 gains or feedforward torque");
    std::lock_guard<std::mutex> lk(mu_);
    kp_ = kp;
    kd_ = kd;
    ff_ = feedforward_torque;
}

float ControlLoop::target() const {
    std::lock_guard<std::mutex> lk(mu_);
    return target_;
}

GripperObservation ControlLoop::observation() const {
    std::lock_guard<std::mutex> lk(mu_);
    GripperObservation o = obs_;
    if (o.valid) {
        o.age_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - obs_time_).count();
    }
    return o;
}

void ControlLoop::on_status_(const MotorStatusSample& s) {
    std::lock_guard<std::mutex> lk(mu_);
    obs_.position = pos_map_.to_position(s.actual_pos);
    obs_.velocity = s.actual_vel;
    obs_.torque   = s.actual_torque;
    obs_.raw_pos  = s.actual_pos;
    obs_.status   = s.status;
    obs_.motor_temp_c = s.motor_temp_c;
    obs_.seq     += 1;
    obs_.valid    = true;
    obs_time_     = std::chrono::steady_clock::now();
    guard_torque_(s, obs_time_);
    guard_stall_(s, obs_time_);
}

// The hard backstop: acts on the motor's own current-derived torque, not on
// what the loop asked for. Once engaged the frame becomes kp=kd=0 + tau_ff, so
// position error stops contributing to the output entirely.
void ControlLoop::guard_torque_(const MotorStatusSample& s,
                                std::chrono::steady_clock::time_point now) {
    if (cfg_.rated_torque_nm <= 0.0f) return;

    if (torque_capped_) {
        // Two ways out. Either the jaw travelled away from where the ceiling
        // engaged -- the obstruction gave way, and a constant tau_ff would
        // otherwise keep accelerating a free jaw -- or the caller commanded a
        // target back past the entry point, which is them backing off.
        const float entry_pos = pos_map_.to_position(cap_entry_raw_);
        const bool moved_free =
            std::abs(s.actual_pos - cap_entry_raw_) > cfg_.rated_release_rad;
        const bool backed_off = cap_closing_ ? (target_ > entry_pos)
                                             : (target_ < entry_pos);
        if (moved_free || backed_off) {
            torque_capped_ = false;
            cap_since_ = {};
            torque_capped_pub_.store(false, std::memory_order_relaxed);
            logger()->info(
                "ControlLoop torque ceiling released ({}), resuming impedance",
                moved_free ? "jaw came free" : "caller backed off");
        }
        return;
    }

    if (std::abs(s.actual_torque) < cfg_.rated_torque_nm) {
        cap_since_ = {};
        return;
    }
    if (cap_since_.time_since_epoch().count() == 0) {
        cap_since_ = now;
        return;
    }
    if (now - cap_since_ < std::chrono::milliseconds(cfg_.rated_hold_ms)) return;

    torque_capped_ = true;
    cap_sign_      = (s.actual_torque >= 0.0f) ? 1.0f : -1.0f;
    cap_entry_raw_ = s.actual_pos;
    cap_closing_   = target_ < pos_map_.to_position(s.actual_pos);
    torque_capped_pub_.store(true, std::memory_order_relaxed);
    torque_caps_.fetch_add(1, std::memory_order_relaxed);
    logger()->warn(
        "ControlLoop torque ceiling engaged at {:.3f} Nm feedback: holding "
        "{:.3f} Nm pure feed-forward with kp=kd=0, so position error can no "
        "longer add to the output",
        s.actual_torque, cfg_.rated_torque_nm);
}

// Torque AND arrested motion, held for stall_hold_ms -- the same shape the
// firmware's own stall detection uses, and for the same reason: torque alone
// trips on the impact transient of a fast approach, which is not a stall.
void ControlLoop::guard_stall_(const MotorStatusSample& s,
                               std::chrono::steady_clock::time_point now) {
    if (cfg_.stall_action == StallAction::None) return;

    const bool candidate = std::abs(s.actual_torque) >= cfg_.stall_torque_nm &&
                           std::abs(s.actual_vel)    <= cfg_.stall_vel_radps;
    if (!candidate) {
        stall_since_ = {};
        stalled_.store(false, std::memory_order_relaxed);
        return;
    }
    if (stall_since_.time_since_epoch().count() == 0) {
        stall_since_ = now;
        return;
    }
    if (now - stall_since_ < std::chrono::milliseconds(cfg_.stall_hold_ms)) return;

    const float here = pos_map_.to_position(s.actual_pos);
    if (!stall_clamped_) {
        stall_clamped_ = true;
        stall_clamp_   = here;
        stall_closing_ = target_ < here;
        stall_trips_.fetch_add(1, std::memory_order_relaxed);
        logger()->warn(
            "ControlLoop stall guard engaged: jaw blocked at {:.4f} while the "
            "target was {:.4f} (torque {:.3f} Nm, |vel| {:.3f} rad/s). Clamping "
            "the effective target here so position error stops growing; command "
            "a target the other way to release.",
            here, target_, s.actual_torque, std::abs(s.actual_vel));
    }
    stalled_.store(true, std::memory_order_relaxed);
}

// The clamp only blocks travel further INTO the obstruction. A target on the
// other side of the stall point is the caller backing off, and releases it.
float ControlLoop::clamped_target_() noexcept {
    if (!stall_clamped_) return target_;
    const bool released = stall_closing_ ? (target_ > stall_clamp_)
                                         : (target_ < stall_clamp_);
    if (released) {
        stall_clamped_ = false;
        stalled_.store(false, std::memory_order_relaxed);
        stall_since_ = {};
        logger()->info("ControlLoop stall guard released at target {:.4f}", target_);
        return target_;
    }
    return stall_closing_ ? std::max(target_, stall_clamp_)
                          : std::min(target_, stall_clamp_);
}

bool ControlLoop::submit_once_() {
    float raw, kp, kd, ff;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (torque_capped_) {
            // Pure feed-forward hold at the ceiling. The position field rides
            // the measurement so the frame carries no error at all, and with
            // both gains zero it could not act on one anyway.
            kp  = 0.0f;
            kd  = 0.0f;
            ff  = cap_sign_ * cfg_.rated_torque_nm;
            raw = obs_.valid ? obs_.raw_pos : pos_map_.to_rad(target_);
        } else {
            kp = kp_; kd = kd_; ff = ff_;
            raw = pos_map_.to_rad(clamped_target_());
            // Error clamp. Skipped until the stream has told us where the jaw
            // actually is -- with no measurement there is no error to bound.
            if (cfg_.max_position_torque_nm > 0.0f && kp > 0.0f && obs_.valid) {
                const float limit = cfg_.max_position_torque_nm / kp;
                raw = std::clamp(raw, obs_.raw_pos - limit, obs_.raw_pos + limit);
            }
        }
    }
    try {
        g_.motor().submit_impedance(raw, kp, kd, ff);
    } catch (const std::exception& e) {
        logger()->error("ControlLoop submit failed, stopping: {}", e.what());
        running_.store(false, std::memory_order_release);
        try { g_.motor().disable(); } catch (...) {}
        return false;
    }
    submit_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void ControlLoop::note_rate_(uint64_t& window_count,
                             std::chrono::steady_clock::time_point& window_start) {
    ++window_count;
    const auto now = std::chrono::steady_clock::now();
    const double win_s = std::chrono::duration<double>(now - window_start).count();
    if (win_s >= 0.5) {
        submit_hz_.store(static_cast<float>(window_count / win_s),
                         std::memory_order_relaxed);
        window_count = 0;
        window_start = now;
    }
}

// Free-running: submit on our own clock, regardless of what the MCU is doing.
// Keeps the configured rate, at the cost of occasionally writing while the MCU
// is mid-transmission -- see SubmitPhase.
}  // namespace xense::taccap

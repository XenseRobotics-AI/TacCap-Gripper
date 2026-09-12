// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/control_loop.hpp>

#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>
#include <cmath>

namespace xense::taccap {

ControlLoop::ControlLoop(FollowerGripper& gripper)
    : ControlLoop(gripper, Config{}) {}

ControlLoop::ControlLoop(FollowerGripper& gripper, Config cfg)
    : g_(gripper), cfg_(cfg) {
    if (cfg_.hz == 0) cfg_.hz = 1;
    kp_ = cfg_.kp;
    kd_ = cfg_.kd;
    ff_ = cfg_.feedforward_torque;
}

ControlLoop::~ControlLoop() {
    stop();
}

void ControlLoop::start_motor_stream_() {
    if (g_.is_streaming()) {
        // Caller already set up streaming — assume it includes motor status and
        // just ride it. We don't reconfigure (that would drop their sources).
        stream_ours_ = false;
        return;
    }
    // Motor-status-only stream so observations arrive without polling 0x50.
    protocol::StreamConfig sc{};
    sc.source_mask  = protocol::StreamSrc::MotorStatus;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.motor_rate   = static_cast<uint16_t>(cfg_.motor_stream_hz);
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    // Drain any leftover stream from a previous host, same as the gripper does.
    try {
        g_.transport().send_cmd(protocol::Cmd::StopStream, {},
                                std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    auto ack = g_.transport().send_cmd(protocol::Cmd::StartStream,
                                       protocol::encode(sc));
    if (ack.is_nack) {
        throw ProtocolError(std::string("ControlLoop: StartStream NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    stream_ours_ = true;
}

void ControlLoop::stop_motor_stream_() {
    if (!stream_ours_) return;
    try {
        g_.transport().send_cmd(protocol::Cmd::StopStream, {},
                                std::chrono::milliseconds(500));
    } catch (...) { /* best-effort */ }
    stream_ours_ = false;
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
        obs_time_  = std::chrono::steady_clock::now();  // staleness runs from now
        stale_safe_ = false;
        stall_clamped_ = false;
        stall_since_   = {};
        torque_capped_ = false;
        cap_since_     = {};
        torque_capped_pub_.store(false, std::memory_order_relaxed);
        stalled_.store(false, std::memory_order_relaxed);
    }

    start_motor_stream_();
    sub_ = g_.motor().on_status(
        [this](const MotorStatusSample& s) { on_status_(s); });
    sub_active_ = true;

    submit_count_.store(0, std::memory_order_relaxed);
    submit_hz_.store(0.0f, std::memory_order_relaxed);
    stop_flag_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { run_(); });

    logger()->info("ControlLoop started: hz={} kp={:.2f} kd={:.2f} stream_hz={}",
                   cfg_.hz, cfg_.kp, cfg_.kd, cfg_.motor_stream_hz);
}

void ControlLoop::stop() {
    stop_flag_.store(true, std::memory_order_release);
    // StreamLocked parks on the doorbell between status frames. Taking the
    // mutex first closes the lost-wakeup window (same reasoning as
    // Transport::join_workers_): if the loop has evaluated its predicate but
    // not yet blocked, it still holds tick_mu_, so this waits until it is
    // genuinely on the cv -- otherwise stop() would sit out the 100ms timeout.
    { std::lock_guard<std::mutex> lk(tick_mu_); }
    tick_cv_.notify_all();
    if (thread_.joinable() &&
        thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);

    // LEAVE THE MOTOR DISABLED, not merely commanded to zero.
    //
    // A zero-stiffness frame de-energizes nothing: the motor stays enabled and
    // the firmware's host watchdog starts counting. At 300 ms with no target
    // update it fires T1, which holds zero speed via the firmware's VELOCITY
    // path -- and that switches the motor's run mode out from under whatever
    // comes next. Measured: every controller shutdown produced stop_reason=6
    // (HOST_TIMEOUT) within 500 ms, and a later session then commanded a motor
    // whose run mode no longer matched what the MCU believed, so its frames
    // were silently ignored while target_seq/applied_seq advanced normally and
    // last_error stayed 0. The jaw simply did not move, and nothing in the
    // stack said why.
    //
    // Disabling is also the honest end state: once this controller is stopped
    // nothing is regulating the jaw, so holding it energized only invites the
    // watchdog to make that decision for us.
    try { g_.motor().disable(); }
    catch (const std::exception& e) {
        logger()->warn("ControlLoop: motor disable on stop failed: {}", e.what());
    }
    if (sub_active_) {
        g_.motor().off(sub_);
        sub_active_ = false;
    }
    stop_motor_stream_();
}

void ControlLoop::set_target(float position01) {
    const float p = std::clamp(position01, 0.0f, 1.0f);
    std::lock_guard<std::mutex> lk(mu_);
    target_ = p;
}

void ControlLoop::set_gains(float kp, float kd, float feedforward_torque) {
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
    {
        std::lock_guard<std::mutex> lk(tick_mu_);
        tick_ = true;
    }
    tick_cv_.notify_one();

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
        // stalled() means "the clamp is holding the effective target back", not
        // "the jaw is stalling right now". Those diverge immediately: clamping
        // the target to where the jaw actually is drops the position error to
        // ~0, so the torque falls back under stall_torque_nm on the very next
        // frame -- that is the clamp WORKING. Storing false here made the flag
        // blink once and then read false for the entire time the gripper was
        // still clamped. The clamp is released by clamped_target_(), and that
        // is the only place the flag may be cleared.
        stalled_.store(stall_clamped_, std::memory_order_relaxed);
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

void ControlLoop::run_() {
    if (cfg_.phase == SubmitPhase::StreamLocked) run_stream_locked_();
    else                                         run_free_();
    {
        // The loop is no longer maintaining obs_, so it is no longer an
        // observation. Nothing else clears it: a failed submit (a dropped USB
        // link surfaces as SerialBus::write: Input/output error) breaks straight
        // out of the phase loop, so guard_stale_() never runs again and a caller
        // polling observation() would keep reading the last frame as live, with
        // only age_ms rising to give it away.
        std::lock_guard<std::mutex> lk(mu_);
        obs_.valid = false;
    }
    running_.store(false, std::memory_order_release);
}

bool ControlLoop::guard_stale_() {
    if (cfg_.status_timeout_ms == 0) return false;

    float raw = 0.0f;
    bool  entering = false;
    bool  dropped_cap = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        const auto now = std::chrono::steady_clock::now();
        const bool stale = now - obs_time_ >=
            std::chrono::milliseconds(cfg_.status_timeout_ms);
        if (!stale) {
            if (stale_safe_) {
                stale_safe_ = false;
                logger()->info("ControlLoop: motor-status stream recovered, "
                               "resuming impedance control");
            }
            return false;
        }
        if (stale_safe_) return true;     // already safed; stay off the wire
        stale_safe_ = true;
        entering    = true;
        obs_.valid  = false;              // it is history now, not an observation
        // A latched ceiling is released only by evidence FROM the status stream,
        // so a stream that dies while it is engaged would pin the motor at
        // rated_torque_nm with nothing left that could ever let go.
        if (torque_capped_) {
            torque_capped_ = false;
            cap_since_     = {};
            torque_capped_pub_.store(false, std::memory_order_relaxed);
            dropped_cap = true;
        }
        raw = obs_.raw_pos;               // last known; zero gains make it inert
    }

    if (entering) {
        logger()->error(
            "ControlLoop: no motor-status frame in {} ms. Commanding zero "
            "torque and holding off the wire until the stream resumes{}",
            cfg_.status_timeout_ms,
            dropped_cap ? " (a latched torque ceiling was released: it could "
                          "not have been released without status frames)" : "");
        try { g_.motor().submit_impedance(raw, 0.0f, 0.0f, 0.0f); }
        catch (const std::exception& e) {
            logger()->error("ControlLoop: zero-torque safe frame failed: {}", e.what());
        }
    }
    return true;
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
        stop_flag_.store(true, std::memory_order_release);
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
void ControlLoop::run_free_() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(cfg_.hz)));
    auto deadline = clock::now();

    uint64_t window_count = 0;
    auto window_start = clock::now();

    while (!stop_flag_.load(std::memory_order_acquire)) {
        deadline += period;
        std::this_thread::sleep_until(deadline);
        if (stop_flag_.load(std::memory_order_acquire)) break;

        if (guard_stale_()) continue;
        if (!submit_once_()) break;
        note_rate_(window_count, window_start);

        // If we fell badly behind (host stalled), don't burst to catch up --
        // resync the deadline to now.
        if (clock::now() > deadline + period) deadline = clock::now();
    }
}

// Phase-locked: one submit per received status frame, issued immediately after
// it arrives, so the write lands in the window the MCU is not transmitting.
// The submit rate follows the stream rate; cfg_.hz plays no part.
void ControlLoop::run_stream_locked_() {
    using clock = std::chrono::steady_clock;
    uint64_t window_count = 0;
    auto window_start = clock::now();
    unsigned silent = 0;
    bool warned = false;

    while (!stop_flag_.load(std::memory_order_acquire)) {
        bool have_tick = false;
        {
            std::unique_lock<std::mutex> lk(tick_mu_);
            // The timeout is a liveness backstop, not a rate: if the stream
            // stalls we wake anyway, notice stop_flag_, and can exit. We do NOT
            // submit on a bare timeout -- that would reintroduce exactly the
            // unsynchronised write this phase exists to avoid.
            tick_cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                return tick_ || stop_flag_.load(std::memory_order_acquire);
            });
            if (stop_flag_.load(std::memory_order_acquire)) break;
            if (!tick_) {
                // No status frames means no doorbell means nothing is ever
                // submitted. guard_stale_() has already safed the motor by now;
                // this says WHY there are no frames, which it cannot. The usual
                // cause is a caller who had already started streaming without
                // StreamSrc::MotorStatus, in which case start_motor_stream_()
                // rode their config instead of setting ours.
                if (++silent >= 20 && !warned) {   // ~2 s
                    warned = true;
                    logger()->error(
                        "ControlLoop: no motor-status frames in 2s, so the "
                        "stream-locked submitter has nothing to fire on. Is "
                        "StreamSrc::MotorStatus enabled on the stream this loop "
                        "is riding? Use SubmitPhase::FreeRunning if you must "
                        "drive without the status stream.");
                }
            } else {
                tick_ = false;
                silent = 0;
                warned = false;
                have_tick = true;
            }
        }
        // Outside tick_mu_: guard_stale_() takes mu_, and nothing may hold both.
        if (guard_stale_()) continue;
        if (!have_tick) continue;
        if (!submit_once_()) break;
        note_rate_(window_count, window_start);
    }
}

}  // namespace xense::taccap

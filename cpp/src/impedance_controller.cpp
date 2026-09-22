// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/impedance_controller.hpp>

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xense::taccap {

namespace {

void validate_config(const ImpedanceConfig& cfg) {
    auto finite = [](float v) { return std::isfinite(v); };
    if (!finite(cfg.kp) || cfg.kp <= 0.0f) {
        throw std::invalid_argument("ImpedanceConfig.kp must be > 0");
    }
    if (!finite(cfg.kd) || cfg.kd < 0.0f) {
        throw std::invalid_argument("ImpedanceConfig.kd must be >= 0");
    }
    if (!finite(cfg.feedforward_torque) ||
        std::abs(cfg.feedforward_torque) > MOTOR_RATED_TORQUE_NM) {
        throw std::invalid_argument(
            "ImpedanceConfig.feedforward_torque must be within +/-1.8 Nm");
    }
    if (!finite(cfg.max_position_torque_nm) || cfg.max_position_torque_nm < 0.0f ||
        cfg.max_position_torque_nm > MOTOR_PEAK_TORQUE_NM) {
        throw std::invalid_argument(
            "ImpedanceConfig.max_position_torque_nm must be in [0, 6.0]");
    }
    // The ceiling holds indefinitely and nothing here times it out, so it is
    // bounded by the RATED torque, not the peak.
    if (!finite(cfg.rated_torque_nm) || cfg.rated_torque_nm < 0.0f ||
        cfg.rated_torque_nm > MOTOR_RATED_TORQUE_NM) {
        throw std::invalid_argument(
            "ImpedanceConfig.rated_torque_nm must be in [0, 1.8] -- the hold it "
            "produces is indefinite, so it is capped at the motor's rated "
            "torque rather than its peak");
    }
    if (cfg.status_timeout_ms == 0 ||
        cfg.motor_stream_hz == 0 || cfg.motor_stream_hz > 100) {
        throw std::invalid_argument(
            "ImpedanceConfig requires status_timeout_ms > 0 and motor_stream_hz "
            "in [1,100]");
    }
}

void validate_tuning(const detail::ImpedanceTuning& t) {
    auto finite = [](float v) { return std::isfinite(v); };
    if (!finite(t.rated_release_rad) || t.rated_release_rad <= 0.0f) {
        throw std::invalid_argument(
            "ImpedanceTuning.rated_release_rad must be > 0");
    }
    if (!finite(t.stall_torque_floor_nm) || t.stall_torque_floor_nm <= 0.0f ||
        t.stall_torque_floor_nm > MOTOR_RATED_TORQUE_NM) {
        throw std::invalid_argument(
            "ImpedanceTuning.stall_torque_floor_nm must be in (0, 1.8]");
    }
    if (!finite(t.stall_vel_radps) || t.stall_vel_radps <= 0.0f) {
        throw std::invalid_argument(
            "ImpedanceTuning.stall_vel_radps must be > 0");
    }
    if (t.rated_hold_ms == 0 || t.stall_hold_ms == 0) {
        throw std::invalid_argument(
            "ImpedanceTuning hold windows must be > 0");
    }
}

// MotorStatusBit::Stalled (0x0004) is deliberately NOT in this mask. Holding a
// blocked position is a stall by the motor's own definition, and the stall
// guard below is what interprets it; treating it as a fault would abort every
// legitimate hold. Same rule as ForcePositionController -- do not "complete"
// this list.
bool has_serious_fault(uint16_t status) noexcept {
    constexpr uint16_t mask =
        protocol::MotorStatusBit::Fault |
        protocol::MotorStatusBit::OverTemp |
        protocol::MotorStatusBit::OverCurrent |
        protocol::MotorStatusBit::OverVolt |
        protocol::MotorStatusBit::UnderVolt |
        protocol::MotorStatusBit::EncoderError |
        protocol::MotorStatusBit::DriverFault |
        protocol::MotorStatusBit::PositionInitError |
        protocol::MotorStatusBit::HardwareIdError |
        protocol::MotorStatusBit::EncoderUncalibrated;
    return (status & mask) != 0;
}

}  // namespace

const char* to_string(ImpedanceState state) noexcept {
    switch (state) {
        case ImpedanceState::Idle:         return "idle";
        case ImpedanceState::Tracking:     return "tracking";
        case ImpedanceState::Stalled:      return "stalled";
        case ImpedanceState::TorqueCapped: return "torque_capped";
        case ImpedanceState::Fault:        return "fault";
    }
    return "unknown";
}

namespace detail {

ImpedancePolicy::ImpedancePolicy(GripperPosition map, ImpedanceConfig cfg)
    : ImpedancePolicy(std::move(map), cfg, ImpedanceTuning{}) {}

ImpedancePolicy::ImpedancePolicy(GripperPosition map, ImpedanceConfig cfg,
                                 ImpedanceTuning tune)
    : map_(std::move(map)), cfg_(cfg), tune_(tune) {
    if (!map_.valid()) {
        throw std::invalid_argument("ImpedancePolicy requires a valid position map");
    }
    validate_config(cfg_);
    validate_tuning(tune_);
    kp_ = cfg_.kp;
    kd_ = cfg_.kd;
    ff_ = cfg_.feedforward_torque;
}

void ImpedancePolicy::reset(const MotorStatusSample& sample) {
    if (state_ == ImpedanceState::Fault) {
        logger()->info("ImpedanceController: leaving Fault ({}), resuming at {:.4f}",
                       fault_reason_, map_.to_position(sample.actual_pos));
    }
    state_ = ImpedanceState::Tracking;
    target_ = effective_ = map_.to_position(sample.actual_pos);
    commanded_torque_nm_ = 0.0f;
    stall_since_ = {};
    stall_clamped_ = false;
    cap_since_ = {};
    torque_capped_ = false;
    fault_reason_.clear();
}

void ImpedancePolicy::set_target(float position) {
    if (!std::isfinite(position)) return;
    target_ = std::clamp(position, 0.0f, 1.0f);
}

void ImpedancePolicy::set_gains(float kp, float kd, float feedforward_torque) {
    if (std::isfinite(kp) && kp >= 0.0f) kp_ = kp;
    if (std::isfinite(kd) && kd >= 0.0f) kd_ = kd;
    if (std::isfinite(feedforward_torque)) ff_ = feedforward_torque;
}

void ImpedancePolicy::fail(std::string reason) {
    // EDGE-TRIGGERED, and it has to be: step() re-calls fail() on every frame
    // while the condition persists, so logging unconditionally would put 100
    // lines a second in the session file. Logging only the transition is what
    // makes the file useful -- a fault used to leave NO record at all, so a run
    // that died because the USB tree dropped underneath it was indistinguishable
    // in the log from one that exited cleanly.
    if (state_ != ImpedanceState::Fault) {
        logger()->error("ImpedanceController: entering Fault -- {}", reason);
    }
    state_ = ImpedanceState::Fault;
    commanded_torque_nm_ = 0.0f;
    fault_reason_ = std::move(reason);
}

void ImpedancePolicy::guard_torque_(const MotorStatusSample& s,
                                    std::chrono::steady_clock::time_point now) {
    if (cfg_.rated_torque_nm <= 0.0f) return;

    if (torque_capped_) {
        // Two ways out. Either the jaw travelled away from where the ceiling
        // engaged -- the obstruction gave way, and a constant tau_ff would
        // otherwise keep accelerating a free jaw -- or the caller commanded a
        // target back past the entry point, which is them backing off.
        const float entry_pos = map_.to_position(cap_entry_raw_);
        const bool moved_free =
            std::abs(s.actual_pos - cap_entry_raw_) > tune_.rated_release_rad;
        const bool backed_off = cap_closing_ ? (target_ > entry_pos)
                                             : (target_ < entry_pos);
        if (moved_free || backed_off) {
            torque_capped_ = false;
            cap_since_ = {};
            logger()->info("ImpedanceController: torque ceiling released ({})",
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
    if (now - cap_since_ < std::chrono::milliseconds(tune_.rated_hold_ms)) return;

    torque_capped_ = true;
    cap_sign_      = (s.actual_torque >= 0.0f) ? 1.0f : -1.0f;
    cap_entry_raw_ = s.actual_pos;
    cap_closing_   = target_ < map_.to_position(s.actual_pos);
    ++torque_caps_;
    logger()->warn(
        "ImpedanceController: torque ceiling engaged at {:.3f} Nm feedback; "
        "holding {:.3f} Nm pure feed-forward with kp=kd=0, so position error "
        "can no longer add to the output",
        s.actual_torque, cfg_.rated_torque_nm);
}

void ImpedancePolicy::guard_stall_(const MotorStatusSample& s,
                                   std::chrono::steady_clock::time_point now) {
    // The clamp binding is the real signal: it means the controller is asking
    // for every bit of torque it is allowed to ask for. Reachable by
    // construction, unlike an absolute trip near the cap -- see the note on
    // ImpedanceTuning::stall_torque_floor_nm.
    bool clamp_binding = false;
    if (cfg_.max_position_torque_nm > 0.0f && kp_ > 0.0f) {
        const float limit = cfg_.max_position_torque_nm / kp_;
        clamp_binding = std::abs(map_.to_rad(target_) - s.actual_pos) >= limit;
    }
    const bool candidate = clamp_binding &&
                           std::abs(s.actual_vel)    <= tune_.stall_vel_radps &&
                           std::abs(s.actual_torque) >= tune_.stall_torque_floor_nm;
    if (!candidate) {
        // The clamp is released by clamped_target_() and nowhere else. Clearing
        // the flag here would make it blink once and read false for the whole
        // time the gripper is still clamped: clamping the target to where the
        // jaw is drops the error to ~0, so the torque falls back under the trip
        // on the very next frame -- that is the clamp WORKING.
        stall_since_ = {};
        return;
    }
    if (stall_since_.time_since_epoch().count() == 0) {
        stall_since_ = now;
        return;
    }
    if (now - stall_since_ < std::chrono::milliseconds(tune_.stall_hold_ms)) return;

    if (!stall_clamped_) {
        const float here = map_.to_position(s.actual_pos);
        stall_clamped_ = true;
        stall_clamp_   = here;
        stall_closing_ = target_ < here;
        ++stall_trips_;
        logger()->warn(
            "ImpedanceController: stall guard engaged, jaw blocked at {:.4f} "
            "while the target was {:.4f} (error clamp binding, torque {:.3f} Nm, "
            "|vel| {:.3f} rad/s). "
            "Clamping the effective target here so position error stops growing; "
            "command a target the other way to release.",
            here, target_, s.actual_torque, std::abs(s.actual_vel));
    }
}

float ImpedancePolicy::clamped_target_() {
    if (!stall_clamped_) return target_;
    const bool released = stall_closing_ ? (target_ > stall_clamp_)
                                         : (target_ < stall_clamp_);
    if (released) {
        stall_clamped_ = false;
        stall_since_ = {};
        logger()->info("ImpedanceController: stall guard released at target {:.4f}",
                       target_);
        return target_;
    }
    return stall_closing_ ? std::max(target_, stall_clamp_)
                          : std::min(target_, stall_clamp_);
}

protocol::MotorImpedanceCtrl ImpedancePolicy::zero_(const MotorStatusSample& s) {
    commanded_torque_nm_ = 0.0f;
    effective_ = map_.to_position(s.actual_pos);
    return {s.actual_pos, 0.0f, 0.0f, 0.0f, 0.0f};
}

protocol::MotorImpedanceCtrl ImpedancePolicy::step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now) {
    if (!std::isfinite(sample.actual_pos) || !std::isfinite(sample.actual_vel) ||
        !std::isfinite(sample.actual_torque)) {
        fail("non-finite motor status");
    } else if (has_serious_fault(sample.status)) {
        fail("motor status reports a fault");
    } else if (std::abs(sample.actual_torque) > MOTOR_PEAK_TORQUE_NM + 1e-4f) {
        fail("measured torque exceeded the motor peak rating");
    }

    if (state_ == ImpedanceState::Fault || state_ == ImpedanceState::Idle) {
        return zero_(sample);
    }

    guard_torque_(sample, now);
    guard_stall_(sample, now);

    if (torque_capped_) {
        state_ = ImpedanceState::TorqueCapped;
        // The position field rides the measurement so the frame carries no
        // error at all, and with both gains zero it could not act on one anyway.
        commanded_torque_nm_ = cfg_.rated_torque_nm;
        effective_ = map_.to_position(sample.actual_pos);
        return {sample.actual_pos, 0.0f, 0.0f,
                cap_sign_ * cfg_.rated_torque_nm, 0.0f};
    }

    const float effective = clamped_target_();
    state_ = stall_clamped_ ? ImpedanceState::Stalled : ImpedanceState::Tracking;

    float raw = map_.to_rad(effective);
    // The error clamp. Applied on every frame, under every state, which is why
    // it is not one of them.
    if (cfg_.max_position_torque_nm > 0.0f && kp_ > 0.0f) {
        const float limit = cfg_.max_position_torque_nm / kp_;
        raw = std::clamp(raw, sample.actual_pos - limit, sample.actual_pos + limit);
    }
    effective_ = map_.to_position(raw);

    const float predicted =
        kp_ * (raw - sample.actual_pos) + kd_ * (0.0f - sample.actual_vel) + ff_;
    commanded_torque_nm_ = std::min(MOTOR_PEAK_TORQUE_NM, std::abs(predicted));
    return {raw, kp_, kd_, ff_, 0.0f};
}

}  // namespace detail

// ---------------------------------------------------------------------------
// ImpedanceController — transport owner
// ---------------------------------------------------------------------------

ImpedanceController::ImpedanceController(FollowerGripper& gripper)
    : ImpedanceController(gripper, ImpedanceConfig{}) {}

ImpedanceController::ImpedanceController(FollowerGripper& gripper,
                                         ImpedanceConfig cfg)
    : g_(gripper), cfg_(cfg) {
    validate_config_();
}

ImpedanceController::~ImpedanceController() { stop(); }

void ImpedanceController::validate_config_() const { validate_config(cfg_); }

void ImpedanceController::start_motor_stream_() {
    if (g_.is_streaming()) { stream_ours_ = false; return; }
    g_.start_streaming(cfg_.motor_stream_hz);
    stream_ours_ = true;
}

void ImpedanceController::stop_motor_stream_() {
    if (!stream_ours_) return;
    g_.stop_streaming();
    stream_ours_ = false;
}

void ImpedanceController::start() {
    if (running()) return;
    validate_config_();
    map_ = g_.position_map();

    const float device_limit = g_.motor().get_startup_limit_torque();
    if (!std::isfinite(device_limit) || device_limit <= 0.0f) {
        throw std::runtime_error(
            "ImpedanceController: stored motor startup torque limit reads " +
            std::to_string(device_limit) +
            " Nm; call set_startup_limit_torque(), power-cycle the gripper, and "
            "verify the value before enabling motion");
    }

    // The ceiling holds indefinitely, so the same sustained-torque cross-check
    // ForcePositionController does applies here. Advisory, never fatal.
    try {
        const auto env = g_.get_envelope();
        const bool enforced =
            (env.flags & protocol::GripperEnvelopeFlag::Enforce) != 0;
        if (std::isfinite(env.cont_torque_nm) && env.cont_torque_nm > 0.0f &&
            cfg_.rated_torque_nm > env.cont_torque_nm) {
            logger()->warn(
                "ImpedanceController: torque ceiling {:.3f} Nm is above the "
                "device's continuous envelope {:.3f} Nm, and the hold it "
                "produces is indefinite. {}",
                cfg_.rated_torque_nm, env.cont_torque_nm,
                enforced ? "The firmware will derate it."
                         : "The envelope is NOT enforced, so nothing below this "
                           "host limits the sustained draw.");
        }
        if (!enforced) {
            logger()->warn(
                "ImpedanceController: the motion envelope is not enforced, so "
                "the firmware's I2t derate and temperature wall are inactive");
        }
    } catch (const std::exception& e) {
        logger()->debug("ImpedanceController: envelope unavailable ({})", e.what());
    }

    const MotorStatusSample initial = g_.motor().read_status();
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        policy_ = std::make_unique<detail::ImpedancePolicy>(map_, cfg_);
        policy_->reset(initial);       // seeds the target with where we are
        latest_ = initial;
        latest_time_ = now;
        have_sample_ = true;
        observation_ = GripperObservation{};
        observation_.valid = true;
        observation_.position = map_.to_position(initial.actual_pos);
        observation_.velocity = initial.actual_vel;
        observation_.torque = initial.actual_torque;
        observation_.raw_pos = initial.actual_pos;
        observation_.status = initial.status;
        observation_.motor_temp_c = initial.motor_temp_c;
        observation_.seq = 1;
        stop_requested_ = false;
        step_requested_ = true;
        device_limit_nm_ = device_limit;
    }

    sub_ = g_.motor().on_status(
        [this](const MotorStatusSample& sample) { on_status_(sample); });
    sub_active_ = true;
    try {
        start_motor_stream_();
    } catch (...) {
        g_.motor().off(sub_);
        sub_active_ = false;
        throw;
    }

    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { run_(); });
    cv_.notify_one();
    logger()->info(
        "ImpedanceController started: kp={:.2f} kd={:.2f} clamp={:.3f}Nm "
        "ceiling={:.3f}Nm stream={}Hz device_limit={:.3f}Nm",
        cfg_.kp, cfg_.kd, cfg_.max_position_torque_nm, cfg_.rated_torque_nm,
        cfg_.motor_stream_hz, device_limit_nm_);
}

void ImpedanceController::stop() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_requested_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);

    MotorStatusSample last{};
    bool have = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        last = latest_;
        have = have_sample_;
        observation_.valid = false;   // nothing maintains it once we are stopped
    }
    if (have) {
        try { g_.motor().submit_impedance(last.actual_pos, 0.0f, 0.0f, 0.0f); }
        catch (...) {}
    }
    // Leave the motor DISABLED, not merely commanded to zero -- a zero-stiffness
    // frame de-energizes nothing, and the firmware's host watchdog then fires
    // T1 within 300 ms and switches the run mode out from under the next
    // session. Full rationale and the measurement in ControlLoop::stop().
    try { g_.motor().disable(); }
    catch (const std::exception& e) {
        logger()->warn("ImpedanceController: motor disable on stop failed: {}", e.what());
    }
    if (sub_active_) {
        g_.motor().off(sub_);
        sub_active_ = false;
    }
    stop_motor_stream_();
}

void ImpedanceController::on_status_(const MotorStatusSample& sample) {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        latest_ = sample;
        latest_time_ = now;
        have_sample_ = true;
        observation_.valid = true;
        observation_.position = map_.to_position(sample.actual_pos);
        observation_.velocity = sample.actual_vel;
        observation_.torque = sample.actual_torque;
        observation_.raw_pos = sample.actual_pos;
        observation_.status = sample.status;
        observation_.motor_temp_c = sample.motor_temp_c;
        ++observation_.seq;
        step_requested_ = true;
    }
    cv_.notify_one();
}

// Target and gain changes only touch policy state; the frame still goes out on
// the status doorbell in run_(), inside the window the MCU is known to be idle.
// A caller-thread submit lands at whatever moment that thread called in and
// costs a telemetry frame when it overlaps the MCU's own transmission.
void ImpedanceController::set_target(float position) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ImpedanceController::set_target called before start");
    }
    if (!std::isfinite(position)) {
        throw std::invalid_argument("target position must be finite");
    }
    policy_->set_target(position);
    command_woke_ = true;
    cv_.notify_one();
}

void ImpedanceController::set_gains(float kp, float kd, float feedforward_torque) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ImpedanceController::set_gains called before start");
    }
    if (!std::isfinite(kp) || kp < 0.0f || !std::isfinite(kd) || kd < 0.0f ||
        !std::isfinite(feedforward_torque) ||
        std::abs(feedforward_torque) > MOTOR_RATED_TORQUE_NM) {
        throw std::invalid_argument(
            "gains require kp >= 0, kd >= 0 and |feedforward| <= 1.8 Nm");
    }
    policy_->set_gains(kp, kd, feedforward_torque);
    command_woke_ = true;
    cv_.notify_one();
}

void ImpedanceController::reset() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ImpedanceController::reset called before start");
    }
    policy_->reset(latest_);
    // The cached sample may still carry the fault bit that was true immediately
    // before Motor::clear_fault() ACKed. Wait for the next streamed status
    // rather than re-faulting the freshly reset policy on stale evidence.
    step_requested_ = false;
}

ImpedanceState ImpedanceController::state() const {
    std::lock_guard<std::mutex> lk(mu_);
    return policy_ ? policy_->state() : ImpedanceState::Idle;
}

ImpedanceSnapshot ImpedanceController::snapshot() const {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mu_);
    ImpedanceSnapshot out;
    out.running = running();
    out.observation = observation_;
    if (out.observation.valid) {
        out.observation.age_ms = std::chrono::duration<double, std::milli>(
            now - latest_time_).count();
    }
    out.device_limit_nm = device_limit_nm_;
    if (policy_) {
        out.state = policy_->state();
        out.target_position = policy_->target_position();
        out.effective_position = policy_->effective_position();
        out.commanded_torque_nm = policy_->commanded_torque_nm();
        out.stalled = policy_->stalled();
        out.torque_capped = policy_->torque_capped();
        out.stall_trips = policy_->stall_trips();
        out.torque_caps = policy_->torque_caps();
        out.fault_reason = policy_->fault_reason();
    }
    return out;
}

void ImpedanceController::run_() {
    const auto timeout = std::chrono::milliseconds(cfg_.status_timeout_ms);
    for (;;) {
        protocol::MotorImpedanceCtrl command{};
        bool send = false;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                return stop_requested_ || step_requested_ || command_woke_;
            });
            if (stop_requested_) break;
            const bool woke_by_command = command_woke_;
            command_woke_ = false;
            const auto now = std::chrono::steady_clock::now();
            if (!policy_ || !have_sample_) continue;

            const bool stale = now - latest_time_ >= timeout;
            if (stale) {
                // Invalidating the observation is independent of WHY the policy
                // is faulted -- the last sample is history the moment frames
                // stop arriving. Gating the two together let a submit failure
                // (which is how a dropped USB link arrives, and which faults the
                // policy well inside status_timeout_ms) leave the observation
                // reading "valid" indefinitely.
                observation_.valid = false;
                if (policy_->state() != ImpedanceState::Fault) {
                    policy_->fail("motor status stream stale");
                    step_requested_ = true;   // push one zero-torque command out
                }
            }
            // A caller command on a dead stream is answered immediately with the
            // zero-torque frame step() produces in Fault, rather than being
            // swallowed with nothing on the wire.
            if (woke_by_command && stale) step_requested_ = true;

            if (!step_requested_) continue;
            step_requested_ = false;

            command = policy_->step(latest_, now);
            send = true;
        }

        if (!send) continue;
        try {
            g_.motor().submit(command);
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu_);
            if (policy_) policy_->fail(std::string("submit failed: ") + e.what());
        }
    }
}

}  // namespace xense::taccap

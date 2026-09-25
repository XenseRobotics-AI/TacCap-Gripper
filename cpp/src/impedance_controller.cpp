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
        cfg.max_position_torque_nm > MOTOR_ABSOLUTE_TORQUE_CEILING_NM) {
        throw std::invalid_argument(
            "ImpedanceConfig.max_position_torque_nm must be in [0, 20]");
    }
    // The ceiling holds indefinitely and nothing here times it out, so it is
    // bounded by the RATED torque, not the peak.
    // Sanity ceiling only -- the binding limit is this DEVICE's rated torque,
    // checked in start(). The hold it produces is indefinite, so the device
    // check uses the rated (rotating) torque and not the peak; but which number
    // that is depends on the motor, and a config is built before the device is
    // reachable.
    if (!finite(cfg.rated_torque_nm) || cfg.rated_torque_nm < 0.0f ||
        cfg.rated_torque_nm > MOTOR_ABSOLUTE_TORQUE_CEILING_NM) {
        throw std::invalid_argument(
            "ImpedanceConfig.rated_torque_nm must be in [0, 20]");
    }
    // The ceiling is a BACKSTOP for torque the control law did not ask for, so
    // it has to sit above the band the law works in. max_position_torque_nm
    // bounds the position term and the ceiling watches the measurement; if what
    // the law can sustain reaches the ceiling, a perfectly normal grasp trips
    // it, the output goes to kp=kd=0 and POSITION CONTROL IS LOST while holding
    // the object. A backstop that fires in the operating band is mis-set, not
    // protective.
    //
    // FEED-FORWARD COUNTS TOWARD IT. It is added to the command on every frame,
    // so the sustained ask is budget + |ff|, not the budget alone -- checking
    // the budget by itself leaves a hole this exact class of bug walks through:
    // at the 1.1 default, an ff of 0.8 puts the steady command at 1.9 and every
    // grasp lands on the 1.8 backstop.
    const float sustained =
        cfg.max_position_torque_nm + std::abs(cfg.feedforward_torque);
    if (cfg.rated_torque_nm > 0.0f && sustained >= cfg.rated_torque_nm) {
        throw std::invalid_argument(
            "ImpedanceConfig: max_position_torque_nm + |feedforward_torque| "
            "must stay below rated_torque_nm -- that sum is what a normal grasp "
            "sustains and the ceiling is the backstop above it; at or past it "
            "every grasp trips the ceiling and drops to kp=kd=0");
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
    if (t.rated_hold_ms == 0) {
        throw std::invalid_argument(
            "ImpedanceTuning.rated_hold_ms must be > 0");
    }
}

// MotorStatusBit::Stalled (0x0004) is deliberately NOT in this mask. Holding a
// blocked position is a stall by the motor's own definition, and holding is what
// this controller is FOR -- an obstruction saturating the error clamp is the
// intended steady state, not a fault. Treating the bit as one would abort every
// legitimate grasp. Same rule as ForcePositionController -- do not "complete"
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

// What the ceiling holds once it engages. NOT rated_torque_nm: it fires
// because the motor is ALREADY producing at or above that, and answering by
// commanding it right back -- indefinitely, with nothing timing it out -- would
// pin a sustained torque above the budget that was chosen precisely because it
// can be held forever. Collapse to the budget instead: the response to "you are
// producing more than rated" is to ask for what you are allowed to keep asking
// for. With the clamp disabled there is no budget, so the rating stands.
float ImpedancePolicy::cap_hold_torque_() const noexcept {
    return (cfg_.max_position_torque_nm > 0.0f)
        ? std::min(cfg_.rated_torque_nm, cfg_.max_position_torque_nm)
        : cfg_.rated_torque_nm;
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
        "ImpedanceController: torque ceiling engaged at {:.3f} Nm feedback "
        "(trip {:.3f} Nm); holding {:.3f} Nm pure feed-forward with kp=kd=0, so "
        "position error can no longer add to the output",
        s.actual_torque, cfg_.rated_torque_nm, cap_hold_torque_());
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

    if (torque_capped_) {
        state_ = ImpedanceState::TorqueCapped;
        // The position field rides the measurement so the frame carries no
        // error at all, and with both gains zero it could not act on one anyway.
        commanded_torque_nm_ = cap_hold_torque_();
        effective_ = map_.to_position(sample.actual_pos);
        return {sample.actual_pos, 0.0f, 0.0f,
                cap_sign_ * cap_hold_torque_(), 0.0f};
    }

    // No stall state and no target clamping: the error clamp below bounds the
    // output on its own, and an obstruction simply saturates it. See the header.
    const float effective = target_;
    state_ = ImpedanceState::Tracking;

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

namespace {

// A spec field is 0 when the firmware's table does not carry that number for
// this model, which means UNKNOWN, never the number zero. Fall back per FIELD,
// not per spec: the RS0x rows carry a real rated torque but no stall rating, so
// an all-or-nothing fallback would quietly pair an RS00 rated torque with an
// EL05 grip budget.
float spec_or(float v, float fallback) {
    return (std::isfinite(v) && v > 0.0f) ? v : fallback;
}

}  // namespace

ImpedanceConfig ImpedanceConfig::for_spec(const protocol::MotorSpec& spec) {
    ImpedanceConfig cfg;
    cfg.max_position_torque_nm =
        spec_or(spec.stall_cont_torque_nm, MOTOR_STALL_CONT_TORQUE_NM);
    cfg.rated_torque_nm = spec_or(spec.rated_torque_nm, MOTOR_RATED_TORQUE_NM);

    // The backstop has to sit above the budget, or it fires inside the
    // operating band and position control is lost mid-grasp. If a model ever
    // reports the two equal, back the BUDGET off rather than raising a rating
    // the motor does not have.
    if (cfg.rated_torque_nm <= cfg.max_position_torque_nm) {
        cfg.max_position_torque_nm = cfg.rated_torque_nm * 0.9f;
    }

    // Approach speed is budget/kd. This is the line that keeps a bigger grip
    // from silently also being a faster one -- the coupling that makes "just
    // raise the torque" wrong.
    cfg.kd = cfg.max_position_torque_nm / MOTOR_APPROACH_SPEED_RADPS;
    return cfg;
}

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

    // The error clamp saturates and HOLDS, so it gets the same sustained-torque
    // cross-check ForcePositionController does.
    //
    // ADVISORY, NEVER FATAL, AND IT MUST NOT WRITE. ensure_envelope() persists
    // to MCU flash; calling it from here would turn every controller start into
    // a flash write, including the ones that only meant to look.
    //
    // Compare against what the firmware ENFORCES, not what the device stores.
    // A unit holding an inflated cont in flash would otherwise silence exactly
    // the warning it needs -- the host would read 1.8 while the firmware ran at
    // 1.1.
    // Bind the ceiling to THIS DEVICE's rated torque. Same reasoning as
    // ForcePositionController: a config is built before a device is reachable,
    // so validate_config() can only sanity-check, and the real bound lives here.
    try {
        const auto spec = g_.motor().get_spec();
        const float rated = spec.rated_torque_nm;
        const std::string model(spec.name, ::strnlen(spec.name, sizeof(spec.name)));
        if (std::isfinite(rated) && rated > 0.0f &&
            cfg_.rated_torque_nm > rated + 1e-4f) {
            throw std::invalid_argument(
                "ImpedanceConfig.rated_torque_nm " +
                std::to_string(cfg_.rated_torque_nm) + " Nm exceeds the " + model +
                "'s rated torque " + std::to_string(rated) +
                " Nm; the hold it produces is indefinite. Build the config with "
                "ImpedanceConfig::for_spec().");
        }
        // Approach speed is budget/kd. Raising the budget without raising kd
        // raises the speed with it, straight towards the regime where a loose
        // object is pushed away before contact can register (measured 5.5 rad/s).
        if (cfg_.kd > 0.0f) {
            const float approach = cfg_.max_position_torque_nm / cfg_.kd;
            if (approach > MOTOR_MAX_APPROACH_SPEED_RADPS) {
                throw std::invalid_argument(
                    "ImpedanceConfig implies an approach speed of " +
                    std::to_string(approach) + " rad/s (budget " +
                    std::to_string(cfg_.max_position_torque_nm) + " / kd " +
                    std::to_string(cfg_.kd) + "), above the " +
                    std::to_string(MOTOR_MAX_APPROACH_SPEED_RADPS) +
                    " rad/s ceiling. Raise kd with the budget -- "
                    "ImpedanceConfig::for_spec() does.");
            }
        }
    } catch (const std::invalid_argument&) {
        throw;
    } catch (const std::exception& e) {
        logger()->warn("ImpedanceController: motor spec unavailable ({}), "
                       "torque ceiling not checked against the device", e.what());
    }

    try {
        const auto audit = g_.audit_envelope();
        // The budget is what a blocked jaw sits on indefinitely. NOT
        // rated_torque_nm: that is a trip level on MEASURED torque and the
        // header requires it to sit ABOVE the operating band, so comparing it
        // against a sustained ceiling is a category error -- it would fire at
        // stock defaults on any correctly configured device.
        const float budget = cfg_.max_position_torque_nm;
        if (audit.effective && std::isfinite(audit.effective->cont_torque_nm) &&
            audit.effective->cont_torque_nm > 0.0f &&
            budget > audit.effective->cont_torque_nm + 1e-4f) {
            logger()->warn(
                "ImpedanceController: torque budget {:.3f} Nm is above the "
                "{:.3f} Nm the firmware sustains, and the hold it produces is "
                "indefinite -- the firmware will derate it, so a blocked jaw "
                "will not hold what was asked for",
                budget, audit.effective->cont_torque_nm);
        }
        if (!audit.ok()) {
            logger()->warn(
                "ImpedanceController: motion envelope: {}. Repair it with "
                "FollowerGripper::ensure_envelope() (writes MCU flash).",
                audit.detail);
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
        observation_.velocity = map_.to_grip_frame(initial.actual_vel);
        observation_.torque = map_.to_grip_frame(initial.actual_torque);
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
    // session. Full rationale and the measurement in
    // ForcePositionController::stop().
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
        observation_.velocity = map_.to_grip_frame(sample.actual_vel);
        observation_.torque = map_.to_grip_frame(sample.actual_torque);
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
        out.torque_capped = policy_->torque_capped();
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

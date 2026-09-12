// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/force_position_controller.hpp>

#include <cstring>

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xense::taccap {

namespace {

constexpr float kEpsilon = 1e-5f;
// Ceiling on the closing/opening velocity-damping gain.
constexpr float kMaxDampingGain = 5.0f;
// "Holding" is the RAMP having run as far ahead of the jaw as the budget lets
// it while the jaw is not following.
//
// Not "the command reached its budget", which is what this used to test and
// which is true the instant a target changes: the setpoint jumps, the command
// saturates, and the jaw has not moved yet because it has not had time to. The
// force-position example caught exactly that -- every blocked step "settled"
// into a grasp in 0.01 s with 0.016 Nm of measured torque, i.e. gripping
// nothing at all, and the verification passed while never moving the jaw.
//
// The lead is self-timing and needs no confirmation window: it starts at zero
// because the ramp is seeded on the jaw, and only fills up if the jaw refuses to
// follow. Free travel sits near friction/kp -- 0.005 rad against a 0.055 rad
// limit at the defaults -- so the separation is an order of magnitude.
constexpr float kHoldingLeadRatio = 0.5f;
constexpr float kHoldingVelRatio  = 0.25f;   // firmware TASK_CANMOTOR_STALL_VEL_RATIO

void validate_config(const ForcePositionConfig& cfg) {
    auto finite = [](float v) { return std::isfinite(v); };
    if (!finite(cfg.close_speed_radps) || cfg.close_speed_radps <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_speed_radps must be > 0");
    }
    if (!finite(cfg.grasp_torque_nm) || cfg.grasp_torque_nm <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.grasp_torque_nm must be > 0");
    }
    if (!finite(cfg.hold_torque_limit_nm) || cfg.hold_torque_limit_nm <= 0.0f ||
        cfg.hold_torque_limit_nm > FORCE_POSITION_MAX_HOLD_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.hold_torque_limit_nm must be in (0, 1.8]");
    }
    if (!finite(cfg.motion_torque_limit_nm) || cfg.motion_torque_limit_nm <= 0.0f ||
        cfg.motion_torque_limit_nm > FORCE_POSITION_MAX_MOTION_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.motion_torque_limit_nm must be in (0, 6.0]");
    }
    if (cfg.hold_torque_limit_nm > cfg.motion_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig hold torque limit must not exceed motion torque limit");
    }
    if (cfg.grasp_torque_nm > cfg.hold_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig.grasp_torque_nm must not exceed hold_torque_limit_nm");
    }
    // No coupling rule between close_speed_radps and grasp_torque_nm any more.
    // It existed because the travel damping gain WAS grasp/speed; the ramp now
    // regulates speed and the budget split sets the gains, so a slow close is
    // just a slow close.
    if (cfg.status_timeout_ms == 0 ||
        cfg.motor_stream_hz == 0 || cfg.motor_stream_hz > 100) {
        throw std::invalid_argument(
            "ForcePositionConfig requires status_timeout_ms > 0 and "
            "motor_stream_hz in [1,100]");
    }
}

void validate_tuning(const detail::ForcePositionTuning& t,
                     const ForcePositionConfig& cfg) {
    auto finite = [](float v) { return std::isfinite(v); };
    (void)cfg;
    if (!finite(t.position_kp) || t.position_kp <= 0.0f ||
        !finite(t.position_kd) || t.position_kd < 0.0f) {
        throw std::invalid_argument(
            "ForcePositionTuning position gains require kp > 0 and kd >= 0");
    }
    if (!finite(t.travel_kd) || t.travel_kd < 0.0f) {
        throw std::invalid_argument("ForcePositionTuning.travel_kd must be >= 0");
    }
    if (!finite(t.arrival_eps_rad) || t.arrival_eps_rad < 0.0f) {
        throw std::invalid_argument(
            "ForcePositionTuning.arrival_eps_rad must be >= 0");
    }
}

void validate_target(const ForcePositionConfig& cfg,
                     float position,
                     float grasp_torque_nm) {
    if (!std::isfinite(position) || position < 0.0f || position > 1.0f) {
        throw std::invalid_argument("target position must be in [0,1]");
    }
    if (!std::isfinite(grasp_torque_nm) || grasp_torque_nm <= 0.0f ||
        grasp_torque_nm > cfg.hold_torque_limit_nm) {
        throw std::invalid_argument(
            "target grasp torque must be in (0, hold_torque_limit_nm]");
    }
}

// MotorStatusBit::Stalled (0x0004) is deliberately NOT in this mask. Holding
// an object is a stall by the motor's own definition, so treating it as a fault
// would abort every successful grasp the instant it succeeds -- an arrested jaw
// at the torque budget is the SUCCESS condition, reported as holding(). This
// mask is only for conditions that make further motion unsafe. Do not
// "complete" it.
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

const char* to_string(ForcePositionState state) noexcept {
    switch (state) {
        case ForcePositionState::Idle:            return "idle";
        case ForcePositionState::HoldingPosition: return "holding_position";
        case ForcePositionState::Closing:         return "closing";
        case ForcePositionState::HoldingForce:    return "holding_force";
        case ForcePositionState::Opening:         return "opening";
        case ForcePositionState::Fault:           return "fault";
    }
    return "unknown";
}

namespace detail {

ForcePositionPolicy::ForcePositionPolicy(GripperPosition map,
                                         ForcePositionConfig cfg)
    : ForcePositionPolicy(std::move(map), cfg, ForcePositionTuning{}) {}

ForcePositionPolicy::ForcePositionPolicy(GripperPosition map,
                                         ForcePositionConfig cfg,
                                         ForcePositionTuning tune)
    : map_(std::move(map)), cfg_(cfg), tune_(tune) {
    if (!map_.valid()) {
        throw std::invalid_argument("ForcePositionPolicy requires a valid position map");
    }
    validate_config(cfg_);
    validate_tuning(tune_, cfg_);
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
}

void ForcePositionPolicy::reset(const MotorStatusSample& sample,
                                std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault) {
        logger()->info("ForcePositionController: leaving Fault ({}), resuming at {:.4f}",
                       fault_reason_, map_.to_position(sample.actual_pos));
    }
    state_ = ForcePositionState::HoldingPosition;
    state_started_ = now;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    holding_ = false;
    arrived_ = true;
    ramp_valid_ = false;
    fault_reason_.clear();
}

void ForcePositionPolicy::release(std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_started_ = now;
    target_position_ = 1.0f;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
}

// Commands only move the setpoint. There is no motion state to disturb and no
// confirmation window to restart, which is what made the old version fragile
// for a caller that streams its target: it reset the startup guard and the
// travel history on every update, so contact could never confirm at 100 Hz.
void ForcePositionPolicy::set_target(
        const MotorStatusSample& sample,
        float target_position,
        float grasp_torque_nm,
        std::chrono::steady_clock::time_point now) {
    (void)sample;
    validate_target(cfg_, target_position, grasp_torque_nm);
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    if (std::abs(target_position - target_position_) > 1e-4f) {
        state_started_ = now;
    }
    target_position_ = target_position;
    grasp_torque_nm_ = grasp_torque_nm;
}

void ForcePositionPolicy::hold_position(const MotorStatusSample& sample) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    ramp_valid_ = false;
}

void ForcePositionPolicy::fail(std::string reason) {
    // EDGE-TRIGGERED, and it has to be: step() re-calls fail() on every frame
    // while the condition persists, so logging unconditionally would put 100
    // lines a second in the session file.
    if (state_ != ForcePositionState::Fault) {
        logger()->error("ForcePositionController: entering Fault -- {}", reason);
    }
    state_ = ForcePositionState::Fault;
    commanded_torque_nm_ = 0.0f;
    holding_ = false;
    ramp_valid_ = false;
    fault_reason_ = std::move(reason);
}

float ForcePositionPolicy::direction_open_() const noexcept {
    return map_.reverse() ? -1.0f : 1.0f;
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::zero_(
        const MotorStatusSample& sample) {
    commanded_torque_nm_ = 0.0f;
    return {sample.actual_pos, 0.0f, 0.0f, 0.0f, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::position_hold_(
        const MotorStatusSample& sample, float desired_raw,
        float torque_budget) {
    const float budget = std::clamp(torque_budget, kEpsilon,
                                    cfg_.motion_torque_limit_nm);
    const float speed = std::abs(sample.actual_vel);
    float kd = tune_.position_kd;
    if (speed > kEpsilon) {
        kd = std::min(kd, budget / speed);
    }
    const float damping = -kd * sample.actual_vel;
    const float position_budget = std::max(0.0f, budget - std::abs(damping));
    const float error_limit = position_budget / tune_.position_kp;
    const float error = std::clamp(desired_raw - sample.actual_pos,
                                   -error_limit, error_limit);
    const float target = sample.actual_pos + error;
    const float predicted = tune_.position_kp * error + damping;
    commanded_torque_nm_ = std::min(budget, std::abs(predicted));
    return {target, tune_.position_kp, kd, 0.0f, 0.0f};
}

/* Move toward target_raw along a TIME-BASED ramp.
 *
 * The ramp is what regulates speed. Commanding "current position plus the error
 * limit" instead would leave the position error permanently saturated, which is
 * a constant-torque push, not a tracked velocity: the jaw then accelerates until
 * damping balances the push, which at the measured friction works out to roughly
 * twice the requested speed. So the setpoint advances at desired_vel per unit
 * time and is anti-windup clamped to stay within the error limit of the jaw --
 * far enough ahead to pull, never so far that a blocked jaw banks up an error
 * it would have to pay back on release.
 *
 * THE BUDGET BACKS THE POSITION TERM ALONE, and that is sound because the
 * velocity feed-forward follows the ramp rather than the requested speed. A
 * blocked jaw pins the ramp against its clamp, the ramp stops, the feed-forward
 * goes to zero, and the damping term asks for nothing -- so at stall the output
 * is exactly kp*error_limit == budget. No more, which bounds the force on the
 * object; no less, which is what makes a grasp hold. kd is then free to be
 * whatever the plant is stable with, instead of being rationed against the grip.
  */
protocol::MotorImpedanceCtrl ForcePositionPolicy::travel_track_(
        const MotorStatusSample& sample, float target_raw, float desired_vel,
        float torque_budget, std::chrono::steady_clock::time_point now) {
    const float budget = std::clamp(torque_budget, kEpsilon,
                                    cfg_.motion_torque_limit_nm);
    const float kd = std::min(tune_.travel_kd, kMaxDampingGain);

    float dt = 0.0f;
    if (ramp_valid_) {
        dt = std::chrono::duration<float>(now - last_step_).count();
        // Floor at 1 ms, not at an epsilon: dt divides into the ramp velocity,
        // so a microsecond dt turns a rounding-sized ramp move into a huge
        // feed-forward. The stream is 100 Hz, so anything under 1 ms is a
        // caller stepping the policy twice on one timestamp, not real time.
        if (!std::isfinite(dt) || dt < 0.001f) { dt = 0.001f; }
        dt = std::min(dt, 0.05f);   // a stalled loop must not jump the ramp
    } else {
        // SEEDING FRAME: the ramp starts where the jaw is and no time has passed,
        // so it cannot have moved and there is nothing to bound. Command nothing
        // and let the next frame, which has a real dt, do the work.
        //
        // It has to be an early return, not just a zero dt. With dt == 0 the
        // ramp velocity is 0 by construction while the jaw may already be moving
        // at the commanded speed, so the damping term invents a kd*|vel| of
        // opposing torque -- 1.25 Nm against a 0.6 Nm budget at these gains.
        // The budget interval then does the only thing it can and drags the ramp
        // backwards to pay for it, which is exactly wrong: the jaw moving at the
        // speed we asked for is the goal, not something to brake.
        ramp_raw_ = sample.actual_pos;
        ramp_valid_ = true;
        last_step_ = now;
        commanded_torque_nm_ = 0.0f;
        return {ramp_raw_, tune_.position_kp, 0.0f, 0.0f, 0.0f};
    }
    last_step_ = now;

    // RE-SEAT A STALE RAMP BEFORE USING IT. The setpoint may legitimately lead
    // the jaw by the budget's error window and no further, so anything beyond
    // that is history: a reversed target, a pause, a jump. Left alone, the
    // budget interval below would haul the ramp back across that gap in a single
    // step and the feed-forward derived from it would spike -- measured at
    // -3.5 rad/s against a 0.5 rad/s command. Re-seat first, and carry the
    // re-seated value into ramp_prev so the correction itself reads as no motion.
    {
        const float max_lead = budget / tune_.position_kp;
        ramp_raw_ = std::clamp(ramp_raw_, sample.actual_pos - max_lead,
                               sample.actual_pos + max_lead);
    }
    const float ramp_prev = ramp_raw_;
    ramp_raw_ += desired_vel * dt;
    if (desired_vel > 0.0f) { ramp_raw_ = std::min(ramp_raw_, target_raw); }
    else                    { ramp_raw_ = std::max(ramp_raw_, target_raw); }
    // Speed limit applied to the ramp POSITION, and applied here -- before the
    // budget clamp, never after deriving the velocity from it. Capping the
    // velocity afterwards would edit one side of the equation the budget
    // interval below was solved for, and the bound would quietly stop holding.
    {
        const float step = std::abs(desired_vel) * dt;
        ramp_raw_ = std::clamp(ramp_raw_, ramp_prev - step, ramp_prev + step);
    }

    // BOUND THE RAMP BY THE BUDGET ITSELF, not by a position-error window.
    //
    // A window on the position error alone is not enough, and the way it fails
    // is easy to miss: while the ramp is still walking away from a jaw that has
    // ALREADY stopped, the damping term is asking for kd*(ramp_vel - 0) on top
    // of kp*error, and both point the same way. The two summed to 1.1 Nm
    // against a 0.6 Nm budget for the ~60 ms the window took to fill -- an 83%
    // overshoot on exactly the controller whose job is to bound force.
    //
    // So bound the thing that is actually promised. With
    //     total = kp*(ramp - pos) + kd*((ramp - prev)/dt - vel)
    // total is affine in ramp, so |total| <= budget is a closed-form interval:
    //     total = a*ramp - b,   a = kp + kd/dt,
    //                           b = kp*pos + kd*prev/dt + kd*vel
    // and the admissible ramp is [(b - budget)/a, (b + budget)/a]. Clamping to
    // that makes the command sit exactly on the budget while the ramp converges
    // and exactly on it once the ramp is pinned -- tight at every step, not just
    // in the steady state.
    {
        const float a = tune_.position_kp + kd / dt;
        const float b = tune_.position_kp * sample.actual_pos
                      + kd * ramp_prev / dt
                      + kd * sample.actual_vel;
        if (a > kEpsilon) {
            ramp_raw_ = std::clamp(ramp_raw_, (b - budget) / a, (b + budget) / a);
        }
    }

    // FEED FORWARD THE RAMP'S OWN VELOCITY, not the speed that was requested.
    // They are the same while the jaw is free, and they differ exactly when it
    // matters: a blocked jaw pins the ramp against the budget clamp, so the ramp
    // stops, the feed-forward goes to zero, and the whole command migrates into
    // the position term. That is what frees kd from the grasp force.
    //
    // Derived from the FINAL ramp and used unmodified in both the frame and the
    // prediction. Do not post-clamp it to the requested speed: the budget
    // interval above was solved for exactly this velocity, so altering it
    // afterwards is altering one side of an equation the bound depends on, and
    // the bound silently stops holding. The interval already limits how far the
    // ramp can move in one step, which is the same protection that clamp was
    // reaching for -- and it also subsumes the old
    // "kd <= motion_limit/|vel_err|" guard, because the interval bounds the
    // TOTAL, damping term included, whatever the jaw is doing.
    const float ramp_vel = (ramp_raw_ - ramp_prev) / dt;

    const float vel_err = ramp_vel - sample.actual_vel;
    const float error = ramp_raw_ - sample.actual_pos;
    // Report the honest prediction, bounded only by the motion limit. Clamping
    // it to the grasp budget would hide the damping term's legitimate excursions.
    commanded_torque_nm_ = std::min(
        cfg_.motion_torque_limit_nm,
        std::abs(tune_.position_kp * error + kd * vel_err));
    return {ramp_raw_, tune_.position_kp, kd, 0.0f, ramp_vel};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now) {
    if (!std::isfinite(sample.actual_pos) || !std::isfinite(sample.actual_vel) ||
        !std::isfinite(sample.actual_torque)) {
        fail("non-finite motor status");
    } else if (has_serious_fault(sample.status)) {
        fail("motor status reports a fault");
    } else if (std::abs(sample.actual_torque) >
               cfg_.motion_torque_limit_nm + 1e-4f) {
        fail("measured torque exceeded motion_torque_limit_nm");
    }

    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) {
        arrived_ = false;
        holding_ = false;
        return zero_(sample);
    }

    const float target_raw = map_.to_rad(target_position_);
    const float to_target = target_raw - sample.actual_pos;
    const float open_position = map_.to_position(sample.actual_pos);

    hold_raw_ = sample.actual_pos;          // where the jaw actually is
    arrived_ = std::abs(to_target) <= tune_.arrival_eps_rad;

    protocol::MotorImpedanceCtrl cmd;
    if (arrived_) {
        ramp_valid_ = false;                // next move restarts the ramp here
        cmd = position_hold_(sample, target_raw, grasp_torque_nm_);
    } else {
        const float dir = (to_target > 0.0f) ? 1.0f : -1.0f;
        cmd = travel_track_(sample, target_raw, dir * cfg_.close_speed_radps,
                            grasp_torque_nm_, now);
    }

    // OBSERVATIONS, not decisions. "Holding" is the clamp binding while the jaw
    // is short of its target -- we are asking for every bit of torque we are
    // allowed to ask for and still not getting there, which is what a grasp is.
    // Same signal ImpedanceController reports as stall_clamped_, and for the
    // same reason: it is reachable by construction, unlike a threshold on
    // measured torque, which never reaches the command at stall.
    const float max_lead = grasp_torque_nm_ / tune_.position_kp;
    const float lead = std::abs(ramp_raw_ - sample.actual_pos);
    holding_ = !arrived_ && ramp_valid_ && max_lead > kEpsilon &&
               lead >= max_lead * kHoldingLeadRatio &&
               std::abs(sample.actual_vel) <=
                   cfg_.close_speed_radps * kHoldingVelRatio;

    state_ = holding_            ? ForcePositionState::HoldingForce
           : arrived_            ? ForcePositionState::HoldingPosition
           : (open_position > target_position_) ? ForcePositionState::Closing
                                                : ForcePositionState::Opening;
    return cmd;
}

}  // namespace detail

ForcePositionController::ForcePositionController(FollowerGripper& gripper)
    : ForcePositionController(gripper, ForcePositionConfig{}) {}

ForcePositionController::ForcePositionController(FollowerGripper& gripper,
                                                 ForcePositionConfig cfg)
    : g_(gripper), cfg_(cfg) {
    validate_config_();
}

ForcePositionController::~ForcePositionController() {
    stop();
}

void ForcePositionController::validate_config_() const {
    validate_config(cfg_);
}

void ForcePositionController::start_motor_stream_() {
    if (g_.is_streaming()) {
        stream_ours_ = false;
        return;
    }
    g_.start_streaming(cfg_.motor_stream_hz);
    stream_ours_ = true;
}

void ForcePositionController::stop_motor_stream_() {
    if (!stream_ours_) return;
    g_.stop_streaming();
    stream_ours_ = false;
}

void ForcePositionController::start() {
    if (running()) return;
    validate_config_();
    map_ = g_.position_map();

    const float device_limit = g_.motor().get_startup_limit_torque();
    if (!std::isfinite(device_limit) || device_limit <= 0.0f ||
        device_limit > cfg_.motion_torque_limit_nm + 1e-4f) {
        throw std::runtime_error(
            "ForcePositionController: stored motor startup torque limit is " +
            std::to_string(device_limit) + " Nm, but motion_torque_limit_nm is " +
            std::to_string(cfg_.motion_torque_limit_nm) +
            " Nm; call set_startup_limit_torque(), power-cycle the gripper, "
            "and verify the value before enabling motion");
    }
    if (device_limit + 1e-4f < cfg_.grasp_torque_nm) {
        logger()->warn(
            "ForcePositionController: device torque limit {:.3f} Nm is below "
            "requested grasp torque {:.3f} Nm; the motor limit wins",
            device_limit, cfg_.grasp_torque_nm);
    } else if (device_limit + 1e-4f < cfg_.motion_torque_limit_nm) {
        logger()->warn(
            "ForcePositionController: device torque limit {:.3f} Nm is below "
            "motion_torque_limit_nm {:.3f} Nm; the device limit wins",
            device_limit, cfg_.motion_torque_limit_nm);
    }

    // Cross-check against the firmware's own stall-detection numbers. The
    // power-on auto-calibration solves the same problem this controller does
    // (close until the jaw is blocked) and its close_stall_torque_nm is the
    // per-device value that has been tuned to survive the jaw's restoring
    // torque. It is advisory here, not a substitute for the caller's force
    // choice -- but a grasp torque far below it will not saturate against the
    // mechanism, and one far above it grips harder than the firmware ever
    // pushes during calibration.
    try {
        const auto ac = g_.get_auto_cal_config();
        if (std::isfinite(ac.close_stall_torque_nm) &&
            ac.close_stall_torque_nm > 0.0f) {
            if (cfg_.grasp_torque_nm < ac.close_stall_torque_nm * 0.5f) {
                logger()->warn(
                    "ForcePositionController: grasp torque {:.3f} Nm is well "
                    "below the firmware's own close stall torque {:.3f} Nm; "
                    "the jaw may not develop enough torque to confirm contact",
                    cfg_.grasp_torque_nm, ac.close_stall_torque_nm);
            }
            logger()->debug(
                "ForcePositionController: firmware auto-cal close={:.3f}Nm "
                "open={:.3f}Nm close_speed={:.3f}rad/s stall_hold={}ms",
                ac.close_stall_torque_nm, ac.open_stall_torque_nm,
                ac.close_speed_rad_s, ac.stall_hold_ms);
        }
    } catch (const std::exception& e) {
        // Older firmware, or a device that has never stored the record. The
        // controller does not depend on it.
        logger()->debug("ForcePositionController: auto-cal config unavailable ({})",
                        e.what());
    }

    // THE DEVICE IS THE AUTHORITY ON ITS OWN RATINGS. The MOTOR_*_TORQUE_NM
    // constants this config defaults to are EL05 numbers compiled into the SDK;
    // on an RS00 (5.0 rated / 14.0 peak) they are simply the wrong motor's
    // limits, and validate_config() would have been enforcing them happily.
    // Read the firmware's motor_spec table and check against that instead.
    //
    // Advisory, not fatal: a config that is too CONSERVATIVE for the installed
    // motor is safe, just weaker than it could be, and refusing to start would
    // strand a caller whose numbers are merely stale. Exceeding the ratings is
    // what gets a warning. Skipped silently on firmware without 0x56.
    try {
        const auto spec = g_.motor().get_spec();
        const std::string model(spec.name,
                                ::strnlen(spec.name, sizeof(spec.name)));
        if (spec.t_max_nm > 0.0f &&
            cfg_.motion_torque_limit_nm > spec.t_max_nm + 1e-4f) {
            logger()->warn(
                "ForcePositionController: motion_torque_limit_nm {:.2f} Nm "
                "exceeds the installed {} peak rating {:.2f} Nm",
                cfg_.motion_torque_limit_nm, model, spec.t_max_nm);
        }
        if (spec.rated_torque_nm > 0.0f &&
            cfg_.hold_torque_limit_nm > spec.rated_torque_nm + 1e-4f) {
            logger()->warn(
                "ForcePositionController: hold_torque_limit_nm {:.2f} Nm "
                "exceeds the installed {} rated torque {:.2f} Nm",
                cfg_.hold_torque_limit_nm, model, spec.rated_torque_nm);
        }
        if (spec.stall_cont_torque_nm > 0.0f &&
            cfg_.grasp_torque_nm > spec.stall_cont_torque_nm + 1e-4f) {
            logger()->warn(
                "ForcePositionController: grasp {:.2f} Nm is above the {} "
                "continuous stall rating {:.2f} Nm -- an indefinite hold will "
                "heat past what this motor can sustain",
                cfg_.grasp_torque_nm, model, spec.stall_cont_torque_nm);
        }
        logger()->info("ForcePositionController: motor {} rated={:.2f} peak={:.2f} "
                       "cont_stall={:.2f} Nm",
                       model, spec.rated_torque_nm, spec.t_max_nm,
                       spec.stall_cont_torque_nm);
    } catch (const std::exception& e) {
        logger()->debug("ForcePositionController: motor spec unavailable ({}) "
                        "-- falling back to the compiled-in EL05 ratings", e.what());
    }

    // Cross-check the grasp against the firmware's own SUSTAINED-torque budget.
    //
    // HoldingForce is indefinite by construction -- that is what it is for --
    // so grasp_torque_nm is a CONTINUOUS rating request, not a transient one,
    // and the envelope's cont_torque_nm is the device's own answer for what it
    // can hold indefinitely. Exceeding it is not primarily a thermal problem:
    // the motor's undervoltage protection is prompt, and a sustained draw can
    // brown the board out and take the USB link with it, which surfaces to the
    // host as SerialBus::write: Input/output error rather than as anything
    // resembling a torque fault. Reported from the field at a 1.5 Nm grasp
    // against a 1.6 Nm continuous envelope, on a gripper stood upright so the
    // jaw's own weight added to the hold.
    try {
        const auto env = g_.get_envelope();
        const bool enforced =
            (env.flags & protocol::GripperEnvelopeFlag::Enforce) != 0;
        if (std::isfinite(env.cont_torque_nm) && env.cont_torque_nm > 0.0f) {
            if (cfg_.grasp_torque_nm > env.cont_torque_nm) {
                logger()->warn(
                    "ForcePositionController: grasp torque {:.3f} Nm exceeds the "
                    "device's continuous envelope {:.3f} Nm, and a force hold is "
                    "indefinite. {}",
                    cfg_.grasp_torque_nm, env.cont_torque_nm,
                    enforced ? "The firmware will derate it, so the actual grip "
                               "will not be what was asked for."
                             : "The envelope is NOT enforced, so nothing below "
                               "this host limits the sustained draw.");
            } else if (cfg_.grasp_torque_nm > env.cont_torque_nm * 0.9f &&
                       cfg_.grasp_torque_nm > ForcePositionConfig{}.grasp_torque_nm) {
                // Only when the caller asked for MORE than the default. The
                // default IS the continuous rating, so testing the 90% band
                // alone would fire on every default-configured session and
                // teach people to tune warnings out.
                logger()->warn(
                    "ForcePositionController: grasp torque {:.3f} Nm is within "
                    "10% of the continuous envelope {:.3f} Nm. A long hold at "
                    "this level has browned out the 24 V rail and dropped the "
                    "USB link; leave headroom if the grasp is load-bearing",
                    cfg_.grasp_torque_nm, env.cont_torque_nm);
            }
        }
        if (!enforced) {
            logger()->warn(
                "ForcePositionController: the motion envelope is not enforced, "
                "so the firmware's I2t derate and temperature wall are inactive "
                "and an indefinite force hold has no protection below this host");
        }
    } catch (const std::exception& e) {
        logger()->debug("ForcePositionController: envelope unavailable ({})",
                        e.what());
    }

    const MotorStatusSample initial = g_.motor().read_status();
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        policy_ = std::make_unique<detail::ForcePositionPolicy>(map_, cfg_);
        policy_->reset(initial, now);
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
        "ForcePositionController started: speed={:.3f}rad/s grasp={:.3f}Nm "
        "hold_limit={:.3f}Nm motion_limit={:.3f}Nm device_limit={:.3f}Nm",
        cfg_.close_speed_radps, cfg_.grasp_torque_nm, cfg_.hold_torque_limit_nm,
        cfg_.motion_torque_limit_nm, device_limit_nm_);
}

void ForcePositionController::stop() {
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
        logger()->warn("ForcePositionController: motor disable on stop failed: {}", e.what());
    }
    if (sub_active_) {
        g_.motor().off(sub_);
        sub_active_ = false;
    }
    stop_motor_stream_();
}

void ForcePositionController::on_status_(const MotorStatusSample& sample) {
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

void ForcePositionController::request_step_() {
    step_requested_ = true;
    cv_.notify_one();
}

void ForcePositionController::set_target(float position) {
    set_target(position, cfg_.grasp_torque_nm);
}

// Caller commands are QUEUED, not applied here.
//
// Applying them on the caller's thread also submitted on the caller's thread,
// at whatever moment that thread happened to call in. The MCU drops bytes out
// of the middle of a status frame it is transmitting if host->MCU traffic
// overlaps it, so an off-phase write costs a telemetry frame. Every submit now
// happens in run_() on the status-frame doorbell, inside the ~9.8 ms the MCU
// is known to be idle -- the same phase discipline ControlLoop's StreamLocked
// uses. The cost is that a command takes effect on the next status frame
// instead of instantly: at motor_stream_hz = 100 that is under 10 ms.
//
// Validation stays eager. Deferring it would turn a caller's out-of-range
// argument into a silent no-op on a background thread instead of the
// std::invalid_argument it has always been.
void ForcePositionController::set_target(float position, float grasp_torque_nm) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::set_target called before start");
    }
    validate_target(cfg_, position, grasp_torque_nm);
    pending_ = PendingCommand::SetTarget;
    pending_position_ = position;
    pending_grasp_torque_nm_ = grasp_torque_nm;
    command_woke_ = true;
    cv_.notify_one();
}

void ForcePositionController::release() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::release called before start");
    }
    pending_ = PendingCommand::Release;
    command_woke_ = true;
    cv_.notify_one();
}

void ForcePositionController::hold_position() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::hold_position called before start");
    }
    pending_ = PendingCommand::HoldPosition;
    command_woke_ = true;
    cv_.notify_one();
}

void ForcePositionController::reset() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::reset called before start");
    }
    policy_->reset(latest_, std::chrono::steady_clock::now());
    // The cached sample may still carry the fault bit that was true immediately
    // before Motor::clear_fault() ACKed. Wait for the next streamed status rather
    // than re-faulting the freshly reset policy on stale evidence.
    step_requested_ = false;
}

ForcePositionState ForcePositionController::state() const {
    std::lock_guard<std::mutex> lk(mu_);
    return policy_ ? policy_->state() : ForcePositionState::Idle;
}

ForcePositionSnapshot ForcePositionController::snapshot() const {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mu_);
    ForcePositionSnapshot out;
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
        out.hold_position = policy_->hold_position();
        out.grasp_torque_nm = policy_->grasp_torque_nm();
        out.commanded_torque_nm = policy_->commanded_torque_nm();
        out.hold_torque_limit_nm = cfg_.hold_torque_limit_nm;
        out.motion_torque_limit_nm = cfg_.motion_torque_limit_nm;
        out.holding = policy_->holding();
        out.arrived = policy_->arrived();
        out.fault_reason = policy_->fault_reason();
    }
    return out;
}

void ForcePositionController::run_() {
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

            // Staleness is checked unconditionally. Doing it only in the
            // !step_requested_ branch let set_target()/release()/hold_position()
            // walk straight past it: those set step_requested_, so a caller
            // command issued after the status stream died would be computed
            // from a stale sample and put on the wire as real motion. Now a
            // dead stream faults first, and step() answers with zero torque
            // whatever the caller just asked for.
            const bool stale = now - latest_time_ >= timeout;
            if (stale) {
                // Invalidating the observation is INDEPENDENT of why the policy
                // is faulted. The last sample is history the moment frames stop
                // arriving, whatever else went wrong.
                //
                // This used to be gated on `state != Fault` along with the
                // fail() below, which meant it only ran when staleness was what
                // CAUSED the fault -- and that is never the case when the link
                // actually drops. A dead USB link makes the next submit throw
                // (SerialBus::write: Input/output error), which faults the
                // policy within one status period, well inside status_timeout_ms
                // -- so by the time the stream was judged stale the policy was
                // already in Fault, this was skipped, and the observation stayed
                // "valid" indefinitely. Reported from the field as a console
                // showing a plausible position and 1.540 Nm of torque beside
                // age=10553.7ms. The one case the flag exists for was the one
                // case it missed.
                observation_.valid = false;
                if (policy_->state() != ForcePositionState::Fault) {
                    // Only the FIRST cause is worth recording. A submit failure
                    // or a motor fault says what went wrong; "stream stale" is
                    // just what happens next, and overwriting with it would
                    // throw away the diagnosis.
                    policy_->fail("motor status stream stale");
                    step_requested_ = true;   // push one zero-torque command out
                }
            }
            // A caller command on a dead stream is answered immediately with the
            // zero-torque frame step() produces in Fault. There is no phase to
            // respect when no status frames are arriving, and the fault may
            // already be latched from an earlier wake -- in which case the
            // block above does not fire and the command would be swallowed with
            // nothing on the wire and nothing for the caller to see.
            if (woke_by_command && stale) step_requested_ = true;

            if (!step_requested_) continue;
            step_requested_ = false;

            // Apply whatever the caller queued, now that we are on the status
            // doorbell and about to submit inside the MCU's idle window. A dead
            // stream drops it: policy_ is in Fault, where every command is a
            // no-op anyway, and holding it back would fire it late on recovery.
            if (pending_ != PendingCommand::None) {
                if (!stale) {
                    switch (pending_) {
                        case PendingCommand::SetTarget:
                            policy_->set_target(latest_, pending_position_,
                                                pending_grasp_torque_nm_, now);
                            break;
                        case PendingCommand::Release:
                            policy_->release(now);
                            break;
                        case PendingCommand::HoldPosition:
                            policy_->hold_position(latest_);
                            break;
                        case PendingCommand::None:
                            break;
                    }
                }
                pending_ = PendingCommand::None;
            }

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

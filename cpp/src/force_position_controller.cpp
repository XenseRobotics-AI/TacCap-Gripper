// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/force_position_controller.hpp>
#include <taccap/control_runtime.hpp>

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xense::taccap {

namespace {

constexpr float kEpsilon = 1e-5f;
constexpr float kPositionTolerance = 0.01f;
constexpr float kSettledVelocity = 0.05f;

void validate_motion(float position, float torque_limit_nm, float speed_rad_s,
                     float maximum_torque_nm) {
    if (!std::isfinite(position) || position < 0.0f || position > 1.0f ||
        !std::isfinite(torque_limit_nm) || torque_limit_nm <= 0.0f ||
        torque_limit_nm > maximum_torque_nm || !std::isfinite(speed_rad_s) ||
        speed_rad_s <= 0.0f || speed_rad_s > 50.0f) {
        throw std::invalid_argument("position requires [0,1], valid torque budget and speed in (0,50]");
    }
}

void validate_config(const ForcePositionConfig& cfg) {
    auto finite = [](float v) { return std::isfinite(v); };
    if (!finite(cfg.close_position) || cfg.close_position < 0.0f ||
        cfg.close_position > 1.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_position must be in [0,1]");
    }
    if (!finite(cfg.close_speed_radps) || cfg.close_speed_radps <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_speed_radps must be > 0");
    }
    if (!finite(cfg.grasp_torque_nm) || cfg.grasp_torque_nm <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.grasp_torque_nm must be > 0");
    }
    if (!finite(cfg.hold_torque_limit_nm) || cfg.hold_torque_limit_nm <= 0.0f ||
        cfg.hold_torque_limit_nm > FORCE_POSITION_MAX_HOLD_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.hold_torque_limit_nm must be in (0, 1.2]");
    }
    if (!finite(cfg.motion_torque_limit_nm) || cfg.motion_torque_limit_nm <= 0.0f ||
        cfg.motion_torque_limit_nm > FORCE_POSITION_MAX_MOTION_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.motion_torque_limit_nm must be in (0, 5.5]");
    }
    if (cfg.hold_torque_limit_nm > cfg.motion_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig hold torque limit must not exceed motion torque limit");
    }
    if (cfg.grasp_torque_nm > cfg.hold_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig.grasp_torque_nm must not exceed hold_torque_limit_nm");
    }
    if (!finite(cfg.contact_torque_nm) || cfg.contact_torque_nm <= 0.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_torque_nm must be > 0");
    }
    // A floor above the torque the closing command can produce is unreachable:
    // the jaw would push at grasp_torque_nm forever and never latch, which is
    // the stall this controller exists to prevent. Feedback torque runs well
    // under the command at stall, so the real headroom is smaller than this
    // check implies -- see the start() warning.
    if (cfg.contact_torque_nm > cfg.grasp_torque_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_torque_nm must not exceed "
            "grasp_torque_nm -- the closing command cannot reach it");
    }
    if (!finite(cfg.contact_vel_radps) || cfg.contact_vel_radps <= 0.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_vel_radps must be > 0");
    }
    if (!finite(cfg.contact_vel_ratio) || cfg.contact_vel_ratio <= 0.0f ||
        cfg.contact_vel_ratio > 1.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_vel_ratio must be in (0, 1]");
    }
    if (!finite(cfg.contact_moved_rad) || cfg.contact_moved_rad < 0.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_moved_rad must be >= 0");
    }
    if (!finite(cfg.position_kp) || cfg.position_kp <= 0.0f ||
        cfg.position_kp > 500.0f || !finite(cfg.position_kd) || cfg.position_kd < 0.0f || cfg.position_kd > 5.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig position gains require kp > 0 and kd >= 0");
    }
    if (!finite(cfg.brake_distance_rad) || cfg.brake_distance_rad < 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.brake_distance_rad must be >= 0");
    }
    if (!finite(cfg.close_compensation_rad) || cfg.close_compensation_rad < 0.0f ||
        cfg.close_compensation_rad > 0.05f)
        throw std::invalid_argument("close_compensation_rad must be in [0,0.05]");
    if (cfg.contact_samples == 0 || cfg.status_timeout_ms == 0 ||
        cfg.motor_stream_hz == 0 || cfg.motor_stream_hz > 100) {
        throw std::invalid_argument(
            "ForcePositionConfig requires contact_samples/status_timeout_ms > 0 "
            "and motor_stream_hz in [1,100]");
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
// would abort every successful grasp the instant it succeeds. Contact detection
// (contact_candidate_) is what interprets an arrested jaw here; this mask is
// only for conditions that make further motion unsafe. Do not "complete" it.
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
        case ForcePositionState::MovingPosition:  return "moving_position";
        case ForcePositionState::Impedance:       return "impedance";
        case ForcePositionState::Blocked:         return "blocked";
    }
    return "unknown";
}

namespace detail {

ForcePositionPolicy::ForcePositionPolicy(GripperPosition map,
                                         ForcePositionConfig cfg)
    : map_(std::move(map)), cfg_(cfg) {
    if (!map_.valid()) {
        throw std::invalid_argument("ForcePositionPolicy requires a valid position map");
    }
    validate_config(cfg_);
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
}

void ForcePositionPolicy::reset(const MotorStatusSample& sample,
                                std::chrono::steady_clock::time_point now) {
    state_ = ForcePositionState::HoldingPosition;
    state_started_ = now;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    compensate_close_ = false;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);
    fault_reason_.clear();
    control_mode_ = ForcePositionMode::Position;
    active_torque_limit_nm_ = cfg_.grasp_torque_nm;
    speed_rad_s_ = cfg_.close_speed_radps;
    reference_raw_ = sample.actual_pos;
    reference_time_ = now;
}

void ForcePositionPolicy::release(std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::Opening;
    state_started_ = now;
    target_position_ = 1.0f;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    control_mode_ = ForcePositionMode::Grasp;
    active_torque_limit_nm_ = cfg_.grasp_torque_nm;
    speed_rad_s_ = cfg_.close_speed_radps;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
}

void ForcePositionPolicy::begin_motion_(const MotorStatusSample& sample) {
    motion_start_raw_ = sample.actual_pos;
    peak_abs_vel_ = std::abs(sample.actual_vel);
}

void ForcePositionPolicy::track_motion_(const MotorStatusSample& sample) noexcept {
    peak_abs_vel_ = std::max(peak_abs_vel_, std::abs(sample.actual_vel));
}

void ForcePositionPolicy::set_target(
        const MotorStatusSample& sample,
        float target_position,
        float grasp_torque_nm,
        std::chrono::steady_clock::time_point now) {
    validate_target(cfg_, target_position, grasp_torque_nm);
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;

    constexpr float kTargetTolerance = 1e-4f;
    const float current_position = map_.to_position(sample.actual_pos);
    target_position_ = target_position;
    grasp_torque_nm_ = grasp_torque_nm;
    control_mode_ = ForcePositionMode::Grasp;
    active_torque_limit_nm_ = grasp_torque_nm;
    speed_rad_s_ = cfg_.close_speed_radps;
    state_started_ = now;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);

    if (current_position > target_position + kTargetTolerance) {
        state_ = ForcePositionState::Closing;
    } else if (current_position < target_position - kTargetTolerance) {
        state_ = ForcePositionState::Opening;
    } else {
        state_ = ForcePositionState::HoldingPosition;
        hold_raw_ = map_.to_rad(target_position);
    }
}

void ForcePositionPolicy::hold_position(const MotorStatusSample& sample) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::HoldingPosition;
    control_mode_ = ForcePositionMode::Position;
    active_torque_limit_nm_ = std::min(active_torque_limit_nm_, cfg_.hold_torque_limit_nm);
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    compensate_close_ = false;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);
}

void ForcePositionPolicy::require_active_() const {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle)
        throw std::logic_error("controller is idle or faulted; stop, clear fault and restart");
}

void ForcePositionPolicy::set_position_target(
        const MotorStatusSample& sample, float position, float torque_limit_nm,
        float speed_rad_s, std::chrono::steady_clock::time_point now) {
    validate_motion(position, torque_limit_nm, speed_rad_s, cfg_.motion_torque_limit_nm);
    require_active_();
    // Preserve the requested opening under load. Only a new target or mode can
    // cancel that intent; inferred contact must not remove gripping torque.
    compensate_close_ = true;
    const float desired = map_.to_rad(position) - direction_open_() *
        cfg_.close_compensation_rad * (1.0f - position);
    const bool continuing = control_mode_ == ForcePositionMode::Position &&
        state_ == ForcePositionState::MovingPosition &&
        (desired - sample.actual_pos) * (position_target_rad() - sample.actual_pos) > 0.0f;
    if (!continuing) {
        begin_motion_(sample);
        state_started_ = now;
        reference_raw_ = sample.actual_pos;
        reference_time_ = now;
        contact_count_ = 0;
    }
    target_position_ = position;
    active_torque_limit_nm_ = torque_limit_nm;
    speed_rad_s_ = speed_rad_s;
    control_mode_ = ForcePositionMode::Position;
    state_ = ForcePositionState::MovingPosition;
    commanded_torque_nm_ = 0.0f;
}

void ForcePositionPolicy::set_grasp_target(
        const MotorStatusSample& sample, float position, float torque_nm,
        float speed_rad_s, std::chrono::steady_clock::time_point now) {
    validate_motion(position, torque_nm, speed_rad_s, cfg_.hold_torque_limit_nm);
    require_active_();
    set_target(sample, position, torque_nm, now);
    speed_rad_s_ = speed_rad_s;
}

void ForcePositionPolicy::set_impedance_target(
        const MotorStatusSample&, float position, float kp, float kd,
        float feedforward_torque_nm, float velocity_rad_s, float torque_limit_nm,
        std::chrono::steady_clock::time_point now) {
    validate_motion(position, torque_limit_nm, 1.0f, cfg_.motion_torque_limit_nm);
    if (!std::isfinite(kp) || kp < 0.0f || kp > 500.0f ||
        !std::isfinite(kd) || kd < 0.0f || kd > 5.0f ||
        !std::isfinite(feedforward_torque_nm) ||
        std::abs(feedforward_torque_nm) > torque_limit_nm ||
        !std::isfinite(velocity_rad_s) || std::abs(velocity_rad_s) > 50.0f)
        throw std::invalid_argument("invalid impedance gains, feed-forward torque or velocity");
    require_active_();
    impedance_target_ = {map_.to_rad(position), kp, kd, feedforward_torque_nm, velocity_rad_s};
    target_position_ = position;
    active_torque_limit_nm_ = torque_limit_nm;
    speed_rad_s_ = std::abs(velocity_rad_s);
    control_mode_ = ForcePositionMode::Impedance;
    state_ = ForcePositionState::Impedance;
    state_started_ = now;
    contact_count_ = 0;
    commanded_torque_nm_ = 0.0f;
}

void ForcePositionPolicy::fail(std::string reason) {
    state_ = ForcePositionState::Fault;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    fault_reason_ = std::move(reason);
}

float ForcePositionPolicy::direction_open_() const noexcept {
    return map_.reverse() ? -1.0f : 1.0f;
}

float ForcePositionPolicy::contact_threshold_() const noexcept {
    // A flat floor. Deliberately NOT derived from the commanded torque -- see
    // the header: the feedback never reaches the command at stall, so any
    // cap-derived threshold is unreachable under a kd-only MIT frame.
    return cfg_.contact_torque_nm;
}

bool ForcePositionPolicy::contact_candidate_(const MotorStatusSample& sample,
                                             float motion_sign) const noexcept {
    // Torque saturation is necessary but NEVER sufficient -- see the header.
    // The jaw's own restoring torque climbs smoothly through an empty close, so
    // torque alone would latch part way down with nothing in the jaws.
    if (std::abs(sample.actual_torque) < contact_threshold_()) return false;

    // Firmware rule "torque": arrested outright, no travel history needed, so a
    // jaw that starts already against the object still latches. Measured
    // margin on 1.1.5 hardware: free travel never drops under 0.183 rad/s.
    if (std::abs(sample.actual_vel) <= cfg_.contact_vel_radps) return true;

    // Firmware rule "velocity": once the jaw has demonstrably moved, motion
    // collapsing to a fraction of the commanded speed is enough. has_moved is
    // what keeps a slow-but-free close from qualifying.
    const float target_speed = speed_rad_s_;
    const bool has_moved =
        peak_abs_vel_ >= target_speed * 0.35f &&
        std::abs(sample.actual_pos - motion_start_raw_) >= cfg_.contact_moved_rad;
    if (!has_moved) return false;
    return sample.actual_vel * motion_sign <= target_speed * cfg_.contact_vel_ratio;
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::zero_(
        const MotorStatusSample& sample) {
    commanded_torque_nm_ = 0.0f;
    return {sample.actual_pos, 0.0f, 0.0f, 0.0f, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::force_hold_() {
    const float hold_torque = std::min(grasp_torque_nm_, cfg_.hold_torque_limit_nm);
    const float signed_torque = -direction_open_() * hold_torque;
    commanded_torque_nm_ = std::abs(signed_torque);
    // kp=kd=0 is the essential safety property: position error cannot add to
    // the requested holding torque after contact.
    return {hold_raw_, 0.0f, 0.0f, signed_torque, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::position_hold_(
        const MotorStatusSample& sample, float desired_raw,
        float torque_budget, float desired_velocity) {
    // Bound the instantaneous PD request before it reaches the motor; motor
    // 0x700B remains the independent hardware backstop. Force holding uses the
    // separate, lower hold limit.
    const float budget = std::clamp(torque_budget, kEpsilon,
                                    cfg_.motion_torque_limit_nm);
    const float velocity_error = desired_velocity - sample.actual_vel;
    const float speed = std::abs(velocity_error);
    float kd = cfg_.position_kd;
    if (speed > kEpsilon) {
        kd = std::min(kd, budget / speed);
    }
    const float damping = kd * velocity_error;
    const float position_budget = std::max(0.0f, budget - std::abs(damping));
    const float error_limit = position_budget / cfg_.position_kp;
    const float error = std::clamp(desired_raw - sample.actual_pos,
                                   -error_limit, error_limit);
    const float target = sample.actual_pos + error;
    const float predicted = cfg_.position_kp * error + damping;
    commanded_torque_nm_ = std::min(budget, std::abs(predicted));
    return {target, cfg_.position_kp, kd, 0.0f, desired_velocity};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::impedance_(
        const MotorStatusSample& sample) {
    auto c = impedance_target_;
    float budget = active_torque_limit_nm_ - std::abs(c.target_torque);
    const float velocity_error = c.vel - sample.actual_vel;
    if (std::abs(velocity_error) > kEpsilon)
        c.kd = std::min(c.kd, budget / std::abs(velocity_error));
    const float damping = c.kd * velocity_error;
    budget = std::max(0.0f, budget - std::abs(damping));
    const float error = c.kp > 0.0f ? std::clamp(c.target_pos - sample.actual_pos,
        -budget / c.kp, budget / c.kp) : 0.0f;
    c.target_pos = sample.actual_pos + error;
    commanded_torque_nm_ = std::abs(c.kp * error + damping + c.target_torque);
    return c;
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now, bool new_sample) {
    if (!std::isfinite(sample.actual_pos) || !std::isfinite(sample.actual_vel) ||
        !std::isfinite(sample.actual_torque) || !std::isfinite(sample.motor_temp_c)) {
        fail("non-finite motor status");
    } else if (has_serious_fault(sample.status)) {
        fail("motor status reports a fault");
    } else if (std::abs(sample.actual_torque) >
               cfg_.motion_torque_limit_nm + 1e-4f) {
        fail("measured torque exceeded motion_torque_limit_nm");
    }

    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) {
        return zero_(sample);
    }

    const float open_position = map_.to_position(sample.actual_pos);

    if (state_ == ForcePositionState::Impedance) return impedance_(sample);

    if (state_ == ForcePositionState::MovingPosition) {
        track_motion_(sample);
        const float desired = position_target_rad();
        const bool near = std::abs(sample.actual_pos - desired) <=
            kPositionTolerance * (map_.max_open_rad() - map_.min_open_rad());
        const bool settled = std::abs(sample.actual_vel) <= kSettledVelocity;
        if (near && settled) {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = desired;
            contact_count_ = 0;
            return position_hold_(sample, desired, active_torque_limit_nm_);
        }
        // Limit reference motion across both frequent updates and scheduling gaps.
        // This is a target ramp; torque and firmware limits may make actual motion slower.
        const float dt = std::clamp(std::chrono::duration<float>(now - reference_time_).count(), 0.0f, 0.05f);
        const float increment = std::clamp(desired - reference_raw_, -speed_rad_s_ * dt, speed_rad_s_ * dt);
        reference_raw_ += increment;
        reference_time_ = now;
        // Track the reference velocity instead of damping all motion toward zero.
        // Position and velocity terms still share the same absolute torque budget.
        float velocity = dt > 0.0f ? increment / dt : 0.0f;
        const float opening = sample.actual_pos * direction_open_();
        const float distance = velocity * direction_open_() > 0.0f ?
            map_.max_open_rad() - opening : opening - map_.min_open_rad();
        // Leave position torque available near calibrated stops, where firmware
        // attenuates velocity commands. Compensation crosses zero using PD only.
        velocity *= std::clamp(distance / 0.10f, 0.0f, 1.0f);
        return position_hold_(sample, reference_raw_, active_torque_limit_nm_, velocity);
    }

    if (state_ == ForcePositionState::Blocked)
        return position_hold_(sample, hold_raw_, active_torque_limit_nm_);

    if (state_ == ForcePositionState::Closing) {
        track_motion_(sample);
        const auto guard = std::chrono::milliseconds(cfg_.startup_guard_ms);
        const bool guard_done = now - state_started_ >= guard;
        // motion_sign: +1 along the closing direction, matching the firmware's
        // s_auto_close_sign.
        const bool contact = contact_candidate_(sample, -direction_open_());
        if (new_sample) {
            if (guard_done && contact) ++contact_count_;
            else                       contact_count_ = 0;
        }

        if (contact_count_ >= cfg_.contact_samples) {
            state_ = ForcePositionState::HoldingForce;
            hold_raw_ = sample.actual_pos;
            compensate_close_ = false;
            return force_hold_();
        }

        if (open_position <= target_position_ + 1e-4f) {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position_);
            return position_hold_(sample, hold_raw_, active_torque_limit_nm_);
        }

        const float close_raw = map_.to_rad(target_position_);
        if (std::abs(sample.actual_pos - close_raw) <= cfg_.brake_distance_rad) {
            // Decelerating onto the target is still part of the caller's grasp,
            // so it is bounded by the grasp torque -- NOT by the 6 Nm motion
            // limit. Measured on hardware: with the motion limit here, a jaw
            // blocked inside the last brake_distance_rad was a plain
            // error-clamped PD push, i.e. exactly the stall this class exists
            // to remove, and its low commanded torque also held the feedback
            // under the contact floor so nothing ever latched.
            return position_hold_(sample, close_raw, grasp_torque_nm_);
        }

        // Kp=0 prevents target-position error from generating torque. At zero
        // actual velocity, base_kd*close_speed equals grasp_torque_nm. Kd is
        // reduced further when the instantaneous velocity error would exceed
        // motion_torque_limit_nm; motor 0x700B is an independent backstop.
        const float close_velocity = -direction_open_() * speed_rad_s_;
        const float velocity_error = close_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / speed_rad_s_);
        const float kd = std::min(base_kd, active_torque_limit_nm_ /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(active_torque_limit_nm_,
                                        std::abs(kd * velocity_error));
        return {sample.actual_pos, 0.0f, kd, 0.0f, close_velocity};
    }

    if (state_ == ForcePositionState::HoldingForce) {
        return force_hold_();
    }

    if (state_ == ForcePositionState::Opening) {
        // No contact latch on the way open: an obstruction there is bounded by
        // the same grasp_torque_nm velocity-damping cap, and stopping short
        // would strand the jaw. Travel history is still tracked so a following
        // Closing starts from an honest peak.
        track_motion_(sample);
        if (open_position >= target_position_ - 1e-4f) {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position_);
            return position_hold_(sample, hold_raw_, active_torque_limit_nm_);
        }
        const float open_velocity = direction_open_() * speed_rad_s_;
        const float velocity_error = open_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / speed_rad_s_);
        const float kd = std::min(base_kd, active_torque_limit_nm_ /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(active_torque_limit_nm_,
                                        std::abs(kd * velocity_error));
        return {sample.actual_pos, 0.0f, kd, 0.0f, open_velocity};
    }

    return position_hold_(sample, hold_raw_, active_torque_limit_nm_);
}

}  // namespace detail

ForcePositionController::ForcePositionController(FollowerGripper& gripper)
    : ForcePositionController(gripper, ForcePositionConfig{}) {}

ForcePositionController::ForcePositionController(FollowerGripper& gripper,
                                                 ForcePositionConfig cfg)
    : g_(gripper), cfg_(cfg), runtime_(std::make_unique<detail::ControlRuntime>(gripper)) {
    validate_config_();
}

ForcePositionController::~ForcePositionController() {
    try { stop(); } catch (...) {}
}

void ForcePositionController::validate_config_() const {
    validate_config(cfg_);
}

void ForcePositionController::start() {
    if (running()) return;
    validate_config_();
    map_ = g_.position_map();
    const auto fw = g_.firmware_version();
    if (cfg_.close_compensation_rad > 0.0f && (!fw ||
        (fw->major == 1 && fw->minor == 1 && fw->patch < 8) || fw->major < 1 ||
        (fw->major == 1 && fw->minor < 1)))
        throw std::runtime_error("closing compensation requires firmware >= 1.1.8");

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

    // Feedback torque runs well under the commanded value once the jaw stalls
    // (~0.59 measured on 1.1.5 hardware), so a grasp torque only marginally
    // above the contact floor produces feedback that never reaches it and the
    // close never latches.
    if (cfg_.grasp_torque_nm < cfg_.contact_torque_nm * 2.0f) {
        logger()->warn(
            "ForcePositionController: grasp torque {:.3f} Nm leaves little "
            "headroom over the {:.3f} Nm contact floor; stall feedback runs "
            "well under the commanded torque, so contact may never confirm",
            cfg_.grasp_torque_nm, cfg_.contact_torque_nm);
    }

    const MotorStatusSample initial = g_.motor().read_status();
    if (!std::isfinite(initial.actual_pos) || !std::isfinite(initial.actual_vel) ||
        !std::isfinite(initial.actual_torque) || !std::isfinite(initial.motor_temp_c) ||
        has_serious_fault(initial.status))
        throw std::runtime_error("ForcePositionController: invalid or faulted initial motor feedback");
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
        step_requested_ = true;
        device_limit_nm_ = device_limit;
        command_sequence_ = 1;
        submitted_sequence_ = 0;
        submission_observation_sequence_ = 0;
        next_execution_check_ = now + std::chrono::milliseconds(200);
        fault_stop_completed_ = false;
        last_step_observation_sequence_ = 0;
        const auto version = g_.firmware_version();
        stream_execution_ = version && (version->major > 1 || (version->major == 1 &&
            (version->minor > 1 || (version->minor == 1 && version->patch >= 8))));
        feedback_frames_ = command_frames_ = 0;
        max_feedback_gap_ms_ = max_command_gap_ms_ = 0.0;
        last_submit_latency_ms_ = max_submit_latency_ms_ = 0.0;
        started_time_ = command_time_ = last_submit_time_ = now;
    }

    running_.store(true, std::memory_order_release);
    try {
        runtime_->start(cfg_.motor_stream_hz, 0,
            [this](const MotorStatusSample& s) { on_status_(s); },
            [this](bool event) { tick_(event); });
        if (stream_execution_) {
            std::unique_lock<std::mutex> lock(mu_);
            if (!sample_cv_.wait_for(lock, std::chrono::milliseconds(cfg_.status_timeout_ms),
                                    [this] { return feedback_frames_ > 0; }))
                throw std::runtime_error("initial execution telemetry timed out");
            if (!latest_.has_execution)
                throw std::runtime_error("firmware omitted negotiated execution telemetry");
        }
    } catch (...) { running_.store(false); throw; }
    runtime_->wake();
    logger()->info(
        "ForcePositionController started: close={:.3f} speed={:.3f}rad/s "
        "grasp={:.3f}Nm hold_limit={:.3f}Nm motion_limit={:.3f}Nm "
        "device_limit={:.3f}Nm",
        cfg_.close_position, cfg_.close_speed_radps, cfg_.grasp_torque_nm,
        cfg_.hold_torque_limit_nm, cfg_.motion_torque_limit_nm, device_limit_nm_);
}

void ForcePositionController::stop() {
    running_.store(false, std::memory_order_release);
    runtime_->stop();
}

void ForcePositionController::on_status_(const MotorStatusSample& sample) {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        // The query used at startup is legacy/clamped. Anchor initial hold to
        // the first raw sample so restarting below zero does not open the jaws.
        if (stream_execution_ && feedback_frames_ == 0 && sample.has_execution &&
            command_sequence_ == 1 && policy_->state() != ForcePositionState::Fault)
            policy_->reset(sample, now);
        if (feedback_frames_ > 0)
            max_feedback_gap_ms_ = std::max(max_feedback_gap_ms_,
                std::chrono::duration<double, std::milli>(now - latest_time_).count());
        ++feedback_frames_;
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
    sample_cv_.notify_all();
    runtime_->wake();
}

void ForcePositionController::request_step_() {
    ++command_sequence_;
    command_time_ = std::chrono::steady_clock::now();
    step_requested_ = true;
    runtime_->wake();
}

void ForcePositionController::require_active_() const {
    if (!running() || !policy_ || !have_sample_)
        throw std::logic_error("controller is not running");
    if (policy_->state() == ForcePositionState::Fault ||
        (latest_.has_execution && (latest_.sample_age_ms > 150 || latest_.execution.owner == 4 ||
            latest_.execution.last_error != 0 || (latest_.execution.flags & 2u))) ||
        has_serious_fault(latest_.status) || !std::isfinite(latest_.actual_pos) ||
        !std::isfinite(latest_.actual_vel) || !std::isfinite(latest_.actual_torque) ||
        !std::isfinite(latest_.motor_temp_c) ||
        std::abs(latest_.actual_torque) > cfg_.motion_torque_limit_nm + 1e-4f ||
        std::chrono::steady_clock::now() - latest_time_ >= std::chrono::milliseconds(cfg_.status_timeout_ms))
        throw std::logic_error("controller has faulted or stale feedback; stop, clear fault and restart");
}

void ForcePositionController::set_position_target(float position, float torque_limit_nm,
                                                  float speed_rad_s) {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    require_active_();
    policy_->set_position_target(latest_, position, torque_limit_nm, speed_rad_s,
                                  std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::set_grasp_target(float position, float torque_nm,
                                               float speed_rad_s) {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    require_active_();
    policy_->set_grasp_target(latest_, position, torque_nm, speed_rad_s,
                               std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::set_impedance_target(float position, float kp, float kd,
        float feedforward_torque_nm, float velocity_rad_s, float torque_limit_nm) {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    require_active_();
    policy_->set_impedance_target(latest_, position, kp, kd, feedforward_torque_nm,
        velocity_rad_s, torque_limit_nm, std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::set_target(float position) {
    set_target(position, cfg_.grasp_torque_nm);
}

void ForcePositionController::set_target(float position, float grasp_torque_nm) {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::set_target called before start");
    }
    policy_->set_target(latest_, position, grasp_torque_nm,
                        std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::release() {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::release called before start");
    }
    policy_->release(std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::hold_position() {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    require_active_();
    policy_->hold_position(latest_);
    request_step_();
}

void ForcePositionController::reset() {
    std::lock_guard<std::mutex> submission(submission_mu_);
    std::lock_guard<std::mutex> lk(mu_);
    require_active_();
    policy_->reset(latest_, std::chrono::steady_clock::now());
    // Strategy reset is available only while healthy. Fault recovery requires a
    // new controller session after the motor fault has been cleared.
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
            now - latest_time_).count() + latest_.sample_age_ms;
    }
    out.device_limit_nm = device_limit_nm_;
    out.execution_telemetry = latest_.has_execution;
    out.execution = latest_.execution;
    out.feedback_frames = feedback_frames_;
    out.command_frames = command_frames_;
    out.elapsed_s = std::chrono::duration<double>(now - started_time_).count();
    out.max_feedback_gap_ms = max_feedback_gap_ms_;
    out.max_command_gap_ms = max_command_gap_ms_;
    out.last_submit_latency_ms = last_submit_latency_ms_;
    out.max_submit_latency_ms = max_submit_latency_ms_;
    out.command_sequence = command_sequence_;
    out.submitted_sequence = submitted_sequence_;
    out.submission_observation_sequence = submission_observation_sequence_;
    if (policy_) {
        out.state = policy_->state();
        out.target_position = policy_->target_position();
        out.target_error_rad = policy_->position_target_rad() - latest_.actual_pos;
        out.hold_position = policy_->hold_position();
        out.grasp_torque_nm = policy_->grasp_torque_nm();
        out.commanded_torque_nm = policy_->commanded_torque_nm();
        out.hold_torque_limit_nm = cfg_.hold_torque_limit_nm;
        out.motion_torque_limit_nm = cfg_.motion_torque_limit_nm;
        out.contact_count = policy_->contact_count();
        out.fault_reason = policy_->fault_reason();
        out.control_mode = policy_->control_mode();
        out.blocked = policy_->state() == ForcePositionState::Blocked;
        out.active_torque_limit_nm = policy_->active_torque_limit_nm();
        out.speed_rad_s = policy_->speed_rad_s();
    }
    return out;
}

void ForcePositionController::tick_(bool event) {
    if (!running()) return;
    // Motor telemetry can stay fresh while the MCU rejects targets or latches
    // its execution watchdog. Sample that independent state without holding
    // either target or observation locks across the ACK wait.
    if (cfg_.monitor_execution && !stream_execution_ &&
        std::chrono::steady_clock::now() >= next_execution_check_) {
        next_execution_check_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        std::string failure;
        try {
            const auto execution = g_.motor().execution_status(std::chrono::milliseconds(50));
            if (execution.owner != 1 || execution.last_error != 0 || (execution.flags & 2u))
                failure = "firmware execution rejected control (owner=" +
                    std::to_string(execution.owner) + ", error=" +
                    std::to_string(execution.last_error) + ")";
        } catch (const std::exception& e) {
            failure = std::string("firmware execution monitor failed: ") + e.what();
        }
        if (!failure.empty()) {
            std::lock_guard<std::mutex> lock(mu_);
            if (policy_ && policy_->state() != ForcePositionState::Fault) {
                policy_->fail(std::move(failure));
                step_requested_ = true;
            }
        }
    }
    std::lock_guard<std::mutex> submission(submission_mu_);
    bool fault = false;
    {
        std::unique_lock<std::mutex> lock(mu_);
        const auto now = std::chrono::steady_clock::now();
        if (!policy_ || !have_sample_) return;
        if (stream_execution_ && feedback_frames_ == 0 &&
            now - latest_time_ < std::chrono::milliseconds(cfg_.status_timeout_ms)) return;
        if (policy_->state() == ForcePositionState::Fault && fault_stop_completed_) return;
        if (now - latest_time_ >= std::chrono::milliseconds(cfg_.status_timeout_ms)) {
            if (policy_->state() != ForcePositionState::Fault) {
                policy_->fail("motor status stream stale");
                step_requested_ = true;
            }
        }
        if (cfg_.monitor_execution && stream_execution_ && observation_.seq > 1) {
            const auto& e = latest_.execution;
            // Allow idle while the first command is reaching the executor.
            if (!latest_.has_execution || latest_.sample_age_ms > 150 ||
                e.last_error || (e.flags & 2u) || e.owner > 1 ||
                (e.owner == 0 && now - started_time_ > std::chrono::milliseconds(200))) {
                policy_->fail("invalid, stale or rejected firmware execution telemetry");
                step_requested_ = true;
            }
        }
        if (!step_requested_ && (!event || policy_->state() == ForcePositionState::Fault)) return;
        // Latest target wins. An API burst cannot emit multiple control frames
        // against the same observation. Fault/disable retries remain immediate.
        if (policy_->state() != ForcePositionState::Fault &&
            observation_.seq == last_step_observation_sequence_) return;
        step_requested_ = false;
        const auto command = policy_->step(latest_, now,
            observation_.seq != last_step_observation_sequence_);
        last_step_observation_sequence_ = observation_.seq;
        const auto observation_seq = observation_.seq;
        fault = policy_->state() == ForcePositionState::Fault;
        // submission_mu_ spans strategy and wire write; telemetry and snapshot
        // readers remain free to run during transport I/O.
        lock.unlock();
        try {
            g_.motor().submit(command);
            lock.lock();
            const auto sent_at = std::chrono::steady_clock::now();
            if (command_frames_ > 0)
                max_command_gap_ms_ = std::max(max_command_gap_ms_,
                    std::chrono::duration<double, std::milli>(sent_at - last_submit_time_).count());
            last_submit_time_ = sent_at;
            ++command_frames_;
            if (!fault && submitted_sequence_ != command_sequence_) {
                last_submit_latency_ms_ = std::chrono::duration<double, std::milli>(sent_at - command_time_).count();
                max_submit_latency_ms_ = std::max(max_submit_latency_ms_, last_submit_latency_ms_);
                submitted_sequence_ = command_sequence_;
                submission_observation_sequence_ = observation_seq;
            }
        } catch (const std::exception& e) {
            if (!lock.owns_lock()) lock.lock();
            if (!fault) policy_->fail(std::string("submit failed: ") + e.what());
            fault = true;
        }
    }
    if (fault) {
        try {
            g_.motor().disable();
            std::lock_guard<std::mutex> lock(mu_);
            fault_stop_completed_ = true;
        } catch (...) {
            std::lock_guard<std::mutex> lock(mu_);
            step_requested_ = true;  // retry even when telemetry has stopped
        }
    }
}

}  // namespace xense::taccap

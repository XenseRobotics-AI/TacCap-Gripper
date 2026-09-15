// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/force_position_controller.hpp>

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xense::taccap {

namespace {

constexpr float kEpsilon = 1e-5f;

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
        !finite(cfg.position_kd) || cfg.position_kd < 0.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig position gains require kp > 0 and kd >= 0");
    }
    if (!finite(cfg.brake_distance_rad) || cfg.brake_distance_rad < 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.brake_distance_rad must be >= 0");
    }
    if (!finite(cfg.close_endpoint_tolerance_rad) || cfg.close_endpoint_tolerance_rad < 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_endpoint_tolerance_rad must be >= 0");
    }
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
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);
    fault_reason_.clear();
}

void ForcePositionPolicy::release(std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::Opening;
    state_started_ = now;
    target_position_ = 1.0f;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
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
    state_started_ = now;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);

    if (current_position > target_position + kTargetTolerance) {
        state_ = ForcePositionState::Closing;
    } else if (current_position < target_position - kTargetTolerance) {
        state_ = ForcePositionState::Opening;
    } else {
        hold_raw_ = sample.actual_pos;
        if (target_position <= cfg_.close_position + kTargetTolerance) {
            state_ = ForcePositionState::HoldingForce;
        } else {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position);
        }
    }
}

void ForcePositionPolicy::hold_position(const MotorStatusSample& sample) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::HoldingPosition;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    begin_motion_(sample);
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
    const float target_speed = cfg_.close_speed_radps;
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
        float torque_budget) {
    // Bound the instantaneous PD request before it reaches the motor; motor
    // 0x700B remains the independent hardware backstop. Force holding uses the
    // separate, lower hold limit.
    const float budget = std::clamp(torque_budget, kEpsilon,
                                    cfg_.motion_torque_limit_nm);
    const float speed = std::abs(sample.actual_vel);
    float kd = cfg_.position_kd;
    if (speed > kEpsilon) {
        kd = std::min(kd, budget / speed);
    }
    const float damping = -kd * sample.actual_vel;
    const float position_budget = std::max(0.0f, budget - std::abs(damping));
    const float error_limit = position_budget / cfg_.position_kp;
    const float error = std::clamp(desired_raw - sample.actual_pos,
                                   -error_limit, error_limit);
    const float target = sample.actual_pos + error;
    const float predicted = cfg_.position_kp * error + damping;
    commanded_torque_nm_ = std::min(budget, std::abs(predicted));
    return {target, cfg_.position_kp, kd, 0.0f, 0.0f};
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
        return zero_(sample);
    }

    const float open_position = map_.to_position(sample.actual_pos);

    if (state_ == ForcePositionState::Closing) {
        track_motion_(sample);
        const auto guard = std::chrono::milliseconds(cfg_.startup_guard_ms);
        const bool guard_done = now - state_started_ >= guard;
        // motion_sign: +1 along the closing direction, matching the firmware's
        // s_auto_close_sign.
        const bool contact = contact_candidate_(sample, -direction_open_());
        if (guard_done && contact) ++contact_count_;
        else                       contact_count_ = 0;

        if (contact_count_ >= cfg_.contact_samples) {
            state_ = ForcePositionState::HoldingForce;
            hold_raw_ = sample.actual_pos;
            return force_hold_();
        }

        const bool endpoint_arrived =
            target_position_ <= cfg_.close_position + 1e-4f &&
            open_position <= cfg_.close_position + cfg_.close_endpoint_tolerance_rad &&
            std::abs(sample.actual_vel) <= cfg_.contact_vel_radps;
        if (endpoint_arrived || open_position <= target_position_ + 1e-4f) {
            hold_raw_ = sample.actual_pos;
            // A close-to-zero command is a grasp request, including an empty
            // close that reaches the calibrated endpoint before the contact
            // detector accumulates enough samples. Keep the configured grasp
            // torque there instead of dropping into zero-error position hold,
            // which otherwise lets the jaw unload and rebound.
            if (target_position_ <= cfg_.close_position + 1e-4f) {
                state_ = ForcePositionState::HoldingForce;
                return force_hold_();
            }
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position_);
            return position_hold_(sample, hold_raw_, cfg_.motion_torque_limit_nm);
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
        const float close_velocity = -direction_open_() * cfg_.close_speed_radps;
        const float velocity_error = close_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / cfg_.close_speed_radps);
        const float kd = std::min(base_kd, cfg_.motion_torque_limit_nm /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(cfg_.motion_torque_limit_nm,
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
            return position_hold_(sample, hold_raw_, cfg_.motion_torque_limit_nm);
        }
        const float open_velocity = direction_open_() * cfg_.close_speed_radps;
        const float velocity_error = open_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / cfg_.close_speed_radps);
        const float kd = std::min(base_kd, cfg_.motion_torque_limit_nm /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(cfg_.motion_torque_limit_nm,
                                        std::abs(kd * velocity_error));
        return {sample.actual_pos, 0.0f, kd, 0.0f, open_velocity};
    }

    return position_hold_(sample, hold_raw_, cfg_.motion_torque_limit_nm);
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
        "ForcePositionController started: close={:.3f} speed={:.3f}rad/s "
        "grasp={:.3f}Nm hold_limit={:.3f}Nm motion_limit={:.3f}Nm "
        "device_limit={:.3f}Nm",
        cfg_.close_position, cfg_.close_speed_radps, cfg_.grasp_torque_nm,
        cfg_.hold_torque_limit_nm, cfg_.motion_torque_limit_nm, device_limit_nm_);
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

void ForcePositionController::set_target(float position, float grasp_torque_nm) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::set_target called before start");
    }
    policy_->set_target(latest_, position, grasp_torque_nm,
                        std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::release() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::release called before start");
    }
    policy_->release(std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::hold_position() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::hold_position called before start");
    }
    policy_->hold_position(latest_);
    request_step_();
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
        out.contact_count = policy_->contact_count();
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
                return stop_requested_ || step_requested_;
            });
            if (stop_requested_) break;
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
            if (stale && policy_->state() != ForcePositionState::Fault) {
                policy_->fail("motor status stream stale");
                step_requested_ = true;   // push one zero-torque command out
            }
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

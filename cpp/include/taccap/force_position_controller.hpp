// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ForcePositionController — contact-aware force/position hybrid grasping.
//
// A plain impedance position command keeps increasing torque while an object
// prevents the jaw from reaching its target. This controller instead runs the
// same hybrid sequence the firmware itself uses:
//
//   velocity-damped close -> contact detection -> pure feed-forward torque hold
//
// In the force-hold state kp=kd=0, so the commanded holding torque cannot grow
// with position error. Position holds are dynamically error-clamped as a second
// line of defence.
//
// CONTACT DETECTION mirrors tc-gu-01's power-on auto-calibration, which solves
// exactly this problem (find the travel limits by closing until the jaw is
// blocked) and has been tuned on this hardware. See
// third_party/firmware/tc-gu-01 App/tasks/task_canmotor.c
// task_canmotor_is_stalled(). The essential point is that a stall there is
// NEVER a bare torque threshold -- it is a torque floor AND arrested motion,
// held for a confirmation window:
//
//   contact = arrested AND |torque| >= contact_torque_nm
//   arrested = |vel| <= 0.035 rad/s, or (having moved) the velocity along the
//              motion direction has fallen under 25% of the commanded speed
//   hold     = contact_samples frames (firmware: stall_hold_ms, default 30 ms)
//
// THE VELOCITY GATE DOES THE SEPARATING, not the torque number. Measured on a
// follower running firmware 1.1.5, closing on empty jaws over the full travel
// (1578 samples): free travel sat at |vel| >= 0.183 rad/s -- five times the
// 0.035 gate -- with |torque| <= 0.142 Nm, while the mechanical stop showed
// |vel| ~ 0.012 rad/s and |torque| ~ 0.21 Nm. Zero free-travel samples passed
// both tests. The torque term only has to reject "stopped but unloaded", which
// is why the firmware's own floor (TASK_CANMOTOR_STALL_TORQUE_FLOOR_NM,
// 0.080 Nm) is the default here.
//
// Do NOT derive the threshold from the commanded torque cap. The firmware can
// use a ratio of 1.00 against its cap because it commands VELOCITY with a
// max_torque and the actuator's own loop winds up to that cap. A kd-only MIT
// frame does not: at the stall above the command asked for 0.359 Nm and the
// feedback saturated at 0.213 Nm (~0.59), so a threshold placed at the cap is
// simply unreachable and the jaw pushes forever without ever latching --
// which is the stall this controller exists to prevent.
//
// WHAT position 0.0 ACTUALLY IS, and why a full close terminates on contact
// rather than on arrival. The firmware auto-calibrates on power-up: it closes
// under a VELOCITY command capped at close_stall_torque_nm (0.35 Nm by
// default) until stalled, records that position, backs off
// TASK_CANMOTOR_BACKOFF_DISTANCE_RAD, returns to it, and calls
// can_motor_set_zero_hold() with the jaw still enabled and loaded. So
// normalized 0.0 is not a geometric hard stop -- it is the depth the jaw
// reaches when PUSHED at the auto-cal stall torque, with that preload baked in.
//
// A kd-only MIT close does not reproduce that push (see below: feedback runs
// ~0.59 of the commanded torque at stall), so it settles short of zero.
// Measured on 1.1.5: an empty close stalls at normalized 0.0167, about 0.020
// rad shy of the calibrated zero. set_target(0.0) is therefore asking for a
// point this control mode cannot reach, and no arrival tolerance fixes that --
// which is precisely why contact detection, not position arrival, is the
// terminal condition for a blocked close.
//
// RS05 manual: 5.5 Nm peak; the 1.2 Nm stalled rating is conditional on
// the manufacturer's thermal test setup. Neither is an unconditional holding
// guarantee. Firmware applies the configured thermal envelope independently.
// start() checks the persisted startup limit, not a live motor-register read.

#pragma once

#include <taccap/control_loop.hpp>
#include <taccap/follower_gripper.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xense::taccap {

constexpr float FORCE_POSITION_MAX_HOLD_TORQUE_NM   = 1.2f;
constexpr float FORCE_POSITION_MAX_MOTION_TORQUE_NM = 5.5f;

enum class ForcePositionState : uint8_t {
    Idle,
    HoldingPosition,
    Closing,
    HoldingForce,
    Opening,
    Fault,
    MovingPosition,
    Impedance,
    Blocked,
};

enum class ForcePositionMode : uint8_t { Position, Grasp, Impedance };

struct ForcePositionConfig {
    float close_position       = 0.0f;   // normalized [0,1], 0 = fully closed
    float close_speed_radps    = 0.5f;   // raw motor speed magnitude
    float grasp_torque_nm      = 0.35f;  // pure torque used after contact
    float hold_torque_limit_nm = FORCE_POSITION_MAX_HOLD_TORQUE_NM;
    float motion_torque_limit_nm = FORCE_POSITION_MAX_MOTION_TORQUE_NM;
    // Contact detection, mirroring task_canmotor_is_stalled() (see above).
    // A FLOOR, not a saturation threshold: it only has to reject "stopped but
    // unloaded", because the velocity gate is what separates travel from
    // contact. Default is the firmware's own
    // TASK_CANMOTOR_STALL_TORQUE_FLOOR_NM. Raise it only if a specific unit
    // shows free-travel torque near it -- and remember feedback torque runs
    // well under the commanded value at stall (~0.59 measured), so a high
    // floor becomes unreachable rather than merely strict.
    float contact_torque_nm    = 0.080f;
    // |vel| at or below this counts as arrested regardless of travel history
    // (TASK_CANMOTOR_STALL_VEL_RAD_S).
    float contact_vel_radps    = 0.035f;
    // Once the jaw has demonstrably moved, velocity along the motion direction
    // at or below this fraction of the commanded speed also counts as arrested
    // (TASK_CANMOTOR_STALL_VEL_RATIO).
    float contact_vel_ratio    = 0.25f;
    // Travel required before the ratio test arms (TASK_CANMOTOR_STALL_MOVED_RAD),
    // together with peak speed reaching 35% of the commanded speed.
    float contact_moved_rad    = 0.010f;
    // Matches ControlLoop::Config::kp. Raising it tightens the position hold
    // without raising the torque ceiling: the PD request is error-clamped
    // against a torque budget either way, so kp only narrows the error window
    // (budget/kp), it does not widen the output.
    float position_kp          = 20.0f;  // safe current-position/endpoint hold
    float position_kd          = 1.0f;
    float close_compensation_rad = 0.0f;  // opt-in position-only closing offset, firmware >= 1.1.8
    float brake_distance_rad   = 0.10f;  // switch close velocity -> clamped PD
    // Consecutive confirming status frames. At motor_stream_hz = 100 the
    // firmware's 30 ms stall_hold_ms is 3 frames.
    unsigned contact_samples   = 3;
    unsigned startup_guard_ms  = 250;    // ignore acceleration torque at close start
    unsigned status_timeout_ms = 350;    // stale stream -> zero command + Fault
    unsigned motor_stream_hz   = 100;
    bool monitor_execution     = false;  // 1.1.8 stream; 1.1.7 legacy synchronous fallback
};

struct ForcePositionSnapshot {
    bool                running             = false;
    ForcePositionState  state               = ForcePositionState::Idle;
    GripperObservation  observation{};
    float               target_position     = 0.0f;  // normalized [0,1]
    float               hold_position       = 0.0f;  // normalized [0,1]
    float               grasp_torque_nm     = 0.0f;  // active contact/hold target
    float               commanded_torque_nm = 0.0f;  // magnitude predicted/requested
    float               hold_torque_limit_nm = 0.0f;
    float               motion_torque_limit_nm = 0.0f;
    float               device_limit_nm     = 0.0f;  // persisted 0x700B boot value
    unsigned            contact_count       = 0;
    std::string         fault_reason;
    ForcePositionMode   control_mode = ForcePositionMode::Position;
    bool                blocked = false;
    float               active_torque_limit_nm = 0.0f;
    float               speed_rad_s = 0.0f;
    uint64_t            command_sequence = 0;
    uint64_t            submitted_sequence = 0;
    uint64_t            submission_observation_sequence = 0;
    float               target_error_rad = 0.0f;
    bool                execution_telemetry = false;
    protocol::MotorExecutionStatus execution{};
    uint64_t            feedback_frames = 0;
    uint64_t            command_frames = 0;
    double              elapsed_s = 0.0;
    double              max_feedback_gap_ms = 0.0;
    double              max_command_gap_ms = 0.0;
    double              last_submit_latency_ms = 0.0;
    double              max_submit_latency_ms = 0.0;
};

namespace detail {

// Pure state machine used by ForcePositionController. Kept separate from the
// transport owner so its safety transitions and command bounds are testable
// without a connected gripper.
class ForcePositionPolicy {
public:
    ForcePositionPolicy(GripperPosition map, ForcePositionConfig cfg);

    void reset(const MotorStatusSample& sample,
               std::chrono::steady_clock::time_point now);
    void release(std::chrono::steady_clock::time_point now);
    void set_target(const MotorStatusSample& sample,
                    float target_position,
                    float grasp_torque_nm,
                    std::chrono::steady_clock::time_point now);
    void hold_position(const MotorStatusSample& sample);
    void set_position_target(const MotorStatusSample& sample, float position,
                             float torque_limit_nm, float speed_rad_s,
                             std::chrono::steady_clock::time_point now);
    void set_grasp_target(const MotorStatusSample& sample, float position,
                          float torque_nm, float speed_rad_s,
                          std::chrono::steady_clock::time_point now);
    void set_impedance_target(const MotorStatusSample& sample, float position,
                              float kp, float kd, float feedforward_torque_nm,
                              float velocity_rad_s, float torque_limit_nm,
                              std::chrono::steady_clock::time_point now);
    void fail(std::string reason);

    protocol::MotorImpedanceCtrl step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now, bool new_sample = true);

    ForcePositionState state() const noexcept { return state_; }
    float target_position() const noexcept { return target_position_; }
    float hold_position() const noexcept { return map_.to_position(hold_raw_); }
    float grasp_torque_nm() const noexcept { return grasp_torque_nm_; }
    float commanded_torque_nm() const noexcept { return commanded_torque_nm_; }
    unsigned contact_count() const noexcept { return contact_count_; }
    const std::string& fault_reason() const noexcept { return fault_reason_; }
    ForcePositionMode control_mode() const noexcept { return control_mode_; }
    float active_torque_limit_nm() const noexcept { return active_torque_limit_nm_; }
    float speed_rad_s() const noexcept { return speed_rad_s_; }
    float position_target_rad() const noexcept {
        if (state_ == ForcePositionState::HoldingPosition) return hold_raw_;
        return map_.to_rad(target_position_) - (map_.reverse() ? -1.0f : 1.0f) *
            (compensate_close_ && control_mode_ == ForcePositionMode::Position ?
                cfg_.close_compensation_rad * (1.0f - target_position_) : 0.0f);
    }

private:
    // Reset / update the per-motion travel history the contact test needs.
    void begin_motion_(const MotorStatusSample& sample);
    void track_motion_(const MotorStatusSample& sample) noexcept;
    // Torque saturation AND arrested motion, per task_canmotor_is_stalled().
    bool contact_candidate_(const MotorStatusSample& sample,
                            float motion_sign) const noexcept;
    protocol::MotorImpedanceCtrl zero_(const MotorStatusSample& sample);
    protocol::MotorImpedanceCtrl force_hold_();
    // torque_budget bounds the instantaneous PD request. The approach ("brake")
    // phase of a motion passes the grasp torque so decelerating onto a target
    // can never push harder than the grasp it belongs to; a settled hold passes
    // the motion limit.
    protocol::MotorImpedanceCtrl position_hold_(const MotorStatusSample& sample,
                                                 float desired_raw,
                                                 float torque_budget, float desired_velocity = 0.0f);
    float contact_threshold_() const noexcept;
    float direction_open_() const noexcept;
    void require_active_() const;
    protocol::MotorImpedanceCtrl impedance_(const MotorStatusSample& sample);

    GripperPosition map_;
    ForcePositionConfig cfg_;
    ForcePositionState state_ = ForcePositionState::Idle;
    std::chrono::steady_clock::time_point state_started_{};
    float target_position_ = 0.0f;
    float hold_raw_ = 0.0f;
    float grasp_torque_nm_ = 0.0f;
    float commanded_torque_nm_ = 0.0f;
    unsigned contact_count_ = 0;
    // Per-motion travel history, reset whenever a new Closing/Opening starts.
    // Mirrors s_home_stall_{motion_start_pos,peak_vel} in the firmware.
    float motion_start_raw_ = 0.0f;
    float peak_abs_vel_ = 0.0f;
    std::string fault_reason_;
    ForcePositionMode control_mode_ = ForcePositionMode::Position;
    float active_torque_limit_nm_ = 0.0f;
    float speed_rad_s_ = 0.0f;
    float reference_raw_ = 0.0f;
    bool compensate_close_ = false;
    std::chrono::steady_clock::time_point reference_time_{};
    protocol::MotorImpedanceCtrl impedance_target_{};
};

}  // namespace detail

class ForcePositionController {
public:
    explicit ForcePositionController(FollowerGripper& gripper);
    ForcePositionController(FollowerGripper& gripper, ForcePositionConfig cfg);
    ~ForcePositionController();

    ForcePositionController(const ForcePositionController&) = delete;
    ForcePositionController& operator=(const ForcePositionController&) = delete;

    // Owns the follower motor-status/control path until stop(). The caller still
    // can request an emergency disable. stop() disables before releasing ownership.
    void start();
    void stop();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    void release();        // open toward 1.0 with bounded velocity damping
    // Immediately move toward a normalized target. A lower target uses the
    // contact-aware grasp path; a higher target uses bounded opening motion.
    void set_target(float position);
    void set_target(float position, float grasp_torque_nm);
    // Position speed limits the reference ramp, not measured physical velocity.
    // Position following retains the requested target under load and keeps
    // applying bounded torque. Only a new command cancels that target.
    void set_position_target(float position, float torque_limit_nm = 1.0f,
                             float speed_rad_s = 0.5f);
    void set_grasp_target(float position, float torque_nm = 0.35f,
                          float speed_rad_s = 0.5f);
    // Velocity and feed-forward torque use the raw motor frame. All three torque
    // terms share torque_limit_nm; zero kp/kd gives bounded feed-forward control.
    void set_impedance_target(float position, float kp, float kd,
                              float feedforward_torque_nm = 0.0f,
                              float velocity_rad_s = 0.0f,
                              float torque_limit_nm = 1.0f);
    void hold_position();  // cancel motion/force and hold the latest position
    void reset();          // reset strategy only; after disable use stop/clear_fault/start

    ForcePositionState state() const;
    ForcePositionSnapshot snapshot() const;
    const ForcePositionConfig& config() const noexcept { return cfg_; }

private:
    void validate_config_() const;
    void on_status_(const MotorStatusSample& sample);
    void request_step_();
    void tick_(bool event);
    void require_active_() const;

    FollowerGripper& g_;
    ForcePositionConfig cfg_;
    GripperPosition map_;

    mutable std::mutex mu_;
    std::condition_variable sample_cv_;
    std::mutex submission_mu_;  // target changes serialize with writes, not telemetry
    bool step_requested_ = false;
    bool have_sample_ = false;
    MotorStatusSample latest_{};
    std::chrono::steady_clock::time_point latest_time_{};
    GripperObservation observation_{};
    std::unique_ptr<detail::ForcePositionPolicy> policy_;
    float device_limit_nm_ = 0.0f;
    uint64_t command_sequence_ = 0;
    uint64_t submitted_sequence_ = 0;
    uint64_t submission_observation_sequence_ = 0;
    std::chrono::steady_clock::time_point next_execution_check_{};
    bool fault_stop_completed_ = false;
    uint64_t last_step_observation_sequence_ = 0;
    bool stream_execution_ = false;
    uint64_t feedback_frames_ = 0, command_frames_ = 0;
    double max_feedback_gap_ms_ = 0.0, max_command_gap_ms_ = 0.0;
    double last_submit_latency_ms_ = 0.0, max_submit_latency_ms_ = 0.0;
    std::chrono::steady_clock::time_point started_time_{}, command_time_{}, last_submit_time_{};

    std::unique_ptr<detail::ControlRuntime> runtime_;
    std::atomic<bool> running_{false};

};

const char* to_string(ForcePositionState state) noexcept;

}  // namespace xense::taccap

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ForcePositionController — bounded-torque grasping.
//
// ONE CONTROL LAW. The jaw is commanded toward target_position with the PD
// request error-clamped against grasp_torque_nm, for the whole move:
//
//   command(target, kp, kd, torque_budget = grasp_torque_nm)
//
// Free travel costs only what friction costs (~0.1 Nm measured on this
// mechanism), well under the budget, so the clamp does not bind and the jaw
// tracks a ramp at close_speed_radps. An obstruction stops the jaw, the ramp
// runs ahead, the clamp saturates, and the command settles at exactly
// grasp_torque_nm and stays there. CONTACT IS NOT DETECTED -- saturation is
// what contact IS, and holding is what saturation does.
//
// WHAT THIS REPLACED, and why. Until 2026-09 this class ran a contact state
// machine on the host: a velocity-damped close, a stall test copied from the
// firmware's auto-calibration, an N-frame confirmation window, then a switch
// into pure feed-forward torque. Three things were wrong with it.
//
//  1. It duplicated the MCU. task_canmotor_is_stalled() runs the same test at
//     500 Hz and docs/CONTROL_LAYERING.md section 3 already assigns contact
//     detection to the firmware. Two copies of one physical event drift apart.
//  2. It made the control soft ON PURPOSE. The travel command carried kp=0 --
//     a position term would have muddied the stall signature -- leaving a
//     proportional velocity loop of grasp_torque/close_speed, 0.7 Nm/(rad/s) at
//     the old defaults. Measured over 10 runs at full stream rate: 37% relative
//     velocity ripple closing, 0.59 rad/s peak-to-peak, a 12 Hz limit cycle,
//     and a mean speed only 77% of what was asked for. Four configurations
//     proved the ripple tracked kd and not speed, so it was loop softness and
//     not mechanical resonance.
//  3. Its central judgement was unmeasurable. Separating "arrived" from
//     "blocked" needed a threshold on distance-to-target, and on empty jaws
//     those two are physically continuous.
//
// THE TORQUE BUDGET IS THE WHOLE DESIGN, so it has to be big enough to command
// with. The default is now the EL05's continuous stall rating, 1.1 Nm -- see
// ForcePositionConfig. The previous 0.35 Nm was the firmware's homing constant
// wearing a grasp-force costume, and left mechanism friction occupying 29% of
// all available authority, which is what the ripple above really measured.
//
// STILL TRUE, AND STILL LOAD-BEARING: the firmware's motion envelope is the
// safety layer, not this class. It clamps position error, runs I2t derating and
// a temperature wall, and it is the only thermal protection in the system --
// the motor's own over-temperature protection did not act at 100 C case
// temperature. start() cross-checks grasp_torque_nm against the device's
// reported continuous envelope and warns; it cannot enforce.
//
// 6.0 Nm is also the firmware's own default AND maximum for the persisted
// 0x700B startup limit (storage.c STORAGE_MOTOR_LIMIT_TORQUE_{DEFAULT,MAX}_NM),
// so the default configuration here matches a factory device. start() verifies
// the V2.2 value persisted for the next boot against the motion limit. After
// changing that stored value, physically power-cycle before calling start().

#pragma once

#include <taccap/components/motor.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/gripper_observation.hpp>
#include <taccap/gripper_position.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xense::taccap {

// Aliases for the motor ratings in components/motor.hpp, kept because they are
// public API and exported to Python.
constexpr float FORCE_POSITION_MAX_HOLD_TORQUE_NM   = MOTOR_RATED_TORQUE_NM;
constexpr float FORCE_POSITION_MAX_MOTION_TORQUE_NM = MOTOR_PEAK_TORQUE_NM;

enum class ForcePositionState : uint8_t {
    Idle,
    HoldingPosition,
    Closing,
    HoldingForce,
    Opening,
    Fault,
};

// THE TUNABLE SURFACE. Two values are the grasp itself and are the only ones a
// task normally sets; two are the motor's own nameplate ratings; two describe
// the transport.
//
// grasp_torque_nm IS THE CONTROL LAW'S TORQUE BUDGET, not a threshold consulted
// after some contact event. The jaw is commanded toward target_position with the
// PD request error-clamped against this budget, so free travel costs only what
// friction costs (~0.1 Nm measured) and an obstruction saturates the clamp at
// exactly this value and holds there. Contact needs no detecting; it is what
// saturation IS.
//
// THE DEFAULT IS THE EL05's CONTINUOUS STALL RATING, 1.1 Nm -- the torque the
// datasheet says it can hold indefinitely, and the grip force this gripper is
// specified to deliver. It replaces an earlier 0.35 Nm that had been copied from
// the firmware's auto-calibration constant
// (GRIPPER_AUTO_CAL_DEFAULT_CLOSE_TORQUE, the push used to find the mechanical
// stop during homing) and was never justified as a grip force at all.
//
// WHAT A LONG HOLD AT THIS TORQUE ACTUALLY DOES, measured on a real workpiece at
// 24 V with the envelope enforced, winding temperature from the status stream:
//
//   1.1 Nm  36 -> 70 C over 600 s, and the RATE decays the whole way:
//           8 -> 4 -> 3 -> 2 -> 1 C/min. That is a first-order approach, not a
//           linear climb; fitting T_inf - T ~ exp(-t/tau) puts the plateau near
//           75 C with tau ~ 350 s, about 15 C under the firmware's 90 C
//           temperature wall. No fault, no undervoltage latch, no link loss, and
//           the grasp did not drift.
//   0.6 Nm  41 -> 49 C over 600 s, last-minute slope 0.00 C/min. Plateaued flat.
//
// Read the RATE, not the last slope. An earlier reading of this same data called
// 1.1 Nm unsustainable because it was "still climbing 1 C/min at the end" -- but
// a first-order system approaching its plateau always still has slope, and the
// rate had already fallen eightfold. The margin is real but it is not generous:
// these runs started from 36-41 C rather than a cold machine, and a warm ambient
// eats into the 15 C directly. A hold measured in tens of minutes is worth
// confirming on the unit and the ambient it will actually run in.
//
// NOT A HARD CEILING EITHER. Measured feedback runs 4-7% above the budget at a
// settled hold (1.149 Nm against 1.100, 0.643 against 0.600). The budget shapes
// the command; the firmware's motion envelope is what actually bounds output.
// See docs/CONTROL_REFACTOR.md.
struct ForcePositionConfig {
    float grasp_torque_nm      = 1.1f;   // torque budget; EL05 continuous rating
    float close_speed_radps    = 0.5f;   // ramp speed, raw motor units
    // The two MOTOR RATINGS, not two arbitrary safety margins -- see the note
    // at the top of this file.
    float hold_torque_limit_nm   = FORCE_POSITION_MAX_HOLD_TORQUE_NM;
    float motion_torque_limit_nm = FORCE_POSITION_MAX_MOTION_TORQUE_NM;
    unsigned status_timeout_ms = 350;    // stale stream -> zero command + Fault
    unsigned motor_stream_hz   = 100;
    // Feed-forward torque added ONLY while holding the closed endpoint, to seat
    // the jaw against its mechanical stop. 0 disables it.
    //
    // WHY A FORCE AND NOT A POSITION OFFSET. position_hold_ commands
    // kp*(target-actual), which goes to zero exactly at the target -- so the jaw
    // arrives at the closed end and then stops pressing, leaving the gear
    // train's backlash unseated. Biasing the target past the stop cannot fix it:
    // the firmware clamps every target to the calibrated range, whose closed end
    // is hard-coded 0.0f in can_motor_get_gripper_target_range(), so a biased
    // target is simply truncated. A feed-forward term has no such clamp.
    //
    // WHY 0.25. Measured on TCGU01A28Z0018s, pure feed-forward (kp=kd=0) swept
    // against the closed stop:
    //     0.05 Nm -> raw -0.00249     0.15 Nm -> raw +0.00000
    //     0.10 Nm -> raw -0.00096     0.20/0.30/0.40/0.50 Nm -> raw +0.00000
    // raw 0 is a HARD STOP, not a compliant region: 0.15 Nm seats the jaw fully
    // and 3.3x more torque moves it not one microradian. So the useful range
    // ends at ~0.15 Nm; 0.25 is that with 1.7x headroom for friction growth and
    // mechanical drift. Anything above buys heat and gear load, never closure.
    //
    // Thermally free at this value: the firmware accumulates I2t only above the
    // envelope's cont_torque_nm (1.1 Nm measured), and 0.25 is 23% of it, so
    // s_i2t_energy stays pinned at 0. It still passes through
    // can_motor_envelope_clamp_torque(), so the temperature wall governs it --
    // that clamp is applied unconditionally alongside the target clamp
    // (can_motor.c:5317-5318), not only on the kp==0 path.
    //
    // Self-compensating, which a fixed position inset is not: however far the
    // stop drifts with wear, the force presses until the jaw is seated again.
    float close_preload_nm     = 0.25f;
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
    // OBSERVATIONS, derived per frame -- not states the controller switches on.
    bool                holding             = false; // budget saturated, not at target
    bool                arrived             = false; // within arrival_eps_rad
    std::string         fault_reason;
};

namespace detail {

// Fixed control constants: the gains measured against this hardware. Not on
// ForcePositionConfig -- there is one right answer for this gripper, and the
// unit tests construct it directly to build scenarios no caller should ask for.
//
// The firmware's stall numbers used to live here too (contact_torque_nm,
// contact_vel_radps, contact_vel_ratio, contact_moved_rad, stall_hold_ms,
// startup_guard_ms) along with arrival_band_rad and brake_distance_rad. They
// are gone with the contact state machine: the MCU already runs that test at
// 500 Hz (task_canmotor_is_stalled) and is the authority per
// docs/CONTROL_LAYERING.md section 3, so the host was duplicating it -- worse
// than doing it in either place alone, because two copies of one physical
// event drift apart.
struct ForcePositionTuning {
    // Raising kp tightens tracking without raising the torque ceiling: the PD
    // request is error-clamped against the budget either way, so kp only
    // narrows the error window (budget/kp), it does not widen the output.
    float position_kp         = 20.0f;
    float position_kd         = 1.0f;
    // Damping gain during travel. FREE OF THE GRASP BUDGET, which is the whole
    // point: the velocity feed-forward follows the RAMP's own advance, and the
    // ramp stops advancing the moment the jaw is blocked (it is anti-windup
    // clamped to the jaw). So at stall the damping term asks for kd*(0-0) = 0,
    // the entire budget goes to the position term, and kd is bounded by
    // stability rather than by how hard the caller wants to grip.
    //
    // The first cut of this controller got that wrong: it fed close_speed_radps
    // forward unconditionally, so kd*close_speed had to be carved out of the
    // grasp budget and kd came out at HALF what the old velocity-damped design
    // used at the same grip force. Ripple is disturbance/gain, so halving the
    // gain undid most of what the refactor was supposed to buy -- measured 33%
    // at a 0.6 Nm grasp against the old design's ~22% extrapolated for the same
    // torque. The feed-forward was simply describing a motion that was not
    // happening.
    float travel_kd           = 2.5f;
    // Within this of the commanded position the jaw counts as arrived. Reported,
    // never acted on -- there is no separate arrival branch any more.
    float arrival_eps_rad     = 0.010f;
};


// ONE CONTROL LAW plus guards. Kept separate from the transport owner so its
// bounds and fault transitions are testable without a connected gripper.
//
// There is no contact state machine any more. step() always issues the same
// bounded-impedance command toward target_position_; ForcePositionState is
// DERIVED from the result each frame and reported, never consulted to decide
// what to command. Fault is the one real state, because a fault has to latch.
class ForcePositionPolicy {
public:
    ForcePositionPolicy(GripperPosition map, ForcePositionConfig cfg);
    // Tests only: override the fixed control constants.
    ForcePositionPolicy(GripperPosition map, ForcePositionConfig cfg,
                        ForcePositionTuning tune);

    void reset(const MotorStatusSample& sample,
               std::chrono::steady_clock::time_point now);
    void release(std::chrono::steady_clock::time_point now);
    void set_target(const MotorStatusSample& sample,
                    float target_position,
                    float grasp_torque_nm,
                    std::chrono::steady_clock::time_point now);
    void hold_position(const MotorStatusSample& sample);
    void fail(std::string reason);

    protocol::MotorImpedanceCtrl step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now);

    ForcePositionState state() const noexcept { return state_; }
    float target_position() const noexcept { return target_position_; }
    float hold_position() const noexcept { return map_.to_position(hold_raw_); }
    float grasp_torque_nm() const noexcept { return grasp_torque_nm_; }
    float commanded_torque_nm() const noexcept { return commanded_torque_nm_; }
    // Observations, refreshed by step().
    bool holding() const noexcept { return holding_; }
    bool arrived() const noexcept { return arrived_; }
    const std::string& fault_reason() const noexcept { return fault_reason_; }

private:
    protocol::MotorImpedanceCtrl zero_(const MotorStatusSample& sample);
    // Signed preload for the current hold: cfg_.close_preload_nm when the
    // commanded target is the closed endpoint, 0 otherwise. Sign from the map.
    float close_preload_signed_() const;
    // Settled hold on a fixed point: damps absolute velocity. preload_nm is a
    // SIGNED feed-forward torque, nonzero only at the closed endpoint (see
    // ForcePositionConfig::close_preload_nm); it is reserved out of the budget
    // so the total request stays bounded by it.
    protocol::MotorImpedanceCtrl position_hold_(const MotorStatusSample& sample,
                                                 float desired_raw,
                                                 float torque_budget,
                                                 float preload_nm);
    // Move toward target_raw along a time-based ramp at desired_vel, with the
    // PD request error-clamped against torque_budget. See the definition.
    protocol::MotorImpedanceCtrl travel_track_(const MotorStatusSample& sample,
                                               float target_raw,
                                               float desired_vel,
                                               float torque_budget,
                                               std::chrono::steady_clock::time_point now);
    float direction_open_() const noexcept;

    GripperPosition map_;
    ForcePositionConfig cfg_;
    ForcePositionTuning tune_;
    ForcePositionState state_ = ForcePositionState::Idle;
    std::chrono::steady_clock::time_point state_started_{};
    float target_position_ = 0.0f;
    float hold_raw_ = 0.0f;
    float grasp_torque_nm_ = 0.0f;
    float commanded_torque_nm_ = 0.0f;
    bool  holding_ = false;
    bool  arrived_ = false;
    // Travel ramp: the commanded setpoint, advanced at the commanded speed and
    // anti-windup clamped to stay within the error limit of the jaw.
    float ramp_raw_ = 0.0f;
    bool  ramp_valid_ = false;
    std::chrono::steady_clock::time_point last_step_{};
    std::string fault_reason_;
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
    // owns motor enable/disable. start() seeds a safe current-position hold.
    void start();
    void stop();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    void release();        // open toward 1.0 with bounded velocity damping
    // Immediately move toward a normalized target. A lower target uses the
    // contact-aware grasp path; a higher target uses bounded opening motion.
    void set_target(float position);
    void set_target(float position, float grasp_torque_nm);
    void hold_position();  // cancel motion/force and hold the latest position
    void reset();          // leave Fault after the caller has cleared the motor fault

    ForcePositionState state() const;
    ForcePositionSnapshot snapshot() const;
    const ForcePositionConfig& config() const noexcept { return cfg_; }

private:
    void validate_config_() const;
    void start_motor_stream_();
    void stop_motor_stream_();
    void on_status_(const MotorStatusSample& sample);
    void request_step_();
    void run_();

    FollowerGripper& g_;
    ForcePositionConfig cfg_;
    GripperPosition map_;

    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool step_requested_ = false;
    bool stop_requested_ = false;
    bool have_sample_ = false;

    // Caller commands are queued here and applied in run_() on the status-frame
    // doorbell, so every serial write lands in the window the MCU is idle.
    // Applying them on the caller's thread submitted off-phase and cost
    // telemetry frames. See the note above set_target() in the .cpp.
    enum class PendingCommand : uint8_t { None, SetTarget, Release, HoldPosition };
    PendingCommand pending_ = PendingCommand::None;
    // One-shot wake flag. Caller commands must still wake run_() so staleness
    // is evaluated promptly, but they must NOT set step_requested_ -- that is
    // the status-frame doorbell, and setting it here is what used to submit
    // off-phase. Kept separate from pending_ so the wake is consumed once and
    // the loop does not spin on a command it is deliberately holding back.
    bool command_woke_ = false;
    float pending_position_ = 0.0f;
    float pending_grasp_torque_nm_ = 0.0f;
    MotorStatusSample latest_{};
    std::chrono::steady_clock::time_point latest_time_{};
    GripperObservation observation_{};
    std::unique_ptr<detail::ForcePositionPolicy> policy_;
    float device_limit_nm_ = 0.0f;

    std::thread thread_;
    std::atomic<bool> running_{false};
    Motor::SubId sub_ = 0;
    bool sub_active_ = false;
    bool stream_ours_ = false;
};

const char* to_string(ForcePositionState state) noexcept;

}  // namespace xense::taccap

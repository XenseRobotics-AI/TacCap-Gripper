// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ImpedanceController — position tracking with bounded torque.
//
// The sibling of ForcePositionController, and shaped the same way on purpose: a
// pure state machine in detail::ImpedancePolicy that a test can drive without
// hardware, wrapped in a transport owner that submits on the status-frame
// doorbell. Use this one when the task is "follow a position" (teleoperation,
// a leader-follower relay) and ForcePositionController when it is "close until
// you feel something, then hold that force".
//
// Every submit happens on that doorbell, inside the window the MCU is known to
// be idle. Submitting free-running, off phase with the status stream, was
// measured to cost status frames: the MCU drops bytes out of the middle of a
// frame it is transmitting when host->MCU traffic overlaps it.
//
// WHY A STATE MACHINE RATHER THAN INDEPENDENT FLAGS. The low-level ControlLoop
// this class replaced (since removed) reported its guards as independent flags
// read through independent locks, so a caller polling two of them could observe
// a combination that never existed, and a failed submit ended its loop with
// nothing saying why. Here the guards are ordered states, a failure is an
// explicit Fault with a reason, and every field comes from one snapshot() under
// one lock.
//
// THE PROTECTIONS, in the order they take precedence:
//
//   Fault        - stale status stream, motor fault bits, or feedback over the
//                  peak rating. Commands zero torque and stays there until
//                  reset().
//   TorqueCapped - |feedback| held at or above rated_torque_nm. Sends a pure
//                  feed-forward hold with kp=kd=0, so position error cannot
//                  contribute to the output at all: the torque is PINNED rather
//                  than bounded by an estimate of it. This is the only layer
//                  that acts on what the motor is ACTUALLY producing, and the
//                  only reason it is not redundant with the clamp below -- the
//                  clamp bounds what we ASK for. It is a BACKSTOP for torque
//                  the control law did not ask for, so it sits above the band
//                  the law works in and normal operation never reaches it.
//                  What it holds is the BUDGET, not the trip level: it fires
//                  because the motor is already producing at or above rated,
//                  and nothing times the hold out.
//
// and underneath both, on every frame, the error clamp: the commanded target is
// held within max_position_torque_nm / kp radians of where the jaw actually is.
// That one is not a state because it never stops applying -- and it is the
// protection, not a supporting measure. See below.
//
// THERE IS NO STALL GUARD, AND REMOVING IT WAS THE FIX. This class used to trip
// a third state that clamped the effective target to wherever the jaw had
// stopped. Three things were wrong with it, and they are the same three that
// got the contact state machine deleted from ForcePositionController:
//
//  1. It duplicated the MCU. task_canmotor_is_stalled() runs an equivalent test
//     at 500 Hz and docs/CONTROL_LAYERING.md section 3 assigns contact
//     detection to the firmware. Two copies of one physical event drift apart.
//  2. Its central judgement was unmeasurable. "Blocked" and "grasping" are the
//     same observation -- a jaw that stopped moving with the clamp binding --
//     and nothing in position or velocity separates them. The guard resolved
//     that ambiguity by always assuming "blocked", which on a GRIPPER is the
//     less common case.
//  3. IT CANCELLED THE PROTECTION IT CLAIMED TO BE. Clamping the target to the
//     jaw's own position drops the position error to ~0, so kp*error and
//     kd*vel both vanish and the output collapses to feedforward_torque --
//     zero by default. Measured: closing onto an object tripped it 60 ms after
//     contact at 0.35 Nm of feedback and then let go entirely.
//
// The error clamp already does what the guard was reaching for, and does it
// without a trip: an obstruction saturates it at exactly
// max_position_torque_nm and the jaw holds there indefinitely. Contact needs no
// detecting; it is what saturation IS -- the same sentence, and the same
// mechanism (budget / kp as an error limit), as ForcePositionController.
//
// So the budget has to be a torque this motor can hold FOREVER. The default is
// the EL05's continuous stall rating, matching ForcePositionConfig's grasp
// budget. Thermal protection is not in this class either way: the firmware's
// motion envelope (I2t derating plus a temperature wall) is the only one in the
// system, and the motor's own over-temperature protection did not act at 100 C
// case temperature.
//
// WHY THE ERROR CLAMP HAS TO BOUND THE APPROACH AND NOT JUST THE HOLD. Measured
// on firmware 1.1.5 with a loose rigid object in the jaws, an unclamped step to
// a far target crossed the object's position at 5.5 rad/s and knocked it clean
// out WITHOUT the torque rising at all -- feedback stayed under 0.03 Nm through
// the crossing. No post-contact guard can catch that; there is no sustained
// contact to detect. Only not arriving that fast works. The clamp costs nothing
// while the jaw keeps up (the error is small and it never binds) and engages
// the instant the jaw falls behind, which is exactly the blocked case. It is
// deliberately NOT a slew-rate limit on the target: a slew limit adds lag to
// every motion including the teleoperation this class exists for.
//
// SUSTAINED TORQUE IS THE FAILURE MODE THAT BREAKS HARDWARE. rated_torque_nm is
// capped at the motor's RATED torque rather than its peak because TorqueCapped
// holds indefinitely and nothing here times it out. Above the rated value the
// risk is not mainly thermal: the motor's undervoltage protection is prompt, so
// a long hold browns out the 24 V rail and drops the USB link, which reaches
// the host as a write error rather than as anything resembling a torque fault.

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

enum class ImpedanceState : uint8_t {
    Idle,
    Tracking,
    TorqueCapped,
    Fault,
};

// Seven fields. The gains and the error clamp are what a task actually tunes;
// rated_torque_nm is the motor's own rating; two describe the transport. The
// measured ceiling constants live in detail::ImpedanceTuning, which only
// the tests construct -- the same split ForcePositionConfig uses, and for the
// same reason: there is one right answer for this hardware.
struct ImpedanceConfig {
    float kp = 20.0f;                      // Nm/rad
    float kd = 1.0f;                       // Nm*s/rad
    float feedforward_torque = 0.0f;       // Nm
    // THE CONTROL LAW'S TORQUE BUDGET, and the protection -- not a threshold
    // consulted after some contact event. The commanded target is error-clamped
    // against it, so free travel costs only what friction costs and an
    // obstruction saturates the clamp at exactly this value and holds there.
    //
    // A BLOCKED JAW SITS HERE INDEFINITELY, so it must be a torque the motor can
    // hold forever. The default is the EL05's continuous stall rating, the same
    // budget ForcePositionConfig::grasp_torque_nm defaults to, for the same
    // reason. It was 1.5 while a stall guard was expected to time the hold out;
    // that guard is gone (see the header note) so the value now has to stand on
    // its own.
    //
    // It also sets the approach speed: the jaw accelerates until damping
    // balances the clamped error torque, roughly max_position_torque_nm / kd
    // rad/s -- 1.1 rad/s at these defaults. 0 disables it, which is not
    // recommended: an unclamped step crosses a loose object at 5.5 rad/s and
    // knocks it out without the torque ever rising enough to notice.
    float max_position_torque_nm = MOTOR_STALL_CONT_TORQUE_NM;
    // Output ceiling, on MEASURED torque -- a backstop, not the grasp. The
    // error clamp above bounds the COMMAND; this watches what the motor
    // actually produces, which is a different quantity and the only reason both
    // exist. It must stay ABOVE max_position_torque_nm and the constructor
    // enforces that: at or below the budget, every normal grasp would trip it
    // and drop to kp=kd=0, losing position control while holding the object.
    //
    // Capped at the motor's rated torque because the resulting hold is
    // indefinite. 0 disables it.
    //
    // IT SITS ABOVE THE FIRMWARE ENVELOPE'S cont_torque_nm BY DESIGN, so do not
    // "fix" that by warning about it: start() once compared this against the
    // envelope and the comparison was a category error. The sustained draw is
    // bounded by max_position_torque_nm above; this is a trip level that must
    // stay out of the operating band, and on a correctly configured device
    // (cont == the stall rating == the budget) such a warning fires every time.
    float rated_torque_nm = MOTOR_RATED_TORQUE_NM;
    unsigned status_timeout_ms = 350;      // stale stream -> zero command + Fault
    unsigned motor_stream_hz   = 100;
};

struct ImpedanceSnapshot {
    bool               running        = false;
    ImpedanceState     state          = ImpedanceState::Idle;
    GripperObservation observation{};
    float              target_position    = 0.0f;  // normalized, as commanded
    float              effective_position = 0.0f;  // normalized, after clamping
    float              commanded_torque_nm = 0.0f; // magnitude predicted/requested
    bool               torque_capped  = false;
    uint64_t           torque_caps    = 0;
    float              device_limit_nm = 0.0f;     // persisted 0x700B boot value
    std::string        fault_reason;
};

namespace detail {

// Measured constants for this hardware. Not on ImpedanceConfig -- see the note
// above it.
struct ImpedanceTuning {
    // Ceiling engages once feedback has held at rated_torque_nm this long.
    unsigned rated_hold_ms    = 20;
    // While the ceiling holds, this much travel away from where it engaged
    // means the obstruction gave way, so impedance control resumes. Without it
    // a constant tau_ff would keep accelerating a jaw that came free.
    float    rated_release_rad = 0.05f;
};

// Pure state machine. Hardware-free and side-effect-free: feed it status
// samples and a target, take MIT frames out.
class ImpedancePolicy {
public:
    ImpedancePolicy(GripperPosition map, ImpedanceConfig cfg);
    ImpedancePolicy(GripperPosition map, ImpedanceConfig cfg, ImpedanceTuning tune);

    void reset(const MotorStatusSample& sample);
    void set_target(float position);       // normalized, clamped to [0,1]
    void set_gains(float kp, float kd, float feedforward_torque);
    void fail(std::string reason);

    protocol::MotorImpedanceCtrl step(const MotorStatusSample& sample,
                                      std::chrono::steady_clock::time_point now);

    ImpedanceState state() const noexcept { return state_; }
    float target_position() const noexcept { return target_; }
    float effective_position() const noexcept { return effective_; }
    float commanded_torque_nm() const noexcept { return commanded_torque_nm_; }
    bool torque_capped() const noexcept { return torque_capped_; }
    uint64_t torque_caps() const noexcept { return torque_caps_; }
    const std::string& fault_reason() const noexcept { return fault_reason_; }

private:
    void guard_torque_(const MotorStatusSample& s,
                       std::chrono::steady_clock::time_point now);
    float cap_hold_torque_() const noexcept;
    protocol::MotorImpedanceCtrl zero_(const MotorStatusSample& s);

    GripperPosition map_;
    ImpedanceConfig cfg_;
    ImpedanceTuning tune_;
    ImpedanceState  state_ = ImpedanceState::Idle;
    float target_    = 0.0f;
    float effective_ = 0.0f;
    float kp_ = 0.0f, kd_ = 0.0f, ff_ = 0.0f;
    float commanded_torque_nm_ = 0.0f;

    std::chrono::steady_clock::time_point cap_since_{};
    bool  torque_capped_ = false;
    float cap_sign_      = 1.0f;
    float cap_entry_raw_ = 0.0f;
    bool  cap_closing_   = false;
    uint64_t torque_caps_ = 0;

    std::string fault_reason_;
};

}  // namespace detail

class ImpedanceController {
public:
    explicit ImpedanceController(FollowerGripper& gripper);
    ImpedanceController(FollowerGripper& gripper, ImpedanceConfig cfg);
    ~ImpedanceController();

    ImpedanceController(const ImpedanceController&) = delete;
    ImpedanceController& operator=(const ImpedanceController&) = delete;

    // Owns the follower motor-status/control path until stop(). The caller still
    // owns motor enable/disable. start() seeds the target with the current
    // position, so starting an already-enabled motor does not produce a jump.
    void start();
    void stop();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    void set_target(float position);                      // normalized [0,1]
    void set_gains(float kp, float kd, float feedforward_torque = 0.0f);
    void reset();   // leave Fault after the caller has cleared the motor fault

    ImpedanceState state() const;
    ImpedanceSnapshot snapshot() const;
    const ImpedanceConfig& config() const noexcept { return cfg_; }

private:
    void validate_config_() const;
    void start_motor_stream_();
    void stop_motor_stream_();
    void on_status_(const MotorStatusSample& sample);
    void run_();

    FollowerGripper& g_;
    ImpedanceConfig cfg_;
    GripperPosition map_;

    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool step_requested_ = false;
    bool stop_requested_ = false;
    bool have_sample_ = false;
    bool command_woke_ = false;

    MotorStatusSample latest_{};
    std::chrono::steady_clock::time_point latest_time_{};
    GripperObservation observation_{};
    std::unique_ptr<detail::ImpedancePolicy> policy_;
    float device_limit_nm_ = 0.0f;

    std::thread thread_;
    std::atomic<bool> running_{false};
    Motor::SubId sub_ = 0;
    bool sub_active_ = false;
    bool stream_ours_ = false;
};

const char* to_string(ImpedanceState state) noexcept;

}  // namespace xense::taccap

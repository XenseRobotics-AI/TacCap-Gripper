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
// RELATIONSHIP TO ControlLoop. ControlLoop is the low-level loop and is not
// going away: it still offers SubmitPhase::FreeRunning, raw gain changes at
// runtime, and the smallest possible wrapper around submit_impedance(). This
// class is the supervised version of the same control law. It gives up the
// free-running phase (measured to cost status frames -- see ControlLoop's own
// notes) in exchange for a state machine, a single consistent snapshot, an
// explicit Fault with a reason, and reset().
//
// WHY A STATE MACHINE RATHER THAN TWO BOOLEANS. ControlLoop reports `stalled`
// and `torque_capped` as independent flags read through independent locks, so a
// caller polling both can observe a combination that never existed. Worse, it
// has no notion of being faulted at all: a failed submit breaks its loop and
// `running` goes false with nothing saying why. Here the guards are ordered
// states and every field comes from one snapshot() under one lock.
//
// THE THREE PROTECTIONS, in the order they take precedence:
//
//   Fault        - stale status stream, motor fault bits, or feedback over the
//                  peak rating. Commands zero torque and stays there until
//                  reset().
//   TorqueCapped - |feedback| held at or above rated_torque_nm. Sends a pure
//                  feed-forward hold with kp=kd=0, so position error cannot
//                  contribute to the output at all: the torque is PINNED at the
//                  ceiling rather than bounded by an estimate of it. This is the
//                  only layer that acts on what the motor is ACTUALLY producing.
//   Stalled      - torque above the stall floor AND the jaw slower than the
//                  stall velocity, held for stall_hold_ms. Clamps the effective
//                  target where the jaw stopped so kp * error stops growing.
//                  Both halves are required: torque alone trips on the impact
//                  transient, slowness alone is just a jaw at rest.
//
// and underneath all of them, on every frame, the error clamp: the commanded
// target is held within max_position_torque_nm / kp radians of where the jaw
// actually is. That one is not a state because it never stops applying.
//
// WHY THE ERROR CLAMP IS THE PRIMARY PROTECTION, not the stall guard. Measured
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

#include <taccap/control_loop.hpp>       // GripperObservation
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

enum class ImpedanceState : uint8_t {
    Idle,
    Tracking,
    Stalled,
    TorqueCapped,
    Fault,
};

// Seven fields. The gains and the error clamp are what a task actually tunes;
// rated_torque_nm is the motor's own rating; two describe the transport. The
// measured stall/ceiling constants live in detail::ImpedanceTuning, which only
// the tests construct -- the same split ForcePositionConfig uses, and for the
// same reason: there is one right answer for this hardware.
struct ImpedanceConfig {
    float kp = 20.0f;                      // Nm/rad
    float kd = 1.0f;                       // Nm*s/rad
    float feedforward_torque = 0.0f;       // Nm
    // The error clamp, expressed as the torque it may produce. Also sets the
    // approach speed: the jaw accelerates until damping balances the clamped
    // error torque, roughly max_position_torque_nm / kd rad/s -- 1.5 rad/s at
    // these defaults. 0 disables it, which is not recommended.
    float max_position_torque_nm = 1.5f;
    // Output ceiling, on MEASURED torque. Capped at the motor's rated torque
    // because the resulting hold is indefinite. 0 disables it.
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
    // Kept alongside `state` rather than folded into it: the stall clamp can
    // still be engaged while the ceiling is what the state reports, and a
    // caller diagnosing a blocked jaw wants both.
    bool               stalled        = false;
    bool               torque_capped  = false;
    uint64_t           stall_trips    = 0;
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
    // STALL = THE ERROR CLAMP IS BINDING AND THE JAW IS NOT MOVING.
    //
    // Not "feedback torque above an absolute number", which is what ControlLoop
    // uses (1.2 Nm) and what this class inherited before it was measured. Under
    // an active error clamp that threshold is unreachable, so the guard is
    // decorative: measured on 1.1.6 hardware with a genuine 5-second stall at
    // max_position_torque_nm = 0.35, the command saturated at 0.362 Nm and the
    // feedback peaked at 0.271 Nm -- a factor of 4.4 below the 1.2 Nm trip. The
    // loop reported TRACKING throughout while the jaw was hard against the
    // mechanism. At the 1.5 Nm default clamp the best observed ratio still puts
    // feedback around 1.1 Nm, i.e. on the wrong side of 1.2.
    //
    // That is not a tuning miss, it is the same structural error the
    // force/position controller documents at length: the command is bounded by
    // max_position_torque_nm and the feedback runs under the command, so any
    // trip placed near the cap can never fire. The clamp binding IS the signal
    // -- it means the controller is asking for everything it is allowed to ask
    // for -- and it is reachable by construction.
    //
    // Why it matters rather than being cosmetic: with the guard silent, a
    // blocked jaw sits at the full clamp torque indefinitely. Sustained torque
    // is what browns out the 24 V rail and drops the USB link, and the default
    // clamp of 1.5 Nm is the same order as the 1.5 Nm grasp that did exactly
    // that in the field.
    //
    // A FLOOR, not a threshold. It only has to reject "commanded but the motor
    // is not actually pushing" (disabled, unpowered); the clamp-binding and
    // velocity tests do the separating. Same role and same value as the
    // force/position contact floor.
    float    stall_torque_floor_nm = 0.080f;
    float    stall_vel_radps  = 0.15f;
    unsigned stall_hold_ms    = 60;
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
    bool stalled() const noexcept { return stall_clamped_; }
    bool torque_capped() const noexcept { return torque_capped_; }
    uint64_t stall_trips() const noexcept { return stall_trips_; }
    uint64_t torque_caps() const noexcept { return torque_caps_; }
    const std::string& fault_reason() const noexcept { return fault_reason_; }

private:
    void guard_torque_(const MotorStatusSample& s,
                       std::chrono::steady_clock::time_point now);
    void guard_stall_(const MotorStatusSample& s,
                      std::chrono::steady_clock::time_point now);
    float clamped_target_();
    protocol::MotorImpedanceCtrl zero_(const MotorStatusSample& s);

    GripperPosition map_;
    ImpedanceConfig cfg_;
    ImpedanceTuning tune_;
    ImpedanceState  state_ = ImpedanceState::Idle;
    float target_    = 0.0f;
    float effective_ = 0.0f;
    float kp_ = 0.0f, kd_ = 0.0f, ff_ = 0.0f;
    float commanded_torque_nm_ = 0.0f;

    std::chrono::steady_clock::time_point stall_since_{};
    bool  stall_clamped_ = false;
    float stall_clamp_   = 0.0f;
    bool  stall_closing_ = false;
    uint64_t stall_trips_ = 0;

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

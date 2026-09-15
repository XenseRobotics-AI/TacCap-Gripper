// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ControlLoop — a fixed-rate send/receive loop for embodied gripper control.
//
// A background thread submits the latest normalized position target as a MIT
// impedance frame at `hz` (fire-and-forget, no ACK), while the firmware's
// motor-status stream keeps the latest observation fresh. The policy thread
// only touches two thread-safe, non-blocking calls:
//
//     loop.set_target(p)        // p in [0,1] (0 = closed, 1 = open)
//     auto obs = loop.observation()
//
// This keeps the firmware's 500Hz control target fresh without the caller
// fighting timing or the GIL, and reads observations from the push stream
// instead of polling GetMotorStatus (polling > ~100Hz can stall the firmware's
// status refresh — the stream avoids that entirely).
//
// Usage:
//   FollowerGripper g = ...;
//   g.motor().enable();
//   ControlLoop loop(g, {.hz = 100, .kp = 20.0f, .kd = 1.0f});
//   loop.start();                       // seeds target = current position
//   for (;;) {                          // your policy, at its own rate
//       auto obs = loop.observation();  // latest open amount, non-blocking
//       loop.set_target(policy(obs));   // 0..1
//   }
//   loop.stop();
//   g.motor().disable();
//
// While the loop runs it OWNS the control + telemetry path for this gripper:
// don't issue other motor commands or start/stop streaming on the same gripper
// concurrently.
//
// The reason is NOT frame corruption on the host. Transport serialises writers
// internally, and a blocking tty write() is whole-call atomic anyway (measured:
// 40 x 100 kB writes against a deliberately starved drain produced zero short
// writes, so the partial-write loop in SerialBus never even runs). The reason
// is the firmware: host->MCU traffic that overlaps the MCU's own transmission
// makes it drop bytes out of the middle of the frame it is sending. Measured on
// hw_v1.1.0 with a 100Hz motor-status stream -- the damaged frame arrives with
// its payload short by a couple of bytes, fails the LEN check (so crc_errors
// stays 0, only resync_bytes moves) and is discarded whole. The rate that
// survives is not a fixed ceiling: 250Hz submits lost frames on one run and not
// on the next, because what matters is whether a command happens to land inside
// the ~137us the MCU spends transmitting each 41-byte status frame.

#pragma once

#include <taccap/components/motor.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/gripper_position.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace xense::taccap {
namespace detail { class ControlRuntime; }

// Latest gripper observation, refreshed from the motor-status stream.
struct GripperObservation {
    bool     valid    = false;   // false until the first status frame arrives
    float    position = 0.0f;    // [0,1] normalized open amount (0=closed,1=open)
    float    velocity = 0.0f;    // rad/s (raw motor frame)
    float    torque   = 0.0f;    // Nm
    float    raw_pos  = 0.0f;    // raw shaft angle (rad)
    uint16_t status   = 0;       // protocol::MotorStatusBit::*
    // Motor temperature (deg C). It rides in every status frame already, and
    // is here so that nothing needs read_status() while a control loop runs.
    // That call is an ACK round-trip: its request can land inside the MCU's own
    // transmission and destroy the status frame in flight, and its reply can be
    // corrupted by the loop's submits. The stream-locked submit phase cannot
    // protect it -- the loop has no idea when a caller decides to poll.
    float    motor_temp_c = 0.0f;
    uint64_t seq      = 0;       // count of stream updates received so far
    double   age_ms   = 0.0;     // age of this sample when observation() returned
};

class ControlLoop {
public:
    // When the loop puts its MIT frame on the wire.
    //
    // The MCU drops bytes out of the middle of a status frame it is
    // transmitting if host->MCU traffic overlaps that transmission (see the
    // note above). A 41-byte status frame at 3 Mbps occupies ~137us of each
    // 10ms period, so a free-running submitter collides only occasionally --
    // which is exactly what makes the resulting rate drop look sporadic and
    // unreproducible.
    //
    // StreamLocked removes the collision instead of making it rarer: it
    // submits once per received status frame, so every write lands in the
    // ~9.8ms the MCU is known to be idle. Config::hz is then ignored -- the
    // submit rate becomes motor_stream_hz. This is the same shape as ARX5's
    // send_recv_once: one owner, one exchange per cycle, send phase-locked to
    // receive. Measured: 8 runs of 60s, free-running lost 5..154 status frames
    // per run, stream-locked lost none while putting MORE traffic on the link.
    //
    // It also holds under a full production load. With every camera on both
    // grippers streaming (4 tactile at 640x480 MJPG 120fps + 2 wrist at 30fps,
    // all on the same USB tree), four 60s runs came back at exactly 6000
    // submits : 6000 frames : 0 missing. Not "close to zero" -- every submit
    // matched by a frame. Free-running at 100Hz on the same bench lost 156..308
    // frames per run, with or without the cameras; the camera load barely moved
    // it, because the variable that matters is *when* a write lands, not how
    // busy the bus is.
    //
    // A warning for whoever re-measures this: an early 600s free-running run at
    // 100Hz came back completely clean, and that was wrong to generalise from.
    // Only one gripper assembly was plugged in at the time (4 USB devices
    // instead of 8). With both assemblies attached, the same 100Hz free-running
    // configuration loses 4% of its frames. Bench population changes the
    // answer, so measure on a bus populated like the one you ship on.
    //
    // The margin is not tight. One 41-byte status frame at 3 Mbps fills ~137us
    // of each 10ms period -- 1.4% -- so the window we aim at is ~9.86ms wide and
    // the sub-millisecond jitter in "the MCU just finished sending" is nowhere
    // near enough to matter.
    //
    // What it does NOT protect, because the honest scope matters more than the
    // headline:
    //
    //   - ACK responses. The loop knows when the MCU emits *telemetry*; it has
    //     no idea when the MCU is answering somebody's command. Measured with
    //     no stream running and a 100Hz GetMotorStatusExt poll, adding 250Hz of
    //     concurrent no-ACK traffic corrupts responses that the control runs
    //     never lose -- on firmware 1.1.2.0 that was 5-6 per 6000 commands, and
    //     on 1.1.4.0 (logging off) 1-2. The count dropped; the exposure did
    //     not. Control runs stayed at zero on both, test runs non-zero on both.
    //     The saving grace is that commands RETRY -- every one of those was
    //     recovered, costing ~31ms each and surfacing to the caller as latency,
    //     never as failure. Stream frames have no retry, which is why the same
    //     defect reads as a rate drop on telemetry and as nothing at all on
    //     commands.
    //
    //     (Firmware 1.1.4.0 also roughly halved command latency outright --
    //     877us to 489us mean on the quiet control arm -- by no longer blocking
    //     tasks on its debug logging. That is a real gain, but it is a gain in
    //     latency, not in collision immunity.)
    //   - (This used to warn that turning on IMU and encoder would shrink the
    //     idle window. It cannot: the follower firmware streams motor status
    //     and nothing else -- every other source sits inside
    //     #ifdef ENABLE_MASTER_GRIPPER in task_data_stream.c. FollowerGripper::
    //     start_streaming() still takes imu_hz and encoder_hz and will set the
    //     mask bits, but the follower ignores them; requesting 1000Hz of each
    //     yields zero frames and leaves byte volume unchanged, measured. So the
    //     transmit duty cycle here is fixed at that 1.4% and the margin cannot
    //     erode. The warning was real for the leader, which does stream all
    //     four sources -- but ControlLoop only takes a FollowerGripper, so it
    //     never applies to this class.)
    //   - Callers that must write asynchronously by construction. If your
    //     architecture sends when an external event says to, no phase this loop
    //     chooses can help you.
    //
    // So this is avoidance, not a fix. The defect is in the firmware's UART
    // path (tc-gu-01 issue #1) and stays worth fixing there.
    enum class SubmitPhase : uint8_t { FreeRunning, StreamLocked };

    // What to do when the jaw is blocked. A position target the jaw cannot
    // reach makes kp * error a constant torque demand that nothing in this
    // loop bounds. Measured on follower firmware 1.1.5, kp=8, stepping the
    // target to fully-closed with a rigid object in the jaws: the jaw
    // accelerated to 8.5 rad/s (17x the speed a bounded close uses), hit the
    // object at 5.4 rad/s, and torque went 0.03 -> 2.41 Nm in 20 ms and was
    // still climbing toward the kp*error plateau of ~4.96 Nm when the test
    // aborted. It also drove 0.078 rad PAST the object's own stop, deforming
    // it.
    //
    // Nothing below this layer stops that on 1.1.5. The firmware's limit-stall
    // guard (can_motor.c can_motor_gripper_stop_on_limit_stall) is wired only
    // into the VELOCITY command paths and only near the travel ends -- the MIT
    // impedance path has no stall protection at all, leaving the motor's own
    // 0x700B ceiling (6 Nm, vs a 1.8 Nm rated torque) as the sole backstop.
    // Historical 1.1.5 behaviour; 1.1.7 adds a firmware envelope to all external modes.
    enum class StallAction : uint8_t {
        // Leave the caller's target alone. The pre-guard behaviour.
        None,
        // Clamp the effective target at the position the jaw actually reached,
        // so position error -- and therefore torque -- stops growing. The loop
        // keeps running and needs no fault clearing; commanding a target back
        // the other way releases the clamp.
        HoldPosition,
    };

    struct Config {
        // Target submission rate (Hz). Only consulted by
        // SubmitPhase::FreeRunning -- StreamLocked takes its rate from the
        // status stream. The default matches what StreamLocked actually
        // produces, so the two phases agree out of the box and switching to
        // FreeRunning does not silently change the rate as well as the phase.
        unsigned status_timeout_ms   = 350;
        unsigned hz                  = 100;
        float    kp                  = 20.0f;  // impedance stiffness (Nm/rad)
        float    kd                  = 1.0f;   // impedance damping (Nm·s/rad)
        float    feedforward_torque  = 0.0f;   // Nm
        unsigned motor_stream_hz     = 100;    // motor-status stream rate (Hz)
        SubmitPhase phase            = SubmitPhase::StreamLocked;

        // Bound the position error the loop is allowed to command, expressed
        // as the torque it would produce: the commanded target is clamped to
        // within max_position_torque_nm / kp radians of where the jaw actually
        // is. 0 disables it.
        //
        // This is the primary protection, and it is deliberately NOT a slew
        // rate limit on the target. A slew limit adds lag to every motion,
        // including teleoperation where the leader is tracked continuously.
        // An error clamp costs nothing while the jaw keeps up -- the error is
        // small and the clamp never binds -- and engages the instant the jaw
        // falls behind, which is exactly the blocked case.
        //
        // It bounds BOTH failure modes measured on 1.1.5, and the stall guard
        // below only catches one of them:
        //
        //   - The approach. An unclamped step to a far target demands
        //     kp * 1.2 rad = 9.7 Nm (the motor's 0x700B trims it to 6) and the
        //     jaw reaches 8.5 rad/s. Clamped, the demand is
        //     max_position_torque_nm and the kd term balances it at roughly
        //     max_position_torque_nm / kd rad/s -- 1.5 rad/s at the defaults.
        //   - The stall. kp * error can no longer grow past the clamp, so a
        //     blocked jaw settles at max_position_torque_nm instead of the
        //     ~4.96 Nm plateau measured with a rigid object at mid-travel.
        //
        // Why the approach matters as much as the stall: measured with a
        // loose rigid object in the jaws, an unclamped step crossed the
        // object's position at 5.5 rad/s and knocked it clean out WITHOUT the
        // torque rising at all (feedback stayed under 0.03 Nm through the
        // crossing). No post-contact guard can catch that -- there is no
        // sustained contact to detect. Only not arriving that fast works.
        float       max_position_torque_nm = 1.5f;

        // Output torque ceiling -- the hard backstop, and the only layer that
        // acts on what the motor is ACTUALLY producing rather than on what the
        // loop asked for. The firmware's actual_torque is derived from motor
        // current, so this is a current measurement.
        //
        // When |feedback| stays at or above rated_torque_nm for
        // rated_hold_ms, the loop stops sending an impedance frame entirely
        // and sends a pure feed-forward hold: kp=0, kd=0, tau_ff = the ceiling
        // with the sign the motor was already pushing. With both gains at
        // zero, position error cannot contribute to the output at all -- the
        // torque is pinned at the ceiling instead of merely bounded by an
        // estimate of it. Same primitive ForcePositionController uses after
        // contact, for the same reason.
        //
        // Why this exists on top of max_position_torque_nm: that clamp bounds
        // kp * error, which is the COMMANDED torque. Measured on 1.1.5, the
        // feedback at stall ran about 0.59 of the command -- but that ratio is
        // one unit, one temperature, one load. A ceiling on the measurement
        // does not care what the ratio is.
        //
        // 0 disables this host guard. RS05 stationary rating is 1.2 Nm under
        // specified cooling; higher values are transient allowances. Firmware
        // 1.1.7 independently applies its configured continuous/peak envelope.
        float       rated_torque_nm   = 2.0f;
        unsigned    rated_hold_ms     = 20;
        // While the ceiling holds, this much travel away from where it engaged
        // means the obstruction is gone, so impedance control resumes. Without
        // it a pure tau_ff hold would keep accelerating a jaw that came free.
        float       rated_release_rad = 0.05f;

        // ---- Stall guard (see StallAction) --------------------------------
        // Stalled = torque at or above stall_torque_nm while the jaw is slower
        // than stall_vel_radps, continuously for stall_hold_ms. Both halves
        // are required: torque alone trips on the impact transient (measured
        // 1.33 Nm while still moving at 1.94 rad/s), and slowness alone is
        // just a jaw at rest.
        //
        // The default trip sits above anything free motion produces and above
        // a firm deliberate grasp. It is a contact threshold, not a guarantee
        // that 1.2 Nm can be held indefinitely under arbitrary cooling.
        // Measured free-motion peaks on 1.1.5: 0.29 Nm stepping a quarter of
        // the travel, 0.68 Nm on a full 1.0 -> 0.0 step. Note that torque is
        // NOT what keeps those from tripping the guard -- 0.68 is close to
        // 1.2 -- the velocity test is: a free jaw runs at several rad/s
        // through the whole transit, orders of magnitude above
        // stall_vel_radps. Raise stall_torque_nm only together with a reason
        // the velocity gate is not doing its job.
        float       stall_torque_nm  = 1.2f;
        float       stall_vel_radps  = 0.15f;
        unsigned    stall_hold_ms    = 60;
        StallAction stall_action     = StallAction::HoldPosition;
    };

    explicit ControlLoop(FollowerGripper& gripper);   // default Config
    ControlLoop(FollowerGripper& gripper, Config cfg);
    ~ControlLoop();

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // Start the motor-status stream + submit thread. Seeds the target with the
    // current position so that (with the motor already enabled) starting does
    // not produce a jump. Throws ProtocolError if the gripper isn't calibrated.
    void start();
    void stop(); // disables motor and releases this controller's ownership
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    // Thread-safe, non-blocking. set_target clamps to [0,1].
    void  set_target(float position01);
    void  set_gains(float kp, float kd, float feedforward_torque);
    float target() const;

    // Thread-safe snapshot of the latest observation (non-blocking).
    GripperObservation observation() const;

    // Loop diagnostics.
    float    submit_hz()    const noexcept { return submit_hz_.load(std::memory_order_relaxed); }
    uint64_t submit_count() const noexcept { return submit_count_.load(std::memory_order_relaxed); }

    // Stall guard state. stalled() is true while the clamp is holding the
    // effective target back; stall_trips() counts how many times it engaged.
    bool     stalled()     const noexcept { return stalled_.load(std::memory_order_relaxed); }
    uint64_t stall_trips() const noexcept { return stall_trips_.load(std::memory_order_relaxed); }

    // Output torque ceiling state. torque_capped() is true while the loop is
    // sending a pure tau_ff hold instead of an impedance frame.
    bool     torque_capped() const noexcept { return torque_capped_pub_.load(std::memory_order_relaxed); }
    uint64_t torque_caps()   const noexcept { return torque_caps_.load(std::memory_order_relaxed); }

    const Config& config() const noexcept { return cfg_; }

private:
    // Read the target under mu_ and put one MIT frame on the wire. False means
    // the write failed and the loop has already set stop_flag_.
    bool submit_once_();
    void note_rate_(uint64_t& window_count,
                    std::chrono::steady_clock::time_point& window_start);
    void on_status_(const MotorStatusSample& s);
    // Called from on_status_ under mu_.
    void guard_stall_(const MotorStatusSample& s,
                      std::chrono::steady_clock::time_point now);
    void guard_torque_(const MotorStatusSample& s,
                       std::chrono::steady_clock::time_point now);
    // Called from submit_once_ under mu_. Returns the target actually sent.
    float clamped_target_() noexcept;

    FollowerGripper& g_;
    Config           cfg_;
    GripperPosition  pos_map_;

    std::unique_ptr<detail::ControlRuntime> runtime_;
    uint64_t rate_count_ = 0;
    std::chrono::steady_clock::time_point rate_start_{};
    std::atomic<bool>  running_{false};

    mutable std::mutex mu_;
    float              target_ = 0.0f;   // normalized [0,1]
    float              kp_     = 0.0f;
    float              kd_     = 0.0f;
    float              ff_     = 0.0f;
    GripperObservation obs_;
    std::chrono::steady_clock::time_point obs_time_{};

    std::chrono::steady_clock::time_point stall_since_{};
    bool  stall_clamped_ = false;
    float stall_clamp_   = 0.0f;   // normalized position the jaw stopped at
    bool  stall_closing_ = false;  // was the blocked motion toward closed?
    std::atomic<bool>     stalled_{false};
    std::atomic<uint64_t> stall_trips_{0};

    // Output torque ceiling, under mu_ except the atomics.
    std::chrono::steady_clock::time_point cap_since_{};
    bool  torque_capped_ = false;
    float cap_sign_      = 1.0f;   // direction the motor was pushing
    float cap_entry_raw_ = 0.0f;   // raw position where the ceiling engaged
    bool  cap_closing_   = false;  // was the blocked motion toward closed?
    std::atomic<bool>     torque_capped_pub_{false};
    std::atomic<uint64_t> torque_caps_{0};

    std::atomic<uint64_t> submit_count_{0};
    std::atomic<float>    submit_hz_{0.0f};
};

}  // namespace xense::taccap

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Motor component: typed wrapper around the motor command set (enable/
// disable/clear_fault, four control modes) and the GetMotorStatus telemetry
// path (one-shot read + continuous DATA stream).
//
// Only relevant to the follower gripper — on the leader these commands
// return NACK with ErrorCode::SensorOffline (firmware reports the motor as
// absent), which the component surfaces as ProtocolError just like any
// other NACK.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

// The EduLite05's two torque ratings. Every ceiling in this SDK is one of these
// rather than an invented margin: RATED is what the motor may hold
// indefinitely, PEAK what it may draw transiently. 6.0 Nm is also the
// firmware's own default AND maximum for the persisted 0x700B startup limit
// (storage.c STORAGE_MOTOR_LIMIT_TORQUE_{DEFAULT,MAX}_NM).
//
// Holding above RATED is not primarily a thermal risk: the motor's undervoltage
// protection is prompt, and a sustained draw can brown out the 24 V rail and
// take the USB link with it.
// FALLBACK DEFAULTS ONLY -- EL05 numbers. They exist so the config structs have
// something to initialise to before a device is open; they are NOT the authority.
// Ask the device with Motor::get_spec(): an RS00 is rated 5.0 Nm and peaks at
// 14.0, and nothing in this repository knows which actuator is plugged in.
constexpr float MOTOR_RATED_TORQUE_NM = 1.8f;   // EL05 rated  — fallback
constexpr float MOTOR_PEAK_TORQUE_NM  = 6.0f;   // EL05 peak   — fallback
// The third number, and the one the other two get mistaken for. RATED is the
// ROTATING rating; what a blocked jaw may hold indefinitely is the STALL
// rating, and the manual's overload curve puts those far apart:
//     6.0 Nm -> 1 s   4.0 -> 6 s   1.8 -> 175 s   1.1 -> indefinite
// A gripper's main duty cycle IS the blocked hold, so this is the number that
// governs it. The firmware clamps a host-supplied envelope cont_torque_nm down
// to its own per-model copy of it (motor_spec.c), which is why writing RATED
// there does not raise the grip -- it only makes the record disagree with what
// is enforced. Same fallback caveat as above: ask the device.
constexpr float MOTOR_STALL_CONT_TORQUE_NM = 1.1f;  // EL05 continuous stall — fallback

struct MotorStatusSample {
    std::chrono::steady_clock::time_point host_time;
    float    actual_pos;        // rad
    float    actual_vel;        // rad/s
    float    actual_torque;     // Nm
    float    motor_temp_c;      // °C
    uint16_t status;            // protocol::MotorStatusBit::* bits
    // target_* / control_mode are correct on V1.9 firmware (31-byte status).
    // (V1.9 dropped actual_current / target_current / current_source.)
    float    target_pos;        // rad   — last applied target
    float    target_vel;        // rad/s
    float    target_torque;     // Nm
    uint8_t  control_mode;      // protocol::MotorMode

    // DIAGNOSTICS, carried by the DATA stream since the follower started
    // emitting the 59-byte V2 status instead of the 31-byte legacy prefix.
    //
    // They used to be reachable only by polling Cmd::GetMotorStatusExt, and
    // polling collides with the phase-locked control frames -- so diagnosing a
    // fault meant stopping control, and faults happen while controlling. Zero
    // on firmware that still streams the 31-byte prefix, because the decoder
    // zero-fills the tail; check `stop_reason`/`monitor_version` against that
    // rather than assuming a field is meaningful.
    uint8_t  monitor_version;   // protocol::MOTOR_MONITOR_VERSION when present
    uint8_t  stop_reason;       // protocol::MotorStopReason
    uint8_t  monitor_reserved;  // MOTOR_MONITOR_DIAG_* bits
    uint32_t fault_code;        // live motor fault word
    uint32_t latched_fault_code;// OR of everything seen since power-on

    protocol::MotorStatusExt raw;
};

class Motor {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const MotorStatusSample&)>;

    explicit Motor(bus::Transport& transport);

    // ---- Lifecycle / fault management --------------------------------------
    void enable();
    void disable();
    void clear_fault();

    // ---- Control modes -----------------------------------------------------
    // Each call sends one Cmd::Motor*Ctrl frame and waits for ACK. Caller is
    // responsible for the control loop cadence — there is no host-side
    // interpolation or trajectory smoothing here.
    void set_position(float target_pos_rad,
                      float max_vel_radps,
                      float max_torque_nm);
    void set_velocity(float target_vel_radps,
                      float max_torque_nm,
                      float profile_acc_radps2);
    void set_torque(float target_torque_nm,
                    float max_vel_radps);
    void set_impedance(float target_pos_rad,
                       float kp_nm_per_rad,
                       float kd_nm_s_per_rad,
                       float feedforward_torque_nm,
                       float feedforward_vel_radps = 0.0f);  // V1.7; MIT only

    // ---- High-rate control submission (no ACK) -----------------------------
    // Fire-and-forget MIT control frames for a host-driven realtime loop.
    // These send a CMD_NO_ACK frame and return immediately: there is NO ACK,
    // NO NACK, NO retry, NO timeout, and NO throw on a target the firmware
    // rejects. The firmware's slave control task consumes the *latest*
    // submitted target. Unlike set_*(), which block on an ACK and throw
    // ProtocolError on NACK, submit() never blocks.
    //
    // ---- On submission rate ------------------------------------------------
    // This used to say you could submit "up to the firmware's 500Hz slave
    // control rate", which read as a budget to spend. It is not one.
    //
    // The 500Hz figure is real — that is how often the firmware's control task
    // applies the latest target — but it says nothing about what submitting at
    // that rate costs you. Every host->MCU frame that lands while the MCU is
    // transmitting makes it drop bytes out of the middle of the frame it is
    // sending, and that frame is then discarded whole (tc-gu-01 issue #1). A
    // 41-byte status frame at 3 Mbps fills only ~137us of each 10ms period, so
    // whether a given submit collides is down to *when* it lands, not how many
    // you send:
    //
    //   - 250Hz lost 154 status frames on one 60s run and none on the next.
    //   - 300Hz was clean on a run where 250Hz was not.
    //   - 1000Hz has produced both 0 and 146 lost frames on the same firmware.
    //
    // So rate does not predict loss; it only sets how many chances to collide
    // you take per second. Submitting faster than the status stream also buys
    // nothing on the observation side, since motor status is capped at 100Hz.
    //
    // Prefer ImpedanceController or ForcePositionController, which submit once
    // per received status frame and therefore never overlap a transmission —
    // stream-locked submission measured at zero lost frames across 8 runs while
    // sending MORE frames than the free-running comparison. Reach for raw submit() at your own cadence
    // only when you cannot ride the status stream.
    //
    // Separately: sustained input above a few hundred Hz used to livelock the
    // firmware's command handler outright (tc-gu-01 issue #2, fixed in firmware
    // 1.1.3). Against older firmware, high-rate submission can leave the device
    // streaming happily while accepting no commands at all until power-cycled.
    //
    // Health/error feedback is OUT-OF-BAND — poll these off the realtime thread,
    // never inside the submit loop:
    //   - control_stats(): target_seq vs applied_seq, actual_hz, error_count,
    //     last_error, target_age_ms (host-submit vs firmware-applied cadence).
    //   - on_status(): MotorStatusBit::Fault/Stalled/OverTemp/... + target/actual mirror.
    //   - SensorErrors stream: async SensorErrorId::Motor reports.
    //
    // MIT protocol is assumed (the impedance `vel` feed-forward is MIT-only);
    // there is no per-call protocol check in the hot path. Preconditions are
    // the caller's: enable() and a cleared fault. The only exception is
    // IoError if the transport has been stopped.
    //
    // V1.9 gate: while the firmware owns the motor (notably during power-on
    // auto-calibration, if enabled), external control is refused — ACK commands
    // (enable/set_*) NACK ErrorCode::SysBusy, and submit_* frames are silently
    // dropped (no-ACK). There is no protocol query for this state; detect it by
    // watching control_stats().applied_seq stop advancing (or the motor not
    // moving) and hold off until auto-cal settles after power-on.
    // NOTHING TIMES THESE OUT ON THE DEVICE. The firmware's slave control task
    // keeps applying the LAST submitted frame forever: there is no host-command
    // watchdog anywhere in tc-gu-01's tasks (no last-command timestamp, no
    // staleness check), and the hardware IWDG in task_monitor.c is commented
    // out. So a control frame is not "valid until superseded", it is valid
    // until the end of time.
    //
    // The consequence is the failure this SDK cannot fix from the host.
    // Measured on 1.1.6: a grasp at 1.2 Nm, the USB link dropped mid-close, and
    // the motor was still pushing 1.256 Nm two minutes later with its
    // temperature risen 33 -> 45 C. Both host-side safe states failed with
    // Input/output error, because the device they needed to write to was gone
    // -- the controllers' stop() zero-torque frame and Motor::disable() alike.
    // Recovery took a manual reconnect and disable; nothing on the host could
    // have done it automatically.
    //
    // The only backstops left are thermal and slow: the motion envelope's I2t
    // derate and temperature wall, and only if the envelope is ENFORCED.
    //
    // FIXED IN FIRMWARE on tc-gu-01 branch feat/actuator-can-timeout: the slave
    // control task now watches how old its cached target is and degrades in two
    // stages (300 ms -> zero-speed hold at 0.35 Nm, 30 s -> disable), reporting
    // MotorStopReason::HostTimeout. Verified on hardware: 0.851 -> 0.157 Nm
    // within 1 s with the workpiece still held.
    //
    // Note the actuator's own 0x7028 CAN timeout does NOT cover this case, even
    // though it sounds like it should: this task re-sends its cached target
    // every 2 ms, so the motor keeps receiving CAN commands and that timeout
    // never fires. 0x7028 covers the MCU dying; the task watchdog covers the
    // host dying. Both are needed.
    //
    // Until a gripper is running firmware with that fix, treat "the link can
    // drop while loaded" as a physical hazard rather than a software state.
    void submit(const protocol::MotorImpedanceCtrl& c);  // primary (MIT hybrid)
    void submit(const protocol::MotorPosCtrl& c);
    void submit(const protocol::MotorVelCtrl& c);
    void submit(const protocol::MotorTorqueCtrl& c);

    // Float-arg convenience forms mirroring set_*; used by the Python bindings.
    void submit_impedance(float target_pos_rad,
                          float kp_nm_per_rad,
                          float kd_nm_s_per_rad,
                          float feedforward_torque_nm,
                          float feedforward_vel_radps = 0.0f);
    void submit_position(float target_pos_rad,
                         float max_vel_radps,
                         float max_torque_nm);
    void submit_velocity(float target_vel_radps,
                         float max_torque_nm,
                         float profile_acc_radps2);
    void submit_torque(float target_torque_nm,
                       float max_vel_radps);

    // ---- Telemetry ---------------------------------------------------------
    MotorStatusSample read_status(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // V2.2 — the 72-byte extended status (Cmd 0x53). Superset of read_status():
    // same pose/torque/status prefix plus the firmware's fault word, its latched
    // and stop-time snapshots, collection-health flags and the raw CAN evidence.
    //
    // read_status() is still the right call for a control loop — it is what the
    // DATA stream carries and it stays cheap. Reach for this one when something
    // has already gone wrong and you need to know *what*.
    //
    // Firmware older than follower 1.1.2 does not implement 0x53 and NACKs
    // InvalidCmd -> ProtocolError. There is no capability query; either gate on
    // FollowerGripper::firmware_version() or catch the throw.
    protocol::MotorStatusExt read_status_ext(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // V2.2 — the 64-byte diagnostic fault report (Cmd 0x52). Merges the motor's
    // own fault word, the MCU's firmware-level fault state and the raw CAN
    // reply into one snapshot.
    //
    // `force` = false returns the firmware's cached report (cheap, safe to poll).
    // `force` = true makes the MCU issue a fresh cmd-5 read on the CAN bus
    // before answering — it costs a bus round trip and can perturb a running
    // control loop, so keep it for on-demand diagnostics, not polling.
    //
    // The firmware ACKs OK even when there is nothing to report; inspect
    // report_flags (MotorFaultReportFlag::MotorValid / FwValid) before trusting
    // the fault code fields. Same 1.1.2 firmware floor as read_status_ext().
    protocol::MotorFaultReport fault_report(
        bool force = false,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // The version of the program inside the MOTOR (firmware >= 1.2.6). Not the
    // gripper's own firmware version — that is FollowerGripper::firmware_version().
    //
    // Check `valid` before reading `version` — this call reports failure in the
    // payload rather than throwing, because "the motor did not answer" and "the
    // command is not supported here" are different diagnoses and one NACK
    // cannot say which.
    //
    // MEASURED: readable under the PRIVATE protocol (0074s and 0018s, both
    // answered 1.0.5.0.x). Whether MIT mode also answers is NOT established —
    // the request is an extended frame and MIT is expected to ignore those, but
    // that expectation has never been tested against a unit running 1.2.6 in
    // MIT. Do not build a startup check on the assumption either way until it
    // is; switching protocols to find out costs a 24 V power cycle each way.
    //
    // MAY STOP THE MOTOR. The request reuses RobStride communication type 4,
    // which is also "motor stop"; a motor that does not recognise the 00 C4
    // magic executes it as a stop. `motor_stopped` says so — re-enable before
    // commanding motion again.
    //
    // The default timeout covers the firmware's own wait: measured round trips
    // run to ~1371 ms, so the old 500 ms default timed out before the reply
    // could possibly arrive and made a working read look like a dead motor.
    protocol::MotorVersion motor_version(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

    // Subscribe to streamed MotorStatus DATA frames (StreamSrc::MotorStatus
    // must be enabled in start_streaming for these to arrive).
    SubId on_status(Callback cb);
    void  off(SubId id);

    // ---- Follower motor admin (zero / CAN id / protocol / stats) -----------
    // Follower-only, validated against firmware hw_v1.1.0. On leader hardware
    // these NACK (SensorOffline) -> ProtocolError, like the other motor
    // commands — that is the correct way to surface a leader/follower mismatch,
    // not a stub.
    void              set_zero();                              // Cmd 0x33 (zero)
    uint8_t           get_can_id();                            // Cmd 0x34
    void              set_can_id(uint8_t can_id);              // Cmd 0x35
    // Cmd 0x36 / 0x37. Both send with a 3000 ms ACK timeout of their own rather
    // than the transport default: measured 2026-09-24 on TCGU01A28Z0015s, each
    // timed out on every attempt at the default 200 ms while the motor spoke
    // the private protocol, and both answered at 3000 ms. The switch triggers a
    // discovery scan of up to ~1.25 s in firmware, during which the receive
    // buffer is flushed repeatedly. Callers need do nothing; before the
    // per-command timeout the symptom was a TimeoutError on a command that
    // works fine in the other protocol, which reads as a dead link.
    //
    // The switch persists and needs a 24 V power cycle to take effect.
    void              switch_protocol(protocol::MotorProtocol);
    protocol::MotorProtocol get_protocol();
    // Single-parameter access to the motor (Cmd 0x38/0x39). The firmware
    // whitelists index + R/W; the 4-byte raw_value is interpreted per
    // MotorPrivateParam::type (u8 / f32).
    //
    // THE MIT GATE IS GONE AS OF THE 1.2.6 FIRMWARE, and this comment used to
    // say the opposite. Measured on firmware 1.1.5 against 0x7017, the firmware
    // rejected every index up front whenever the protocol mode was MIT, so both
    // calls raised ProtocolError(InvalidParam) there no matter the index. That
    // blanket gate was removed on purpose -- protocol_handler.c records that it
    // was what made vBus / iqf / 0x7028 unreadable under MIT, while MIT's own
    // instruction 17/18 parameter channel existed all along. What remains is
    // narrower: protocol_check_motor_param_access_allowed() refuses during OTA,
    // and returns SysBusy while the firmware's control task is running.
    //
    // MEASURED 2026-09-23, and the answer is no: the motor does not reply under
    // MIT even though the firmware now forwards the request. On TCGU01A28Z0015s,
    // firmware 1.2.6, get_protocol() == Mit, control loop stopped, indices
    // 0x701C / 0x7017 / 0x7028 / 0x700B all came back
    // ProtocolError(Timeout) -- not InvalidParam. Timeout is the proof the
    // forward happened: the old blanket gate answered InvalidParam up front,
    // before any CAN traffic.
    //
    // THE CONTROL WAS RUN 2026-09-24 and it pins the cause to MIT. Same unit,
    // same firmware, switched to the private protocol and power-cycled: all four
    // indices answered immediately -- 0x701C 24.1063 V, 0x7017 50.0, 0x7028 0,
    // 0x700B 6.0 N*m. So the motor is not broken and the index set is not
    // wrong; the parameter channel simply does not reply while it speaks MIT.
    //
    // The practical rule therefore stands -- no motor parameters under MIT --
    // and it is now a property of the protocol rather than an open question.
    // Reading them means switching protocols, which costs a 24 V power cycle
    // each way, so there is still no runtime path. For the 0x700B limit use
    // set_startup_limit_torque(), which needs no parameter access at all.
    //
    // For the 0x700B limit use set_startup_limit_torque(), which is applied at
    // boot and needs no parameter access at all.
    protocol::MotorPrivateParam get_private_param(uint16_t index);
    void set_private_param(uint16_t index, uint32_t raw_value);
    protocol::MotorControlStats control_stats(                 // Cmd 0x51
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // V2.2 — power-on limit-torque config (Cmd 0x3A / 0x3B), persisted in MCU
    // flash. On every boot the firmware writes this value to the motor's 0x700B
    // limit_torque instead of the old hard-coded 6 Nm. Independent of the
    // current CAN protocol: unlike set_private_param(0x700B, ...) these two work
    // under MIT as well as Private.
    //
    // Note the coupling in the other direction — a successful
    // set_private_param(0x700B, ...) *also* rewrites this stored value, so a
    // private-protocol torque tweak silently becomes the new boot default.
    void  set_startup_limit_torque(float torque_nm);
    float get_startup_limit_torque();

    // The device's own motor ratings. Prefer these over the MOTOR_*_TORQUE_NM
    // constants below, which are EL05 values compiled in as a fallback and are
    // wrong for any other actuator (RS00: 5.0 rated / 14.0 peak). A zero field
    // means the firmware's table does not have that number yet -- unknown, not
    // zero. Throws on firmware older than the command (1.1.6.26).
    //
    // timeout of zero keeps the transport's configured ack timeout, which is
    // what every caller got before the parameter existed.
    protocol::MotorSpec get_spec(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{0});

    static MotorStatusSample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap

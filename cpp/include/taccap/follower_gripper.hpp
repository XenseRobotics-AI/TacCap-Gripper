// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// FollowerGripper — aggregate object representing one TacCap-Gripper follower
// (executor / robot end). Same sensor surface as LeaderGripper plus a Motor
// component that drives the FDCAN-attached actuator on the MCU.
//
// Owns:
//   - bus::Transport                       MCU control + sensor stream link
//   - IMU            via .imu()            Cmd::GetImu / DATA stream
//   - Encoder        via .encoder()        Cmd::GetEncoder / DATA stream
//   - Motor          via .motor()          enable/control + GetMotorStatus
//
// The wrist UVC camera is NOT opened by default (an external camera service
// owns that V4L2 device). It is reachable via .wrist_camera() only when
// constructed with `open_cameras=true` and the matching path; otherwise the
// accessor throws. (The OG visuotactile sensors are not handled here — they
// are read at the Python level via the xensesdk wheel.)
//
// Streaming lifecycle (extends LeaderGripper with optional motor telemetry):
//   start_streaming(motor_hz=100)
//     - motor status is the only source a follower streams; 0 throws rather
//       than starting a stream that would carry nothing
//   stop_streaming()
//
// Discovery is shared with LeaderGripper: hardware enumeration cannot
// distinguish leader-build from follower-build PCBs (firmware role is a
// runtime distinction). The caller picks which class to instantiate. A
// FollowerGripper.open() on leader hardware will succeed at construction but
// Motor commands will return ProtocolError(SensorOffline) at runtime — which
// is the right time to surface the mismatch.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/calibration.hpp>
#include <taccap/components/diagnostics.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/led.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/discovery.hpp>
#include <taccap/error.hpp>
#include <taccap/gripper_position.hpp>
#include <taccap/ota.hpp>

#include <cerrno>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace xense::taccap {

// ---- Motion safety envelope: what is wrong with a stored record -------------
//
// The firmware's clamp is opt-in and its rejections are silent, so "the device
// has an envelope" is not a yes/no question -- a record can be present, print
// as ENFORCED, and still leave the command path unbounded. These bits name each
// way that happens so a caller can act on the difference instead of guessing.
//
// These are an SDK judgement, not a wire concept: nothing here crosses the
// link, and protocol/payloads.hpp stays a pure firmware mirror.
namespace GripperEnvelopeIssue {

constexpr uint32_t None = 0;

// ---- Gating: the firmware ignores the record wholesale ----
// Mutually exclusive, evaluated in this order. A factory record is flags==0,
// which also fails the layout comparison; reporting both would read as two
// unrelated faults when there is only one -- nothing has been written.
constexpr uint32_t NotWritten     = 1u << 0;  // Valid clear
constexpr uint32_t NotEnforced    = 1u << 1;  // Valid set, Enforce clear
constexpr uint32_t LayoutMismatch = 1u << 2;  // layout nibble != LayoutVersion

// ---- Values: the record is readable, but leaves a hole or misreports ----
// Only evaluated when the layout nibble is current. That is the whole point of
// the nibble: on a mismatch the field bytes are not what they claim to be, and
// the incident it exists for looked exactly like plausible numbers.
//
// PeakUnlimited is a hole, not a nit. The firmware's position-error clamp
// returns the target untouched when peak/kp is not positive, so peak==0
// alongside Valid|Enforce leaves kp*error unbounded -- the ~12 Nm brown-out
// path the envelope exists to stop -- while the record prints as ENFORCED.
constexpr uint32_t PeakUnlimited = 1u << 3;   // peak <= 0
constexpr uint32_t ContUnlimited = 1u << 4;   // cont <= 0

// The stored value is not the enforced one: firmware clamps cont down to the
// installed motor's stall rating and says so only on a UART that is not wired
// to USB. Set only when the spec was actually read -- with no spec we do not
// know the rating and must not guess one.
constexpr uint32_t ContAboveStallRating = 1u << 5;

// ---- Advisory: reported, never repaired ----
// With peak <= cont the firmware skips the I2t derate entirely and only peak
// plus the temperature wall apply. That is TIGHTER than the recommendation,
// so it is not a hole and nothing should be widened to open the band.
constexpr uint32_t PeakNotAboveCont = 1u << 6;

// What ensure_envelope() will rewrite. PeakNotAboveCont is deliberately absent.
constexpr uint32_t RepairMask = NotWritten | NotEnforced | LayoutMismatch |
                                PeakUnlimited | ContUnlimited |
                                ContAboveStallRating;

}  // namespace GripperEnvelopeIssue

// What the device stores, what it would be told to store, and what it is
// actually enforcing right now -- which are three different things.
struct EnvelopeAudit {
    protocol::GripperEnvelope stored{};       // the record in flash
    protocol::GripperEnvelope recommended{};  // derived from this device's spec

    // What the firmware applies TODAY. Empty means it applies NOTHING: the
    // record is unwritten, not enforced, or carries a layout this firmware
    // refuses. It cannot be a zeroed struct -- in this record 0 means
    // "unlimited", which would state the opposite.
    std::optional<protocol::GripperEnvelope> effective{};

    uint32_t issues = GripperEnvelopeIssue::None;

    // Two flags, not one. The firmware's table gives EL05 a stall rating but
    // leaves it zero for every RS0x while still giving them real rated torque,
    // so a single flag would read "from device" on an RS00 whose cont silently
    // came from the compiled-in EL05 number.
    bool peak_from_device = false;
    bool cont_from_device = false;

    std::string motor_model;  // "EL05"; empty when the spec could not be read
    std::string detail;       // one human-readable line per issue

    bool ok() const noexcept { return issues == GripperEnvelopeIssue::None; }
    bool needs_write() const noexcept {
        return (issues & GripperEnvelopeIssue::RepairMask) != 0;
    }
};

struct EnvelopeWrite {
    EnvelopeAudit             before{};   // the audit that drove the decision
    bool                      wrote = false;
    protocol::GripperEnvelope written{};  // meaningful only when wrote
};

namespace detail {

// Pure policy -- no transport, no logging, no clock, so the decision table is
// testable without a device or a pty. spec absent = the device could not be
// asked, which is different from a spec that answered with zeros.
protocol::GripperEnvelope recommend_envelope(
    const std::optional<protocol::MotorSpec>& spec);

EnvelopeAudit audit_envelope(const protocol::GripperEnvelope& stored,
                             const std::optional<protocol::MotorSpec>& spec);

// Never widens. A unit deliberately tightened keeps its tighter numbers; only
// what is absent, untrustworthy or unenforceable is replaced.
protocol::GripperEnvelope repair_envelope(const EnvelopeAudit& audit);

}  // namespace detail

class FollowerGripper {
public:
    struct Config {
        std::string mcu_device;             // /dev/serial/by-id/... -if02
        std::string wrist_video;            // /dev/v4l/by-id/... -video-index0
        uint32_t    baudrate            = 3'000'000;
        // Match LeaderGripper defaults: the firmware can be slow to ACK
        // StartStream / StopStream while a previous stream is still
        // flushing — generous window + a couple of retries cover that.
        unsigned    ack_timeout_ms      = 1000;
        unsigned    max_retries         = 2;
        // Wrist camera off by default: the wrist UVC device is owned by an
        // external camera service. Set open_cameras=true (with wrist_video
        // populated) to have this gripper open it.
        bool        open_cameras        = false;
        Camera::Config wrist_cam_extra{};   // width/height/fps overrides

        // ---- Wrist fisheye undistortion (V2.0+ firmware) -------------------
        // Off by default. When true (and the wrist camera is actually opened),
        // open() reads the fisheye intrinsics the firmware persisted and every
        // frame from wrist_camera() comes out rectified.
        //
        // Degrades to raw frames with a warning when the firmware holds no
        // calibration or is too old to answer. Throws when the camera is not
        // configured at the calibrated 640x480 — the firmware record stores no
        // image size, so rescaling the intrinsics would be a guess.
        bool        undistort_wrist     = false;
        // 0 = the calibrated focal length (natural view, matches the PC tool's
        // default); 1 = 0.70x focal length for the widest field of view, with
        // correspondingly more black border. Clamped to [0,1].
        float       fisheye_balance     = 0.0f;

        // Channel order the wrist frames come out in. **RGB by default**, unlike
        // the bare Camera (which keeps OpenCV's native BGR): the wrist stream's
        // consumers are vision/learning pipelines, and every one of them wants
        // RGB — LeRobot datasets store RGB, so a BGR default meant each of them
        // converting at its own call site, or forgetting to and recording
        // swapped channels with nothing raised. Set Bgr here to get the old
        // behaviour back for code that feeds cv::imshow/imwrite directly.
        ColorMode   wrist_color_mode    = ColorMode::Rgb;

        // Refuse to open a follower whose firmware predates kMinFirmware*
        // below -- 1.2.5. Two reasons stack up to it, and they differ in kind.
        //
        // EVERY NUMBER IN THIS COMMENT IS ON THE FOLLOWER VERSION LINE. The two
        // roles number independently, so a leader's 1.2.4 has nothing to do with
        // the follower 1.2.4 named below -- do not compare them, and do not
        // widen this gate to leaders on the strength of a number that looks
        // familiar.
        //
        // 1.1.6 is where the motion safety envelope landed, and it is not
        // optional: before it, the MIT command path has NO stall protection at
        // all in firmware (can_motor_gripper_stop_on_limit_stall is wired only
        // into the velocity commands, and only near the travel ends), so a
        // blocked jaw lets kp*error grow until the motor's own 0x700B ceiling.
        // Measured on 24 V with a rigid object and kp=20: the command asked for
        // ~12 Nm, the jaw hit the object at 5.5 rad/s, and the current draw
        // browned out the whole board -- the gripper dropped what it was
        // holding and the USB link disappeared. That is the failure this SDK
        // now assumes cannot happen.
        //
        // 1.2.5 is where auto-calibration stopped writing the closed zero with
        // an inset (20 mrad through 1.2.3, 5 mrad in 1.2.4). This SDK's
        // closed-endpoint preload assumes normalized 0.0 IS the mechanical
        // stop, so on older firmware it presses past what the position map
        // calls fully closed. Bounded, not dangerous -- but a scale that
        // quietly means something else is worse to debug than a refusal.
        //
        // Set true only to talk to an old device deliberately (diagnostics,
        // reading its config before an upgrade). OTA itself is unaffected: it
        // goes through LeaderGripper, which is role-agnostic, so the upgrade
        // path is never blocked by this check.
        bool        allow_outdated_firmware = false;
    };

    // Minimum FOLLOWER firmware this SDK will drive -- a follower-line
    // number, not comparable with a leader's. See
    // Config::allow_outdated_firmware for why it is a hard requirement.
    static constexpr uint8_t kMinFirmwareMajor = 1;
    static constexpr uint8_t kMinFirmwareMinor = 2;
    static constexpr uint8_t kMinFirmwarePatch = 5;

    explicit FollowerGripper(const Config& cfg);
    ~FollowerGripper();

    FollowerGripper(const FollowerGripper&)            = delete;
    FollowerGripper& operator=(const FollowerGripper&) = delete;

    // Auto-discover. Same enumeration path as LeaderGripper::open() — the
    // caller is responsible for knowing they have follower hardware.
    static std::unique_ptr<FollowerGripper> open();

    // Component accessors.
    IMU&            imu()            noexcept { return imu_; }
    Encoder&        encoder()        noexcept { return encoder_; }
    // Wrist camera accessor throws IoError(ENODEV) unless the gripper was
    // constructed with open_cameras=true and the matching device path.
    Camera&         wrist_camera()   { return deref_(wrist_, "wrist camera",  "wrist_video"); }
    Motor&          motor()          noexcept { return motor_; }

    // ---- Follower gripper open/close limit config (Cmd 0x66/0x67) ----------
    // Read / write the follower's open/close limit config. NACKs as
    // SensorOffline on a leader (no follower config there).
    protocol::GripperConfig get_gripper_config(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void set_gripper_config(const protocol::GripperConfig& cfg);

    // ---- Motion safety envelope (rides inside GripperConfig 0x66/0x67) -----
    // Read-modify-write over the gripper config record, so the envelope and the
    // travel calibration cannot be written independently and one can never
    // silently clobber the other. Returns flags==0 on a device that has never
    // had an envelope written.
    //
    // The firmware clamps every MIT frame against this: the commanded position
    // is held within peak_torque_nm/kp of the measured position, the feed-forward
    // torque against cont_torque_nm. Setting Enforce changes real motion
    // behaviour — the
    // jaw will no longer slam a far target — so write it deliberately.
    protocol::GripperEnvelope get_envelope(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void set_envelope(const protocol::GripperEnvelope& env);

    // ---- Envelope policy --------------------------------------------------
    // Which numbers belong in the envelope is not the caller's question: the
    // device knows its own motor's ratings (0x56) and the firmware clamps
    // against them regardless of what was written. These derive the record
    // from that spec so the answer lives in one place instead of in every
    // script that ever writes one.
    //
    // The timeout covers both round trips (0x67 then 0x56) and defaults to the
    // transport's own ack timeout rather than get_gripper_config()'s 100 ms --
    // 0x56 has always run on the longer one, and shortening it here would be a
    // silent regression on a slow ACK.
    //
    // Both block on ACKs, so neither may be called while a controller is
    // running: the reply collides with the phase-locked 100 Hz control frames.
    // Before start(), or after stop().

    // Read-only. Never writes.
    EnvelopeAudit audit_envelope(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

    // Idempotent, and WRITES MCU FLASH -- persistent, survives power loss --
    // but only when the stored record is ineffective or misreports what is
    // enforced. Calling it twice writes once.
    //
    // It never widens an envelope. To put a unit back on its device-derived
    // numbers, that is a different intent and says so at the call site:
    //     g.set_envelope(g.audit_envelope().recommended);
    EnvelopeWrite ensure_envelope(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

    // ---- Power-on auto-calibration config (V1.9 — Cmd 0x68/0x69) ------------
    // When enabled, the firmware auto-calibrates on power-up (close-to-stall =>
    // zero, open-to-stall => max_open). Read / write that config here.
    protocol::GripperAutoCalConfig get_auto_cal_config(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void set_auto_cal_config(const protocol::GripperAutoCalConfig& cfg);

    // V2.2 — patch only the stall-detection fields, leaving speeds and the
    // Enable flag as stored. Saves a read-modify-write when tuning stall torque,
    // and cannot accidentally clobber the rest of the config from a stale read.
    // The Ex form additionally patches the two delays and the (compat-only)
    // confirm counts. Firmware older than follower 1.1.2 NACKs LengthMismatch —
    // fall back to the full set_auto_cal_config() there.
    void set_auto_cal_stall_param(const protocol::GripperAutoCalStallParam& p);
    void set_auto_cal_stall_param(const protocol::GripperAutoCalStallParamEx& p);

    // ---- Normalized gripper position (0 = closed, 1 = open) -----------------
    // Convenience layer over the motor + GripperConfig so callers work in a
    // normalized [0,1] position instead of raw shaft radians. NOTE: this is
    // distinct from motor().set_position(), which takes RAW radians — these
    // FollowerGripper methods are normalized [0,1].
    //
    // The mapping (closed = motor zero, travel = max_open_rad, direction from
    // the Reverse flag) is read once from the firmware via Cmd::GetGripperConfig
    // and cached; call reload_config() after re-calibrating. All methods throw
    // ProtocolError if the gripper isn't calibrated (config not Valid).
    //
    // The motor must be enabled before set_position() moves anything; like
    // Motor::submit_*, set_position() is fire-and-forget (no ACK) for a host
    // realtime loop — poll motor().control_stats() for health.
    float position(                                       // read: raw -> [0,1]
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void  set_position(float position,                    // command in [0,1], no-ACK
                       float kp_nm_per_rad,
                       float kd_nm_s_per_rad,
                       float feedforward_torque_nm = 0.0f);
    float pos_to_rad(float position);                     // [0,1] -> raw rad
    float rad_to_pos(float raw_rad);                      // raw rad -> [0,1]
    const GripperPosition& position_map();                // cached converter (loads if needed)
    void  reload_config();                                // re-read + rebuild converter

    Key&            key()            noexcept { return key_; }            // V1.4
    Led&            led()            noexcept { return led_; }            // V1.9
    SensorErrors&   sensor_errors()  noexcept { return errors_; }         // V1.6
    // Firmware UART counters and log control. Works on both roles; needs
    // firmware 1.1.3 (counters) / 1.1.4 (log control).
    Diagnostics&    diagnostics()    noexcept { return diag_; }           // fw 1.1.3
    // Fisheye camera calibration works on the follower too; the encoder-max
    // methods are leader-only and NACK with InvalidCmd here.
    Calibration&    calibration()    noexcept { return cal_; }            // V2.0
    OtaSession&     ota()            noexcept { return ota_; }            // V1.3
    // The firmware version reported at open(), or nullopt when the MCU did not
    // answer GetVersion. Read once during open rather than on demand: the
    // command competes with the control stream, and the answer cannot change
    // while the port is held.
    //
    // Callers gate features on it — wrist fisheye intrinsics need V2.0+, and a
    // caller that skips the check gets a confusing protocol error instead of a
    // "your firmware is too old" one.
    std::optional<protocol::FirmwareVersion> firmware_version() const noexcept {
        return fw_version_;
    }

    bus::Transport& transport()      noexcept { return t_; }

    // Streaming lifecycle.
    //
    // Motor status is the ONLY thing a follower streams. This used to mirror
    // the leader's (imu_hz, encoder_hz, motor_hz) signature, which was worse
    // than useless: the follower firmware emits IMU, encoder and eskin only
    // under #ifdef ENABLE_MASTER_GRIPPER, so those two rates set their mask
    // bits and then produced nothing (measured: 1000Hz of each yields zero
    // frames and leaves byte volume unchanged). Worse, the old defaults were
    // imu=100, encoder=100, motor=0 -- so the bare call `g.start_streaming()`
    // started a stream that carried NOTHING and reported success.
    //
    // Motor status is capped by the firmware at 100 Hz
    // (STREAM_MOTOR_MAX_RATE_HZ, "leave bandwidth for the control channel"),
    // so motor_hz=200 is served at 100 Hz. If you need faster feedback than
    // that, it is not available on this stream at any setting.
    //
    // Rates: 0 turns a source OFF. The firmware gates emission on the
    // source_mask bit alone, so a 0 rate with the bit set would stream at its
    // 100 Hz default — we clear the bit instead. All-zero throws
    // IoError(EINVAL) rather than starting a stream that carries nothing.
    //
    // The firmware divides a 1 kHz tick by an integer, so only divisors of
    // 1000 are exact: 300 Hz arrives as 333 Hz, 150 Hz as 167 Hz, and
    // anything above 1000 Hz collapses to 100 Hz. StartStream is never NACKed
    // for a bad rate, so the SDK logs a warning whenever it has to adjust one.
    // See cpp/src/stream_rate.hpp for the firmware model this mirrors.
    void start_streaming(unsigned motor_hz = 100);
    void stop_streaming();
    bool is_streaming() const noexcept { return streaming_; }

    const Config& config() const noexcept { return cfg_; }

private:
    std::optional<protocol::FirmwareVersion> fw_version_{};

    // Dereference an optional component, or throw a clear IoError naming the
    // Config field the caller must set (with open_cameras=true) to enable it.
    template <typename T>
    static T& deref_(const std::unique_ptr<T>& p, const char* what,
                     const char* cfg_field) {
        if (!p) {
            throw IoError(std::string(what) + " not opened (construct with "
                          "open_cameras=true and " + cfg_field + " set)",
                          ENODEV);
        }
        return *p;
    }

    // Load + cache the GripperPosition converter from the firmware GripperConfig
    // on first use. Throws ProtocolError if the config isn't Valid (uncalibrated).
    void ensure_position_map_();

    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Motor                           motor_;
    Key                             key_;       // V1.4
    Led                             led_;       // V1.9
    SensorErrors                    errors_;    // V1.6
    Diagnostics    diag_;
    Calibration                     cal_;       // V2.0
    OtaSession                      ota_;       // V1.3
    std::unique_ptr<Camera>         wrist_;
    bool                            streaming_ = false;
    GripperPosition                 pos_map_;             // raw<->position, cached
    bool                            pos_map_loaded_ = false;
};

}  // namespace xense::taccap

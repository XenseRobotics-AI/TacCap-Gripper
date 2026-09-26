// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// LeaderGripper — aggregate object representing one TacCap-Gripper leader
// (master / teleop end). Owns:
//
//   - bus::Transport                       MCU control + sensor stream link
//   - IMU            via .imu()            Cmd::GetImu / DATA stream
//   - Encoder        via .encoder()        Cmd::GetEncoder / DATA stream
//
// The wrist UVC camera is NOT opened by default: an external camera service
// owns that V4L2 device now. It is still reachable via .wrist_camera() IF the
// gripper was constructed with `open_cameras=true` and the matching device
// path; otherwise the accessor throws. (The OG visuotactile sensors are not
// handled here — they are read at the Python level via the xensesdk wheel.)
//
// Leader gripper has no motor (TacCap-G1 design). Follower gripper adds
// `Motor` and is implemented in follower_gripper.hpp.
//
// Streaming lifecycle (MCU control link only — cameras stream independently):
//   start_streaming(imu_hz, encoder_hz)  -> Cmd::StartStream (IMU + Encoder)
//   stop_streaming()                     -> Cmd::StopStream
//
// Subscribers (.imu().on_data(...) etc.) can be installed before or after
// start_streaming(); they accumulate frames as soon as data starts flowing.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/calibration.hpp>
#include <taccap/components/diagnostics.hpp>
#include <taccap/components/device.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/led.hpp>
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

class LeaderGripper {
public:
    struct Config {
        std::string mcu_device;             // /dev/ttyACM1 / /dev/taccap-leader / by-id
        std::string wrist_video;            // /dev/video3 / by-id
        uint32_t    baudrate            = 3'000'000;
        // The firmware can be slow to ACK StartStream / StopStream while
        // a previous stream is still flushing — give it a generous window
        // so a back-to-back run doesn't time out before the queued DATA
        // frames drain through the kernel rx buffer.
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

        // ---- Normalized encoder position (0 = closed, 1 = open) ------------
        // Off by default: every EncoderSample reports position_rad in radians
        // and leaves .position NaN.
        //
        // When true, the constructor builds the raw-rad <-> [0,1] converter and
        // installs it on the Encoder, so read_once() AND every on_data()
        // subscriber additionally get .position in [0,1]. position_rad keeps
        // reporting radians either way — normalization adds a field, it does
        // not repurpose one.
        //
        // The travel span comes from the firmware's Cmd::EncoderMaxCal (0x2C,
        // command set >= V2.1, i.e. leader >= 1.2.0). Construction THROWS when the
        // firmware has no encoder-max calibration or is too old to answer —
        // asking for normalization and silently not getting it would be worse.
        // Set encoder_max_rad to bypass the firmware read entirely.
        bool        normalize_position  = false;
        // Override the max travel angle (rad) instead of reading it from the
        // firmware. 0 (default) = read from firmware. Useful on pre-V2.1
        // firmware, or to normalize against a span the host already knows.
        float       encoder_max_rad     = 0.0f;
    };

    // Construct from explicit config (for tests, custom topologies).
    explicit LeaderGripper(const Config& cfg);
    ~LeaderGripper();

    LeaderGripper(const LeaderGripper&)            = delete;
    LeaderGripper& operator=(const LeaderGripper&) = delete;

    // Auto-discover everything. Throws IoError if hardware not found.
    // Returns a unique_ptr because LeaderGripper itself isn't copyable or
    // movable (its members hold mutexes / threads / atomics).
    static std::unique_ptr<LeaderGripper> open();

    // Component accessors.
    IMU&            imu()            noexcept { return imu_; }
    Encoder&        encoder()        noexcept { return encoder_; }
    // Wrist camera accessor throws IoError(ENODEV) unless the gripper was
    // constructed with open_cameras=true and the matching device path.
    Camera&         wrist_camera()   { return deref_(wrist_, "wrist camera",  "wrist_video"); }
    Key&            key()            noexcept { return key_; }            // V1.4
    Led&            led()            noexcept { return led_; }            // V1.9
    SensorErrors&   sensor_errors()  noexcept { return errors_; }         // V1.6
    // The firmware version reported at open(), or nullopt when the MCU did not
    // answer GetVersion. Read once during open rather than on demand: the
    // command competes with the sensor stream, and the answer cannot change
    // while the port is held.
    //
    // The constructor always read this to build its log line; it just never
    // kept it, so callers had to re-issue GetVersion by hand to gate on
    // firmware age. FollowerGripper has had the accessor all along.
    std::optional<protocol::FirmwareVersion> firmware_version() const noexcept {
        return fw_version_;
    }
    // The SN read at open(), empty when the MCU did not answer GetSn or none is
    // burned. Same reasoning as firmware_version(): read once, it cannot change
    // while the port is held. device().get_sn() asks again.
    const std::string& firmware_sn() const noexcept { return fw_sn_; }
    // Firmware UART counters and log control. Works on both roles; needs
    // firmware 1.1.3 (counters) / 1.1.4 (log control).
    Diagnostics&    diagnostics()    noexcept { return diag_; }           // fw 1.1.3
    Calibration&    calibration()    noexcept { return cal_; }            // V2.0/V2.1
    // Heartbeat, reset, SN and device type -- the MCU-level commands both roles
    // answer. The SN/device-type setters are factory operations; see device.hpp.
    Device&         device()         noexcept { return dev_; }
    OtaSession&     ota()            noexcept { return ota_; }            // V1.3
    bus::Transport& transport()      noexcept { return t_; }

    // ---- Normalized gripper position (0 = closed, 1 = open) ----------------
    // Mirror of the FollowerGripper surface, built on the leader's encoder-max
    // calibration (Cmd::EncoderMaxCal 0x2C) instead of a follower
    // GripperConfig. Encoder zero is the fully-closed pose (latch it with
    // encoder().set_zero() while the gripper is held closed); the travel span
    // to fully open is the calibrated max_rad.
    //
    // These work whether or not Config::normalize_position is set — that flag
    // only controls whether EncoderSample::position is filled in. The map is
    // loaded once on first use and cached; call reload_position_map() after
    // re-calibrating. All of them throw ProtocolError when the gripper has no
    // encoder-max calibration.
    float position(                                       // read: raw -> [0,1]
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    float pos_to_rad(float position);                     // [0,1] -> raw rad
    float rad_to_pos(float raw_rad);                      // raw rad -> [0,1]
    const GripperPosition& position_map();                // cached converter (loads if needed)
    void  reload_position_map();                          // re-read + rebuild converter

    // Streaming lifecycle.
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
    void start_streaming(unsigned imu_hz = 100, unsigned encoder_hz = 100);
    void stop_streaming();
    bool is_streaming() const noexcept { return streaming_; }

    const Config& config() const noexcept { return cfg_; }

private:
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

    // Load + cache the GripperPosition converter on first use, from
    // Config::encoder_max_rad when set, otherwise from the firmware's
    // encoder-max calibration. Throws ProtocolError when neither is available.
    void ensure_position_map_();

    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Key                             key_;       // V1.4
    Led                             led_;       // V1.9
    SensorErrors                    errors_;    // V1.6
    std::optional<protocol::FirmwareVersion> fw_version_{};
    std::string                     fw_sn_;
    Diagnostics    diag_;
    Calibration                     cal_;       // V2.0/V2.1
    Device                          dev_;
    OtaSession                      ota_;       // V1.3
    std::unique_ptr<Camera>         wrist_;
    bool                            streaming_ = false;
    GripperPosition                 pos_map_;             // raw<->position, cached
    bool                            pos_map_loaded_ = false;
};

}  // namespace xense::taccap

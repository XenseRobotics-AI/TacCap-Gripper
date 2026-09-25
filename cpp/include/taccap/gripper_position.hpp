// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// GripperPosition — pure converter between a normalized gripper position in
// [0, 1] (0 = fully closed, 1 = fully open) and the follower motor's raw shaft
// angle (rad), derived from the firmware GripperConfig.
//
// Reference frame (matches the firmware, see can_motor.c):
//   - The motor zero is the fully-CLOSED pose (set via Cmd::MotorSetZero while
//     the gripper is held closed). The normalized position grows from there.
//   - The open travel advances in the motor's NEGATIVE direction when
//     GripperConfig has the Reverse flag set (positive otherwise).
//   - Travel spans [min_open_rad, max_open_rad] (min is currently always 0; the
//     firmware sanitizes it to 0). The firmware itself clamps every commanded
//     target to this same [0, max_open] range, so a host using this converter
//     stays consistent with the firmware's own limit enforcement.
//
// Hardware-free and header-only: the realtime upper layer can use it directly
// without opening a gripper. Construct from a GripperConfig read via
// FollowerGripper::get_gripper_config() (Cmd::GetGripperConfig 0x67), or from
// a bare travel span via from_travel() — that is how the LEADER builds its map,
// since the leader's calibration is a single max travel angle read through
// Cmd::EncoderMaxCal (0x2C) rather than a full GripperConfig record.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <algorithm>

namespace xense::taccap {

class GripperPosition {
public:
    // Default-constructed converter is invalid() until built from a config.
    GripperPosition() = default;

    explicit GripperPosition(const protocol::GripperConfig& cfg)
        : max_open_(cfg.max_open_rad),
          min_open_(cfg.min_open_rad),
          dir_((cfg.flags & protocol::GripperConfigFlag::Reverse) ? -1.0f : 1.0f),
          valid_((cfg.flags & protocol::GripperConfigFlag::Valid) != 0 &&
                 cfg.max_open_rad > cfg.min_open_rad) {}

    // Build from an explicit travel span, for calibration sources that aren't a
    // GripperConfig record — chiefly the leader's Cmd::EncoderMaxCal (0x2C)
    // max travel angle, which carries no flags and no reverse bit (the leader
    // encoder counts positive as it opens away from its zero).
    //
    // Valid iff max_rad > min_rad, matching the GripperConfig constructor.
    static GripperPosition from_travel(float max_rad, float min_rad = 0.0f,
                                       bool reverse = false) noexcept {
        GripperPosition p;
        p.max_open_ = max_rad;
        p.min_open_ = min_rad;
        p.dir_      = reverse ? -1.0f : 1.0f;
        p.valid_    = max_rad > min_rad;
        return p;
    }

    // Valid only when the firmware config is marked Valid and the travel span is
    // positive — otherwise the gripper isn't calibrated and a normalized
    // position is meaningless.
    bool  valid()        const noexcept { return valid_; }
    float max_open_rad() const noexcept { return max_open_; }
    float min_open_rad() const noexcept { return min_open_; }
    bool  reverse()      const noexcept { return dir_ < 0.0f; }

    // Raw shaft angle (rad) -> normalized position, clamped to [0, 1].
    float to_position(float raw_rad) const noexcept {
        const float span = max_open_ - min_open_;
        if (span <= 0.0f) return 0.0f;
        const float travel = raw_rad * dir_ - min_open_;
        return std::clamp(travel / span, 0.0f, 1.0f);
    }

    // Motor-frame rate or torque -> GRIP frame: positive means "towards closed"
    // on every device, whatever direction its motor happens to be mounted.
    //
    // Why this exists: GripperObservation used to report a normalized position
    // (which goes through dir_) next to a raw motor-frame velocity and torque
    // (which did not). Two frames in one struct. It stayed invisible while every
    // follower in the field was an EL05 with the Reverse flag set -- the first
    // gripper built the other way round reported velocity and torque with the
    // OPPOSITE sign for the same physical motion, and nothing in the SDK said so.
    //
    // The sign convention is deliberately NOT d(position)/dt. Position is an
    // extent in [0,1] where 1 is open; velocity and torque are signed, and for a
    // gripper the quantity a caller reaches for is the GRIP -- so closing is
    // positive and a grasp reads as a positive torque. The two conventions
    // disagree by construction; this one was chosen because "grip force is
    // negative" is the more surprising of the two.
    float to_grip_frame(float motor_frame) const noexcept {
        return motor_frame * -dir_;
    }

    // Normalized position [0, 1] -> raw shaft angle (rad). The input is clamped
    // to [0, 1] first so a caller can never command beyond the calibrated travel
    // (the firmware clamps too, but this keeps the host command honest).
    float to_rad(float position) const noexcept {
        const float p = std::clamp(position, 0.0f, 1.0f);
        const float travel = min_open_ + p * (max_open_ - min_open_);
        return travel * dir_;   // dir_ is ±1, so dividing by it == multiplying
    }

private:
    float max_open_ = 0.0f;
    float min_open_ = 0.0f;
    float dir_      = 1.0f;   // +1 normal, -1 when Reverse
    bool  valid_    = false;
};

}  // namespace xense::taccap

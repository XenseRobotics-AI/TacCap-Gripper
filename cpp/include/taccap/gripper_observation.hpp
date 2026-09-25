// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// The gripper-level view of the motor-status stream, shared by every
// background controller.
//
// It lived in control_loop.hpp until that class was removed. ControlLoop was
// the low-level loop and the two supervised controllers were documented
// against it, but nothing in the SDK ever instantiated it, no recipe selected
// it, and it carried a second copy of the impedance law that had already
// drifted from ImpedanceController's -- different torque budgets, and a stall
// guard on one side that collapses the grip. This type is what actually had to
// be kept.

#pragma once

#include <cstdint>

namespace xense::taccap {

// Latest gripper observation, refreshed from the motor-status stream.
struct GripperObservation {
    bool     valid    = false;   // false until the first status frame arrives
    float    position = 0.0f;    // [0,1] normalized open amount (0=closed,1=open)
    // Velocity and torque are in the GRIP frame: **positive means closing**, so
    // a grasp reads as a positive torque. Same on every device -- the mounting
    // direction is taken out by GripperPosition::to_grip_frame().
    //
    // Note this is deliberately NOT d(position)/dt: position counts upwards
    // towards OPEN while these count positive towards CLOSED. Position is an
    // extent, these are signed, and for a gripper the signed quantity a caller
    // wants is the grip.
    //
    // They were raw motor-frame values until 0.3.1, which meant their sign
    // depended on how that unit's motor was mounted -- invisible until a
    // follower built the other way round showed up.
    float    velocity = 0.0f;    // rad/s, + = closing
    float    torque   = 0.0f;    // Nm, + = closing (i.e. the grip)
    float    raw_pos  = 0.0f;    // raw shaft angle (rad) -- MOTOR frame, unrotated
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

}  // namespace xense::taccap

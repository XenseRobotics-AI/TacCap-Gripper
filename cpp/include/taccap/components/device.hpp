// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Device — the MCU-level commands every gripper answers, leader and follower
// alike: liveness (Heartbeat 0x01), reset (0x03), and the factory identity
// burned into flash (SN 0x04/0x05, device type 0x06/0x07).
//
// THE WRITES ARE FACTORY OPERATIONS. The SN is not a label: discovery derives
// the gripper's SIDE from its sequence digit and its ROLE (leader/follower) from
// the m/s suffix, and ota_update.py picks the image by that suffix. A wrong SN
// therefore swaps left and right, or sends the wrong role's firmware to the
// board -- which bricks it. Both setters read the value back and throw if it
// did not land; neither is something a test tool should call in passing.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <string>

namespace xense::taccap {

struct Heartbeat {
    uint32_t                  uptime_ms;  // MCU HAL tick since boot; wraps at ~49.7 days
    protocol::FirmwareVersion version;    // same four bytes GetVersion returns
};

class Device {
public:
    explicit Device(bus::Transport& transport);

    // Cmd 0x01. Cheap and side-effect-free: the liveness probe. uptime_ms going
    // backwards between two calls means the MCU rebooted in between.
    Heartbeat heartbeat(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Cmd 0x03. The firmware ACKs first and then resets, so this returns once
    // the ACK is in; the port disappears ~10 ms later and every further command
    // on this transport times out. Re-open after USB re-enumerates (1-3 s).
    //
    // A SOFT reset, like the OTA bank swap: it restarts the MCU but not the
    // USB-serial bridge or the motor. It is not a substitute for a power cycle.
    void reset(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Cmd 0x04. The SN as burned; empty when none has been.
    std::string get_sn(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Cmd 0x05. FACTORY OPERATION -- see the header note. `sn` is at most 16
    // characters. Reads it back and throws ProtocolError if it differs.
    void set_sn(const std::string& sn,
                std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

    // Cmd 0x06. The flash-burned LEFT/RIGHT byte; Unknown when unset. Discovery
    // prefers the SN's sequence digit and only falls back to this.
    protocol::DeviceType get_device_type(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // Cmd 0x07. FACTORY OPERATION. Left or Right only -- the firmware refuses
    // Unknown. Reads it back and throws ProtocolError if it differs.
    void set_device_type(protocol::DeviceType type,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap

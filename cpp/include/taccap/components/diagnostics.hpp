// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Diagnostics component — firmware UART counters and log control.
//
// Both commands live in the firmware's UART layer rather than in any
// gripper-role subsystem, so this component is available on leader and
// follower alike.
//
// Firmware floor: uart_stats() needs 1.1.3, set_log_config() needs 1.1.4.
// Older firmware NACKs InvalidCmd, surfaced as ProtocolError like any other
// unsupported command.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>

namespace xense::taccap {

class Diagnostics {
public:
    explicit Diagnostics(bus::Transport& transport);

    // Free-running UART counters since MCU boot (Cmd 0x54).
    //
    // Successful HAL-call counts are not pin-level capture. Differences from
    // aligned host byte counts narrow the fault domain but cannot by themselves
    // exclude MCU firmware or identify the bridge/USB/TTY layer. RX errors
    // describe host-to-MCU reception, not MCU-to-host telemetry loss.
    //
    // Counters are cumulative, so take a reading before and after a
    // measurement window and subtract.
    //
    // `log_dropped` reads 0 against firmware 1.1.3, which answered with a
    // 32-byte packet that had no such field — it is not distinguishable from a
    // genuine zero, so gate on firmware_version() if that matters to you.
    protocol::UartStats uart_stats(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Explicit opt-in; unsupported firmware raises ProtocolError. Never
    // silently report missing subtype counters as zero. No periodic polling.
    protocol::UartRxErrors uart_rx_errors(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Turn firmware logging on or off at runtime (Cmd 0x55). Returns the
    // configuration the firmware reports as active afterwards.
    //
    // **This is a diagnostic lever, not a setting to leave on.** The firmware's
    // log sink is a blocking polled UART write — roughly 0.5 ms per line at
    // 921600 — and it blocks whichever task emitted the line. Logging on every
    // received command is what livelocked the firmware's command channel badly
    // enough to need a power cycle, which is why 1.1.4 ships with logging off.
    // Turn it on to look at something, then turn it back off.
    //
    // Output goes to the MCU's own DEBUG UART, which is not routed through the
    // USB bridge — you need a probe on that pin to read it. Enabling logging
    // without one costs you the realtime penalty and shows you nothing.
    protocol::LogConfig set_log_config(
        protocol::LogLevel level,
        uint8_t output_mask = protocol::LogOutput::None,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Convenience wrapper for the common "off again" case.
    protocol::LogConfig disable_logging(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap

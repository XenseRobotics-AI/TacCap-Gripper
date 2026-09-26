// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// RAII serial-port wrapper for talking to the TC-GU-01 MCU.
// Configures termios for 8N1 at the requested baud (default 3 Mbps to match
// the firmware's USART3) and exposes blocking read/write with a configurable
// VTIME-based timeout.
//
// This is a transport primitive: it knows nothing about TC-GU-01 framing.
// Pair it with bus::FrameParser for a complete receive pipeline.

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace xense::taccap::bus {

class SerialBus {
public:
    struct Config {
        std::string device;
        uint32_t    baudrate         = 3'000'000;
        // VTIME granularity is 100 ms. We round up internally; values <100 ms
        // are interpreted as "return ASAP" (VMIN=0, VTIME=1).
        unsigned    read_timeout_ms  = 1;
        unsigned    write_timeout_ms = 1000;
    };

    // Throws IoError on open failure.
    explicit SerialBus(const Config& cfg);
    ~SerialBus();

    SerialBus(const SerialBus&)            = delete;
    SerialBus& operator=(const SerialBus&) = delete;
    SerialBus(SerialBus&&) noexcept;
    SerialBus& operator=(SerialBus&&) noexcept;

    // Read up to `max` bytes. Returns 0 if the timeout elapses before any
    // data arrives. Throws IoError on hard failures (closed fd, etc.).
    std::size_t read(uint8_t* buf, std::size_t max);
    std::vector<uint8_t> read(std::size_t max);

    // Write all bytes (loops over partial writes). Throws IoError or
    // TimeoutError on failure.
    void write(const uint8_t* data, std::size_t len);
    void write(const std::vector<uint8_t>& bytes);

    // Discard whatever is queued in the kernel rx/tx buffer.
    void flush_input();
    void flush_output();

    bool is_open() const noexcept { return fd_ >= 0; }

    // Release the device: drops the exclusive lock and closes the fd, so the
    // port can be opened again (by this process or another). Idempotent. Any
    // read/write afterwards throws IoError(EBADF).
    void close() noexcept { close_(); }
    const Config& config() const noexcept { return cfg_; }

    // Best-effort enumeration of likely TacCap serial endpoints. Today we
    // walk /dev/serial/by-id/ (stable USB-by-VID/PID symlinks) plus
    // /dev/ttyACM* and /dev/ttyUSB*. Filtering to TacCap-only happens at a
    // higher layer.
    static std::vector<std::string> list_ports();

private:
    void apply_termios_();
    void close_();

    Config cfg_;
    int    fd_ = -1;
};

}  // namespace xense::taccap::bus

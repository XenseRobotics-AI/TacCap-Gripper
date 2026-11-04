// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Exception hierarchy for the TacCap-Gripper SDK. All recoverable errors
// derive from xense::taccap::Error so user code can catch a single base.

#pragma once

#include <stdexcept>
#include <string>
#include <system_error>
#include <cstdint>

namespace xense::taccap {

class Error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Wire-protocol parse / packing failures (bad CRC, oversized payload,
// unknown frame type, truncation, etc.). Recoverable: caller can resync.
class ProtocolError : public Error {
public:
    using Error::Error;
};

class CrcError : public ProtocolError {
public:
    using ProtocolError::ProtocolError;
};

// Preserve ProtocolError compatibility while allowing capability negotiation
// to fall back ONLY on an explicit unsupported-command response.
class NackError : public ProtocolError {
public:
    NackError(const std::string& what, std::uint8_t code)
        : ProtocolError(what), code_(code) {}
    std::uint8_t code() const noexcept { return code_; }
private:
    std::uint8_t code_;
};

// Errno-backed I/O failure (open/read/write/tcsetattr).
class IoError : public Error {
public:
    IoError(const std::string& what, int err)
        : Error(what + ": " + std::system_category().message(err)),
          errno_value_(err) {}
    int errno_value() const noexcept { return errno_value_; }
private:
    int errno_value_;
};

// Read/write completed without progress within the configured deadline.
class TimeoutError : public IoError {
public:
    using IoError::IoError;
};

}  // namespace xense::taccap

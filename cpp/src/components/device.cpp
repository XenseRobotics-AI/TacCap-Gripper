// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/device.hpp>

#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>
#include <stdexcept>

namespace xense::taccap {

namespace {

bus::AckResponse send_or_throw(bus::Transport& t, protocol::Cmd cmd,
                               const std::vector<uint8_t>& payload, const char* what,
                               std::chrono::milliseconds timeout) {
    auto ack = t.send_cmd(cmd, payload, timeout);
    if (const auto err = bus::ack_error_code(ack); err != protocol::ErrorCode::Ok) {
        throw ProtocolError(std::string("Device::") + what + " NACK: " +
                            protocol::to_string(err));
    }
    return ack;
}

}  // namespace

Device::Device(bus::Transport& transport) : t_(transport) {}

Heartbeat Device::heartbeat(std::chrono::milliseconds timeout) {
    auto ack = send_or_throw(t_, protocol::Cmd::Heartbeat, {}, "heartbeat", timeout);
    if (ack.data.size() < 8) {
        throw ProtocolError("Device::heartbeat: short payload (" +
                            std::to_string(ack.data.size()) + " bytes, want 8)");
    }
    Heartbeat hb{};
    std::memcpy(&hb.uptime_ms, ack.data.data(), 4);   // little-endian on the wire
    hb.version = protocol::decode_version(ack.data.data() + 4, 4);
    return hb;
}

void Device::reset(std::chrono::milliseconds timeout) {
    send_or_throw(t_, protocol::Cmd::ResetDevice, {}, "reset", timeout);
    logger()->warn("Device::reset: MCU is rebooting; this transport is dead until re-opened");
}

std::string Device::get_sn(std::chrono::milliseconds timeout) {
    auto ack = send_or_throw(t_, protocol::Cmd::GetSn, {}, "get_sn", timeout);
    return ack.data.empty() ? std::string{}
                            : protocol::decode_sn(ack.data.data(), ack.data.size());
}

void Device::set_sn(const std::string& sn, std::chrono::milliseconds timeout) {
    if (sn.empty() || sn.size() > 16) {
        throw std::invalid_argument("Device::set_sn: SN must be 1..16 characters, got " +
                                    std::to_string(sn.size()));
    }
    const std::string before = get_sn(timeout);
    send_or_throw(t_, protocol::Cmd::SetSn, protocol::encode_sn(sn), "set_sn", timeout);
    const std::string after = get_sn(timeout);
    if (after != sn) {
        throw ProtocolError("Device::set_sn: wrote '" + sn + "' but read back '" + after + "'");
    }
    logger()->warn("Device::set_sn: SN changed '{}' -> '{}' (factory operation)", before, after);
}

protocol::DeviceType Device::get_device_type(std::chrono::milliseconds timeout) {
    // NOT send_or_throw: the answer is exactly one byte, which is also the
    // shape of a legacy one-byte error reply, so ack_error_code() would read
    // an unset type (0xFF) as a NACK. A real NACK arrives as is_nack (the
    // pure-ACK error form, firmware >= 1.2.3). Motor::get_can_id does the same.
    auto ack = t_.send_cmd(protocol::Cmd::GetDevType, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Device::get_device_type NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_dev_type(ack.data.data(), ack.data.size());
}

void Device::set_device_type(protocol::DeviceType type, std::chrono::milliseconds timeout) {
    if (type != protocol::DeviceType::Left && type != protocol::DeviceType::Right) {
        throw std::invalid_argument("Device::set_device_type: only Left or Right can be written");
    }
    send_or_throw(t_, protocol::Cmd::SetDevType, {static_cast<uint8_t>(type)},
                  "set_device_type", timeout);
    const auto after = get_device_type(timeout);
    if (after != type) {
        throw ProtocolError("Device::set_device_type: read back " +
                            std::to_string(static_cast<unsigned>(after)) + ", wrote " +
                            std::to_string(static_cast<unsigned>(type)));
    }
    logger()->warn("Device::set_device_type: device type set to {} (factory operation)",
                   type == protocol::DeviceType::Left ? "LEFT" : "RIGHT");
}

}  // namespace xense::taccap

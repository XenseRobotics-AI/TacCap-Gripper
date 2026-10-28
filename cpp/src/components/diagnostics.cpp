// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/diagnostics.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

namespace xense::taccap {

Diagnostics::Diagnostics(bus::Transport& transport) : t_(transport) {}

protocol::UartRxErrors Diagnostics::uart_rx_errors(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetUartStats, {1}, timeout);
    const auto error = bus::ack_error_code(ack);
    if (error != protocol::ErrorCode::Ok)
        throw ProtocolError(std::string("Diagnostics::uart_rx_errors NACK: ") +
                            protocol::to_string(error));
    return protocol::decode_uart_rx_errors(ack.data.data(), ack.data.size());
}

protocol::UartStats Diagnostics::uart_stats(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetUartStats, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Diagnostics::uart_stats NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_uart_stats(ack.data.data(), ack.data.size());
}

protocol::LogConfig Diagnostics::set_log_config(protocol::LogLevel level,
                                                uint8_t output_mask,
                                                std::chrono::milliseconds timeout) {
    protocol::LogConfig req{static_cast<uint8_t>(level), output_mask};
    auto ack = t_.send_cmd(protocol::Cmd::SetLogConfig, protocol::encode(req), timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Diagnostics::set_log_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_log_config(ack.data.data(), ack.data.size());
}

protocol::LogConfig Diagnostics::disable_logging(std::chrono::milliseconds timeout) {
    return set_log_config(protocol::LogLevel::None, protocol::LogOutput::None, timeout);
}

}  // namespace xense::taccap

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Shared PTY-backed fake-firmware harness for transport-level gtest cases.
//
// The host SDK opens the PTY's slave (a real /dev/pts/N tty so termios
// setup works); the test acts as the "fake firmware" on the master end,
// reading the host's frames and writing back ACK / NACK / DATA frames
// per the protocol_handler.c contract.
//
// Two response variants from firmware:
//   send_response(seq, cmd, payload) → protocol_send_response, ACK with
//                                      typed data
//   send_nack(seq, err)              → protocol_send_ack(seq, err), wire
//                                      frame has cmd=0, payload=[err]
//
// See test_transport.cpp for the original PTY-based ACK/NACK round-trip
// tests; this header is the same code extracted so test_upper_v16_pty.cpp
// can drive Key / SensorErrors / OtaSession against a fake firmware too.

#pragma once

#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>
#include <taccap/bus/transport.hpp>
#include <taccap/protocol/commands.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <chrono>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace taccap_test {

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

class Pty {
public:
    Pty() {
        char slave_path[128] = {0};
        if (::openpty(&master_, &slave_, slave_path, nullptr, nullptr) != 0) {
            ADD_FAILURE() << "openpty failed: " << std::strerror(errno);
            master_ = slave_ = -1;
            return;
        }
        slave_path_ = slave_path;
        ::fcntl(master_, F_SETFL, ::fcntl(master_, F_GETFL, 0) | O_NONBLOCK);
    }
    ~Pty() {
        if (master_ >= 0) ::close(master_);
        if (slave_  >= 0) ::close(slave_);
    }
    Pty(const Pty&)            = delete;
    Pty& operator=(const Pty&) = delete;

    int master() const { return master_; }

    // Drop the link the way a yanked USB cable does: writes from the slave end
    // fail with EIO. Idempotent; the destructor tolerates the double close.
    void close_master() {
        if (master_ >= 0) { ::close(master_); master_ = -1; }
    }
    const std::string& slave_path() const { return slave_path_; }

    std::optional<tb::Frame> expect_frame(int timeout_ms = 1000) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        uint8_t buf[256];
        for (;;) {
            tb::Frame out;
            if (parser_.try_pop(out)) return out;

            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) return std::nullopt;
            const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    deadline - now).count();

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(master_, &rfds);
            timeval tv{};
            tv.tv_sec  = remain / 1000;
            tv.tv_usec = (remain % 1000) * 1000;
            const int r = ::select(master_ + 1, &rfds, nullptr, nullptr, &tv);
            if (r <= 0) continue;
            const ssize_t n = ::read(master_, buf, sizeof(buf));
            if (n > 0) parser_.feed(buf, static_cast<std::size_t>(n));
        }
    }

    // success_with_data: protocol_send_response(seq, cmd, ERR_OK, payload, n).
    void send_response(uint8_t seq, tp::Cmd cmd,
                       std::vector<uint8_t> data) {
        if (data.empty()) data.push_back(static_cast<uint8_t>(tp::ErrorCode::Ok));
        write_frame(tp::Address::MCU, seq, tp::FrameType::ACK, cmd, data);
    }

    // failure: protocol_send_ack(seq, err) — wire frame.cmd = 0, payload = [err].
    void send_nack(uint8_t seq, tp::ErrorCode err) {
        std::vector<uint8_t> body{static_cast<uint8_t>(err)};
        write_frame(tp::Address::MCU, seq,
                    tp::FrameType::ACK,
                    static_cast<tp::Cmd>(0),
                    body);
    }

    void send_ack_ok(uint8_t seq, tp::Cmd echoed_cmd) {
        send_response(seq, echoed_cmd, {});
    }

    void send_data(uint8_t seq, tp::Cmd cmd,
                   const std::vector<uint8_t>& payload) {
        write_frame(tp::Address::MCU, seq, tp::FrameType::DATA, cmd, payload);
    }

    void send_raw(const std::vector<uint8_t>& bytes) {
        const ssize_t n = ::write(master_, bytes.data(), bytes.size());
        ASSERT_EQ(static_cast<std::size_t>(n), bytes.size());
    }

private:
    void write_frame(tp::Address addr, uint8_t seq, tp::FrameType type,
                     tp::Cmd cmd, const std::vector<uint8_t>& payload) {
        auto wire = tb::pack_frame(addr, seq, type, cmd, payload);
        const ssize_t n = ::write(master_, wire.data(), wire.size());
        ASSERT_EQ(static_cast<std::size_t>(n), wire.size())
            << "PTY write short: " << std::strerror(errno);
    }

    int             master_ = -1;
    int             slave_  = -1;
    std::string     slave_path_;
    tb::FrameParser parser_;
};

// Standard Transport::Config for PTY-backed tests. PTY ignores actual
// baud but B9600 is universal across kernels; we keep ack_timeout / max
// retries small so failure-path tests don't drag on.
inline tb::Transport::Config base_config(const std::string& dev) {
    tb::Transport::Config c;
    c.serial.device          = dev;
    c.serial.baudrate        = 9600;
    c.serial.read_timeout_ms = 1;
    c.peer                   = tp::Address::MCU;
    c.ack_timeout            = std::chrono::milliseconds(50);
    c.max_retries            = 2;
    c.retry_interval         = std::chrono::milliseconds(5);
    return c;
}

}  // namespace taccap_test

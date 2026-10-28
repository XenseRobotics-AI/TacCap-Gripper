#include <gtest/gtest.h>
#include <taccap/components/motor.hpp>
#include <taccap/components/diagnostics.hpp>
#include <taccap/protocol/codec.hpp>
#include "pty_helper.hpp"
#include <future>
#include <termios.h>
namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;
using namespace std::chrono_literals;
using taccap_test::Pty;
using taccap_test::base_config;

TEST(TransportReliability, WrongCommandCannotConsumePendingAck) {
    Pty p; auto cfg = base_config(p.slave_path()); cfg.max_retries = 0;
    tb::Transport host(cfg);
    auto request = std::async(std::launch::async, [&] {
        return host.send_cmd(tp::Cmd::GetVersion, {}, 500ms);
    });
    auto f = p.expect_frame(); ASSERT_TRUE(f);
    p.send_ack_ok(f->seq, tp::Cmd::Heartbeat);
    EXPECT_EQ(request.wait_for(20ms), std::future_status::timeout);
    p.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    EXPECT_EQ(request.get().cmd, tp::Cmd::GetVersion);
    EXPECT_EQ(host.stats().unexpected_frames, 1u);
}

TEST(TransportReliability, WrapCannotOverwritePendingRequest) {
    Pty p; auto cfg = base_config(p.slave_path()); cfg.max_retries = 0;
    tb::Transport host(cfg);
    auto first = std::async(std::launch::async, [&] {
        return host.send_cmd(tp::Cmd::GetVersion, {}, 2000ms);
    });
    auto f = p.expect_frame(); ASSERT_TRUE(f);
    for (unsigned i = 0; i < 256; ++i) {
        host.send_cmd_no_ack(tp::Cmd::MotorImpedanceCtrl, {0});
        auto next = p.expect_frame(); ASSERT_TRUE(next);
        EXPECT_NE(next->seq, f->seq); // active ACK slots are reserved even for no-ACK TX
    }
    auto second = std::async(std::launch::async, [&] {
        return host.send_cmd(tp::Cmd::Heartbeat, {}, 500ms);
    });
    auto g = p.expect_frame(); ASSERT_TRUE(g); EXPECT_NE(g->seq, f->seq);
    p.send_ack_ok(g->seq, tp::Cmd::Heartbeat);
    EXPECT_EQ(second.get().cmd, tp::Cmd::Heartbeat);
    p.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    EXPECT_EQ(first.get().cmd, tp::Cmd::GetVersion);
}

TEST(TransportReliability, DeliverySnapshotExposesUnacceptedTargetsWithoutExtraControl) {
    Pty p; tb::Transport host(base_config(p.slave_path()));
    xense::taccap::Motor motor(host);
    EXPECT_EQ(motor.target_writes().count, 0u);
    tp::MotorImpedanceCtrl target{};
    for (int i = 0; i < 3; ++i) {
        motor.submit(target); auto f = p.expect_frame(); ASSERT_TRUE(f);
        EXPECT_EQ(f->type, tp::FrameType::CMD_NO_ACK);
    }
    auto request = std::async(std::launch::async, [&] { return motor.delivery_snapshot(500ms); });
    auto f = p.expect_frame(); ASSERT_TRUE(f); EXPECT_EQ(f->cmd, tp::Cmd::GetMotorControlStats);
    tp::MotorControlStats stats{}; stats.target_seq = 2; stats.applied_seq = 2;
    std::vector<uint8_t> data(sizeof(stats)); std::memcpy(data.data(), &stats, sizeof(stats));
    p.send_response(f->seq, f->cmd, data);
    auto out = request.get();
    EXPECT_EQ(out.before.count, 3u); EXPECT_EQ(out.after.count, 3u);
    EXPECT_EQ(out.firmware.target_seq, 2u);
    EXPECT_GT(out.after.last_write_ns, 0);
    EXPECT_GE(out.after.max_gap_us, out.after.last_gap_us);
    EXPECT_FALSE(p.expect_frame(20)); // no automatic polling, target replay or stop
}

TEST(TransportReliability, InheritedParityFlagsAreCleared) {
    Pty p; int fd = ::open(p.slave_path().c_str(), O_RDWR | O_NOCTTY);
    ASSERT_GE(fd, 0); termios tio{}; ASSERT_EQ(tcgetattr(fd, &tio), 0);
    tio.c_iflag |= INPCK | IGNPAR; ASSERT_EQ(tcsetattr(fd, TCSANOW, &tio), 0);
    { tb::Transport host(base_config(p.slave_path()));
      ASSERT_EQ(tcgetattr(fd, &tio), 0); EXPECT_EQ(tio.c_iflag & (INPCK | IGNPAR), 0u); }
    ::close(fd);
}

TEST(DiagnosticsCodec, ExtendedErrorsRequireExplicitCompleteReply) {
    tp::UartRxErrors errors{1,2,3,4,5,8,1234};
    std::vector<uint8_t> wire(72, 0); std::memcpy(wire.data()+44, &errors, 28);
    auto out = tp::decode_uart_rx_errors(wire.data(), wire.size());
    EXPECT_EQ(out.overrun, 1u); EXPECT_EQ(out.framing, 2u); EXPECT_EQ(out.noise, 3u);
    EXPECT_EQ(out.parity, 4u); EXPECT_EQ(out.receiver_timeout, 5u);
    EXPECT_EQ(out.last_error, 8u); EXPECT_EQ(out.last_error_ms, 1234u);
    EXPECT_THROW(tp::decode_uart_rx_errors(wire.data(), 44), xense::taccap::ProtocolError);
    EXPECT_THROW(tp::decode_uart_rx_errors(wire.data(), 71), xense::taccap::ProtocolError);
    EXPECT_THROW(tp::decode_uart_rx_errors(nullptr, 72), xense::taccap::ProtocolError);
}

TEST(TransportReliability, ExtendedErrorsAreOptInAndOldFirmwareIsNotZeroFilled) {
    Pty p; tb::Transport host(base_config(p.slave_path()));
    xense::taccap::Diagnostics diag(host);
    auto first = std::async(std::launch::async, [&] { return diag.uart_stats(500ms); });
    auto f = p.expect_frame(); ASSERT_TRUE(f); EXPECT_TRUE(f->payload.empty());
    p.send_response(f->seq, f->cmd, std::vector<uint8_t>(44));
    EXPECT_EQ(first.get().rx_errors, 0u);
    auto second = std::async(std::launch::async, [&] { return diag.uart_rx_errors(500ms); });
    f = p.expect_frame(); ASSERT_TRUE(f); EXPECT_EQ(f->payload, std::vector<uint8_t>{1});
    p.send_response(f->seq, f->cmd, std::vector<uint8_t>(44));
    EXPECT_THROW(second.get(), xense::taccap::ProtocolError);
}

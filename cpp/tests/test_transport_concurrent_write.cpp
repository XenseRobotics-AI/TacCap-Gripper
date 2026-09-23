// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Contract: frames from concurrent writers reach the far end intact.
//
// Read the result honestly -- this test passes with and without
// Transport's write mutex, and that is the finding, not a gap in the test.
// SerialBus::write() loops over partial writes, but on a BLOCKING tty
// ::write() never returns short: n_tty holds the tty's atomic_write_lock and
// loops internally until every byte is buffered. Measured directly (40 x
// 100 kB writes against a starved drain, zero short writes), so the loop's
// second iteration is unreachable and two threads cannot splice their frames.
//
// What this pins, then, is the invariant rather than a fixed bug: if the fd
// ever becomes non-blocking, or the transport moves onto something that is not
// a tty, the partial-write loop becomes reachable and the mutex in
// Transport::write_frame_ is what keeps this test green.
//
// The harness still drains slower than the writers fill so the output buffer
// stays saturated -- that is the state the race would need, and arranging it
// is what makes the test meaningful rather than vacuous.

#include <gtest/gtest.h>

#include <taccap/bus/frame.hpp>
#include <taccap/bus/transport.hpp>
#include <taccap/protocol/commands.hpp>

#include <pty.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

struct Damage {
    uint64_t frames_ok   = 0;
    uint64_t crc_errors  = 0;
    uint64_t resync_bytes = 0;
};

// Run `writers` threads, each sending `per_writer` CMD_NO_ACK frames with a
// `payload_bytes` payload, while this thread plays the MCU: it drains the pty
// master in small chunks and parses what it gets.
Damage hammer(unsigned writers, unsigned per_writer, std::size_t payload_bytes) {
    int master = -1, slave = -1;
    char slave_path[128] = {0};
    EXPECT_EQ(::openpty(&master, &slave, slave_path, nullptr, nullptr), 0)
        << std::strerror(errno);
    ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL, 0) | O_NONBLOCK);

    tb::Transport::Config cfg;
    cfg.serial.device          = slave_path;
    cfg.serial.baudrate        = 9600;   // ignored by a pty
    cfg.serial.read_timeout_ms = 1;
    cfg.peer                   = tp::Address::MCU;

    Damage d{};
    {
        tb::Transport t(cfg);

        std::atomic<bool> writers_done{false};
        std::atomic<unsigned> live{writers};
        std::vector<std::thread> pool;
        const std::vector<uint8_t> payload(payload_bytes, 0x42);

        for (unsigned w = 0; w < writers; ++w) {
            pool.emplace_back([&, w] {
                for (unsigned i = 0; i < per_writer; ++i) {
                    try {
                        t.send_cmd_no_ack(tp::Cmd::MotorImpedanceCtrl, payload);
                    } catch (...) {
                        break;   // write timeout: the drain fell too far behind
                    }
                }
                if (live.fetch_sub(1) == 1) writers_done.store(true);
            });
        }

        // The "MCU": drain deliberately slower than the writers fill, so the
        // kernel's output buffer stays saturated. That is what makes ::write()
        // return SHORT, which is the only state in which the partial-write loop
        // can be interrupted mid-frame. Drain as fast as the writers and the
        // race simply never arms -- an earlier version of this test did exactly
        // that and passed against the unfixed code.
        tb::FrameParser parser;
        uint8_t buf[512];
        for (;;) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(master, &rfds);
            timeval tv{0, 2000};   // 2 ms
            const int r = ::select(master + 1, &rfds, nullptr, nullptr, &tv);
            if (r > 0) {
                const ssize_t n = ::read(master, buf, sizeof(buf));
                if (n > 0) parser.feed(buf, static_cast<std::size_t>(n));
                std::this_thread::sleep_for(std::chrono::microseconds(300));
            } else if (writers_done.load()) {
                break;   // nothing left in flight and every writer is finished
            }
            tb::Frame f;
            while (parser.try_pop(f)) ++d.frames_ok;
        }

        for (auto& th : pool) th.join();
        d.crc_errors   = parser.stats().crc_errors;
        d.resync_bytes = parser.stats().resync_bytes;
    }

    ::close(master);
    ::close(slave);
    return d;
}

// Frames the writers handed to the transport, all of which must reach the
// far end intact.
constexpr unsigned kWriters   = 4;
constexpr unsigned kPerWriter = 150;

void expect_intact(const Damage& d, const char* what) {
    std::cout << "[ " << what << " ] frames_ok=" << d.frames_ok
              << " expected=" << (kWriters * kPerWriter)
              << " crc_errors=" << d.crc_errors
              << " resync_bytes=" << d.resync_bytes << std::endl;
    EXPECT_EQ(d.crc_errors, 0u)   << what << ": frames were spliced on the wire";
    EXPECT_EQ(d.resync_bytes, 0u) << what << ": bytes were discarded by the parser";
    EXPECT_EQ(d.frames_ok, kWriters * kPerWriter) << what << ": frames went missing";
}

// OTA-sized payloads (1030 B) are split by any link that is not instantly
// drained, so this is the case that breaks first.
TEST(TransportConcurrentWrite, LargeFramesSurviveConcurrentWriters) {
    expect_intact(hammer(kWriters, kPerWriter, 1030), "large/1030B");
}

// The realtime case: controller-sized MIT frames (20 B payload) racing each
// other. Small writes are atomic while the buffer has room, so this only
// splices once the link backs up -- exactly the condition a busy bus creates.
TEST(TransportConcurrentWrite, SmallFramesSurviveConcurrentWriters) {
    expect_intact(hammer(kWriters, kPerWriter, 20), "small/20B");
}

}  // namespace

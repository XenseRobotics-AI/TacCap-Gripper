// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Transport tests use openpty(): the host-side SerialBus opens the PTY's
// slave (a real /dev/pts/N tty so termios setup works), and the test acts
// as a "fake firmware" on the master end, reading host-issued frames and
// writing back ACK / DATA frames.

#include <gtest/gtest.h>
#include <taccap/bus/transport.hpp>
#include <taccap/bus/frame.hpp>
#include <taccap/protocol/payloads.hpp>
#include <taccap/error.hpp>

#include "pty_helper.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <cstring>
#include <thread>
#include <vector>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

using taccap_test::Pty;
using taccap_test::base_config;

TEST(Transport, SendCmdReceivesAck) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    tb::Transport host(base_config(pty.slave_path()));

    // Drive the fake firmware in a side thread.
    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        EXPECT_EQ(f->cmd,  tp::Cmd::GetVersion);
        EXPECT_EQ(f->type, tp::FrameType::CMD_NEED_ACK);
        // Firmware "success no data" path: send_response with empty payload
        // → wire ACK with cmd=GetVersion and payload=[ERR_OK].
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });

    auto ack = host.send_cmd(tp::Cmd::GetVersion);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);
    EXPECT_FALSE(ack.is_nack);
    EXPECT_EQ(ack.cmd, tp::Cmd::GetVersion);
    EXPECT_EQ(ack.data.size(), 1u);  // single ERR_OK byte

    fw.join();
    EXPECT_EQ(host.stats().ack_timeouts, 0u);
    EXPECT_EQ(host.stats().retries,      0u);
}

TEST(Transport, SendCmdNackThrowsProtocolError) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::InvalidParam);
    });

    EXPECT_THROW(host.send_cmd(tp::Cmd::SetSn, std::vector<uint8_t>{1, 2, 3}),
                 xense::taccap::ProtocolError);

    fw.join();
}

TEST(Transport, RetryThenSucceed) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    auto cfg = base_config(pty.slave_path());
    cfg.max_retries    = 3;
    cfg.ack_timeout    = std::chrono::milliseconds(40);
    cfg.retry_interval = std::chrono::milliseconds(5);
    tb::Transport host(cfg);

    std::thread fw([&]() {
        // Drop the first two attempts, ACK the third.
        for (int i = 0; i < 2; ++i) {
            auto f = pty.expect_frame(500);
            ASSERT_TRUE(f.has_value());
            // intentionally don't ACK
        }
        auto f = pty.expect_frame(500);
        ASSERT_TRUE(f.has_value());
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });

    auto ack = host.send_cmd(tp::Cmd::GetVersion);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);

    fw.join();
    EXPECT_GE(host.stats().retries,      2u);
    EXPECT_GE(host.stats().ack_timeouts, 2u);
    EXPECT_GE(host.stats().frames_sent,  3u);
}

TEST(Transport, AckTimeoutThrowsAfterRetries) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    auto cfg = base_config(pty.slave_path());
    cfg.max_retries    = 1;
    cfg.ack_timeout    = std::chrono::milliseconds(20);
    cfg.retry_interval = std::chrono::milliseconds(2);
    tb::Transport host(cfg);

    // Side thread just drains the master so the kernel buffer doesn't fill,
    // but never replies. (Not strictly needed for small commands.)
    std::thread drain([&]() {
        for (int i = 0; i <= cfg.max_retries; ++i) {
            (void)pty.expect_frame(200);
        }
    });

    EXPECT_THROW(host.send_cmd(tp::Cmd::Heartbeat),
                 xense::taccap::TimeoutError);

    drain.join();
    EXPECT_EQ(host.stats().ack_timeouts, cfg.max_retries + 1);
    EXPECT_EQ(host.stats().retries,      cfg.max_retries);
}

TEST(Transport, DataFrameDispatchedToSubscriber) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::mutex            m;
    std::condition_variable cv;
    bool                  got = false;
    std::vector<uint8_t>  seen;

    auto sub = host.subscribe(tp::Cmd::GetImu, [&](const tb::Frame& f) {
        std::lock_guard<std::mutex> lk(m);
        seen = f.payload;
        got  = true;
        cv.notify_one();
    });
    (void)sub;

    std::vector<uint8_t> body(sizeof(tp::ImuData), 0);
    body[0] = 0xDE; body[1] = 0xAD; body[2] = 0xBE; body[3] = 0xEF;
    pty.send_data(/*seq=*/7, tp::Cmd::GetImu, body);

    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                            [&]() { return got; }));
    EXPECT_EQ(seen.size(), sizeof(tp::ImuData));
    EXPECT_EQ(seen[0], 0xDE);
}

TEST(Transport, UnsubscribeStopsCallback) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::atomic<int> count{0};
    auto sub = host.subscribe(tp::Cmd::GetEncoder,
                              [&](const tb::Frame&) { ++count; });

    // First frame should be delivered.
    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    auto wait_until = [](auto pred, int ms) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end) {
            if (pred()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    };
    EXPECT_TRUE(wait_until([&]() { return count.load() >= 1; }, 500));

    host.unsubscribe(sub);

    // Subsequent frame should NOT bump the counter.
    pty.send_data(2, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(count.load(), 1);
}

// stop() must drop subscriptions before joining the reader, so no callback
// can be entered once shutdown has begun and every callback object is
// destroyed on the caller's thread. The Python bindings depend on this: their
// callback destructor takes the GIL, and running it on the reader thread
// during interpreter teardown aborts the process.
TEST(Transport, StopDropsSubscribersAndDestroysThemOnCaller) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    std::atomic<int> count{0};
    // Records the thread that destroyed the captured state — i.e. where the
    // real binding's GIL-acquiring deleter would have run.
    struct ThreadWitness {
        std::thread::id* out;
        ~ThreadWitness() { *out = std::this_thread::get_id(); }
    };
    std::thread::id destroyed_on{};

    {
        tb::Transport host(base_config(pty.slave_path()));
        auto witness = std::make_shared<ThreadWitness>(ThreadWitness{&destroyed_on});
        host.subscribe(tp::Cmd::GetEncoder,
                       [&count, witness](const tb::Frame&) { ++count; });

        pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < end && count.load() < 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        ASSERT_GE(count.load(), 1) << "subscriber never fired before stop()";

        host.stop();
        EXPECT_EQ(destroyed_on, std::this_thread::get_id())
            << "callback state must be destroyed on the thread calling stop(), "
               "not on the reader thread";

        // A frame arriving after stop() must not reach the callback. (The
        // reader is joined by now, so this only reaches the kernel buffer.)
        const int after_stop = count.load();
        pty.send_data(2, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        EXPECT_EQ(count.load(), after_stop);
    }
}

// A double stop() must not double-drop the subscribers (they are already
// gone) or trip over the empty vector. Complements StopIsIdempotent below,
// which covers the reader/join half of idempotency.
TEST(Transport, RepeatedStopWithSubscribersIsSafe) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));
    host.subscribe(tp::Cmd::GetEncoder, [](const tb::Frame&) {});
    host.subscribe(tp::Cmd::GetImu,     [](const tb::Frame&) {});
    host.stop();
    host.stop();          // and ~Transport makes a third
    EXPECT_FALSE(host.is_running());
}

TEST(Transport, SendCmdNoAckDoesNotBlock) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    const auto t0 = std::chrono::steady_clock::now();
    host.send_cmd_no_ack(tp::Cmd::Heartbeat);
    const auto dt = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(),
              50)
        << "send_cmd_no_ack should not block";

    auto f = pty.expect_frame(500);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->type, tp::FrameType::CMD_NO_ACK);
    EXPECT_EQ(f->cmd,  tp::Cmd::Heartbeat);
}

TEST(Transport, StrayAckIgnored) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    // No outstanding command — push a stray ACK with a random seq.
    pty.send_ack_ok(/*seq=*/123, tp::Cmd::GetVersion);

    // Give the reader a moment to process.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // The transport should still be running and stats should reflect the
    // unmatched frame as unexpected.
    EXPECT_TRUE(host.is_running());
    EXPECT_GE(host.stats().unexpected_frames, 1u);
}

TEST(Transport, StatsCountersIncrement) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });
    host.send_cmd(tp::Cmd::GetVersion);
    fw.join();

    auto s = host.stats();
    EXPECT_GE(s.frames_sent,     1u);
    EXPECT_GE(s.frames_received, 1u);
    EXPECT_GT(s.bytes_written,   0u);
    EXPECT_GT(s.bytes_read,      0u);
}

TEST(Transport, PureAckCmdZeroOkIsSuccess) {
    // Firmware quirk (TC-GU-01 v1.1): some handlers (notably StopStream) take
    // the protocol_send_ack(seq, ERR_OK) wire path, producing an ACK frame
    // with cmd=0 and payload=[0x00]. That's "success with no data", NOT a
    // NACK — the host must accept it.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::Ok);   // cmd=0, payload=[0x00]
    });
    auto ack = host.send_cmd(tp::Cmd::StopStream);
    EXPECT_FALSE(ack.is_nack);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);
    EXPECT_EQ(static_cast<uint8_t>(ack.cmd), 0u);   // cmd is 0 on this path
    fw.join();
}

TEST(Transport, PureAckCmdZeroNonOkIsNack) {
    // A real NACK still produces ProtocolError.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::InvalidParam);
    });
    EXPECT_THROW(host.send_cmd(tp::Cmd::SetSn, std::vector<uint8_t>{1}),
                 xense::taccap::ProtocolError);
    fw.join();
}

TEST(Transport, StopIsIdempotent) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    EXPECT_TRUE(host.is_running());
    host.stop();
    EXPECT_FALSE(host.is_running());
    host.stop();   // second call must not crash / hang
    EXPECT_FALSE(host.is_running());

    EXPECT_THROW(host.send_cmd(tp::Cmd::Heartbeat),
                 xense::taccap::IoError);
}

// ---------------------------------------------------------------------------
// Reader / dispatcher decoupling.
//
// Subscriber callbacks are user code. Running them on the reader thread makes
// read() wait on them, so a slow callback stops the host draining the tty, the
// kernel buffer overflows, bytes are lost mid-frame, and the stream silently
// drops to a fraction of its configured rate. The dispatcher thread exists to
// make callback cost pay in latency and queue depth instead of in data.
//
// Every test below fails against the pre-dispatcher single-thread design.
// ---------------------------------------------------------------------------

namespace {

bool poll_until(const std::function<bool()>& pred, int ms) {
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

// A subscriber that parks on its first invocation until the test releases it,
// then records every frame's seq. Models "the callback is slower than the
// stream" without depending on wall-clock sleeps.
struct BlockingSubscriber {
    std::mutex              m;
    std::condition_variable cv;
    bool                    entered  = false;
    bool                    released = false;
    std::vector<uint8_t>    seen;

    void operator()(const tb::Frame& f) {
        std::unique_lock<std::mutex> lk(m);
        if (!entered) {
            entered = true;
            cv.notify_all();
            cv.wait(lk, [this] { return released; });
        }
        seen.push_back(f.seq);
    }

    void wait_entered() {
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [this] { return entered; });
    }
    void release() {
        { std::lock_guard<std::mutex> lk(m); released = true; }
        cv.notify_all();
    }
    std::vector<uint8_t> snapshot() {
        std::lock_guard<std::mutex> lk(m);
        return seen;
    }
};

}  // namespace

// The core regression guard: while a subscriber is parked, the reader must
// keep reading, parsing and counting frames. Pre-dispatcher this stalled at
// frames_received == 1 and the rest of the burst rotted in the tty buffer.
TEST(Transport, SlowSubscriberDoesNotStallReader) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    BlockingSubscriber sub;
    host.subscribe(tp::Cmd::GetEncoder, std::ref(sub));

    constexpr int kFrames = 50;
    for (int i = 1; i <= kFrames; ++i) {
        pty.send_data(static_cast<uint8_t>(i), tp::Cmd::GetEncoder,
                      std::vector<uint8_t>(16, 0));
    }
    sub.wait_entered();

    // The callback is still parked here — the reader must have drained the
    // whole burst regardless.
    EXPECT_TRUE(poll_until(
        [&] { return host.stats().frames_received >= kFrames; }, 2000))
        << "reader stalled behind the subscriber: frames_received="
        << host.stats().frames_received;

    sub.release();
    EXPECT_TRUE(poll_until(
        [&] { return sub.snapshot().size() == kFrames; }, 2000));
    EXPECT_EQ(host.stats().queue_dropped, 0u)   // 50 frames, 256-deep queue
        << "burst fit in the queue and must not have been evicted";
    EXPECT_GE(host.stats().queue_high_water, 1u);
}

// send_cmd() must not queue behind subscriber work. handle_ack_ deliberately
// stays on the reader thread; pre-dispatcher this test timed out and threw.
TEST(Transport, AckNotBlockedBySlowSubscriber) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));   // ack_timeout 50ms, 2 retries

    BlockingSubscriber sub;
    host.subscribe(tp::Cmd::GetEncoder, std::ref(sub));

    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    sub.wait_entered();   // dispatcher is now parked inside user code

    std::thread fw([&] {
        auto f = pty.expect_frame(1000);
        ASSERT_TRUE(f.has_value());
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });

    const auto t0 = std::chrono::steady_clock::now();
    auto ack = host.send_cmd(tp::Cmd::GetVersion);   // must not throw
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
    fw.join();
    sub.release();

    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);
    EXPECT_EQ(host.stats().retries, 0u)
        << "ACK matching queued behind the subscriber";
    EXPECT_LT(dt, 200) << "send_cmd took " << dt << "ms behind a parked callback";
}

// A consumer that cannot keep up must lose the OLDEST frames, not the newest:
// these are state samples, so currency beats history. The eviction has to be
// counted, otherwise a rate drop is indistinguishable from the firmware
// simply sending less.
TEST(Transport, FullQueueDropsOldestAndCountsIt) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    auto cfg = base_config(pty.slave_path());
    cfg.dispatch_queue_frames = 4;
    tb::Transport host(cfg);

    BlockingSubscriber sub;
    host.subscribe(tp::Cmd::GetEncoder, std::ref(sub));

    // Park the dispatcher on frame 1 BEFORE the burst. Sending everything at
    // once would race the dispatcher's own startup: the reader can fill a
    // 4-deep queue before the thread first runs, and then which frame parks
    // is a scheduling detail rather than something the test states.
    constexpr int kFrames = 20;
    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    sub.wait_entered();
    for (int i = 2; i <= kFrames; ++i) {
        pty.send_data(static_cast<uint8_t>(i), tp::Cmd::GetEncoder,
                      std::vector<uint8_t>(16, 0));
    }
    ASSERT_TRUE(poll_until(
        [&] { return host.stats().frames_received >= kFrames; }, 2000));

    sub.release();
    // Frame 1 was already popped into the callback; 2..20 contended for four
    // slots, so the four newest survive and 15 are evicted.
    EXPECT_TRUE(poll_until([&] { return sub.snapshot().size() == 5u; }, 2000));

    const auto seen = sub.snapshot();
    ASSERT_EQ(seen.size(), 5u);
    EXPECT_EQ(seen, (std::vector<uint8_t>{1, 17, 18, 19, 20}))
        << "queue must evict from the front, keeping the freshest samples";
    EXPECT_EQ(host.stats().queue_dropped, 15u);
    EXPECT_EQ(host.stats().queue_high_water, 4u);
}

// Ordering is a contract: one dispatcher thread, one queue, no reordering.
TEST(Transport, DispatchPreservesFrameOrder) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::mutex           m;
    std::vector<uint8_t> seen;
    host.subscribe(tp::Cmd::GetImu, [&](const tb::Frame& f) {
        std::lock_guard<std::mutex> lk(m);
        seen.push_back(f.seq);
    });

    constexpr int kFrames = 100;
    for (int i = 0; i < kFrames; ++i) {
        pty.send_data(static_cast<uint8_t>(i), tp::Cmd::GetImu,
                      std::vector<uint8_t>(8, 0));
    }
    ASSERT_TRUE(poll_until([&] {
        std::lock_guard<std::mutex> lk(m);
        return seen.size() == kFrames;
    }, 2000));

    std::lock_guard<std::mutex> lk(m);
    for (int i = 0; i < kFrames; ++i) {
        ASSERT_EQ(seen[i], static_cast<uint8_t>(i)) << "reordered at " << i;
    }
}

// stop() must not deliver whatever is still queued — entering user code during
// teardown is what the shutdown ordering exists to prevent (the Python
// binding's callback destructor takes the GIL). The leftovers are counted so
// the drop stays visible rather than silent.
TEST(Transport, StopDropsQueuedFramesInsteadOfDelivering) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    BlockingSubscriber sub;
    host.subscribe(tp::Cmd::GetEncoder, std::ref(sub));

    constexpr int kFrames = 10;
    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    sub.wait_entered();                       // dispatcher parked on frame 1
    for (int i = 2; i <= kFrames; ++i) {
        pty.send_data(static_cast<uint8_t>(i), tp::Cmd::GetEncoder,
                      std::vector<uint8_t>(16, 0));
    }
    ASSERT_TRUE(poll_until(
        [&] { return host.stats().frames_received >= kFrames; }, 2000));

    // stop() sets stop_requested_, joins the reader, then blocks joining the
    // dispatcher — which is parked in frame 1's callback. Waiting for
    // is_running() to clear guarantees the stop flag is already published
    // before we release; otherwise the dispatcher would drain a few frames in
    // the gap and the count below would be a race.
    std::thread stopper([&] { host.stop(); });
    ASSERT_TRUE(poll_until([&] { return !host.is_running(); }, 2000));
    sub.release();
    stopper.join();

    EXPECT_EQ(sub.snapshot().size(), 1u)
        << "no callback may be entered once stop() has begun";
    EXPECT_EQ(host.stats().queue_dropped, static_cast<uint64_t>(kFrames - 1));
    EXPECT_FALSE(host.is_running());
}

// callback_max_us is the number that tells a user their own callback is why
// queue_dropped is climbing.
TEST(Transport, RecordsSlowestCallback) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::atomic<int> count{0};
    host.subscribe(tp::Cmd::GetEncoder, [&](const tb::Frame&) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ++count;
    });

    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    ASSERT_TRUE(poll_until([&] { return count.load() >= 1; }, 2000));

    // Allow for coarse timer resolution; the point is the order of magnitude.
    EXPECT_GE(host.stats().callback_max_us, 40'000u);
}

// Byte loss upstream of the parser has to be attributable. A well-framed
// frame with a bad CRC is the signature of dropped bytes, and it must land in
// crc_errors rather than vanishing into a generic resync.
TEST(Transport, CountsCrcErrorsFromCorruptedStream) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::atomic<int> count{0};
    host.subscribe(tp::Cmd::GetEncoder,
                   [&](const tb::Frame&) { ++count; });

    auto bad = tb::pack_frame(tp::Address::MCU, 1, tp::FrameType::DATA,
                              tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    bad[bad.size() - 2] ^= 0xFF;          // corrupt the CRC, keep the framing
    pty.send_raw(bad);
    pty.send_data(2, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));

    ASSERT_TRUE(poll_until([&] { return count.load() >= 1; }, 2000));
    auto s = host.stats();
    EXPECT_EQ(s.crc_errors, 1u);
    EXPECT_GE(s.resync_bytes, 1u);
    EXPECT_EQ(s.queue_dropped, 0u)
        << "loss was upstream of the queue; the counters must not blur that";
}

// ---- Firmware collision resend (follower >= 1.2.17) ------------------------
// A pushed frame is resent verbatim right after the original when a host write
// overlapped it. On a link where the overlap no longer corrupts bytes both
// copies arrive; the transport must deliver one.
namespace {
struct DataLog {
    std::mutex m;
    std::condition_variable cv;
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> got;   // (seq, payload)
    void add(const tb::Frame& f) {
        std::lock_guard<std::mutex> lk(m);
        got.emplace_back(f.seq, f.payload);
        cv.notify_all();
    }
    bool wait_for(std::size_t n, int ms = 1000) {
        std::unique_lock<std::mutex> lk(m);
        return cv.wait_for(lk, std::chrono::milliseconds(ms), [&] { return got.size() >= n; });
    }
    std::size_t size() { std::lock_guard<std::mutex> lk(m); return got.size(); }
};
}  // namespace

TEST(Transport, DropsVerbatimResendOfPreviousDataFrame) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));
    DataLog log;
    auto s1 = host.subscribe(tp::Cmd::GetMotorStatus, [&](const tb::Frame& f) { log.add(f); });
    auto s2 = host.subscribe(tp::Cmd::GetImu, [&](const tb::Frame& f) { log.add(f); });
    (void)s1; (void)s2;

    const std::vector<uint8_t> a{1, 2, 3, 4}, b{5, 6, 7, 8};
    pty.send_data(10, tp::Cmd::GetMotorStatus, a);
    pty.send_data(10, tp::Cmd::GetMotorStatus, a);   // resend: dropped
    pty.send_data(10, tp::Cmd::GetMotorStatus, a);   // second resend: dropped
    pty.send_data(11, tp::Cmd::GetMotorStatus, a);   // same payload, next seq: delivered
    pty.send_data(12, tp::Cmd::GetImu, b);
    pty.send_data(12, tp::Cmd::GetImu, b);           // resend: dropped
    pty.send_data(13, tp::Cmd::GetImu, b);
    ASSERT_TRUE(log.wait_for(4));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(log.size(), 4u);
    EXPECT_EQ(log.got[0].first, 10); EXPECT_EQ(log.got[1].first, 11);
    EXPECT_EQ(log.got[2].first, 12); EXPECT_EQ(log.got[3].first, 13);
    EXPECT_EQ(host.stats().duplicate_data_dropped, 3u);
    EXPECT_EQ(host.stats().frames_received, 7u);    // received, then filtered
}

TEST(Transport, KeepsDataFramesThatDifferInSeqCmdOrPayload) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));
    DataLog log;
    auto s1 = host.subscribe(tp::Cmd::GetMotorStatus, [&](const tb::Frame& f) { log.add(f); });
    auto s2 = host.subscribe(tp::Cmd::GetImu, [&](const tb::Frame& f) { log.add(f); });
    (void)s1; (void)s2;

    pty.send_data(20, tp::Cmd::GetMotorStatus, {1, 2});
    pty.send_data(20, tp::Cmd::GetMotorStatus, {1, 3});   // same seq, other payload
    pty.send_data(20, tp::Cmd::GetImu, {1, 3});           // same seq + payload, other cmd
    pty.send_data(21, tp::Cmd::GetImu, {1, 3});           // other seq
    // Only the immediately previous frame counts: A B A is not a resend.
    pty.send_data(30, tp::Cmd::GetMotorStatus, {9});
    pty.send_data(31, tp::Cmd::GetMotorStatus, {8});
    pty.send_data(30, tp::Cmd::GetMotorStatus, {9});
    ASSERT_TRUE(log.wait_for(7));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(log.size(), 7u);
    EXPECT_EQ(host.stats().duplicate_data_dropped, 0u);
}

TEST(Transport, AckBetweenDataFramesDoesNotHideAResend) {
    // ACKs carry the host's seq and are never DATA: they neither reset nor
    // match the previous-DATA record.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));
    DataLog log;
    auto s = host.subscribe(tp::Cmd::GetMotorStatus, [&](const tb::Frame& f) { log.add(f); });
    (void)s;
    pty.send_data(40, tp::Cmd::GetMotorStatus, {7, 7});
    pty.send_ack_ok(40, tp::Cmd::GetVersion);              // stray ACK, same seq
    pty.send_data(40, tp::Cmd::GetMotorStatus, {7, 7});    // still the resend
    pty.send_data(41, tp::Cmd::GetMotorStatus, {7, 7});
    ASSERT_TRUE(log.wait_for(2));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(log.size(), 2u);
    EXPECT_EQ(host.stats().duplicate_data_dropped, 1u);
}

TEST(Transport, InjectedExchangeRecordsAreNeverDeduplicated) {
    // Exchange replies are split into records that all carry the reply's seq;
    // identical consecutive records (an unchanged cached motor status) must
    // all be delivered, and must not interact with the wire-DATA record.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));
    DataLog log;
    auto s = host.subscribe(tp::Cmd::GetMotorStatus, [&](const tb::Frame& f) { log.add(f); });
    (void)s;
    pty.send_data(50, tp::Cmd::GetMotorStatus, {4, 4});
    ASSERT_TRUE(log.wait_for(1));
    for (int i = 0; i < 3; ++i) {
        tb::Frame f{};
        f.addr = tp::Address::MCU; f.seq = 50; f.cmd = tp::Cmd::GetMotorStatus; f.payload = {4, 4};
        host.inject_data(std::move(f));
    }
    ASSERT_TRUE(log.wait_for(4));
    pty.send_data(50, tp::Cmd::GetMotorStatus, {4, 4});    // wire resend of the wire frame
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_EQ(log.size(), 4u);
    EXPECT_EQ(host.stats().duplicate_data_dropped, 1u);
}

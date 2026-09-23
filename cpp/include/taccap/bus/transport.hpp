// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Asynchronous transport for TC-GU-01:
//   - owns a SerialBus + FrameParser + a reader thread + a dispatcher thread
//   - matches CMD_NEED_ACK frames to incoming ACK frames by seq, with
//     host-side timeout/retry honouring the firmware's 10ms / 3-attempt spec
//   - dispatches DATA frames to per-Cmd subscriber callbacks
//
// Threading contract (two threads, deliberately):
//
//   reader thread      read() -> parse -> { ACK: fulfil the promise inline
//                                           DATA: push to dispatch_queue_ }
//   dispatcher thread  pop -> fan out to matching subscriber callbacks
//
// The split exists because subscriber callbacks are user code: a Python
// callback must take the GIL, and CPython's default 5ms switch interval means
// a single contended acquire can exceed the whole per-frame budget (at 200Hz
// x 3 stream sources the reader has ~1.67ms per frame). With callbacks on the
// reader thread, that stalls read(), the kernel tty buffer (4KB on n_tty)
// overflows, bytes are lost mid-frame, and the stream silently drops to a
// fraction of its configured rate. Keeping read() free of user code makes
// callback cost cost *latency and queue depth* instead of *data*.
//
// It also decouples ACK matching from callback load: handle_ack_ still runs
// on the reader, so send_cmd() no longer times out behind a slow subscriber.
//
// Writers are serialised by an internal mutex, so send_cmd() and
// send_cmd_no_ack() may be called from any number of threads without their
// frames interleaving on the wire. That is frame integrity only -- it says
// nothing about the ORDER two threads' commands reach the firmware, and it
// cannot stop concurrent traffic from disturbing the firmware's own timing.
//
// Callbacks are still expected to be reasonably quick. The queue is bounded
// and drops the OLDEST frame when full (state telemetry wants currency, not
// history); Stats::queue_dropped and Stats::callback_max_us make both the
// loss and its cause visible. Synchronous send_cmd() blocks the caller.

#pragma once

#include <taccap/bus/frame.hpp>
#include <taccap/bus/serial_bus.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace xense::taccap::bus {

// Decoded ACK / NACK from the firmware. The wire format mirrors firmware
// `protocol_handler.c` exactly:
//
//   - Failure path → `protocol_send_ack(seq, err)`:
//       frame.cmd == 0, frame.payload = [err_code]   (1 byte)
//   - Success path → `protocol_send_response(seq, cmd, ERR_OK, data, n)`:
//       frame.cmd == cmd
//       frame.payload = [ERR_OK] when n == 0 (handler had no data)
//       frame.payload = data    when n > 0  (NO err_code prefix)
//
// There is no retry_count on the wire — that field of `protocol::AckPayload`
// is firmware-internal retry book-keeping, not the ACK frame layout.
struct AckResponse {
    uint8_t              seq;         // matched against the request seq
    protocol::Cmd        cmd;         // 0 ↔ NACK; original cmd ↔ success
    bool                 is_nack;     // true iff static_cast<uint8_t>(cmd) == 0
    protocol::ErrorCode  error_code;  // Ok on success; payload[0] on NACK
    std::vector<uint8_t> data;        // wire payload as-is (response data on
                                      // success, [err_code] on NACK)
};

// Recover the firmware's error code from a response, including the echoed-cmd
// path that `is_nack` cannot see.
//
// Only handler *dispatch* failures take the cmd==0 wire path. A handler that
// returns non-OK goes through protocol_send_response(seq, cmd, err, NULL, 0),
// which echoes the command byte and packs the error as a single payload byte
// — indistinguishable at the transport layer from a legitimate 1-byte
// success response.
//
// This resolves the ambiguity in favour of "error", so it is ONLY valid for
// commands whose successful response is never a single non-zero byte. That
// holds for Cmd::Ota* (success is [0x00] or an 8-byte OtaStatus) and for the
// V2.0 calibration commands (success is [0x00], 33 B or 5 B). It does NOT
// hold for Cmd::MotorGetCanId / Cmd::MotorGetProtocol, whose success payload
// is exactly one meaningful non-zero byte — never use this for those.
protocol::ErrorCode ack_error_code(const AckResponse& ack) noexcept;

// How send_cmd allocates the sequence number across retry attempts.
enum class RetryMode : uint8_t {
    // Every attempt gets a fresh seq. Correct for idempotent commands, and
    // the historical behaviour.
    NewSeq,
    // Every attempt reuses the first seq. Use for commands the firmware must
    // NOT execute twice: the firmware keeps the last request's seq/cmd/payload
    // hash plus its real response, and replays that response for a byte-
    // identical repeat instead of re-running the handler (see
    // protocol_resend_cached_response in protocol_handler.c).
    //
    // This matters for Cmd::OtaWriteBlock above all: the firmware demands
    // strictly sequential offsets and marks the whole session failed on a
    // repeat, so a NewSeq retry after a merely-slow ACK would kill an
    // otherwise fine update.
    //
    // The firmware caches exactly one request, so nothing else may be sent on
    // this transport between attempts — true for the single-threaded OTA flow.
    SameSeq,
};

class Transport {
public:
    using DataCallback   = std::function<void(const Frame&)>;
    using SubscriptionId = uint64_t;

    struct Config {
        SerialBus::Config         serial;
        protocol::Address         peer            = protocol::Address::MCU;
        std::chrono::milliseconds ack_timeout     {10};
        unsigned                  max_retries     = 3;
        std::chrono::milliseconds retry_interval  {10};
        std::size_t               rx_chunk_bytes  = 4096;
        std::size_t               parser_max_buf  = 64 * 1024;
        // Depth of the reader -> dispatcher queue, in frames. Sized to absorb
        // a scheduler/GC hiccup without loss: 256 frames is ~400ms at the
        // 600 frames/s of a 200Hz three-source stream.
        //
        // It is a burst absorber, not a backlog. A consumer that is
        // *persistently* slower than the stream keeps the queue full, which
        // costs depth/rate of added staleness (and shows up as a rising
        // queue_dropped) — lower this when latency matters more than
        // surviving hiccups. Clamped to >= 1.
        std::size_t               dispatch_queue_frames = 256;
    };

    struct Stats {
        uint64_t bytes_read           = 0;
        uint64_t bytes_written        = 0;
        uint64_t frames_received      = 0;
        uint64_t frames_sent          = 0;
        uint64_t ack_timeouts         = 0;
        uint64_t retries              = 0;
        uint64_t unexpected_frames    = 0;
        uint64_t callback_exceptions  = 0;

        // ---- Loss accounting ------------------------------------------
        // Three distinct ways a frame can fail to reach a subscriber. Read
        // together they say *where* a rate drop came from:
        //
        //   crc_errors / resync_bytes > 0  -> bytes were lost before the
        //       parser. Two different causes, told apart by bytes_read:
        //         - bytes_read also down: the host stopped draining the tty
        //           long enough to overflow the kernel/USB buffers.
        //           Cross-check callback_max_us.
        //         - bytes_read at full volume, crc_errors flat, resync_bytes
        //           climbing in ~one-frame steps: the MCU dropped bytes out of
        //           the middle of a frame it was transmitting, which it does
        //           when host->MCU traffic overlaps its own send. The short
        //           payload fails the LEN check before CRC is ever reached,
        //           which is why crc_errors stays at zero. Nothing on the host
        //           is broken; stop overlapping the writes (submit on the
        //           status-frame doorbell, as both controllers do).
        //   queue_dropped > 0              -> bytes arrived fine, but the
        //       subscriber could not keep up and old frames were evicted.
        //   both zero, rate still low      -> the firmware really is sending
        //       slower; nothing on the host side to fix.
        uint64_t crc_errors           = 0;  // well-framed, bad CRC → byte loss
        uint64_t resync_bytes         = 0;  // bytes discarded without a frame
        uint64_t parser_overflow_bytes = 0; // dropped by the parser_max_buf cap
        uint64_t queue_dropped        = 0;  // DATA frames evicted, queue full
        uint64_t queue_high_water     = 0;  // deepest the queue has ever been
        uint64_t callback_max_us      = 0;  // slowest single fan-out, in us
    };

    // Opens the serial port, starts the reader thread.
    explicit Transport(const Config& cfg);

    // Stops the reader, fails any pending ACKs with IoError, joins.
    ~Transport();

    Transport(const Transport&)            = delete;
    Transport& operator=(const Transport&) = delete;

    // Send CMD_NEED_ACK and block until matching ACK arrives or all retries
    // are exhausted. `timeout` of 0 falls back to Config::ack_timeout.
    // Throws ProtocolError on NACK, TimeoutError on retry exhaustion,
    // IoError on transport failure.
    AckResponse send_cmd(protocol::Cmd cmd,
                         const std::vector<uint8_t>& payload = {},
                         std::chrono::milliseconds timeout = std::chrono::milliseconds{0},
                         RetryMode retry = RetryMode::NewSeq);

    // Send CMD_NO_ACK fire-and-forget.
    void send_cmd_no_ack(protocol::Cmd cmd,
                         const std::vector<uint8_t>& payload = {});

    // Register a callback for DATA frames whose `cmd` byte matches.
    // Callbacks run on the dispatcher thread, never on the reader, and are
    // serialised with each other. Subscriptions are resolved at dispatch
    // time, so unsubscribe() takes effect even for already-queued frames.
    SubscriptionId subscribe(protocol::Cmd cmd, DataCallback cb);
    void unsubscribe(SubscriptionId id);

    bool is_running() const noexcept;

    // Graceful, idempotent shutdown. Signals both workers, joins them, and
    // only THEN drops the subscriptions, so every callback object is
    // destroyed on the calling thread — never on a worker. That ordering
    // matters for the Python bindings, whose callback destructor takes the
    // GIL. The dispatcher exits without draining whatever is still queued
    // (entering user code during teardown is what this ordering forbids);
    // those frames are counted into Stats::queue_dropped.
    void stop() noexcept;

    Stats stats() const noexcept;

    const Config& config() const noexcept { return cfg_; }

private:
    struct PendingAck {
        std::promise<AckResponse> promise;
    };
    struct Sub {
        SubscriptionId  id;
        protocol::Cmd   cmd;
        DataCallback    cb;
    };

    // Serialise every byte we put on the link. SerialBus::write() loops over
    // partial writes, and that loop is only interruptible when ::write()
    // returns short -- which a BLOCKING tty never does (n_tty holds
    // atomic_write_lock and loops internally; measured at 40 x 100 kB writes
    // against a starved drain, zero short writes). So this mutex prevents no
    // failure we can currently produce; it makes the invariant ours instead of
    // the kernel tty layer's, which a non-blocking fd or a non-tty transport
    // would quietly take away. Held across the (blocking) write, so a submit
    // can queue behind a large frame; that is the cost of one link.
    void write_frame_(const std::vector<uint8_t>& wire);

    void reader_loop_();
    void dispatch_loop_();
    void dispatch_(Frame&& f);
    void handle_ack_(const Frame& f);
    void enqueue_data_(Frame&& f);
    void handle_data_(const Frame& f);

    // Set stop_requested_, wake the dispatcher, join both workers. Safe to
    // call when the threads are already gone.
    void join_workers_() noexcept;

    // Fail every pending ACK with the given error message. Called on
    // reader exit and on stop().
    void fail_pending_(const std::string& reason) noexcept;

    // Drop every subscription. The callbacks are destroyed OUTSIDE sub_mu_ —
    // a callback destructor may take arbitrary locks (the Python binding's
    // takes the GIL), and holding the subscription mutex across that would
    // invite a lock-order inversion against the reader thread.
    void clear_subs_() noexcept;

    Config         cfg_;
    SerialBus      serial_;
    FrameParser    parser_;

    std::atomic<bool>          stop_requested_{false};
    std::atomic<bool>          running_{false};
    std::thread                reader_;
    std::thread                dispatcher_;

    // Serialises writers; see write_frame_().
    std::mutex                 write_mu_;

    // reader -> dispatcher hand-off. Bounded; drop-oldest on overflow.
    std::mutex                 queue_mu_;
    std::condition_variable    queue_cv_;
    std::deque<Frame>          queue_;

    // ACK matching
    std::mutex                                 pending_mu_;
    std::unordered_map<uint8_t, PendingAck>    pending_acks_;
    std::atomic<uint8_t>                       next_seq_{0};

    // Subscriptions
    std::mutex                                 sub_mu_;
    std::vector<Sub>                           subs_;
    std::atomic<SubscriptionId>                next_sub_id_{1};

    // Stats (atomic counters; read via Stats snapshot)
    std::atomic<uint64_t> stat_bytes_read_         {0};
    std::atomic<uint64_t> stat_bytes_written_      {0};
    std::atomic<uint64_t> stat_frames_received_    {0};
    std::atomic<uint64_t> stat_frames_sent_        {0};
    std::atomic<uint64_t> stat_ack_timeouts_       {0};
    std::atomic<uint64_t> stat_retries_            {0};
    std::atomic<uint64_t> stat_unexpected_frames_  {0};
    std::atomic<uint64_t> stat_callback_exceptions_{0};
    // Mirrored from FrameParser::stats() by the reader thread (the parser
    // itself is single-threaded and keeps plain counters).
    std::atomic<uint64_t> stat_crc_errors_         {0};
    std::atomic<uint64_t> stat_resync_bytes_       {0};
    std::atomic<uint64_t> stat_parser_overflow_    {0};
    std::atomic<uint64_t> stat_queue_dropped_      {0};
    std::atomic<uint64_t> stat_queue_high_water_   {0};
    std::atomic<uint64_t> stat_callback_max_us_    {0};
};

}  // namespace xense::taccap::bus

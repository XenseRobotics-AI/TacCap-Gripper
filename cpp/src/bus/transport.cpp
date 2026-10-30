// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>
#include <cerrno>
#include <utility>

namespace xense::taccap::bus {

namespace {
using namespace std::chrono_literals;
}

// ---- ctor / dtor ----------------------------------------------------------

Transport::Transport(const Config& cfg)
    : cfg_(cfg),
      serial_(cfg.serial),
      parser_(cfg.parser_max_buf) {
    if (cfg_.dispatch_queue_frames == 0) cfg_.dispatch_queue_frames = 1;
    if (cfg_.control_queue_frames == 0) cfg_.control_queue_frames = 1;
    running_.store(true, std::memory_order_release);
    // Dispatcher first: the reader must never find itself without a consumer.
    dispatcher_ = std::thread(&Transport::dispatch_loop_, this);
    try {
        control_worker_ = std::thread(&Transport::control_loop_, this);
        reader_ = std::thread(&Transport::reader_loop_, this);
    } catch (...) {
        // The ctor did not complete, so ~Transport never runs — and a joinable
        // std::thread destructor calls std::terminate. Wind the dispatcher
        // down by hand before letting the exception out.
        running_.store(false, std::memory_order_release);
        join_workers_();
        throw;
    }
}

Transport::~Transport() {
    stop();
}

void Transport::close() noexcept {
    stop();              // workers joined first: nothing is reading the fd any more
    serial_.close();
}

// ---- public API -----------------------------------------------------------

protocol::ErrorCode ack_error_code(const AckResponse& ack) noexcept {
    if (ack.is_nack) return ack.error_code;
    // Echoed-cmd error path: protocol_send_response(seq, cmd, err, NULL, 0)
    // packs the error as the whole payload with the command byte intact.
    if (ack.data.size() == 1 && ack.data[0] != 0) {
        return static_cast<protocol::ErrorCode>(ack.data[0]);
    }
    return protocol::ErrorCode::Ok;
}

AckResponse Transport::send_cmd(protocol::Cmd cmd,
                                const std::vector<uint8_t>& payload,
                                std::chrono::milliseconds timeout,
                                RetryMode retry) {
    const auto t = (timeout.count() == 0) ? cfg_.ack_timeout : timeout;
    uint8_t seq = 0;
    bool reserved = false;
    std::shared_ptr<PendingAck> pending;
    auto release = [&]() {
        std::lock_guard<std::mutex> lk(pending_mu_);
        if (!reserved) return;
        auto it = pending_acks_.find(seq);
        if (it != pending_acks_.end() && it->second == pending)
            pending_acks_.erase(it);
        reserved_seqs_[seq] = false;
        reserved = false;
    };
    const unsigned max_retries = retry == RetryMode::Once ? 0u : cfg_.max_retries;
    try {
        for (unsigned attempt = 0; attempt <= max_retries; ++attempt) {
            if (reserved && retry == RetryMode::NewSeq) release();
            std::future<AckResponse> fut;
            {
                std::lock_guard<std::mutex> lk(pending_mu_);
                if (!running_.load(std::memory_order_acquire))
                    throw IoError("send_cmd on stopped transport", EBADF);
                if (!reserved) {
                    seq = allocate_seq_locked_();
                    reserved_seqs_[seq] = true;
                    reserved = true;
                }
                pending = std::make_shared<PendingAck>();
                pending->cmd = cmd;
                fut = pending->promise.get_future();
                pending_acks_[seq] = pending;
            }
            auto turn = half_duplex_turn_();
            write_frame_(pack_frame(cfg_.peer, seq,
                         protocol::FrameType::CMD_NEED_ACK, cmd, payload), cmd);
            const bool answered = fut.wait_for(t) == std::future_status::ready;
            if (turn.owns_lock()) turn.unlock();
            if (answered) {
                auto ack = fut.get();
                release();
                return ack;
            }
            {
                std::lock_guard<std::mutex> lk(pending_mu_);
                auto it = pending_acks_.find(seq);
                if (it != pending_acks_.end() && it->second == pending)
                    pending_acks_.erase(it);
            }
            // SameSeq owns its slot throughout retry delays (OTA deduplication).
            stat_ack_timeouts_.fetch_add(1, std::memory_order_relaxed);
            if (attempt < max_retries) {
                stat_retries_.fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_for(cfg_.retry_interval);
            }
        }
        throw TimeoutError("send_cmd " + std::string(protocol::to_string(cmd)) +
                           ": no ACK after " + std::to_string(max_retries + 1) +
                           " attempts", ETIMEDOUT);
    } catch (...) {
        release();
        throw;
    }
}

// pending_mu_ guards reservation across the entire request lifetime, including
// timeout/ACK races and SameSeq retries. Never overwrite another promise.
uint8_t Transport::allocate_seq_locked_() {
    for (unsigned i = 0; i < 256; ++i) {
        const uint8_t seq = next_seq_++;
        if (!reserved_seqs_[seq]) return seq;
    }
    throw IoError("Transport: all ACK sequence slots occupied", EBUSY);
}

void Transport::send_cmd_no_ack(protocol::Cmd cmd,
                                const std::vector<uint8_t>& payload) {
    uint8_t seq;
    {
        std::lock_guard<std::mutex> lk(pending_mu_);
        if (!running_.load(std::memory_order_acquire))
            throw IoError("send_cmd_no_ack on stopped transport", EBADF);
        seq = allocate_seq_locked_();
    }
    auto turn = half_duplex_turn_();
    write_frame_(pack_frame(cfg_.peer, seq,
                 protocol::FrameType::CMD_NO_ACK, cmd, payload), cmd);
}

std::unique_lock<std::mutex> Transport::half_duplex_turn_() {
    if (!half_duplex_.load(std::memory_order_acquire)) return {};
    return std::unique_lock<std::mutex>(half_duplex_mu_);
}

void Transport::inject_data(Frame f) {
    f.type = protocol::FrameType::DATA;
    if (f.cmd == protocol::Cmd::GetMotorStatus) enqueue_control_(f);
    enqueue_data_(std::move(f));
}

Transport::TargetWrites Transport::target_writes() {
    std::lock_guard<std::mutex> lk(write_mu_);
    return target_writes_;
}

// The one place bytes leave for the MCU. Packing stays outside the lock; only
// the write is serialised.
void Transport::write_frame_(const std::vector<uint8_t>& wire, protocol::Cmd cmd) {
    {
        std::lock_guard<std::mutex> lk(write_mu_);
        serial_.write(wire);
        if (cmd == protocol::Cmd::MotorPosCtrl || cmd == protocol::Cmd::MotorVelCtrl ||
            cmd == protocol::Cmd::MotorTorqueCtrl || cmd == protocol::Cmd::MotorImpedanceCtrl) {
            const auto now = std::chrono::steady_clock::now();
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();
            if (target_writes_.count != 0) {
                target_writes_.last_gap_us = (ns - target_writes_.last_write_ns) / 1000;
                target_writes_.max_gap_us = std::max(target_writes_.max_gap_us,
                                                    target_writes_.last_gap_us);
            }
            ++target_writes_.count;
            target_writes_.last_write_ns = ns;
        }
    }
    stat_bytes_written_.fetch_add(wire.size(), std::memory_order_relaxed);
    stat_frames_sent_.fetch_add(1, std::memory_order_relaxed);
}

Transport::SubscriptionId
Transport::subscribe(protocol::Cmd cmd, DataCallback cb) {
    SubscriptionId id = next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(sub_mu_);
    subs_.push_back({id, cmd, std::move(cb)});
    return id;
}

void Transport::unsubscribe(SubscriptionId id) {
    if (unsubscribe_control_(id)) return;
    std::lock_guard<std::mutex> lk(sub_mu_);
    subs_.erase(std::remove_if(subs_.begin(), subs_.end(),
                               [id](const Sub& s) { return s.id == id; }),
                subs_.end());
}

bool Transport::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

void Transport::clear_subs_() noexcept {
    { std::lock_guard<std::mutex> lock(control_sub_mu_);
      control_subs_.clear(); control_sub_count_.store(0); }
    std::vector<Sub> doomed;
    {
        std::lock_guard<std::mutex> lk(sub_mu_);
        doomed.swap(subs_);
    }
    // `doomed` destructs here, with sub_mu_ released: the Python bindings'
    // callback destructor acquires the GIL, and the reader thread may be
    // holding the GIL inside a callback while it waits for sub_mu_.
}

void Transport::join_workers_() noexcept {
    // stop_requested_ is already set by stop(); re-assert it for the direct
    // callers (there are none today, but the join below deadlocks without it).
    stop_requested_.store(true, std::memory_order_release);

    // The reader only reads/parses/enqueues — no user code — so it exits
    // within one VTIME tick regardless of how slow subscribers are.
    if (reader_.joinable()) reader_.join();

    { std::lock_guard<std::mutex> lk(control_queue_mu_); }
    control_queue_cv_.notify_all();
    if (control_worker_.joinable()) control_worker_.join();

    // Taking queue_mu_ here closes the lost-wakeup window: if the dispatcher
    // has evaluated its predicate but not yet blocked, it still holds the
    // mutex, so this blocks until it is genuinely waiting on the cv.
    { std::lock_guard<std::mutex> lk(queue_mu_); }
    queue_cv_.notify_all();
    if (dispatcher_.joinable()) dispatcher_.join();
}

void Transport::stop() noexcept {
    // Order matters: stop_requested_ is the workers' exit signal, is_running()
    // is what callers observe. Publishing the signal first means anyone who
    // sees is_running() == false knows the workers are already committed to
    // exiting — no window where the transport looks stopped but a queued
    // frame can still reach a callback.
    //
    // Both stores are unconditional: the reader's I/O-failure path clears
    // running_ without ever setting stop_requested_, so gating this on
    // running_ would leave the dispatcher parked on the cv forever.
    stop_requested_.store(true, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    join_workers_();

    // Subscriptions are dropped only AFTER both workers are joined. The
    // dispatcher holds a copy of each matching callback while it fans out, so
    // clearing earlier could make the dispatcher's copy the last reference —
    // and the Python binding's deleter takes the GIL. Clearing here, with
    // both workers gone, guarantees the callbacks die on the caller's thread.
    // No callback can be entered in the meantime: the dispatcher checks
    // stop_requested_ before every dequeue.
    clear_subs_();

    // Whatever is still queued is dropped rather than delivered during
    // teardown; account for it so the counter stays honest.
    {
        std::lock_guard<std::mutex> lk(queue_mu_);
        if (!queue_.empty()) {
            stat_queue_dropped_.fetch_add(queue_.size(),
                                          std::memory_order_relaxed);
            queue_.clear();
        }
    }

    fail_pending_("transport stopped");
    {
        std::lock_guard<std::mutex> lock(control_queue_mu_);
        stat_control_queue_dropped_.fetch_add(control_queue_.size());
        control_queue_.clear();
    }
}

Transport::Stats Transport::stats() const noexcept {
    Stats s;
    s.bytes_read          = stat_bytes_read_.load(std::memory_order_relaxed);
    s.bytes_written       = stat_bytes_written_.load(std::memory_order_relaxed);
    s.frames_received     = stat_frames_received_.load(std::memory_order_relaxed);
    s.frames_sent         = stat_frames_sent_.load(std::memory_order_relaxed);
    s.ack_timeouts        = stat_ack_timeouts_.load(std::memory_order_relaxed);
    s.retries             = stat_retries_.load(std::memory_order_relaxed);
    s.unexpected_frames   = stat_unexpected_frames_.load(std::memory_order_relaxed);
    s.callback_exceptions = stat_callback_exceptions_.load(std::memory_order_relaxed);
    s.crc_errors            = stat_crc_errors_.load(std::memory_order_relaxed);
    s.resync_bytes          = stat_resync_bytes_.load(std::memory_order_relaxed);
    s.parser_overflow_bytes = stat_parser_overflow_.load(std::memory_order_relaxed);
    s.parser_no_head = stat_parser_no_head_.load(std::memory_order_relaxed);
    s.parser_oversize = stat_parser_oversize_.load(std::memory_order_relaxed);
    s.parser_short_body = stat_parser_short_body_.load(std::memory_order_relaxed);
    s.parser_bad_length = stat_parser_bad_length_.load(std::memory_order_relaxed);
    s.queue_dropped         = stat_queue_dropped_.load(std::memory_order_relaxed);
    s.queue_high_water      = stat_queue_high_water_.load(std::memory_order_relaxed);
    s.callback_max_us       = stat_callback_max_us_.load(std::memory_order_relaxed);
    s.control_queue_dropped = stat_control_queue_dropped_.load(std::memory_order_relaxed);
    s.control_queue_high_water = stat_control_queue_high_water_.load(std::memory_order_relaxed);
    s.control_queue_overflows = stat_control_queue_overflows_.load(std::memory_order_relaxed);
    return s;
}

// ---- reader thread --------------------------------------------------------

void Transport::reader_loop_() {
    std::vector<uint8_t> buf(cfg_.rx_chunk_bytes);

    while (!stop_requested_.load(std::memory_order_acquire)) {
        std::size_t n = 0;
        try {
            n = serial_.read(buf.data(), buf.size());
        } catch (const std::exception& e) {
            running_.store(false, std::memory_order_release);
            fail_pending_(std::string("reader: ") + e.what());
            return;
        }
        if (n == 0) continue;   // VTIME wakeup, no data
        const auto received = std::chrono::steady_clock::now();

        stat_bytes_read_.fetch_add(n, std::memory_order_relaxed);
        parser_.feed(buf.data(), n);

        // The parser keeps plain counters (it is single-threaded by
        // contract); publish them here so stats() can be read from any
        // thread. They are monotonic, so a store is enough.
        const auto& ps = parser_.stats();
        stat_crc_errors_.store(ps.crc_errors, std::memory_order_relaxed);
        stat_resync_bytes_.store(ps.resync_bytes, std::memory_order_relaxed);
        stat_parser_overflow_.store(ps.overflow_bytes, std::memory_order_relaxed);
        stat_parser_no_head_.store(ps.no_head, std::memory_order_relaxed);
        stat_parser_oversize_.store(ps.oversize, std::memory_order_relaxed);
        stat_parser_short_body_.store(ps.short_body, std::memory_order_relaxed);
        stat_parser_bad_length_.store(ps.bad_length, std::memory_order_relaxed);

        Frame f;
        while (parser_.try_pop(f)) {
            f.receive_time = received;
            stat_frames_received_.fetch_add(1, std::memory_order_relaxed);
            dispatch_(std::move(f));
        }
    }
}

// Consumes the reader's hand-off queue and runs subscriber callbacks. This is
// the ONLY thread that touches user code.
void Transport::dispatch_loop_() {
    for (;;) {
        Frame f;
        {
            std::unique_lock<std::mutex> lk(queue_mu_);
            queue_cv_.wait(lk, [this] {
                return !queue_.empty() ||
                       stop_requested_.load(std::memory_order_acquire);
            });
            // Shutdown beats drain: entering a callback during teardown is
            // exactly what stop()'s ordering contract rules out. Leftovers
            // are counted as dropped by stop().
            if (stop_requested_.load(std::memory_order_acquire)) return;
            f = std::move(queue_.front());
            queue_.pop_front();
        }
        handle_data_(f);   // queue_mu_ released — callbacks may re-enter
    }
}

void Transport::dispatch_(Frame&& f) {
    switch (f.type) {
        case protocol::FrameType::ACK:
            // Stays on the reader thread on purpose: it only fulfils a
            // promise, and routing it through the queue would put send_cmd()
            // latency behind whatever the subscribers are doing.
            handle_ack_(f);
            break;
        case protocol::FrameType::DATA:
            if (f.cmd == protocol::Cmd::GetMotorStatus) enqueue_control_(f);
            enqueue_data_(std::move(f));
            break;
        case protocol::FrameType::CMD_NEED_ACK:
        case protocol::FrameType::CMD_NO_ACK:
        default:
            stat_unexpected_frames_.fetch_add(1, std::memory_order_relaxed);
            break;
    }
}

void Transport::handle_ack_(const Frame& f) {
    // Match by frame.seq (firmware echoes the request seq into the response
    // frame's seq field).
    std::promise<AckResponse> winning_promise;
    bool found = false;
    {
        std::lock_guard<std::mutex> lk(pending_mu_);
        auto it = pending_acks_.find(f.seq);
        if (it != pending_acks_.end() &&
            (f.cmd == it->second->cmd ||
             (static_cast<uint8_t>(f.cmd) == 0 && f.payload.size() == 1))) {
            winning_promise = std::move(it->second->promise);
            pending_acks_.erase(it);
            found = true;
        }
    }
    if (!found) {
        // Stray ACK (sender already timed out). Drop quietly.
        stat_unexpected_frames_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    AckResponse r{};
    r.seq      = f.seq;
    r.cmd      = f.cmd;
    r.is_nack  = false;
    r.data     = f.payload;
    r.receive_time = f.receive_time;

    if (static_cast<uint8_t>(f.cmd) == 0) {
        // cmd == 0 means the firmware took the protocol_send_ack(seq, err)
        // wire path instead of protocol_send_response. The first payload
        // byte carries the error code:
        //   - ERR_OK : the handler succeeded but didn't echo the command
        //              (e.g. StopStream uses this path on TC-GU-01 v1.1
        //              firmware). Treat as success with no data.
        //   - !ERR_OK: a real NACK.
        const auto err = (!f.payload.empty())
            ? static_cast<protocol::ErrorCode>(f.payload[0])
            : protocol::ErrorCode::InvalidCmd;
        if (err != protocol::ErrorCode::Ok) {
            r.is_nack    = true;
            r.error_code = err;
            try {
                winning_promise.set_exception(std::make_exception_ptr(
                    NackError(std::string("NACK: ") +
                              protocol::to_string(err), static_cast<uint8_t>(err))));
            } catch (...) { /* promise already satisfied — ignore */ }
            return;
        }
        // else: pure-ACK success → fall through to the success path below
        r.error_code = protocol::ErrorCode::Ok;
    } else {
        // cmd != 0: standard send_response path. The wire payload is the
        // response data verbatim. A single 0x00 byte means "no data";
        // longer payloads carry typed data (firmware_version_t, sn_info_t,
        // ImuData, ...).
        r.error_code = protocol::ErrorCode::Ok;
    }

    try {
        winning_promise.set_value(std::move(r));
    } catch (...) { /* promise already satisfied — ignore */ }
}

// Reader-thread side of the hand-off. Must stay O(1) and lock-light: every
// microsecond spent here is a microsecond not spent draining the tty.
void Transport::enqueue_data_(Frame&& f) {
    std::size_t depth = 0;
    {
        std::lock_guard<std::mutex> lk(queue_mu_);
        // Drop-oldest. These are state samples: a backed-up consumer should
        // lose history, not currency. Blocking instead would reintroduce the
        // very coupling this queue exists to break.
        while (queue_.size() >= cfg_.dispatch_queue_frames) {
            queue_.pop_front();
            stat_queue_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        queue_.push_back(std::move(f));
        depth = queue_.size();
    }
    queue_cv_.notify_one();

    uint64_t hw = stat_queue_high_water_.load(std::memory_order_relaxed);
    while (depth > hw &&
           !stat_queue_high_water_.compare_exchange_weak(
               hw, depth, std::memory_order_relaxed)) {
    }
}

void Transport::handle_data_(const Frame& f) {
    // Snapshot matching callbacks under lock; release before calling so
    // callbacks can safely subscribe/unsubscribe (re-entrant).
    std::vector<DataCallback> hits;
    {
        std::lock_guard<std::mutex> lk(sub_mu_);
        for (const auto& s : subs_) {
            if (s.cmd == f.cmd) hits.push_back(s.cb);
        }
    }
    if (hits.empty()) return;

    // Time the fan-out. This is the number that tells a user their callback
    // is the reason queue_dropped is climbing, so it is worth two clock reads
    // per frame (steady_clock::now() is a vDSO call, ~20ns).
    const auto t0 = std::chrono::steady_clock::now();
    for (auto& cb : hits) {
        try {
            cb(f);
        } catch (...) {
            stat_callback_exceptions_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    const auto us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count());
    uint64_t worst = stat_callback_max_us_.load(std::memory_order_relaxed);
    while (us > worst &&
           !stat_callback_max_us_.compare_exchange_weak(
               worst, us, std::memory_order_relaxed)) {
    }
}

Transport::SubscriptionId Transport::subscribe_control_(DataCallback cb) {
    {
        std::lock_guard<std::mutex> lock(control_queue_mu_);
        if (control_failed_) throw IoError("motor control receive lane overflowed; reopen transport", EOVERFLOW);
    }
    auto sub = std::make_shared<ControlSub>();
    sub->id = next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    sub->cb = std::move(cb);
    sub->since = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(control_sub_mu_);
    control_subs_.push_back(sub);
    control_sub_count_.fetch_add(1, std::memory_order_release);
    return sub->id;
}

bool Transport::unsubscribe_control_(SubscriptionId id) {
    std::shared_ptr<ControlSub> sub;
    {
        std::lock_guard<std::mutex> lock(control_sub_mu_);
        auto it = std::find_if(control_subs_.begin(), control_subs_.end(),
                              [id](const auto& s) { return s->id == id; });
        if (it == control_subs_.end()) return false;
        sub = *it;
        control_subs_.erase(it);
        control_sub_count_.fetch_sub(1, std::memory_order_release);
    }
    std::unique_lock<std::mutex> lock(sub->mutex);
    sub->active = false;
    // A copied subscription may already be executing. Controller destruction
    // must wait for it; simply removing the callback from a vector is unsafe.
    sub->idle.wait(lock, [&] { return sub->inflight == 0; });
    return true;
}

void Transport::enqueue_control_(const Frame& f) {
    if (!control_sub_count_.load(std::memory_order_acquire)) return;
    {
        std::lock_guard<std::mutex> lock(control_queue_mu_);
        if (control_failed_) {
            stat_control_queue_dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (control_queue_.size() >= cfg_.control_queue_frames) {
            // Do not silently evict an old fault and continue on a healthy
            // newest sample. Stop delivering control feedback; its existing
            // receive-age watchdog must then fail safe, even if user delivery
            // continues. Recreating the transport explicitly clears the latch.
            control_failed_ = true;
            stat_control_queue_overflows_.fetch_add(1, std::memory_order_relaxed);
            stat_control_queue_dropped_.fetch_add(control_queue_.size() + 1,
                                                  std::memory_order_relaxed);
            control_queue_.clear();
            return;
        }
        control_queue_.push_back(f);
        const auto depth = control_queue_.size();
        auto high = stat_control_queue_high_water_.load(std::memory_order_relaxed);
        if (depth > high) stat_control_queue_high_water_.store(depth, std::memory_order_relaxed);
    }
    control_queue_cv_.notify_one();
}

void Transport::control_loop_() {
    for (;;) {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(control_queue_mu_);
            control_queue_cv_.wait(lock, [&] {
                return stop_requested_.load(std::memory_order_acquire) || !control_queue_.empty();
            });
            if (stop_requested_.load(std::memory_order_acquire)) return;
            frame = std::move(control_queue_.front());
            control_queue_.pop_front();
        }
        std::vector<std::shared_ptr<ControlSub>> hits;
        { std::lock_guard<std::mutex> lock(control_sub_mu_); hits = control_subs_; }
        for (const auto& sub : hits) {
            {
                std::lock_guard<std::mutex> lock(sub->mutex);
                if (!sub->active || frame.receive_time < sub->since) continue;
                ++sub->inflight;
            }
            try { sub->cb(frame); }
            catch (...) { stat_callback_exceptions_.fetch_add(1, std::memory_order_relaxed); }
            {
                std::lock_guard<std::mutex> lock(sub->mutex);
                --sub->inflight;
            }
            sub->idle.notify_all();
        }
    }
}

void Transport::fail_pending_(const std::string& reason) noexcept {
    std::lock_guard<std::mutex> lk(pending_mu_);
    for (auto& kv : pending_acks_) {
        try {
            kv.second->promise.set_exception(std::make_exception_ptr(
                IoError("transport: " + reason, EIO)));
        } catch (...) { /* already satisfied */ }
    }
    pending_acks_.clear();
}

}  // namespace xense::taccap::bus

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include "leader_exchange.hpp"

#include <taccap/error.hpp>

#include <cstring>
#include <optional>

namespace xense::taccap::detail {

namespace {

using clock = std::chrono::steady_clock;

// Back-to-back exchanges allowed per period while the firmware reports a
// backlog; a backlog larger than this is left for the next period.
constexpr unsigned kMaxContinuations = 8;

}  // namespace

LeaderExchange::LeaderExchange(bus::Transport& transport,
                               std::chrono::microseconds period,
                               std::chrono::milliseconds timeout)
    : t_(transport), period_(period), timeout_(timeout) {
    worker_ = std::thread([this] { run_(); });
}

LeaderExchange::~LeaderExchange() { stop(); }

void LeaderExchange::stop() noexcept {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) worker_.join();
}

LeaderExchangeStats LeaderExchange::stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}

bool LeaderExchange::decode(const std::vector<uint8_t>& d,
                            protocol::LeaderExchangeResponseHeader& h,
                            std::vector<bus::Frame>& frames) {
    frames.clear();
    if (d.size() < sizeof(h)) return false;
    std::memcpy(&h, d.data(), sizeof(h));
    if (h.version != protocol::LEADER_EXCHANGE_VERSION) return false;
    std::size_t pos = sizeof(h);
    for (unsigned i = 0; i < h.record_count; ++i) {
        if (d.size() - pos < 2) { frames.clear(); return false; }
        const std::size_t n = d[pos + 1];
        if (d.size() - pos - 2 < n) { frames.clear(); return false; }
        bus::Frame f{};
        f.addr = protocol::Address::MCU;
        f.type = protocol::FrameType::DATA;
        f.cmd = static_cast<protocol::Cmd>(d[pos]);
        f.payload.assign(d.begin() + static_cast<std::ptrdiff_t>(pos + 2),
                         d.begin() + static_cast<std::ptrdiff_t>(pos + 2 + n));
        pos += 2 + n;
        frames.push_back(std::move(f));
    }
    if (pos != d.size()) { frames.clear(); return false; }
    return true;
}

void LeaderExchange::run_() {
    protocol::LeaderExchangeRequest req{};
    req.version = protocol::LEADER_EXCHANGE_VERSION;
    std::vector<uint8_t> wire(sizeof(req));
    std::memcpy(wire.data(), &req, sizeof(req));

    auto next_tick = clock::now();
    unsigned continuations = 0;
    std::unique_lock<std::mutex> lk(mu_);
    while (!stop_) {
        cv_.wait_until(lk, next_tick, [&] { return stop_; });
        if (stop_) break;
        lk.unlock();

        const auto started = clock::now();
        std::optional<bus::AckResponse> ack;
        bool nack = false, link_down = false;
        try {
            // SameSeq: a retry after a lost reply makes the firmware replay its
            // cached reply instead of handing out (and losing) new records.
            ack = t_.send_cmd(protocol::Cmd::LeaderExchange, wire, timeout_, bus::RetryMode::SameSeq);
        } catch (const NackError&) {
            nack = true;
        } catch (const TimeoutError&) {
        } catch (const std::exception&) {
            link_down = !t_.is_running();
        }
        const auto rtt_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - started).count());

        protocol::LeaderExchangeResponseHeader h{};
        std::vector<bus::Frame> frames;
        const bool ok = ack && decode(ack->data, h, frames);
        LeaderExchangeStats add{};
        for (auto& f : frames) {
            f.seq = ack->seq;
            f.receive_time = ack->receive_time;
            switch (f.cmd) {
                case protocol::Cmd::GetImu:
                case protocol::Cmd::GetImuV2:          ++add.imu_samples; break;
                case protocol::Cmd::GetEncoder:        ++add.encoder_samples; break;
                case protocol::Cmd::KeyStatus:         ++add.key_events; break;
                case protocol::Cmd::SensorErrorReport: ++add.sensor_errors; break;
                default:                               ++add.other_records; break;
            }
            t_.inject_data(std::move(f));
        }

        lk.lock();
        ++stats_.exchanges;
        if (continuations) ++stats_.continuation_exchanges;
        if (ok) {
            stats_.records += h.record_count;
            stats_.records_dropped += h.dropped;
            stats_.imu_samples += add.imu_samples;
            stats_.encoder_samples += add.encoder_samples;
            stats_.key_events += add.key_events;
            stats_.sensor_errors += add.sensor_errors;
            stats_.other_records += add.other_records;
            if (!(h.flags & protocol::LeaderExchangeRespFlag::Polled)) ++stats_.not_polled;
            stats_.last_rtt_us = rtt_us;
            if (rtt_us > stats_.max_rtt_us) stats_.max_rtt_us = rtt_us;
        } else if (nack) {
            ++stats_.nacks;
        } else if (ack) {
            ++stats_.malformed;
        } else {
            ++stats_.timeouts;
        }
        if (link_down) break;
        if (ok && h.pending && continuations < kMaxContinuations) {
            ++continuations;            // backlog: fetch again now, keep the tick
            continue;
        }
        continuations = 0;
        next_tick += period_;
        const auto now = clock::now();
        if (next_tick <= now) next_tick = now + period_;   // never burst to catch up
    }
}

}  // namespace xense::taccap::detail

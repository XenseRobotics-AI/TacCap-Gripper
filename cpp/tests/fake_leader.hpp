// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// A minimal fake leader firmware over a pty for the leader exchange tests.
// It answers what LeaderGripper's constructor and start_streaming()/
// start_exchange() issue, produces 100 Hz IMU (legacy 28 B) + encoder samples
// while streaming, and either pushes them as DATA frames (StreamMode
// Separate) or holds them as [cmd][len][payload] records for Cmd::
// LeaderExchange (StreamMode::Polled), mirroring tc-gu-01 leader_uplink.c:
// 1 KB queue, oldest record dropped when full, 504 B of records per reply,
// and a byte-identical repeat of the last request replays the cached reply.

#pragma once

#include "pty_helper.hpp"

#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace taccap_test {

class FakeLeader {
public:
    explicit FakeLeader(Pty& pty) : pty_(pty) { thread_ = std::thread([this] { run_(); }); }
    ~FakeLeader() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

    // Firmware without 0x5E: InvalidCmd for the exchange, InvalidParam for mode 2.
    void set_exchange_supported(bool on) { exchange_supported_.store(on); }
    // Produce samples every this many ms while streaming (default 10).
    void set_sample_period_ms(int ms) { sample_ms_.store(ms); }
    // Hold each exchange reply this long and count host frames that arrive meanwhile.
    void set_exchange_reply_delay_ms(int ms) { delay_ms_.store(ms); }
    // Do not answer the next n exchange requests (records are still taken).
    void drop_exchange_replies(unsigned n) { drop_.store(n); }
    // Make the next exchange reply's record lengths inconsistent.
    void corrupt_next_reply() { corrupt_.store(true); }
    // A button event, held or pushed like any other DATA frame. Emitted from
    // the fake's own thread so the PTY never has two writers.
    void press_key(uint8_t state) { key_state_.store(static_cast<int>(state)); }

    bool polled() const { return polled_.load(); }
    bool streaming() const { return streaming_.load(); }
    uint8_t stream_mode() const { return mode_.load(); }
    unsigned exchanges() const { return exchanges_.load(); }
    unsigned replays() const { return replays_.load(); }
    unsigned overlaps() const { return overlaps_.load(); }
    // DATA frames written on the wire (only outside StreamMode::Polled).
    unsigned data_frames_pushed() const { return pushed_.load(); }
    uint16_t imu_seq_produced() const { return imu_seq_.load(); }

private:
    struct Record { uint8_t cmd; std::vector<uint8_t> payload; };

    void emit_(tp::Cmd cmd, const void* p, std::size_t n) {
        std::lock_guard<std::mutex> lk(mu_);
        std::vector<uint8_t> bytes(static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n);
        if (polled_.load()) {
            queue_bytes_ += 2 + n;
            queue_.push_back({static_cast<uint8_t>(cmd), std::move(bytes)});
            while (queue_bytes_ > 1024) {
                queue_bytes_ -= 2 + queue_.front().payload.size();
                queue_.pop_front();
                if (dropped_ != 0xFFFF) ++dropped_;
            }
        } else {
            pty_.send_data(0, cmd, bytes);
            ++pushed_;
        }
    }

    void produce_() {
        tp::ImuData imu{};
        imu.valid_flag = 0x0F;
        imu.accel_z = 1000;
        imu.seq = imu_seq_.load();
        imu.timestamp_us = 10000u * imu.seq + 1u;
        emit_(tp::Cmd::GetImu, &imu, sizeof(imu));
        tp::EncoderData enc{};
        enc.seq = imu.seq;
        enc.timestamp_us = imu.timestamp_us;
        enc.position_rad = 0.01f * static_cast<float>(imu.seq % 100);
        emit_(tp::Cmd::GetEncoder, &enc, sizeof(enc));
        imu_seq_.fetch_add(1);
    }

    std::vector<uint8_t> take_reply_() {
        std::lock_guard<std::mutex> lk(mu_);
        tp::LeaderExchangeResponseHeader h{};
        h.version = tp::LEADER_EXCHANGE_VERSION;
        h.flags = polled_.load() ? tp::LeaderExchangeRespFlag::Polled : 0;
        std::vector<uint8_t> out(sizeof(h));
        while (!queue_.empty() && out.size() + 2 + queue_.front().payload.size() <= 512) {
            auto& r = queue_.front();
            out.push_back(r.cmd);
            out.push_back(static_cast<uint8_t>(r.payload.size()));
            out.insert(out.end(), r.payload.begin(), r.payload.end());
            queue_bytes_ -= 2 + r.payload.size();
            queue_.pop_front();
            ++h.record_count;
        }
        h.dropped = dropped_;
        dropped_ = 0;
        h.pending = static_cast<uint16_t>(queue_.size());
        std::memcpy(out.data(), &h, sizeof(h));
        return out;
    }

    void handle_(const tb::Frame& f) {
        if (pending_reply_) ++overlaps_;
        // Duplicate request (same seq/cmd/payload): replay the cached reply.
        if (last_ && last_->seq == f.seq && last_->cmd == f.cmd && last_->payload == f.payload) {
            ++replays_;
            if (f.cmd == tp::Cmd::LeaderExchange) pty_.send_response(f.seq, f.cmd, last_reply_);
            else pty_.send_ack_ok(f.seq, f.cmd);
            return;
        }
        last_ = f;
        switch (f.cmd) {
            case tp::Cmd::StopStream: {
                std::lock_guard<std::mutex> lk(mu_);
                streaming_.store(false); polled_.store(false);
                queue_.clear(); queue_bytes_ = 0; dropped_ = 0;
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            }
            case tp::Cmd::StartStream: {
                const auto sc = tp::decode_stream_config(f.payload.data(), f.payload.size());
                if (sc.mode == static_cast<uint8_t>(tp::StreamMode::Polled) && !exchange_supported_.load()) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidParam);
                    return;
                }
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    queue_.clear(); queue_bytes_ = 0; dropped_ = 0;
                    polled_.store(sc.mode == static_cast<uint8_t>(tp::StreamMode::Polled));
                }
                mode_.store(sc.mode);
                streaming_.store(true);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            }
            case tp::Cmd::LeaderExchange: {
                if (!exchange_supported_.load()) { pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd); return; }
                if (f.payload.size() != sizeof(tp::LeaderExchangeRequest) ||
                    f.payload[0] != tp::LEADER_EXCHANGE_VERSION || f.payload[1] || f.payload[2] || f.payload[3]) {
                    pty_.send_nack(f.seq, tp::ErrorCode::InvalidParam);
                    return;
                }
                ++exchanges_;
                last_reply_ = take_reply_();
                if (corrupt_.exchange(false) && last_reply_.size() > 9) last_reply_[9] = 0xFF;
                if (drop_.load()) { drop_.fetch_sub(1); return; }
                if (const int ms = delay_ms_.load()) {
                    pending_reply_ = true;
                    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
                    while (std::chrono::steady_clock::now() < until) {
                        if (auto g = pty_.expect_frame(1)) { ++overlaps_; deferred_.push_back(*g); }
                    }
                    pending_reply_ = false;
                }
                pty_.send_response(f.seq, f.cmd, last_reply_);
                return;
            }
            case tp::Cmd::GetVersion: {
                tp::FirmwareVersion v{1, 2, 6, 0};
                std::vector<uint8_t> d(sizeof(v));
                std::memcpy(d.data(), &v, sizeof(v));
                pty_.send_response(f.seq, f.cmd, d);
                return;
            }
            case tp::Cmd::GetSn: {
                const char sn[] = "TCGU01FAKELEADm";
                pty_.send_response(f.seq, f.cmd, std::vector<uint8_t>(sn, sn + sizeof(sn)));
                return;
            }
            case tp::Cmd::Heartbeat:
                pty_.send_response(f.seq, f.cmd, {1, 0, 0, 0, 1, 2, 6, 0});
                return;
            default:
                // TimeSession and anything else: legacy firmware.
                pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                return;
        }
    }

    void run_() {
        auto next = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            if (!deferred_.empty()) {
                auto f = deferred_.front();
                deferred_.erase(deferred_.begin());
                handle_(f);
            } else if (auto f = pty_.expect_frame(1)) {
                handle_(*f);
            }
            const int key = key_state_.exchange(-1);
            if (key >= 0) {
                tp::KeyStatusPayload k{0, static_cast<uint8_t>(key)};
                emit_(tp::Cmd::KeyStatus, &k, sizeof(k));
            }
            const auto now = std::chrono::steady_clock::now();
            if (streaming_.load() && now >= next) {
                next = now + std::chrono::milliseconds(sample_ms_.load());
                produce_();
            }
        }
    }

    Pty&                    pty_;
    std::thread             thread_;
    std::atomic<bool>       stop_{false};
    std::atomic<bool>       exchange_supported_{true};
    std::atomic<int>        sample_ms_{10};
    std::atomic<int>        delay_ms_{0};
    std::atomic<unsigned>   drop_{0};
    std::atomic<bool>       corrupt_{false};
    std::atomic<bool>       streaming_{false}, polled_{false};
    std::atomic<uint8_t>    mode_{0};
    std::atomic<unsigned>   exchanges_{0}, replays_{0}, overlaps_{0}, pushed_{0};
    std::atomic<int>        key_state_{-1};
    std::atomic<uint16_t>   imu_seq_{0};
    bool                    pending_reply_ = false;
    std::vector<tb::Frame>  deferred_;
    std::optional<tb::Frame> last_;
    std::vector<uint8_t>    last_reply_;
    std::mutex              mu_;
    std::deque<Record>      queue_;
    std::size_t             queue_bytes_ = 0;
    uint16_t                dropped_ = 0;
};

}  // namespace taccap_test

// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include "follower_exchange.hpp"

#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>

namespace xense::taccap::detail {

namespace {

using clock = std::chrono::steady_clock;

// A target submitted when the next telemetry exchange is at most this far away
// waits for it and rides on it, instead of costing a target-only exchange.
// At the high-rate period (2 ms) that is every target: a controller answering
// each status sample used to double the line to 1000 exchanges/s (9s,
// 2026-11-13), for at most ~2 ms of latency saved. At the 100 Hz period a target
// submitted right after a status reply is ~10 ms from the tick and still goes
// out at once, so low-rate latency is unchanged.
constexpr auto kMergeWindow = std::chrono::microseconds(3000);

// MCU-trajectory keep-alive. Below the firmware's 100 ms buffer-restart gap and
// its 300 ms host watchdog, and long against any stream worth interpolating:
// resending the last target in the middle of a stream would put a flat step
// into the firmware's reference, so it is only resent once the caller has been
// quiet this long (the target is being held, not streamed).
constexpr auto kMcuKeepAlive = std::chrono::milliseconds(80);

std::optional<protocol::ExchangeTarget> target_kind(protocol::Cmd cmd) {
    switch (cmd) {
        case protocol::Cmd::MotorPosCtrl:       return protocol::ExchangeTarget::Position;
        case protocol::Cmd::MotorVelCtrl:       return protocol::ExchangeTarget::Velocity;
        case protocol::Cmd::MotorTorqueCtrl:    return protocol::ExchangeTarget::Torque;
        case protocol::Cmd::MotorImpedanceCtrl: return protocol::ExchangeTarget::Impedance;
        default:                                return std::nullopt;
    }
}

std::size_t imu_size(uint8_t cmd) {
    if (cmd == static_cast<uint8_t>(protocol::Cmd::GetImu))   return sizeof(protocol::ImuData);
    if (cmd == static_cast<uint8_t>(protocol::Cmd::GetImuV2)) return sizeof(protocol::ImuData) + 16;
    return 0;
}

}  // namespace

FollowerExchange::FollowerExchange(bus::Transport& transport,
                                   std::chrono::microseconds period,
                                   std::chrono::milliseconds timeout, bool protected_supported)
    : t_(transport), period_(period), timeout_(timeout), protected_supported_(protected_supported) {
    worker_ = std::thread([this] { run_(); });
}

FollowerExchange::~FollowerExchange() { stop(); }

void FollowerExchange::stop() noexcept {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) worker_.join();
}

void FollowerExchange::submit(protocol::Cmd cmd, std::vector<uint8_t> payload, bool mcu_trajectory) {
    const auto kind = target_kind(cmd);
    if (!kind) throw IoError("FollowerExchange: not a motor target command", EINVAL);
    if (mcu_trajectory && *kind != protocol::ExchangeTarget::Position &&
        *kind != protocol::ExchangeTarget::Impedance)
        throw IoError("FollowerExchange: MCU trajectory takes position or impedance targets only", EINVAL);
    const uint8_t flags = mcu_trajectory ? protocol::ExchangeFlag::McuTrajectory : 0;
    queue_target_(Target{*kind, std::move(payload), flags});
}

bool FollowerExchange::submit_protected(const protocol::ProtectedImpedanceCtrl& target) {
    if (!protected_supported_) return false;
    return queue_target_(Target{protocol::ExchangeTarget::Impedance, protocol::encode(target),
                                protocol::ExchangeFlag::ProtectedImpedance});
}

bool FollowerExchange::queue_target_(Target target) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (stop_ || finishing_) return false;
        if (pending_) ++stats_.targets_superseded;
        if (target.flags != protocol::ExchangeFlag::McuTrajectory) keepalive_.reset();
        pending_ = std::move(target);
        ++stats_.targets_submitted;
    }
    cv_.notify_all();
    return true;
}

void FollowerExchange::finish_targets(std::optional<protocol::MotorImpedanceCtrl> zero) {
    std::unique_lock<std::mutex> lk(mu_);
    finishing_ = true;
    pending_.reset();
    keepalive_.reset();
    if (zero && !stop_)
        pending_ = Target{protocol::ExchangeTarget::Impedance, protocol::encode(*zero), 0};
    cv_.notify_all();
    // send_cmd itself has bounded attempts. During shutdown its timeout never
    // requeues a target. The final disable still runs if the zero ACK was lost.
    targets_done_.wait(lk, [this] { return !pending_ && !target_in_flight_; });
    finishing_ = false;
}

ExchangeStats FollowerExchange::stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}

void FollowerExchange::run_() {
    auto next_tick = clock::now();
    std::unique_lock<std::mutex> lk(mu_);
    while (!stop_) {
        // Wake for a target only when the tick is not close; otherwise sleep to
        // the tick and carry it there.
        cv_.wait_until(lk, next_tick, [&] {
            return stop_ || (pending_.has_value() && (finishing_ || next_tick - clock::now() > kMergeWindow));
        });
        if (stop_) break;
        const auto woke = clock::now();
        const bool telemetry = woke >= next_tick;
        std::optional<Target> target = std::move(pending_);
        pending_.reset();
        bool keepalive = false;
        if (target && target->flags == protocol::ExchangeFlag::McuTrajectory) {
            keepalive_ = *target;
            keepalive_sent_ = woke;
        } else if (!target && telemetry && keepalive_ && woke - keepalive_sent_ >= kMcuKeepAlive) {
            target = *keepalive_;
            keepalive_sent_ = woke;
            keepalive = true;
        }
        target_in_flight_ = target.has_value();
        lk.unlock();

        protocol::ExchangeRequestHeader head{};
        head.version     = protocol::EXCHANGE_VERSION;
        head.flags       = telemetry ? protocol::ExchangeFlag::Telemetry : 0;
        if (target) head.flags |= target->flags;
        head.target_kind = static_cast<uint8_t>(target ? target->kind : protocol::ExchangeTarget::None);
        std::vector<uint8_t> req(sizeof(head));
        std::memcpy(req.data(), &head, sizeof(head));
        if (target) req.insert(req.end(), target->payload.begin(), target->payload.end());

        const auto started = clock::now();
        std::optional<bus::AckResponse> ack;
        bool link_down = false;
        try {
            // SameSeq: when a reply is lost the retry is byte-identical, so the
            // firmware's command layer replays its cached reply (same telemetry,
            // the target not applied twice) instead of running a new exchange --
            // otherwise the IMU samples that reply carried would be lost.
            ack = t_.send_cmd(protocol::Cmd::SlaveExchange, req, timeout_, bus::RetryMode::SameSeq);
        } catch (const TimeoutError&) {
        } catch (const std::exception&) {
            link_down = !t_.is_running();
        }
        const auto rtt_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - started).count());

        if (ack) handle_reply_(*ack, target.has_value(), target ? target->flags : 0);

        lk.lock();
        ++stats_.exchanges;
        if (telemetry) ++stats_.telemetry_exchanges;
        if (target) ++stats_.target_exchanges;
        if (target && target->flags == protocol::ExchangeFlag::McuTrajectory)
            ++(keepalive ? stats_.mcu_keepalives : stats_.mcu_targets);
        if (ack) {
            stats_.last_rtt_us = rtt_us;
            if (rtt_us > stats_.max_rtt_us) stats_.max_rtt_us = rtt_us;
        } else {
            ++stats_.timeouts;
            if (target && target->flags == protocol::ExchangeFlag::ProtectedImpedance)
                ++stats_.protected_unacknowledged;
            // Only legacy targets survive the transport's final timeout.
            if (target && target->flags != protocol::ExchangeFlag::ProtectedImpedance &&
                !finishing_ && !pending_ && !keepalive) {
                pending_ = std::move(target);
                ++stats_.targets_resent;
            }
        }
        target_in_flight_ = false;
        if (finishing_) targets_done_.notify_all();
        if (link_down) break;
        if (telemetry) {
            next_tick += period_;
            const auto now = clock::now();
            if (next_tick <= now) next_tick = now + period_;   // never burst to catch up
        }
    }
    stop_ = true;
    pending_.reset();
    keepalive_.reset();
    targets_done_.notify_all();
}

void FollowerExchange::handle_reply_(const bus::AckResponse& ack, bool carried_target, uint8_t target_flags) {
    const auto& d = ack.data;
    protocol::ExchangeResponseHeader h{};
    bool ok = !ack.is_nack && d.size() >= sizeof(h);
    if (ok) {
        std::memcpy(&h, d.data(), sizeof(h));
        ok = h.version == protocol::EXCHANGE_VERSION && h.imu_count <= 3;
    }
    std::size_t pos = sizeof(h);
    const std::size_t motor = (ok && (h.flags & protocol::ExchangeRespFlag::Motor))
                                  ? protocol::MOTOR_STATUS_V2_SIZE : 0;
    const std::size_t imu = ok ? imu_size(h.imu_cmd) : 0;
    if (ok && h.imu_count && !imu) ok = false;
    if (ok && d.size() != pos + motor + std::size_t(h.imu_count) * imu) ok = false;

    std::vector<bus::Frame> frames;
    if (ok) {
        auto frame = [&](protocol::Cmd cmd, std::size_t n) {
            bus::Frame f{};
            f.addr = protocol::Address::MCU;
            f.seq = ack.seq;
            f.type = protocol::FrameType::DATA;
            f.cmd = cmd;
            f.payload.assign(d.begin() + static_cast<std::ptrdiff_t>(pos),
                             d.begin() + static_cast<std::ptrdiff_t>(pos + n));
            f.receive_time = ack.receive_time;
            pos += n;
            frames.push_back(std::move(f));
        };
        if (motor) frame(protocol::Cmd::GetMotorStatus, motor);
        for (unsigned i = 0; i < h.imu_count; ++i) frame(static_cast<protocol::Cmd>(h.imu_cmd), imu);
    }
    for (auto& f : frames) t_.inject_data(std::move(f));

    std::lock_guard<std::mutex> lk(mu_);
    if (target_flags == protocol::ExchangeFlag::ProtectedImpedance) {
        if (ack.is_nack) ++stats_.protected_rejected;
        else if (!ok || !(h.flags & protocol::ExchangeRespFlag::ProtectedImpedance) ||
                 (h.flags & protocol::ExchangeRespFlag::McuTrajectory)) ++stats_.protected_unacknowledged;
        else if (h.target_result != 0) ++stats_.protected_rejected;
        else ++stats_.protected_accepted;
    }
    if (ack.is_nack) { ++stats_.nacks; return; }
    if (!ok) { ++stats_.malformed; return; }
    if (motor) ++stats_.motor_samples;
    stats_.imu_samples += h.imu_count;
    stats_.imu_dropped += h.imu_dropped;
    if (target_flags == protocol::ExchangeFlag::McuTrajectory && !(h.flags & protocol::ExchangeRespFlag::McuTrajectory))
        ++stats_.mcu_unacknowledged;
    if (carried_target) {
        stats_.last_target_result = h.target_result;
        if (h.target_result != 0) ++stats_.targets_rejected;
    }
}

}  // namespace xense::taccap::detail

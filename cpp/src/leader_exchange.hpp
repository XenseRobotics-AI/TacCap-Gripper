// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Host side of the leader half-duplex exchange (Cmd::LeaderExchange).
//
// Why: same as FollowerExchange. With the leader streaming on its own clock, a
// host write that lands while the leader is transmitting loses the overlapping
// uplink bytes (tc-gu-01 docs/LEADER_EXCHANGE.md). In StreamMode::Polled the
// leader holds every DATA frame it would have pushed (IMU, encoder, key,
// sensor error); this loop is the only thing that asks for them, and the
// transport's half-duplex turn keeps user queries from overlapping a reply.
//
// Cadence: one exchange per period; when the reply says records are still
// pending, the next exchange goes out at once (bounded) instead of waiting.
// A timed-out exchange is retried with the same seq, so the firmware replays
// its cached reply and the records it already handed out are not lost.
//
// Records are re-dispatched as the DATA frames they are, so imu().on_data,
// encoder().on_data, key().on_event and sensor-error subscribers see exactly
// what streaming would have delivered.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/leader_gripper.hpp>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace xense::taccap::detail {

class LeaderExchange {
public:
    LeaderExchange(bus::Transport& transport, std::chrono::microseconds period,
                   std::chrono::milliseconds timeout);
    ~LeaderExchange();

    LeaderExchange(const LeaderExchange&)            = delete;
    LeaderExchange& operator=(const LeaderExchange&) = delete;

    // Idempotent; joins the worker.
    void stop() noexcept;

    LeaderExchangeStats stats() const;

    // Split one reply into its header and the DATA frames its records carry.
    // False (and no frames) when the reply is malformed. Public for tests.
    static bool decode(const std::vector<uint8_t>& reply,
                       protocol::LeaderExchangeResponseHeader& head,
                       std::vector<bus::Frame>& frames);

private:
    void run_();

    bus::Transport&                 t_;
    const std::chrono::microseconds period_;
    const std::chrono::milliseconds timeout_;

    mutable std::mutex       mu_;
    std::condition_variable  cv_;
    bool                     stop_ = false;
    LeaderExchangeStats      stats_{};
    std::thread              worker_;
};

}  // namespace xense::taccap::detail

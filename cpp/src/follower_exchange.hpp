// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Host side of the follower half-duplex exchange (Cmd::SlaveExchange).
//
// Why: with the firmware streaming on its own clock, a host write that lands
// while the follower is transmitting loses the overlapping uplink bytes
// (tc-gu-01 docs/TX_COLLISION_RETRY.md). In StreamMode::Polled the follower
// pushes nothing; this loop is the only thing that asks it to talk, and the
// transport's half-duplex turn keeps every other request out of the way while
// a reply may be on the wire. The line therefore carries one direction at a
// time by construction, not by luck.
//
// Cadence: one telemetry exchange per period. A target submitted between
// ticks goes out at once in a target-only exchange (header-only reply), so a
// controller that reacts to a status sample keeps today's ~1 ms latency
// instead of waiting a whole period. A request that times out keeps its
// target for the next exchange unless a newer one superseded it; targets are
// absolute set-points, so applying one twice is harmless.
//
// Replies are split back into the DATA frames a stream would have carried
// (GetMotorStatus, GetImu/GetImuV2) and injected into the transport's normal
// dispatch, so subscribers and the controllers' control lane see the same
// samples as in streaming mode.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/follower_gripper.hpp>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace xense::taccap::detail {

class FollowerExchange {
public:
    FollowerExchange(bus::Transport& transport, std::chrono::microseconds period,
                     std::chrono::milliseconds timeout);
    ~FollowerExchange();

    FollowerExchange(const FollowerExchange&)            = delete;
    FollowerExchange& operator=(const FollowerExchange&) = delete;

    // Latest target wins. Only the four Motor*Ctrl commands are accepted;
    // anything else throws IoError(EINVAL). After stop() it is ignored.
    void submit(protocol::Cmd cmd, std::vector<uint8_t> payload);

    // Idempotent; joins the worker.
    void stop() noexcept;

    ExchangeStats stats() const;

private:
    struct Target {
        protocol::ExchangeTarget kind;
        std::vector<uint8_t>     payload;
    };

    void run_();
    void handle_reply_(const bus::AckResponse& ack, bool carried_target);

    bus::Transport&                 t_;
    const std::chrono::microseconds period_;
    const std::chrono::milliseconds timeout_;

    mutable std::mutex       mu_;
    std::condition_variable  cv_;
    std::optional<Target>    pending_;
    bool                     stop_ = false;
    ExchangeStats            stats_{};
    std::thread              worker_;
};

}  // namespace xense::taccap::detail

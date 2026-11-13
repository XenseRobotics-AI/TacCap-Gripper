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
// Cadence: one telemetry exchange per period. A target submitted when the next
// tick is more than 3 ms away goes out at once in a target-only exchange
// (header-only reply), so a 100 Hz controller keeps ~1 ms latency instead of
// waiting a whole period; one submitted within 3 ms of the tick rides on that
// telemetry exchange, so a controller stepping on every high-rate status
// sample does not double the exchange count. A lost reply is retried with the same seq
// (the firmware replays its cached reply, so no IMU sample is lost); a request
// still unanswered after the retries normally keeps its target for the next
// exchange. Protected P+V commands use only the original absolute deadline
// and leave the queue after their transport attempts, including on timeout.
//
// Replies are split back into the DATA frames a stream would have carried
// (GetMotorStatus, GetImu/GetImuV2) and injected into the transport's normal
// dispatch, so subscribers and the controllers' control lane see the same
// samples as in streaming mode.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/detail/monotonic_cv.hpp>
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
                     std::chrono::milliseconds timeout, bool protected_supported = false);
    ~FollowerExchange();

    FollowerExchange(const FollowerExchange&)            = delete;
    FollowerExchange& operator=(const FollowerExchange&) = delete;

    // Latest target wins. Only the four Motor*Ctrl commands are accepted;
    // anything else throws IoError(EINVAL). After stop() it is ignored.
    // mcu_trajectory (V2.10): carry the target with ExchangeFlag::McuTrajectory.
    // Only Position/Impedance may carry it (EINVAL otherwise). The last such
    // target is resent every kMcuKeepAlive while no newer target arrives, so
    // the follower's 300 ms host watchdog sees a live host during a long hold;
    // any target without the bit ends that.
    void submit(protocol::Cmd cmd, std::vector<uint8_t> payload, bool mcu_trajectory = false);

    bool submit_protected(const protocol::ProtectedImpedanceCtrl& target);
    bool supports_protected_impedance() const noexcept { return protected_supported_; }

    // The motor serializes new submits with disable. Retire pending targets,
    // retries and keepalives before it sends the final disable on the same bus.
    void finish_targets(std::optional<protocol::MotorImpedanceCtrl> zero);

    // Idempotent; joins the worker.
    void stop() noexcept;

    ExchangeStats stats() const;

private:
    struct Target {
        protocol::ExchangeTarget kind;
        std::vector<uint8_t>     payload;
        uint8_t                  flags = 0;
    };

    void run_();
    void handle_reply_(const bus::AckResponse& ack, bool carried_target, uint8_t target_flags);
    bool queue_target_(Target target);

    bus::Transport&                 t_;
    const std::chrono::microseconds period_;
    const std::chrono::milliseconds timeout_;

    const bool                      protected_supported_;

    mutable std::mutex       mu_;
    detail::MonotonicCondition cv_;   // monotonic even on old-sysroot toolchains
    std::optional<Target>    pending_;
    std::optional<Target>    keepalive_;          // last MCU-trajectory target
    std::chrono::steady_clock::time_point keepalive_sent_{};
    std::condition_variable  targets_done_;
    bool                     finishing_ = false;
    bool                     target_in_flight_ = false;
    bool                     stop_ = false;
    ExchangeStats            stats_{};
    std::thread              worker_;
};

}  // namespace xense::taccap::detail

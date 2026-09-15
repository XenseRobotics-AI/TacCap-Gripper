// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
#pragma once

#include <taccap/follower_gripper.hpp>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace xense::taccap::detail {

// Shared transport/stream/thread lifecycle. Strategies retain their own state.
class ControlRuntime {
public:
    using Tick = std::function<void(bool)>; // true: sample/target event, false: timer
    explicit ControlRuntime(FollowerGripper& gripper) : g_(gripper) {}
    ~ControlRuntime() { try { stop(); } catch (...) {} }
    void start(unsigned stream_hz, unsigned free_hz,
               Motor::Callback sample, Tick tick);
    void stop();
    void wake();
private:
    struct CallbackGate { std::mutex mutex; bool active = true; };
    std::shared_ptr<CallbackGate> gate_;
    FollowerGripper& g_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::thread thread_;
    bool stopping_ = false;
    bool pending_ = false;
    bool claimed_ = false;
    bool stream_ours_ = false;
    Motor::SubId sub_ = 0;
    bool subscribed_ = false;
};
}

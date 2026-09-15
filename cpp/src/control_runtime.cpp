// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
#include <taccap/control_runtime.hpp>
#include <taccap/log.hpp>
#include <stdexcept>
#include <exception>

namespace xense::taccap::detail {
void ControlRuntime::start(unsigned stream_hz, unsigned free_hz,
                           Motor::Callback sample, Tick tick) {
    if (claimed_) throw std::logic_error("control runtime already started");
    g_.motor().claim_controller(this);
    claimed_ = true;
    try {
        stopping_ = false;
        pending_ = false;
        gate_ = std::make_shared<CallbackGate>();
        sub_ = g_.motor().on_status([this, sample, gate = gate_](const MotorStatusSample& s) {
            std::lock_guard<std::mutex> lock(gate->mutex);
            if (!gate->active) return;
            sample(s);
            wake();
        });
        subscribed_ = true;
        if (!g_.is_streaming()) {
            g_.start_streaming(stream_hz);
            stream_ours_ = true;
        }
        thread_ = std::thread([this, free_hz, tick] {
            g_.motor().controller_thread(this);
            using Clock = std::chrono::steady_clock;
            const auto period = free_hz ?
                std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / free_hz)) :
                std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(100));
            auto deadline = Clock::now() + period;
            for (;;) {
                bool event;
                {
                    std::unique_lock<std::mutex> lock(mu_);
                    cv_.wait_until(lock, deadline, [this, free_hz] {
                        return stopping_ || (!free_hz && pending_);
                    });
                    if (stopping_) break;
                    event = pending_ || free_hz;
                    pending_ = false;
                }
                try { tick(event); }
                catch (const std::exception& e) {
                    logger()->error("control runtime failed: {}", e.what());
                    try { g_.motor().disable(); } catch (...) {}
                    break;
                }
                deadline = Clock::now() + period;
            }
        });
    } catch (...) { stop(); throw; }
}
void ControlRuntime::wake() {
    { std::lock_guard<std::mutex> lock(mu_); pending_ = true; }
    cv_.notify_one();
}
void ControlRuntime::stop() {
    if (!claimed_) return;
    { std::lock_guard<std::mutex> lock(mu_); stopping_ = true; }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (gate_) {
        std::lock_guard<std::mutex> lock(gate_->mutex);
        gate_->active = false; // wait for callbacks already in flight before destroying strategies
    }
    if (subscribed_) { g_.motor().off(sub_); subscribed_ = false; }
    std::exception_ptr disable_error;
    try { g_.motor().disable(); } catch (...) { disable_error = std::current_exception(); }
    if (stream_ours_) {
        try { g_.stop_streaming(); } catch (...) {}
        stream_ours_ = false;
    }
    g_.motor().release_controller(this);
    claimed_ = false;
    if (disable_error) std::rethrow_exception(disable_error);
}
}

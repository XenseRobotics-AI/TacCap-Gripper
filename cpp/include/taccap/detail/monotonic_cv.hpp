// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Timed waits that really run on CLOCK_MONOTONIC.
//
// Why: libstdc++ only implements condition_variable::wait_until(steady_clock)
// and timed_mutex::try_lock_for on the monotonic clock when it was built
// against glibc >= 2.30 (pthread_cond_clockwait / pthread_mutex_clocklock).
// Toolchains built on an older sysroot -- conda-forge's, manylinux's -- fall
// back to pthread_cond_timedwait / pthread_mutex_timedlock on CLOCK_REALTIME:
// a steady deadline is converted to wall-clock time, so when NTP steps the
// wall clock back by X, the wait lasts X longer. That stalled a half-duplex
// exchange loop past the controllers' 200 ms status timeout (observed:
// "Clock change detected" 0.2 s before the fault).
//
// A pthread condition variable whose attribute selects CLOCK_MONOTONIC is
// available in every glibc, so these waits are immune to wall-clock steps
// whatever the toolchain. std::chrono::steady_clock is CLOCK_MONOTONIC on
// Linux (libstdc++ and libc++), so its time points convert directly.

#pragma once

#include <pthread.h>
#include <time.h>

#include <chrono>
#include <mutex>

namespace xense::taccap::detail {

class MonotonicCondition {
public:
    MonotonicCondition() {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&cond_, &attr);
        pthread_condattr_destroy(&attr);
    }
    ~MonotonicCondition() { pthread_cond_destroy(&cond_); }
    MonotonicCondition(const MonotonicCondition&)            = delete;
    MonotonicCondition& operator=(const MonotonicCondition&) = delete;

    void notify_one() noexcept { pthread_cond_signal(&cond_); }
    void notify_all() noexcept { pthread_cond_broadcast(&cond_); }

    // Same contract as std::condition_variable::wait_until with a predicate:
    // returns pred() (false only when the deadline passed with pred false).
    template <typename Pred>
    bool wait_until(std::unique_lock<std::mutex>& lk,
                    std::chrono::steady_clock::time_point deadline, Pred pred) {
        while (!pred()) {
            if (std::chrono::steady_clock::now() >= deadline) return pred();
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                deadline.time_since_epoch()).count();
            timespec ts{};
            ts.tv_sec  = static_cast<time_t>(ns / 1'000'000'000);
            ts.tv_nsec = static_cast<long>(ns % 1'000'000'000);
            pthread_cond_timedwait(&cond_, lk.mutex()->native_handle(), &ts);
        }
        return true;
    }

    template <typename Rep, typename Period, typename Pred>
    bool wait_for(std::unique_lock<std::mutex>& lk,
                  std::chrono::duration<Rep, Period> d, Pred pred) {
        return wait_until(lk, std::chrono::steady_clock::now() +
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(d),
                          std::move(pred));
    }

private:
    pthread_cond_t cond_;
};

// A lock that can be waited for with a monotonic timeout (replacement for
// std::timed_mutex, whose try_lock_for has the same wall-clock fallback).
class MonotonicTimedLock {
public:
    bool try_lock_for(std::chrono::steady_clock::duration wait) {
        std::unique_lock<std::mutex> lk(mu_);
        if (!cv_.wait_for(lk, wait, [this] { return !held_; })) return false;
        held_ = true;
        return true;
    }
    void unlock() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            held_ = false;
        }
        cv_.notify_one();
    }

private:
    std::mutex         mu_;
    MonotonicCondition cv_;
    bool               held_ = false;
};

}  // namespace xense::taccap::detail

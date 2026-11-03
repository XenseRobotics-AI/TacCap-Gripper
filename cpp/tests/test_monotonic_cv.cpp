// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// detail::MonotonicCondition / MonotonicTimedLock: timed waits on
// CLOCK_MONOTONIC (see monotonic_cv.hpp for why std:: ones are not enough on
// old-sysroot toolchains). Wall-clock steps cannot be produced without root,
// so this pins the contract; steady_clock == CLOCK_MONOTONIC is checked below.

#include <taccap/detail/monotonic_cv.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace td = xense::taccap::detail;
using clk = std::chrono::steady_clock;
using namespace std::chrono_literals;

TEST(MonotonicCv, TimesOutAfterTheDeadlineWithPredicateFalse) {
    std::mutex m; td::MonotonicCondition cv;
    std::unique_lock<std::mutex> lk(m);
    const auto t0 = clk::now();
    EXPECT_FALSE(cv.wait_for(lk, 30ms, [] { return false; }));
    const auto dt = clk::now() - t0;
    EXPECT_GE(dt, 30ms);
    EXPECT_LT(dt, 500ms);
    EXPECT_TRUE(lk.owns_lock());
}

TEST(MonotonicCv, NotifyWakesBeforeTheDeadline) {
    std::mutex m; td::MonotonicCondition cv; bool ready = false;
    std::thread th([&] {
        std::this_thread::sleep_for(20ms);
        { std::lock_guard<std::mutex> g(m); ready = true; }
        cv.notify_all();
    });
    std::unique_lock<std::mutex> lk(m);
    const auto t0 = clk::now();
    EXPECT_TRUE(cv.wait_until(lk, t0 + 5s, [&] { return ready; }));
    EXPECT_LT(clk::now() - t0, 2s);
    lk.unlock();
    th.join();
}

TEST(MonotonicCv, PredicateAlreadyTrueReturnsImmediately) {
    std::mutex m; td::MonotonicCondition cv;
    std::unique_lock<std::mutex> lk(m);
    EXPECT_TRUE(cv.wait_until(lk, clk::now() - 1s, [] { return true; }));
}

TEST(MonotonicCv, SteadyClockIsTheMonotonicClock) {
    // The helper hands steady_clock time points to a CLOCK_MONOTONIC cond.
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const auto mono = std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec);
    const auto steady = clk::now().time_since_epoch();
    EXPECT_LT(std::chrono::abs(steady - mono), 50ms);
}

TEST(MonotonicTimedLock, ExcludesTimesOutAndHandsOver) {
    td::MonotonicTimedLock lock;
    ASSERT_TRUE(lock.try_lock_for(0ms));
    const auto t0 = clk::now();
    EXPECT_FALSE(lock.try_lock_for(30ms)) << "second owner admitted";
    EXPECT_GE(clk::now() - t0, 30ms);
    std::atomic<bool> got{false};
    std::thread th([&] { got = lock.try_lock_for(2s); if (got) lock.unlock(); });
    std::this_thread::sleep_for(20ms);
    lock.unlock();
    th.join();
    EXPECT_TRUE(got.load());
    EXPECT_TRUE(lock.try_lock_for(0ms));
    lock.unlock();
}

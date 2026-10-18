// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Reading follower status: one-shot reads, streamed reads, and how to tell whether the status
// is fresh.
//
// The follower streams **only motor status** — the firmware compiles out the IMU / encoder /
// e-skin streams in this role, so start_streaming() takes a single motor_hz parameter.
//
// Do not use read_status() during control: it is an ACK round trip, and the request can land in
// the middle of the MCU's own transmission and corrupt the in-flight status frame. The
// controller owns the status stream itself; see follower_impedance.cpp.
//
// Freshness must be judged by the **firmware's** timestamp, not by "frames are still arriving".
// The firmware once stopped its own polling after requesting active reporting from a motor that
// does not support it, so the status froze permanently while idle, yet the version, the data
// stream and the counters all looked normal — the only symptom was that the values read were
// silently stale.

#include "example_target.hpp"

#include <taccap/follower_gripper.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    using namespace std::chrono_literals;

    auto g = example::open_follower(argc, argv);
    if (auto v = g->firmware_version()) {
        std::printf("[fw] %u.%u.%u.%u\n", v->major, v->minor, v->patch, v->build);
    }

    // ---- One-shot read ------------------------------------------------------
    const auto s = g->motor().read_status();
    std::printf("\n[one-shot] motor().read_status()\n");
    std::printf("  position %+.5f rad   (raw motor-shaft angle; zero set by power-on "
                "calibration)\n", s.actual_pos);
    std::printf("  velocity %+.5f rad/s\n", s.actual_vel);
    std::printf("  torque   %+.4f Nm    (feedback value, not the command)\n", s.actual_torque);
    std::printf("  temp     %.1f C\n", s.motor_temp_c);
    std::printf("  status=0x%04X stop_reason=%u\n", s.status,
                static_cast<unsigned>(s.stop_reason));

    const auto cfg = g->get_gripper_config();
    std::printf("\n[normalized] position() = %.4f   travel min=%.5f max=%.5f rad\n",
                g->position(), cfg.min_open_rad, cfg.max_open_rad);

    // ---- Faults ------------------------------------------------------------
    const auto fr = g->motor().fault_report();
    std::printf("\n[fault] motor 0x%08X  latched 0x%08X  firmware 0x%08X\n",
                fr.motor_fault_code, fr.motor_latched_fault_code,
                fr.firmware_fault_code);
    std::printf("  Latched bits only go away when explicitly cleared — 0 now does not mean "
                "there was never a fault.\n");

    // ---- Streamed read -----------------------------------------------------
    std::atomic<uint64_t> n{0};
    const auto sub = g->motor().on_status([&](const auto&) { ++n; });

    const uint32_t fw_before = fr.status_timestamp_ms;
    const auto t0 = std::chrono::steady_clock::now();
    g->start_streaming(/*motor_hz=*/100);
    std::this_thread::sleep_for(3s);
    g->stop_streaming();
    g->motor().off(sub);
    const double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf("\n[streamed] start_streaming(100) + on_status()\n");
    if (n.load() == 0) {
        std::printf("  **not a single frame received** — the stream did not start, or the "
                    "callback was registered after streaming started.\n");
        return 1;
    }
    std::printf("  %llu frames / %.2f s = %.1f Hz\n",
                static_cast<unsigned long long>(n.load()), dt, n.load() / dt);

    // Firmware-side tick of the last status frame. Use it instead of "how many frames arrived":
    // frames still being pushed does not mean the contents are updating — that is exactly the
    // hole described above.
    const uint32_t fw_after = g->motor().fault_report().status_timestamp_ms;
    const uint32_t advance = fw_after - fw_before;
    const double wall_ms = dt * 1000.0;
    std::printf("  firmware status timestamp advanced %u ms, wall clock %.0f ms  %s\n",
                advance, wall_ms,
                advance > wall_ms * 0.5
                    ? "OK"
                    : "**status is stale — the stream is pushing, the contents are not "
                      "updating**");
    return advance > wall_ms * 0.5 ? 0 : 1;
}

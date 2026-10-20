// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Impedance control, and **reading status while controlling**.
//
// The controller submits MIT frames in the background, phased to the status stream; the policy
// side touches only two non-blocking calls:
//
//     c.set_target(p);        // p in [0,1], 0=closed 1=open
//     auto s = c.snapshot();  // state + observation + protection flags, one-lock consistent view
//
// Status comes for free: the controller already owns the motor status stream, so **do not call
// read_status() during control** — that is an ACK round trip, and its request collides with the
// control frames on the serial link. observation even includes the motor temperature, precisely
// so a running loop never needs any extra command.
//
// snapshot() takes the whole view under one lock, so state, observation and the commanded torque
// describe the same instant; reading field by field stitches together a state that never
// existed.
//
// observation.age_ms is the correct basis for judging "the stream is still alive" — see
// follower_status.cpp.

#include "example_target.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/impedance_controller.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    using xense::taccap::ImpedanceConfig;
    using xense::taccap::ImpedanceController;
    using xense::taccap::ImpedanceState;

    auto g = example::open_follower(argc, argv);
    if (auto v = g->firmware_version()) {
        std::printf("[fw] %u.%u.%u.%u\n", v->major, v->minor, v->patch, v->build);
    }

    ImpedanceConfig cfg;
    cfg.kp = 20.0f;   // stiffness Nm/rad
    cfg.kd = 1.0f;    // damping Nm·s/rad, also sets the approach speed
    ImpedanceController c(*g, cfg);
    c.start();

    const float targets[] = {1.0f, 0.5f, 0.0f, 1.0f};
    int stale = 0;

    std::printf("\n  %5s %5s %8s %9s %8s %7s %s\n",
                "t", "tgt", "pos", "torque", "cmd", "age", "state");
    const auto t0 = std::chrono::steady_clock::now();
    for (float tgt : targets) {
        c.set_target(tgt);
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto s = c.snapshot();          // non-blocking, sends no command
            const auto& o = s.observation;
            if (s.state == ImpedanceState::Fault) {
                std::printf("  FAULT: %s\n", s.fault_reason.c_str());
                c.stop();
                return 1;
            }
            // A stale observation means the stream broke, not that the jaw is not moving — the
            // two are handled completely differently.
            if (o.valid && o.age_ms > 100.0f) ++stale;
            std::printf("  %5.1f %5.2f %8.4f %+9.4f %8.4f %6.0fms %s\n",
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t0).count(),
                        tgt, o.position, o.torque, s.commanded_torque_nm,
                        o.age_ms, to_string(s.state));
            std::this_thread::sleep_for(200ms);   // read-side rate is independent of control
        }
    }
    c.stop();

    std::printf("\nstale observations (age>100ms): %d\n", stale);
    if (stale) {
        std::printf("  **stream is intermittent** — the controller is still sending frames, but "
                    "the status it sees is stale.\n");
        return 1;
    }
    std::printf("  status fresh throughout: control and reads do not interfere; the read side "
                "needs no commands.\n");
    return 0;
}

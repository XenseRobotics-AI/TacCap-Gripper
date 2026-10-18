// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Force-position hybrid control.
//
// **There is a single control law throughout**, with no contact detection:
//
//     command(target, kp, kd, torque_budget = grasp_torque_nm)
//
// The PD request is error-clamped to the budget. Free travel only pays friction; when blocked,
// it clamps at exactly the budget and holds — contact is not detected, **saturation is the
// contact**. SDK 0.2.0 removed the host-side contact state machine: the MCU already runs the
// same stall criterion at 500Hz and is authoritative; running a second copy on the host means
// two copies of the same physical event drift apart, which is worse than running it on either
// side alone.
//
// The closed endpoint additionally carries a bounded feed-forward preload (close_preload_nm,
// default 0.25 Nm) that seats the jaw firmly against the mechanical stop. Without it,
// kp*(target-actual) goes exactly to zero at the target, the jaw stops applying force once it
// arrives, and the backlash stays there — measured closed-position holding torque was only
// 0.001 Nm. The preload is **reserved** out of the budget rather than added on top, so the
// invariant "total request never exceeds grasp_torque_nm" still holds.
//
// holding is an observation, not a state transition: the command has used up the budget and
// the jaw is still not moving. Reporting holding at the commanded position means the budget
// was used up by a move that should have arrived — that is a calibration or mapping problem.

#include "example_target.hpp"

#include <taccap/follower_gripper.hpp>
#include <taccap/force_position_controller.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    using xense::taccap::ForcePositionConfig;
    using xense::taccap::ForcePositionController;
    using xense::taccap::ForcePositionState;

    auto g = example::open_follower(argc, argv);
    if (auto v = g->firmware_version()) {
        std::printf("[fw] %u.%u.%u.%u\n", v->major, v->minor, v->patch, v->build);
    }

    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.1f;   // budget: unused in free travel, settles here when blocked
    std::printf("[cfg] grasp=%.2f Nm  close_preload=%.2f Nm\n",
                cfg.grasp_torque_nm, cfg.close_preload_nm);

    ForcePositionController c(*g, cfg);
    c.start();

    const float targets[] = {1.0f, 0.5f, 0.0f, 1.0f};
    int failures = 0;

    for (float tgt : targets) {
        c.set_target(tgt);
        // Wait for settle: wait for "arrived" or "blocked" instead of sleeping a fixed time —
        // a fixed wait both truncates slow moves and wastes time on fast ones.
        //
        // But you cannot just wait for arrived to become true: **set_target() does not clear it
        // immediately**; it only updates on the next control tick. arrived=true left over from
        // the previous move would make this loop exit before the jaw has even started moving,
        // so the measured position would be mid-travel. So first wait for it to go false (the
        // command has been accepted), then wait for it to go true.
        const auto t_accept = std::chrono::steady_clock::now() + 500ms;
        while (std::chrono::steady_clock::now() < t_accept && c.snapshot().arrived) {
            std::this_thread::sleep_for(5ms);
        }
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        xense::taccap::ForcePositionSnapshot s{};
        while (std::chrono::steady_clock::now() < deadline) {
            s = c.snapshot();
            if (s.state == ForcePositionState::Fault) break;
            if (s.arrived || s.holding) break;
            std::this_thread::sleep_for(10ms);
        }
        std::this_thread::sleep_for(300ms);       // let the hold settle before sampling
        s = c.snapshot();

        const float err = s.observation.position - tgt;
        std::printf("  target=%.2f -> %-17s pos=%.4f err=%+.4f cmd=%.3f "
                    "measured=%+.3f Nm\n",
                    tgt, to_string(s.state), s.observation.position, err,
                    s.commanded_torque_nm, s.observation.torque);

        if (s.state == ForcePositionState::Fault) {
            std::printf("    FAULT: %s\n", s.fault_reason.c_str());
            ++failures;
        } else if (s.holding && std::abs(err) < 0.02f) {
            // The invariant above: holding at the commanded position is a calibration/mapping
            // problem.
            std::printf("    **holding reported at the commanded position — check calibration "
                        "and the normalization mapping**\n");
            ++failures;
        } else if (!s.arrived && !s.holding) {
            std::printf("    **neither arrived nor blocked: timeout**\n");
            ++failures;
        }
    }
    c.stop();

    std::printf("\n%zu steps, %d failures\n", sizeof(targets) / sizeof(targets[0]), failures);
    return failures ? 1 : 0;
}

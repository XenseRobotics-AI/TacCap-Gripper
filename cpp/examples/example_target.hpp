// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Target selection shared by the examples: `left` / `right` / an explicit SN, the same
// convention as the Python examples.
//
// The reason it exists is concrete: the workstation permanently has four units plugged in (two
// leaders, two followers), and FollowerGripper::open() goes through find_one(), which throws
// IoError if there is more than one. An example without a selector cannot run even once on a
// real bench — and "which unit did I get" must not be guessed; flashing or calibrating the wrong
// one costs too much.

#pragma once

#include <taccap/discovery.hpp>
#include <taccap/follower_gripper.hpp>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace example {

inline void print_candidates(const std::vector<xense::taccap::discovery::GripperEndpoints>& all) {
    std::printf("Currently connected:\n");
    for (const auto& e : all) {
        const char* side =
            e.side == xense::taccap::discovery::Side::Left    ? "left "
            : e.side == xense::taccap::discovery::Side::Right ? "right"
                                                   : "?    ";
        const char* role =
            e.role == xense::taccap::discovery::Role::Follower ? "follower"
            : e.role == xense::taccap::discovery::Role::Leader ? "leader  "
                                                    : "?       ";
        std::printf("  [%s] %s  sn=%s  %s\n", side, role,
                    e.firmware_sn.c_str(), e.mcu_device.c_str());
    }
}

// argv[1] can be "left" / "right" / a full SN; if omitted, exactly one follower must be
// connected.
inline std::unique_ptr<xense::taccap::FollowerGripper>
open_follower(int argc, char** argv) {
    using namespace xense::taccap;
    using discovery::GripperEndpoints;
    using discovery::Role;
    using discovery::Side;

    const std::string want = argc > 1 ? argv[1] : "";
    auto all = discovery::scan_all();

    std::vector<GripperEndpoints> hits;
    for (const auto& e : all) {
        if (e.role != Role::Follower) continue;
        if (want.empty()) {
            hits.push_back(e);
        } else if (want == "left" && e.side == Side::Left) {
            hits.push_back(e);
        } else if (want == "right" && e.side == Side::Right) {
            hits.push_back(e);
        } else if (want == e.firmware_sn) {
            hits.push_back(e);
        }
    }

    if (hits.size() != 1) {
        std::printf("%s\n",
                    hits.empty() ? "No matching follower."
                                 : "More than one follower matched; specify one by SN.");
        print_candidates(all);
        std::printf("Usage: %s [left|right|SN]\n", argv[0]);
        throw std::runtime_error("target selection failed");
    }

    const auto& ep = hits.front();
    std::printf("[discovery] %s  %s\n", ep.firmware_sn.c_str(),
                ep.mcu_device.c_str());

    FollowerGripper::Config cfg;
    cfg.mcu_device = ep.mcu_device;
    return std::make_unique<FollowerGripper>(cfg);
}

}  // namespace example

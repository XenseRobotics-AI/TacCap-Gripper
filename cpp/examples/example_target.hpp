// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 示例共用的目标选择:`left` / `right` / 明确的 SN,和 Python 示例同一套约定。
//
// 存在的理由很具体:工位上常年插着四台(两主两从),而 FollowerGripper::open()
// 走的是 find_one(),多于一台就抛 IoError。没有选择器的示例在真实台架上一次
// 都跑不起来 —— 而且「拿到的是哪一台」不能靠猜,刷错、标定错的代价太高。

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
    std::printf("当前接着的:\n");
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

// argv[1] 可以是 "left" / "right" / 一个完整 SN;省略则要求恰好接着一台从爪。
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
                    hits.empty() ? "没有匹配的从爪。"
                                 : "匹配到不止一台从爪,请用 SN 指定。");
        print_candidates(all);
        std::printf("用法: %s [left|right|SN]\n", argv[0]);
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

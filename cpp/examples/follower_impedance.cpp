// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 阻抗控制,以及**边控制边读状态**。
//
// 控制器在后台按状态流的相位提交 MIT 帧,策略侧只碰两个非阻塞调用:
//
//     c.set_target(p);        // p in [0,1],0=闭合 1=张开
//     auto s = c.snapshot();  // 状态 + 观测 + 保护标志,一把锁一致视图
//
// 状态是白拿的:控制器已经持有电机状态流,所以**控制期间不要再调 read_status()**
// —— 那是一次 ACK 往返,它的请求会和控制帧在串口上撞车。observation 里连电机
// 温度都有,就是为了让运行中的回路不需要任何额外命令。
//
// snapshot() 在一把锁下取出整份视图,所以 state、observation、命令力矩描述的是
// 同一时刻;分字段去读会拼出一个从没存在过的状态。
//
// observation.age_ms 是判断「流还活着」的正确依据 —— 见 follower_status.cpp。

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
    cfg.kp = 20.0f;   // 刚度 Nm/rad
    cfg.kd = 1.0f;    // 阻尼 Nm·s/rad,同时决定接近速度
    ImpedanceController c(*g, cfg);
    c.start();

    const float targets[] = {1.0f, 0.5f, 0.0f, 1.0f};
    int stale = 0;

    std::printf("\n  %5s %5s %8s %9s %8s %7s %s\n",
                "t", "目标", "位置", "力矩", "命令", "年龄", "状态");
    const auto t0 = std::chrono::steady_clock::now();
    for (float tgt : targets) {
        c.set_target(tgt);
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto s = c.snapshot();          // 非阻塞,不发任何命令
            const auto& o = s.observation;
            if (s.state == ImpedanceState::Fault) {
                std::printf("  FAULT: %s\n", s.fault_reason.c_str());
                c.stop();
                return 1;
            }
            // 观测太旧说明流断了,而不是爪子不动 —— 两者的处理完全不同。
            if (o.valid && o.age_ms > 100.0f) ++stale;
            std::printf("  %5.1f %5.2f %8.4f %+9.4f %8.4f %6.0fms %s\n",
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t0).count(),
                        tgt, o.position, o.torque, s.commanded_torque_nm,
                        o.age_ms, to_string(s.state));
            std::this_thread::sleep_for(200ms);   // 读侧频率和控制无关
        }
    }
    c.stop();

    std::printf("\n观测过期(age>100ms)次数: %d\n", stale);
    if (stale) {
        std::printf("  **流有断续** —— 控制器还在发帧,但它看到的状态是旧的。\n");
        return 1;
    }
    std::printf("  状态全程新鲜:控制和读互不干扰,读侧不需要发任何命令。\n");
    return 0;
}

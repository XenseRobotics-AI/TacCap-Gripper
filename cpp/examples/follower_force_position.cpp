// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 力位混合控制。
//
// **全程只有一条控制律**,没有接触检测:
//
//     command(target, kp, kd, torque_budget = grasp_torque_nm)
//
// PD 请求对预算做误差钳位。自由行程只付摩擦,被挡住就钳在恰好的预算上并保持 ——
// 不检测接触,**饱和本身就是接触**。SDK 0.2.0 删掉了主机侧的接触状态机:MCU 已经
// 在 500Hz 跑同一个堵转判据并且是权威,主机再跑一份,同一个物理事件的两份副本
// 会漂开,比只在任一侧跑都糟。
//
// 闭合端点额外带一个有界前馈预压(close_preload_nm,默认 0.25 Nm),把爪子坐死在
// 机械止点上。没有它,kp*(target-actual) 在目标处恰好归零,爪子到了就不再施力,
// 齿隙留在那儿 —— 实测闭合位保持力矩只有 0.001 Nm。预压是从预算里**预留**而非
// 叠加,所以「总请求不超过 grasp_torque_nm」这条不变量仍然成立。
//
// holding 是观测量,不是状态跳转:命令用满了预算而爪子仍然没在走。在命令位置上
// 报 holding 意味着预算被一次本该到位的移动用满了 —— 那是标定或映射出了问题。

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
    cfg.grasp_torque_nm = 1.1f;   // 预算:自由行程用不到,被挡住时就停在这个值
    std::printf("[cfg] grasp=%.2f Nm  close_preload=%.2f Nm\n",
                cfg.grasp_torque_nm, cfg.close_preload_nm);

    ForcePositionController c(*g, cfg);
    c.start();

    const float targets[] = {1.0f, 0.5f, 0.0f, 1.0f};
    int failures = 0;

    for (float tgt : targets) {
        c.set_target(tgt);
        // 等停:等「到位」或「被挡住」,而不是固定睡一段 —— 固定等待既会截断慢
        // 的运动,又会在快的运动上白等。
        //
        // 但不能直接等 arrived 变真:**set_target() 不会立刻清掉它**,要到下一个
        // 控制拍才更新。上一次运动留下的 arrived=true 会让这个循环在爪子还没
        // 开始动的时候就退出,于是量到的是行进中的位置。所以先等它变假(命令
        // 被采纳),再等它变真。
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
        std::this_thread::sleep_for(300ms);       // 让保持稳下来再采样
        s = c.snapshot();

        const float err = s.observation.position - tgt;
        std::printf("  target=%.2f -> %-17s pos=%.4f err=%+.4f cmd=%.3f "
                    "实测=%+.3f Nm\n",
                    tgt, to_string(s.state), s.observation.position, err,
                    s.commanded_torque_nm, s.observation.torque);

        if (s.state == ForcePositionState::Fault) {
            std::printf("    FAULT: %s\n", s.fault_reason.c_str());
            ++failures;
        } else if (s.holding && std::abs(err) < 0.02f) {
            // 上面那条不变式:在命令位置上报 holding 是标定/映射的问题。
            std::printf("    **在命令位置上报 holding —— 检查标定与归一化映射**\n");
            ++failures;
        } else if (!s.arrived && !s.holding) {
            std::printf("    **既没到位也没被挡住:超时**\n");
            ++failures;
        }
    }
    c.stop();

    std::printf("\n%zu 步,%d 个失败\n", sizeof(targets) / sizeof(targets[0]), failures);
    return failures ? 1 : 0;
}

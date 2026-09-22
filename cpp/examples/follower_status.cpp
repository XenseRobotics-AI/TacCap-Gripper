// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 读从爪的状态:一次性读、开流读、以及怎么判断状态是不是新鲜的。
//
// 从爪**只流电机状态**这一种数据 —— 固件在这个角色上把 IMU / 编码器 / 电子皮肤
// 的流编译掉了,所以 start_streaming() 只有 motor_hz 一个参数。
//
// 控制期间不要用 read_status():它是一次 ACK 往返,请求可能落在 MCU 自己的发送
// 中间、毁掉在途的状态帧。控制器自己持有状态流,见 follower_impedance.cpp。
//
// 新鲜度必须看**固件的**时间戳,不能看「帧还在到」。固件曾经因为向不支持主动
// 上报的电机请求了上报就停掉自己的轮询,于是空闲时状态永久冻结,而版本号、
// 数据流、计数全都正常 —— 唯一的症状是读到的数悄悄是旧的。

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

    // ---- 一次性读 ----------------------------------------------------------
    const auto s = g->motor().read_status();
    std::printf("\n[一次性读] motor().read_status()\n");
    std::printf("  位置   %+.5f rad   (电机轴原始角,零点由上电标定确定)\n", s.actual_pos);
    std::printf("  速度   %+.5f rad/s\n", s.actual_vel);
    std::printf("  力矩   %+.4f Nm    (反馈值,不是命令值)\n", s.actual_torque);
    std::printf("  温度   %.1f C\n", s.motor_temp_c);
    std::printf("  status=0x%04X stop_reason=%u\n", s.status,
                static_cast<unsigned>(s.stop_reason));

    const auto cfg = g->get_gripper_config();
    std::printf("\n[归一化] position() = %.4f   行程 min=%.5f max=%.5f rad\n",
                g->position(), cfg.min_open_rad, cfg.max_open_rad);

    // ---- 故障 --------------------------------------------------------------
    const auto fr = g->motor().fault_report();
    std::printf("\n[故障] 电机 0x%08X  锁存 0x%08X  固件 0x%08X\n",
                fr.motor_fault_code, fr.motor_latched_fault_code,
                fr.firmware_fault_code);
    std::printf("  锁存位要显式清除才会消失 —— 当前为 0 不代表从没出过故障。\n");

    // ---- 开流读 ------------------------------------------------------------
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

    std::printf("\n[开流读] start_streaming(100) + on_status()\n");
    if (n.load() == 0) {
        std::printf("  **一帧都没收到** —— 流没起来,或者回调注册晚于开流。\n");
        return 1;
    }
    std::printf("  %llu 帧 / %.2f s = %.1f Hz\n",
                static_cast<unsigned long long>(n.load()), dt, n.load() / dt);

    // 固件侧最后一帧状态的 tick。用它而不是「收到了多少帧」:帧还在推不等于
    // 内容在更新,那正是上面说的那个洞。
    const uint32_t fw_after = g->motor().fault_report().status_timestamp_ms;
    const uint32_t advance = fw_after - fw_before;
    const double wall_ms = dt * 1000.0;
    std::printf("  固件状态时间戳推进 %u ms,墙钟 %.0f ms  %s\n",
                advance, wall_ms,
                advance > wall_ms * 0.5 ? "OK"
                                        : "**状态是旧的 —— 流在推,内容没更新**");
    return advance > wall_ms * 0.5 ? 0 : 1;
}

# 示例覆盖:主爪读流、从爪读状态、两个控制器。
# 刻意不提供任何电机原语(submit_position / submit_velocity / submit_torque /
# submit_impedance)的示例 —— 它们不经过主机侧的误差钳位、力矩天花板和堵转保护。
foreach(_ex leader_demo follower_status follower_impedance follower_force_position)
    add_executable(${_ex} ${_ex}.cpp)
    target_link_libraries(${_ex} PRIVATE taccap_core)
endforeach()

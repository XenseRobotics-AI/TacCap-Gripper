# Examples cover: leader streaming, follower status reads, and the two controllers.
# There are deliberately no examples for any motor primitive (submit_position / submit_velocity /
# submit_torque / submit_impedance) — they bypass the host-side error clamp, torque ceiling and
# stall protection.
foreach(_ex leader_demo follower_status follower_impedance follower_force_position)
    add_executable(${_ex} ${_ex}.cpp)
    target_link_libraries(${_ex} PRIVATE taccap_core)
endforeach()

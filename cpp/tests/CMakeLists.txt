find_package(GTest CONFIG REQUIRED)

add_executable(taccap_unit_tests
    test_crc.cpp
    test_frame.cpp
    test_codec.cpp
    test_codec_v16.cpp
    test_codec_v17.cpp
    test_codec_v21.cpp
    test_codec_v22.cpp
    test_codec_motor_model.cpp
    test_codec_motor_can_xfer.cpp
    test_fisheye_undistort.cpp
    test_calibration_pty.cpp
    test_ota_retry_pty.cpp
    test_python_compat.cpp
    test_stuff.cpp
    test_transport.cpp
    test_transport_concurrent_write.cpp
    test_diagnostics_codec.cpp
    test_components.cpp
    test_discovery.cpp
    test_gripper_position.cpp
    test_gripper_envelope.cpp
    test_envelope_pty.cpp
    test_force_position_controller.cpp
    test_force_position_pty.cpp
    test_impedance_controller.cpp
    test_impedance_pty.cpp
    test_upper_v16.cpp
    test_upper_v16_pty.cpp
    test_crc32_boundaries.cpp
    test_log.cpp
    test_stream_rate.cpp
)

# stream_rate.hpp is a private helper under src/ (not an installed header),
# but its firmware model is exactly the kind of thing that must be pinned by
# tests rather than re-derived by hand.
target_include_directories(taccap_unit_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

target_link_libraries(taccap_unit_tests
    PRIVATE
        taccap_core
        GTest::gtest
        GTest::gtest_main
        util            # openpty(), forkpty()
)

include(GoogleTest)
gtest_discover_tests(taccap_unit_tests)

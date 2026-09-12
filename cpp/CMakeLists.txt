# cpp/ — TacCap-Gripper core library + C++ tests/examples

add_library(taccap_core SHARED
    src/protocol/commands.cpp
    src/protocol/codec.cpp
    src/bus/frame.cpp
    src/bus/serial_bus.cpp
    src/bus/transport.cpp
    src/components/imu.cpp
    src/components/encoder.cpp
    src/components/camera.cpp
    src/components/fisheye_undistorter.cpp
    src/components/motor.cpp
    src/components/key.cpp
    src/components/led.cpp
    src/components/calibration.cpp
    src/components/sensor_errors.cpp
    src/components/diagnostics.cpp
    src/discovery.cpp
    src/leader_gripper.cpp
    src/follower_gripper.cpp
    src/control_loop.cpp
    src/force_position_controller.cpp
    src/impedance_controller.cpp
    src/ota.cpp
    src/log.cpp
)

# Generate the version header into the build tree.
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/taccap/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/taccap/version.hpp
    @ONLY
)

target_include_directories(taccap_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

# Camera component uses cv::VideoCapture (opencv_videoio) and cv::Mat
# (opencv_core). FisheyeUndistorter adds cv::fisheye::initUndistortRectifyMap
# (opencv_calib3d) and cv::remap (opencv_imgproc). spdlog backs the shared
# logger (taccap/log.hpp). All used to come transitively through libxensesdk;
# now that it is gone, request them directly. spdlog is consumed header-only
# (matching the prior transitive use — no libspdlog was ever linked).
find_package(OpenCV REQUIRED COMPONENTS core videoio calib3d imgproc)
find_package(spdlog REQUIRED)

target_link_libraries(taccap_core
    PUBLIC  opencv_core                 # cv::Mat in Camera component
    PUBLIC  opencv_videoio              # cv::VideoCapture in Camera component
    PRIVATE opencv_calib3d              # cv::fisheye::* in FisheyeUndistorter
    PRIVATE opencv_imgproc              # cv::remap in FisheyeUndistorter
    PUBLIC  spdlog::spdlog_header_only  # logging (taccap/log.hpp)
    PRIVATE Threads::Threads            # poll() + future async transport
)

set_target_properties(taccap_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    SOVERSION ${PROJECT_VERSION_MAJOR}
    VERSION   ${PROJECT_VERSION}
    OUTPUT_NAME taccap_core
    # In wheel/install: $ORIGIN = same dir as libtaccap_core.so so the Python
    # extension finds it without LD_LIBRARY_PATH.
    INSTALL_RPATH "$ORIGIN"
)

# C++ examples (smoke executables, not shipped in the wheel).
if(TACCAP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# C++ unit tests. enable_testing() is called at top level so ctest sees us.
if(TACCAP_BUILD_TESTS)
    add_subdirectory(tests)
endif()

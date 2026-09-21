cmake_minimum_required(VERSION 3.20)
project(taccap_gripper VERSION 0.2.0 LANGUAGES CXX)

# ==============================================================================
# Standards & global settings
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ==============================================================================
# Options
# ==============================================================================
option(TACCAP_BUILD_PYTHON   "Build pybind11 module (xense.taccap)" ON)
option(TACCAP_BUILD_EXAMPLES "Build C++ examples"                   OFF)
option(TACCAP_BUILD_TESTS    "Build C++ unit tests"                 OFF)

# Threads (used by SerialBus + future async Transport)
find_package(Threads REQUIRED)

if(TACCAP_BUILD_TESTS)
    enable_testing()
endif()

# ==============================================================================
# TacCap C++ core (declares taccap_core SHARED library inside cpp/)
# ==============================================================================
add_subdirectory(cpp)

# ==============================================================================
# Python bindings
# ==============================================================================
if(TACCAP_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# ==============================================================================
# Summary
# ==============================================================================
message(STATUS "")
message(STATUS "taccap-gripper configuration:")
message(STATUS "  Version:           ${PROJECT_VERSION}")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Python bindings:   ${TACCAP_BUILD_PYTHON}")
message(STATUS "  C++ examples:      ${TACCAP_BUILD_EXAMPLES}")
message(STATUS "  C++ tests:         ${TACCAP_BUILD_TESTS}")
message(STATUS "")

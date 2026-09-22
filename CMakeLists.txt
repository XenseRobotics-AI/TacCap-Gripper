cmake_minimum_required(VERSION 3.20)

# ==============================================================================
# Version — ONE hand-written source: [project].version in pyproject.toml
# ==============================================================================
# Everything else derives from it: PROJECT_VERSION -> TACCAP_VERSION_STRING
# (cpp/include/taccap/version.hpp.in) -> the extension's __version__ -> the
# package's __version__. The wheel metadata reads the same key directly.
#
# It used to be written by hand in three places -- here, pyproject.toml, and a
# checked-in python/xense/taccap/_version.py -- with nothing tying them
# together, and it had already drifted: the package reported 0.2.0 while the
# extension, hello() and the wheel metadata all said 0.2.1. That is the same
# shape as the firmware's two #ifdef-guarded version structs (one number, two
# copies, no link), and it gets the same treatment: one source, the rest derived.
# python/tests/test_version_single_source.py fails if they ever disagree again.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml"
     _taccap_version_lines REGEX "^version[ \t]*=")
list(LENGTH _taccap_version_lines _taccap_version_count)
if(NOT _taccap_version_count EQUAL 1)
    message(FATAL_ERROR
        "Expected exactly one line-anchored 'version =' in pyproject.toml, "
        "found ${_taccap_version_count}. The build reads the project version "
        "from there; fix pyproject.toml rather than hard-coding it here.")
endif()
string(REGEX REPLACE "^version[ \t]*=[ \t]*\"([^\"]+)\".*$" "\\1"
       TACCAP_VERSION "${_taccap_version_lines}")
unset(_taccap_version_lines)
unset(_taccap_version_count)

project(taccap_gripper VERSION ${TACCAP_VERSION} LANGUAGES CXX)

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

cmake_minimum_required(VERSION 3.20)
# Python, wheel metadata and native code share one full release version.
set(_taccap_version_file "${CMAKE_CURRENT_SOURCE_DIR}/python/xense/taccap/_version.py")
file(STRINGS "${_taccap_version_file}" _taccap_version_line REGEX "^__version__ = ")
string(REGEX MATCH "\"([0-9]+\\.[0-9]+\\.[0-9]+((a|b|rc)[0-9]+)?)\""
    _taccap_version_match "${_taccap_version_line}")
if(NOT _taccap_version_match)
    message(FATAL_ERROR "Invalid SDK version in ${_taccap_version_file}")
endif()
set(TACCAP_VERSION "${CMAKE_MATCH_1}")
# CMake library VERSION only accepts the numeric release component.
string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" TACCAP_VERSION_BASE "${TACCAP_VERSION}")
project(taccap_gripper VERSION ${TACCAP_VERSION_BASE} LANGUAGES CXX)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_taccap_version_file}")

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
message(STATUS "  Version:           ${TACCAP_VERSION}")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Python bindings:   ${TACCAP_BUILD_PYTHON}")
message(STATUS "  C++ examples:      ${TACCAP_BUILD_EXAMPLES}")
message(STATUS "  C++ tests:         ${TACCAP_BUILD_TESTS}")
message(STATUS "")

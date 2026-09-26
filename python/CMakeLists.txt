find_package(pybind11 CONFIG REQUIRED)
find_package(spdlog REQUIRED)

pybind11_add_module(_taccap_native MODULE
    bindings/module.cpp
    bindings/components.cpp
    bindings/bind_sensors.cpp
    bindings/bind_ota.cpp
    bindings/bind_gripper_types.cpp
    bindings/bind_calibration.cpp
    bindings/bind_motor.cpp
    bindings/bind_camera.cpp
    bindings/bind_gripper.cpp
    bindings/bind_control.cpp
    bindings/log.cpp
)

# pybind11 adds Python's include directory before transitive dependency
# includes. Some environments also place an older spdlog copy under
# include/pythonX.Y, which can be selected ahead of the spdlog package found by
# CMake and then fail against the paired fmt headers. Put a build-local shim in
# front so <spdlog/...> and <fmt/...> resolve to the packages found by CMake.
set(_taccap_dependency_include_dir
    ${CMAKE_CURRENT_BINARY_DIR}/dependency_include)
file(MAKE_DIRECTORY ${_taccap_dependency_include_dir})

function(_taccap_link_dependency_header_dir target_name header_dir_name)
    get_target_property(_taccap_include_dirs
        ${target_name} INTERFACE_INCLUDE_DIRECTORIES)
    foreach(_taccap_include_dir IN LISTS _taccap_include_dirs)
        if(EXISTS ${_taccap_include_dir}/${header_dir_name})
            set(_taccap_link_path
                ${_taccap_dependency_include_dir}/${header_dir_name})
            if(EXISTS ${_taccap_link_path} OR IS_SYMLINK ${_taccap_link_path})
                file(REMOVE ${_taccap_link_path})
            endif()
            file(CREATE_LINK
                ${_taccap_include_dir}/${header_dir_name}
                ${_taccap_link_path}
                SYMBOLIC COPY_ON_ERROR)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR
        "Could not find ${header_dir_name} include directory from ${target_name}")
endfunction()

_taccap_link_dependency_header_dir(spdlog::spdlog_header_only spdlog)
_taccap_link_dependency_header_dir(fmt::fmt fmt)
target_include_directories(_taccap_native BEFORE PRIVATE
    ${_taccap_dependency_include_dir})

target_link_libraries(_taccap_native PRIVATE taccap_core)

set_target_properties(_taccap_native PROPERTIES
    # Find libtaccap_core.so in dev (build dir) and in the installed wheel
    # ($ORIGIN = the directory the .so is loaded from).
    BUILD_RPATH "$<TARGET_FILE_DIR:taccap_core>"
    INSTALL_RPATH "$ORIGIN"
)

# In-tree development build ONLY: place the .so (and libtaccap_core.so) next to
# the Python sources, so `python -c 'import xense.taccap'` from python/ and the
# pytest conftest (which puts python/ first on sys.path) test what
# `cmake --build build` just produced.
#
# NOT under scikit-build (pip install, editable or not). Those builds ship the
# extension through the install() rules below; writing into the source tree as
# well let ANY install -- from any conda env, any build dir -- overwrite the
# checkout's .so with one linked against that other env's libraries. It
# happened: a second tool's `pip install` (build dir in /tmp) replaced the .so
# this checkout's tests load with one whose RPATH pointed into its own env.
if(NOT SKBUILD)
    set_target_properties(_taccap_native PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap)
    add_custom_command(TARGET _taccap_native POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:taccap_core>
                ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_FILE_NAME:taccap_core>
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                $<TARGET_FILE_NAME:taccap_core>
                ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_SONAME_FILE_NAME:taccap_core>
        VERBATIM)
endif()

# Wheel: ship _taccap_native + libtaccap_core.so together.
install(TARGETS _taccap_native taccap_core
    LIBRARY DESTINATION xense/taccap)

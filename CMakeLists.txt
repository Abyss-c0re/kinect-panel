cmake_minimum_required(VERSION 3.21)

# CMake ≥ 3.21 HIP language wants Clang, not the hipcc wrapper.
if(NOT DEFINED CMAKE_HIP_COMPILER)
  foreach(_hip_clang IN ITEMS
      /opt/rocm/lib/llvm/bin/clang++
      /opt/rocm/llvm/bin/clang++
      /opt/rocm/bin/amdclang++)
    if(EXISTS "${_hip_clang}")
      set(CMAKE_HIP_COMPILER "${_hip_clang}")
      break()
    endif()
  endforeach()
endif()
if(EXISTS /opt/rocm AND NOT DEFINED CMAKE_PREFIX_PATH)
  list(APPEND CMAKE_PREFIX_PATH /opt/rocm)
endif()

project(kinect-panel
  VERSION 0.3.0
  DESCRIPTION "Xbox 360 Kinect panel: RGB/IR webcam, tilt, depth cloud, ROCm frame-gen"
  LANGUAGES C
  HOMEPAGE_URL "https://github.com/Abyss-c0re/kinect-panel"
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

include(GNUInstallDirs)
include(CheckLanguage)

option(KINECT_ENABLE_HIP "Build ROCm/HIP frame-gen if a compiler is found" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)
find_package(Threads REQUIRED)

find_path(FREENECT_INCLUDE_DIR
  NAMES libfreenect.h
  PATHS /usr/include/libfreenect /usr/local/include/libfreenect
)
find_library(FREENECT_LIBRARY NAMES freenect)
find_library(FREENECT_SYNC_LIBRARY NAMES freenect_sync)
if(NOT FREENECT_INCLUDE_DIR OR NOT FREENECT_LIBRARY)
  message(FATAL_ERROR "libfreenect not found (headers + libfreenect)")
endif()

set(KINECT_HAVE_HIP OFF)
if(KINECT_ENABLE_HIP)
  check_language(HIP)
  if(CMAKE_HIP_COMPILER)
    enable_language(HIP)
    if(NOT CMAKE_HIP_ARCHITECTURES)
      set(CMAKE_HIP_ARCHITECTURES gfx1201)
    endif()
    set(KINECT_HAVE_HIP ON)
    message(STATUS "HIP: ${CMAKE_HIP_COMPILER} arch=${CMAKE_HIP_ARCHITECTURES}")
  else()
    message(STATUS "HIP compiler not found — frame-gen CPU lerp only")
  endif()
endif()

# --- libraries ---

add_library(kinect_core STATIC
  src/kinect_v4l.c
  src/kinect_motor.c
  src/kinect_bridge.c
  src/kinect_ir.c
  src/kinect_cloud_view.c
  src/cube_kinect_cloud.c
)
target_include_directories(kinect_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${FREENECT_INCLUDE_DIR}"
)
target_compile_definitions(kinect_core PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_link_libraries(kinect_core
  PUBLIC
    PkgConfig::GTK3
    Threads::Threads
    "${FREENECT_LIBRARY}"
    m
)

add_library(kinect_fg STATIC src/fg/fg.c)
target_include_directories(kinect_fg PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_definitions(kinect_fg PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_link_libraries(kinect_fg PUBLIC m)

if(KINECT_HAVE_HIP)
  add_library(kinect_fg_hip STATIC src/fg/fg_hip.hip)
  set_source_files_properties(src/fg/fg_hip.hip PROPERTIES LANGUAGE HIP)
  target_include_directories(kinect_fg_hip PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
  find_library(AMDHIP64_LIBRARY NAMES amdhip64 PATHS /opt/rocm/lib)
  if(AMDHIP64_LIBRARY)
    target_link_libraries(kinect_fg_hip PUBLIC "${AMDHIP64_LIBRARY}")
    set_target_properties(kinect_fg_hip PROPERTIES
      BUILD_RPATH /opt/rocm/lib
      INSTALL_RPATH /opt/rocm/lib
    )
  endif()
  target_link_libraries(kinect_fg PUBLIC kinect_fg_hip)
  target_compile_definitions(kinect_fg PUBLIC FG_HAVE_HIP)
  target_compile_definitions(kinect_core PUBLIC FG_HAVE_HIP)
endif()

target_link_libraries(kinect_core PUBLIC kinect_fg)

# --- executables ---

add_executable(kinect_panel src/kinect_panel.c)
target_link_libraries(kinect_panel PRIVATE kinect_core)
set_target_properties(kinect_panel PROPERTIES OUTPUT_NAME kinect_panel)

add_executable(kinect_depth_dump src/kinect_depth_dump.c)
target_include_directories(kinect_depth_dump PRIVATE "${FREENECT_INCLUDE_DIR}")
target_link_libraries(kinect_depth_dump PRIVATE "${FREENECT_LIBRARY}")
if(FREENECT_SYNC_LIBRARY)
  target_link_libraries(kinect_depth_dump PRIVATE "${FREENECT_SYNC_LIBRARY}")
endif()
target_link_libraries(kinect_depth_dump PRIVATE m)

add_executable(fg_probe src/fg/fg_probe.c)
target_link_libraries(fg_probe PRIVATE kinect_fg)

# --- install ---

install(TARGETS kinect_panel kinect_depth_dump fg_probe
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(PROGRAMS setup_loopback.sh DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/kinect-panel")

# Convenience copy next to the historic launcher
add_custom_command(TARGET kinect_panel POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_panel> "${CMAKE_SOURCE_DIR}/bin/kinect_panel"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_depth_dump> "${CMAKE_SOURCE_DIR}/bin/kinect_depth_dump"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:fg_probe> "${CMAKE_SOURCE_DIR}/bin/fg_probe"
  COMMENT "Copy binaries to source bin/ for the local launcher"
)

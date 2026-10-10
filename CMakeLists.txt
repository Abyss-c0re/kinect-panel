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
  VERSION 0.6.0
  DESCRIPTION "Kinect panel: Xbox 360 and Kinect 2 depth clouds, RGB/IR webcam, ROCm frame-gen, OpenXR"
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
option(KINECT_ENABLE_OPENXR "Build libkinect_xr + OpenXR sample if OpenXR is found" ON)
option(KINECT_ENABLE_V2 "Build Kinect 2 depth via libfreenect2" ON)
option(KINECT_ENABLE_GTK "Build the GTK panel and camera session" ON)

find_package(PkgConfig REQUIRED)
if(KINECT_ENABLE_GTK)
  pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)
else()
  message(STATUS "GTK off — kinect_share and libraries only")
endif()
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

# libfreenect2 is C++. Prefer the system prefix, then a staged DESTDIR.
set(KINECT_HAVE_V2 OFF)
set(KINECT2_LIBDIR "")
set(KINECT2_INCLUDE "")
if(KINECT_ENABLE_V2)
  foreach(_k2_root IN ITEMS /usr/local /tmp/kinect2-build/destdir/usr/local)
    if(EXISTS "${_k2_root}/lib/libfreenect2.so")
      set(KINECT2_LIBDIR "${_k2_root}/lib")
      set(KINECT2_INCLUDE "${_k2_root}/include")
      set(KINECT_HAVE_V2 ON)
      break()
    endif()
  endforeach()
endif()
if(KINECT_HAVE_V2)
  enable_language(CXX)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  message(STATUS "Kinect v2: ${KINECT2_LIBDIR}/libfreenect2.so")
else()
  message(STATUS "Kinect v2: libfreenect2 not found — stub (v1 only)")
endif()

function(kinect_v2_link tgt)
  if(NOT KINECT_HAVE_V2 OR NOT TARGET ${tgt})
    return()
  endif()
  set_target_properties(${tgt} PROPERTIES
    LINKER_LANGUAGE CXX
    BUILD_RPATH "${KINECT2_LIBDIR}"
    INSTALL_RPATH "${KINECT2_LIBDIR}"
  )
endfunction()

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

# OBJECT, not a second static lib: the wrapper calls cube_cloud / ksync, and
# fuse calls the wrapper. One archive avoids a static-lib cycle. This target
# does not get _POSIX_C_SOURCE (that macro breaks libstdc++).
if(KINECT_HAVE_V2)
  add_library(kinect2 OBJECT src/kinect2_device.cpp)
  target_include_directories(kinect2 PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${KINECT2_INCLUDE}"
  )
else()
  add_library(kinect2 OBJECT src/kinect2_stub.c)
  target_include_directories(kinect2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_compile_definitions(kinect2 PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
endif()

add_library(kinect_cloud STATIC
  src/cube_kinect_cloud.c
  src/cube_kinect_mesh.c
  src/kinect_sync.c
  src/kinect_fuse.c
  src/kinect_net.c
  $<TARGET_OBJECTS:kinect2>
)
target_include_directories(kinect_cloud PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${FREENECT_INCLUDE_DIR}"
)
target_compile_definitions(kinect_cloud PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_link_libraries(kinect_cloud PUBLIC m Threads::Threads "${FREENECT_LIBRARY}")
if(KINECT_HAVE_V2)
  target_link_libraries(kinect_cloud PUBLIC "${KINECT2_LIBDIR}/libfreenect2.so")
endif()

if(KINECT_ENABLE_GTK)
add_library(kinect_core STATIC
  src/kinect_v4l.c
  src/kinect_motor.c
  src/kinect_bridge.c
  src/kinect_ir.c
  src/kinect_cloud_view.c
  src/kinect_session.c
)
target_include_directories(kinect_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${FREENECT_INCLUDE_DIR}"
)
target_compile_definitions(kinect_core PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_link_libraries(kinect_core
  PUBLIC
    kinect_cloud
    PkgConfig::GTK3
    Threads::Threads
    "${FREENECT_LIBRARY}"
    m
)
endif()

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
  if(TARGET kinect_core)
    target_compile_definitions(kinect_core PUBLIC FG_HAVE_HIP)
  endif()
endif()

if(TARGET kinect_core)
  target_link_libraries(kinect_core PUBLIC kinect_fg)
endif()

# --- executables ---

if(KINECT_ENABLE_GTK)
  add_executable(kinect_panel src/kinect_panel.c)
  target_link_libraries(kinect_panel PRIVATE kinect_core)
  set_target_properties(kinect_panel PROPERTIES OUTPUT_NAME kinect_panel)
  kinect_v2_link(kinect_panel)
endif()

add_executable(kinect_depth_dump src/kinect_depth_dump.c)
target_include_directories(kinect_depth_dump PRIVATE "${FREENECT_INCLUDE_DIR}")
target_link_libraries(kinect_depth_dump PRIVATE "${FREENECT_LIBRARY}")
if(FREENECT_SYNC_LIBRARY)
  target_link_libraries(kinect_depth_dump PRIVATE "${FREENECT_SYNC_LIBRARY}")
endif()
target_link_libraries(kinect_depth_dump PRIVATE m)

add_executable(fg_probe src/fg/fg_probe.c)
target_link_libraries(fg_probe PRIVATE kinect_fg)

add_executable(kinect_sync_proof src/kinect_sync_proof.c)
target_link_libraries(kinect_sync_proof PRIVATE kinect_cloud)
kinect_v2_link(kinect_sync_proof)

add_executable(kinect_share src/kinect_share.c)
target_link_libraries(kinect_share PRIVATE kinect_cloud)
kinect_v2_link(kinect_share)

add_executable(kinect_net_proof src/kinect_net_proof.c)
target_link_libraries(kinect_net_proof PRIVATE kinect_cloud)
kinect_v2_link(kinect_net_proof)

# --- OpenXR library (no GTK) ---

add_library(kinect_xr STATIC src/xr/kinect_xr.c src/xr/kxr_menu.c)
target_include_directories(kinect_xr
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/xr"
    "${FREENECT_INCLUDE_DIR}"
)
target_compile_definitions(kinect_xr PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_link_libraries(kinect_xr PUBLIC kinect_cloud Threads::Threads "${FREENECT_LIBRARY}" m)

add_executable(kxr_proof src/kxr_proof.c)
target_link_libraries(kxr_proof PRIVATE kinect_xr)
kinect_v2_link(kxr_proof)

pkg_check_modules(GLFW3 IMPORTED_TARGET glfw3)
pkg_check_modules(GLEW IMPORTED_TARGET glew)
if(GLFW3_FOUND AND GLEW_FOUND)
  add_executable(kxr_local src/kxr_local.c src/xr/kinect_xr_gl.c)
  target_include_directories(kxr_local PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  target_compile_definitions(kxr_local PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
  target_link_libraries(kxr_local PRIVATE kinect_xr PkgConfig::GLFW3 PkgConfig::GLEW)
  kinect_v2_link(kxr_local)
  message(STATUS "Local viewer: kxr_local (no VR stream)")
endif()

if(KINECT_ENABLE_OPENXR)
  pkg_check_modules(OPENXR IMPORTED_TARGET openxr)
  if(OPENXR_FOUND AND GLFW3_FOUND AND GLEW_FOUND)
    add_executable(kxr_openxr src/kxr_openxr.c src/xr/kinect_xr_gl.c)
    target_include_directories(kxr_openxr PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(kxr_openxr PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
    target_link_libraries(kxr_openxr PRIVATE
      kinect_xr
      PkgConfig::OPENXR
      PkgConfig::GLFW3
      PkgConfig::GLEW
    )
    kinect_v2_link(kxr_openxr)
    message(STATUS "OpenXR sample: kxr_openxr")
  else()
    message(STATUS "OpenXR sample skipped (need openxr + glfw3 + glew)")
  endif()
endif()

# --- install ---

install(TARGETS kinect_depth_dump fg_probe kxr_proof kinect_sync_proof kinect_share kinect_net_proof
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
if(TARGET kinect_panel)
  install(TARGETS kinect_panel RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
install(TARGETS kinect_cloud kinect_xr
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
install(FILES src/xr/kinect_xr.h src/xr/kxr_menu.h src/cube_kinect_cloud.h src/cube_kinect_mesh.h
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/kinect_xr"
)
if(TARGET kxr_local)
  install(TARGETS kxr_local RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
if(TARGET kxr_openxr)
  install(TARGETS kxr_openxr RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
install(PROGRAMS setup_loopback.sh DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/kinect-panel")

# Convenience copy next to the historic launcher
if(TARGET kinect_panel)
  add_custom_command(TARGET kinect_panel POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
    COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_panel> "${CMAKE_SOURCE_DIR}/bin/kinect_panel"
    COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_depth_dump> "${CMAKE_SOURCE_DIR}/bin/kinect_depth_dump"
    COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:fg_probe> "${CMAKE_SOURCE_DIR}/bin/fg_probe"
    COMMENT "Copy binaries to source bin/ for the local launcher"
  )
endif()
add_custom_command(TARGET kinect_share POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_share> "${CMAKE_SOURCE_DIR}/bin/kinect_share"
  COMMENT "Copy kinect_share to source bin/"
)
add_custom_command(TARGET kxr_proof POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kxr_proof> "${CMAKE_SOURCE_DIR}/bin/kxr_proof"
  COMMENT "Copy kxr_proof to source bin/"
)
add_custom_command(TARGET kinect_sync_proof POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
  COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kinect_sync_proof> "${CMAKE_SOURCE_DIR}/bin/kinect_sync_proof"
  COMMENT "Copy kinect_sync_proof to source bin/"
)
if(TARGET kxr_local)
  add_custom_command(TARGET kxr_local POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
    COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kxr_local> "${CMAKE_SOURCE_DIR}/bin/kxr_local"
    COMMENT "Copy kxr_local to source bin/"
  )
endif()
if(TARGET kxr_openxr)
  add_custom_command(TARGET kxr_openxr POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/bin"
    COMMAND "${CMAKE_COMMAND}" -E copy $<TARGET_FILE:kxr_openxr> "${CMAKE_SOURCE_DIR}/bin/kxr_openxr"
    COMMENT "Copy kxr_openxr to source bin/"
  )
endif()

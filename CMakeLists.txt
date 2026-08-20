cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED ENV{LLVM_MINGW_ROOT} AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/.tools/llvm-mingw")
  set(ENV{LLVM_MINGW_ROOT} "${CMAKE_CURRENT_LIST_DIR}/.tools/llvm-mingw")
endif()

if(UNIX AND NOT WIN32 AND NOT CMAKE_TOOLCHAIN_FILE)
  set(_codespaces_toolchain "${CMAKE_CURRENT_LIST_DIR}/cmake/llvm-mingw-x86_64.cmake")
  if(EXISTS "${_codespaces_toolchain}" AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/.tools/llvm-mingw/bin/x86_64-w64-mingw32-clang++")
    set(CMAKE_TOOLCHAIN_FILE "${_codespaces_toolchain}" CACHE FILEPATH
        "Automatically selected LLVM-MinGW toolchain for Codespaces" FORCE)
  endif()
endif()

project(Pico4VRMotionPredictionTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED OPENVR_ROOT)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/openvr/headers/openvr.h")
    set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/openvr")
  else()
    set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/openvr")
  endif()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OPENVR_LIBRARY "${OPENVR_ROOT}/lib/win64/openvr_api.lib")
  set(OPENVR_DLL     "${OPENVR_ROOT}/bin/win64/openvr_api.dll")
endif()

# 1. The Benchmark Tool
add_executable(Pico4VRMotionTest main.cpp)
target_include_directories(Pico4VRMotionTest PRIVATE "${OPENVR_ROOT}/headers")
target_compile_definitions(Pico4VRMotionTest PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
if(NOT MSVC)
  target_compile_options(Pico4VRMotionTest PRIVATE -Wall -Wextra -Wpedantic)
  target_link_options(Pico4VRMotionTest PRIVATE -static)
endif()
target_link_libraries(Pico4VRMotionTest PRIVATE "${OPENVR_LIBRARY}" d3d12 dxgi d3dcompiler dxguid user32 gdi32 shell32 ole32 uuid)
add_custom_command(TARGET Pico4VRMotionTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENVR_DLL}" "$<TARGET_FILE_DIR:Pico4VRMotionTest>/openvr_api.dll" VERBATIM)

# 2. The Drop-In Proxy DLL (Statically Linked)
add_library(driver_pico SHARED proxy_main.cpp)
target_include_directories(driver_pico PRIVATE "${OPENVR_ROOT}/headers")
target_compile_definitions(driver_pico PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
if(NOT MSVC)
  target_compile_options(driver_pico PRIVATE -Wall -Wextra -Wpedantic)
  # CRITICAL: -static eliminates all MinGW DLL dependencies
  target_link_options(driver_pico PRIVATE -static)
endif()
set_target_properties(driver_pico PROPERTIES PREFIX "" OUTPUT_NAME "driver_pico")
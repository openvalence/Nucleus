cmake_minimum_required(VERSION 3.24)
project(valencesim CXX)

# Host build only -- never part of a PlatformIO firmware build. Windows-first
# (WinLibs MinGW-w64 GCC), the same toolchain as Valence Bench.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(NUCLEUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
# The sibling checkout valence.pin names. Read-only from here: its bench host
# WS port and IXWebSocket patch are COMPILED from their home, never copied.
set(VALENCE_ROOT "${NUCLEUS_ROOT}/../Valence" CACHE PATH "Sibling Valence checkout")
set(BENCH_SRC "${VALENCE_ROOT}/hub/bench/src")

include(FetchContent)
set(USE_TLS OFF CACHE BOOL "" FORCE)
set(USE_ZLIB OFF CACHE BOOL "" FORCE)
set(IXWEBSOCKET_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ixwebsocket
    GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
    GIT_TAG v11.4.6
    GIT_SHALLOW ON
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${VALENCE_ROOT}/hub/bench/cmake/PatchIXWebSocket.cmake)
FetchContent_MakeAvailable(ixwebsocket)

# The vendored Ruckig step solvers are real translation units; everything else
# the engine needs is header-only. Same list the P4 component compiles.
file(GLOB RUCKIG_SRC "${NUCLEUS_ROOT}/lib/ruckig/src/ruckig/*.cpp")

add_executable(valencesim
    src/main.cpp
    src/SimMotion.cpp
    src/SimMotorSwitch.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/motion/MotionArbiter.cpp"
    src/SimPattern.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/PatternEngine.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/AdvancedPattern.cpp"
    src/SimUiToken.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceDevice.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceDiscovery.cpp"
    "${BENCH_SRC}/net/WsServerPort.cpp"
    ${RUCKIG_SRC})

target_include_directories(valencesim PRIVATE
    src
    "${NUCLEUS_ROOT}/flagship_p4/src"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub"
    "${NUCLEUS_ROOT}/lib/valence/include"
    "${NUCLEUS_ROOT}/lib/kinetic/include"
    "${NUCLEUS_ROOT}/lib/geiger/include"
    "${NUCLEUS_ROOT}/lib/ruckig/include"
    "${NUCLEUS_ROOT}/lib/strokeengine_patterns/include"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/arduino_compat"
    "${BENCH_SRC}")

# Device GLOG sites are audible here; without it Geiger's host branch is mute.
target_compile_definitions(valencesim PRIVATE GEIGER_HOST_PLATFORM)

# The board's catalog capacities, from their one home, so the twin's catalog
# and scratch are the P4's.
include("${NUCLEUS_ROOT}/flagship_p4/valence_capacity.cmake")
target_compile_definitions(valencesim PRIVATE ${VALENCE_CAPACITY_DEFINITIONS})

target_compile_options(valencesim PRIVATE -O2 -Wall -Wextra)

# Standalone exe: no libstdc++/libgcc/winpthread DLL hunting.
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_options(valencesim PRIVATE -static)
endif()

target_link_libraries(valencesim PRIVATE ixwebsocket)
if(WIN32)
    target_link_libraries(valencesim PRIVATE ws2_32 dnsapi shell32 winmm)
endif()

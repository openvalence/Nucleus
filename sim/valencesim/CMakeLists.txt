cmake_minimum_required(VERSION 3.24)
project(valencesim CXX)

# Host build only -- never part of a PlatformIO firmware build. GCC or Clang:
# MinGW-w64 on Windows (the Valence Bench toolchain, never MSVC), Linux and
# macOS. Phosphor bundles the result as its simulator sidecar.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(NUCLEUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
# The sibling checkout valence.pin names. Read-only from here: its bench host
# WS port and IXWebSocket patch are COMPILED from their home, never copied.
set(VALENCE_ROOT "${NUCLEUS_ROOT}/../Valence" CACHE PATH "Sibling Valence checkout")
set(BENCH_SRC "${VALENCE_ROOT}/hub/bench/src")
# The sibling checkout kinetic.pin names. Read-only from here, like Valence.
set(KINETIC_ROOT "${NUCLEUS_ROOT}/../Kinetic" CACHE PATH "Sibling Kinetic checkout")

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

# Kinetic's Ruckig step solvers are real translation units; everything else
# the engine needs is header-only. Same list the P4 component compiles.
file(GLOB RUCKIG_SRC "${KINETIC_ROOT}/third_party/ruckig/src/ruckig/*.cpp")

# The board's kernel switch (flagship_p4/src/motion/ValenceMotion.h), so a Mac
# or Linux bench can A/B the two planners on one build host. A Kinetic² build
# compiles no Ruckig.
option(NUCLEUS_KINETIC2 "plan with Kinetic² (kinetic2::Engine<1>) instead of Kinetic" OFF)
if(NUCLEUS_KINETIC2)
    set(RUCKIG_SRC "")
endif()

add_executable(valencesim
    src/main.cpp
    src/SimMotion.cpp
    src/SimMotorSwitch.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/motion/MotionArbiter.cpp"
    src/SimPattern.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/PatternEngine.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/advanced/AdvancedPattern.cpp"
    src/SimUiToken.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceDevice.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceDiscovery.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceEstopDatagram.cpp"
    "${BENCH_SRC}/net/WsServerPort.cpp"
    ${RUCKIG_SRC})

target_include_directories(valencesim PRIVATE
    src
    "${NUCLEUS_ROOT}/flagship_p4/src"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub"
    "${NUCLEUS_ROOT}/lib/valence/include"
    "${KINETIC_ROOT}/include"
    "${NUCLEUS_ROOT}/lib/geiger/include"
    "${KINETIC_ROOT}/third_party/ruckig/include"
    "${NUCLEUS_ROOT}/lib/strokeengine_patterns/include"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/arduino_compat"
    "${BENCH_SRC}")

# Device GLOG sites are audible here; without it Geiger's host branch is mute.
target_compile_definitions(valencesim PRIVATE GEIGER_HOST_PLATFORM)
if(NUCLEUS_KINETIC2)
    target_compile_definitions(valencesim PRIVATE NUCLEUS_KINETIC2=1)
endif()

# The board's catalog capacities, from their one home, so the twin's catalog
# and scratch are the P4's.
include("${NUCLEUS_ROOT}/flagship_p4/valence_capacity.cmake")
target_compile_definitions(valencesim PRIVATE ${VALENCE_CAPACITY_DEFINITIONS})

target_compile_options(valencesim PRIVATE -O2 -Wall -Wextra)

# Standalone exe: no libstdc++/libgcc/winpthread DLL hunting.
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_options(valencesim PRIVATE -static)
elseif(UNIX AND NOT APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # A bundled sidecar meets distros older than the build host's libstdc++.
    target_link_options(valencesim PRIVATE -static-libstdc++ -static-libgcc)
endif()

# Firmware sources include lwIP's socket header on every non-Winsock host.
if(NOT WIN32)
    target_include_directories(valencesim PRIVATE posix)
endif()

target_link_libraries(valencesim PRIVATE ixwebsocket)
if(WIN32)
    target_link_libraries(valencesim PRIVATE ws2_32 dnsapi shell32 winmm)
endif()

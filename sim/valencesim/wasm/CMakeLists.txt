# Integral's in-process front: integral.wasm + integral.js (README.md, "The
# wasm front").
#
# Build (from the Nucleus root, with emsdk activated):
#   emcmake cmake -S sim/valencesim/wasm -B sim/valencesim/wasm/build -G Ninja
#   cmake --build sim/valencesim/wasm/build
# Output: sim/valencesim/wasm/build/integral.{js,wasm} (gitignored).
#
# The machine's sources are the native exe's, by path; nothing is copied. No
# pthreads and no asyncify: the host drives every call synchronously.

cmake_minimum_required(VERSION 3.24)
project(integral CXX)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "integral builds with em++ (emcmake). The native front is ../CMakeLists.txt.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(SIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
get_filename_component(NUCLEUS_ROOT "${SIM_ROOT}/../.." ABSOLUTE)
set(VALENCE_ROOT "${NUCLEUS_ROOT}/../Valence" CACHE PATH "Sibling Valence checkout")
set(KINETIC_ROOT "${NUCLEUS_ROOT}/../Kinetic" CACHE PATH "Sibling Kinetic checkout")

add_executable(integral
    integral.cpp
    "${SIM_ROOT}/src/SimCore.cpp"
    "${SIM_ROOT}/src/SimMotion.cpp"
    "${SIM_ROOT}/src/SimMotorSwitch.cpp"
    "${SIM_ROOT}/src/SimPattern.cpp"
    "${SIM_ROOT}/src/SimUiToken.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/motion/MotionArbiter.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/PatternEngine.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/advanced/AdvancedPattern.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceDevice.cpp"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub/ValenceEstopDatagram.cpp")

target_include_directories(integral PRIVATE
    "${SIM_ROOT}/src"
    "${SIM_ROOT}/posix"
    "${NUCLEUS_ROOT}/flagship_p4/src"
    "${NUCLEUS_ROOT}/flagship_p4/src/hub"
    "${NUCLEUS_ROOT}/lib/valence/include"
    "${KINETIC_ROOT}/include"
    "${NUCLEUS_ROOT}/lib/geiger/include"
    "${NUCLEUS_ROOT}/lib/strokeengine_patterns/include"
    "${NUCLEUS_ROOT}/flagship_p4/src/patterns/arduino_compat"
    # Header-only here: HostPlatform.h (HostRandom) and SessionLog.h.
    "${VALENCE_ROOT}/hub/bench/src")

target_compile_definitions(integral PRIVATE GEIGER_HOST_PLATFORM)
include("${NUCLEUS_ROOT}/flagship_p4/valence_capacity.cmake")
target_compile_definitions(integral PRIVATE ${VALENCE_CAPACITY_DEFINITIONS})
target_compile_options(integral PRIVATE -O2 -Wall -Wextra)

target_link_options(integral PRIVATE
    -O2
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createIntegral
    -sENVIRONMENT=web,worker,node
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=1MB
    -sFILESYSTEM=0
    -sEXPORTED_FUNCTIONS=_malloc,_free,_integral_create,_integral_tick,_integral_connect,_integral_disconnect,_integral_send,_integral_poll,_integral_http,_integral_state_get,_integral_destroy
    -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPU32,stringToNewUTF8,UTF8ToString)
set_target_properties(integral PROPERTIES SUFFIX ".js")

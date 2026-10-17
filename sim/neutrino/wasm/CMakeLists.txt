# Neutrino's in-process front: neutrino.wasm + neutrino.js (README.md, "The
# wasm front").
#
# Build (from the Nucleus root, with emsdk activated):
#   emcmake cmake -S sim/neutrino/wasm -B sim/neutrino/wasm/build -G Ninja
#   cmake --build sim/neutrino/wasm/build
# Output: sim/neutrino/wasm/build/neutrino.{js,wasm} (gitignored).
#
# The machine's sources are the native exe's, by path; nothing is copied. No
# pthreads and no asyncify: the host drives every call synchronously.

cmake_minimum_required(VERSION 3.24)
project(neutrino CXX)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "neutrino builds with em++ (emcmake). The native front is ../CMakeLists.txt.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(SIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
get_filename_component(NUCLEUS_ROOT "${SIM_ROOT}/../.." ABSOLUTE)
set(VALENCE_ROOT "${NUCLEUS_ROOT}/../Valence" CACHE PATH "Sibling Valence checkout")
set(KINETIC_ROOT "${NUCLEUS_ROOT}/../Kinetic" CACHE PATH "Sibling Kinetic checkout")

add_executable(neutrino
    neutrino.cpp
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

target_include_directories(neutrino PRIVATE
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

target_compile_definitions(neutrino PRIVATE GEIGER_HOST_PLATFORM)
include("${NUCLEUS_ROOT}/flagship_p4/valence_capacity.cmake")
target_compile_definitions(neutrino PRIVATE ${VALENCE_CAPACITY_DEFINITIONS})
target_compile_options(neutrino PRIVATE -O2 -Wall -Wextra)

target_link_options(neutrino PRIVATE
    -O2
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createNeutrino
    -sENVIRONMENT=web,worker,node
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=1MB
    -sFILESYSTEM=0
    -sEXPORTED_FUNCTIONS=_malloc,_free,_neutrino_create,_neutrino_tick,_neutrino_connect,_neutrino_disconnect,_neutrino_send,_neutrino_poll,_neutrino_http,_neutrino_state_get,_neutrino_destroy,_neutrino_pair_press,_neutrino_pair_double_press,_neutrino_datagram
    -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPU32,stringToNewUTF8,UTF8ToString)
set_target_properties(neutrino PROPERTIES SUFFIX ".js")

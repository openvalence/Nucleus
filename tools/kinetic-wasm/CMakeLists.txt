# kinetic.wasm -- the machine's own planner for an offline renderer (README.md).
#
# Build (from the Nucleus root, with emsdk activated):
#   emcmake cmake -S tools/kinetic-wasm -B tools/kinetic-wasm/build -G Ninja
#   cmake --build tools/kinetic-wasm/build
# Output: tools/kinetic-wasm/build/kinetic.wasm (gitignored, never committed).
#
# Every planning source is the one the P4 compiles, by path; nothing is copied.
# The determinism rule (README.md) is the compile flags below: no FP
# contraction, no fast-math. Never add either.

cmake_minimum_required(VERSION 3.24)

# Script mode: `cmake -DOUT=<file> -DROOT=<dir> -P CMakeLists.txt` writes the
# git identity header, rewriting it only when it changed. Runs on every build
# so a commit or an edit never leaves a stale sha baked into the binary.
if(CMAKE_SCRIPT_MODE_FILE)
    execute_process(COMMAND git -C "${ROOT}" describe --always --dirty=-dirty --abbrev=12
                    OUTPUT_VARIABLE sha OUTPUT_STRIP_TRAILING_WHITESPACE
                    RESULT_VARIABLE rc ERROR_QUIET)
    if(NOT rc EQUAL 0 OR sha STREQUAL "")
        set(sha "unknown")
    endif()
    set(text "#define NUCLEUS_GIT_SHA \"${sha}\"\n")
    set(old "")
    if(EXISTS "${OUT}")
        file(READ "${OUT}" old)
    endif()
    if(NOT old STREQUAL text)
        file(WRITE "${OUT}" "${text}")
    endif()
    return()
endif()

project(kinetic_wasm CXX)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "kinetic-wasm builds with em++ (emcmake). Its native twin is "
                        "the suite test/native/test_kinetic_wasm_trace.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(NUCLEUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(KINETIC_ROOT "${NUCLEUS_ROOT}/../Kinetic")
file(GLOB RUCKIG_SRC "${KINETIC_ROOT}/third_party/ruckig/src/ruckig/*.cpp")

set(SHA_HEADER "${CMAKE_CURRENT_BINARY_DIR}/gen/kinetic_git_sha.h")
add_custom_target(kinetic_git_sha
    COMMAND "${CMAKE_COMMAND}" -DOUT=${SHA_HEADER} -DROOT=${NUCLEUS_ROOT}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
    BYPRODUCTS "${SHA_HEADER}"
    VERBATIM)

add_executable(kinetic
    kinetic_wasm.cpp
    "${NUCLEUS_ROOT}/flagship_p4/src/motion/MotionArbiter.cpp"
    ${RUCKIG_SRC})
add_dependencies(kinetic kinetic_git_sha)
set_target_properties(kinetic PROPERTIES SUFFIX ".wasm")

target_include_directories(kinetic PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/gen"
    "${NUCLEUS_ROOT}/flagship_p4/src"
    "${NUCLEUS_ROOT}/lib/valence/include"
    "${KINETIC_ROOT}/include"
    "${NUCLEUS_ROOT}/lib/geiger/include"
    "${KINETIC_ROOT}/third_party/ruckig/include")

# Geiger's mute host branch: no GEIGER_HOST_PLATFORM, so every GLOG site
# compiles to nothing and the binary imports no I/O.
# MotionArbiter.cpp's log tag is unused under the mute branch; clang says so.
target_compile_options(kinetic PRIVATE -O2 -ffp-contract=off -fno-fast-math -Wall
    -Wno-unused-const-variable)

# Standalone wasm, no JS glue, no filesystem: a Worker instantiates it with
# WebAssembly.instantiate and calls the exports. The stack is sized for
# commit(), which nests KB-scale Ruckig temporaries (motion-control.md).
target_link_options(kinetic PRIVATE
    -O2
    --no-entry
    -sSTANDALONE_WASM=1
    -sFILESYSTEM=0
    -sALLOW_MEMORY_GROWTH=0
    -sINITIAL_MEMORY=4MB
    -sSTACK_SIZE=256KB
    -sMALLOC=emmalloc
    -sEXPORTED_FUNCTIONS=_malloc,_free)

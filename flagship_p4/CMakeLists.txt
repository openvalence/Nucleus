cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Catalog capacities for EVERY component, before project(): one Catalog32 type
# in the whole image (valence_capacity.cmake says why).
include(${CMAKE_CURRENT_LIST_DIR}/valence_capacity.cmake)
foreach(_def ${VALENCE_CAPACITY_DEFINITIONS})
    idf_build_set_property(COMPILE_DEFINITIONS "${_def}" APPEND)
endforeach()
# The bench profile (valence_config.h says what it changes). OFF for every
# release image; only the flagship_p4_bench environment turns it on. A build
# profile, never a runtime switch (bd val-091.58).
option(NUCLEUS_BENCH_NO_MOTOR "devkit bench: motor-power gate advisory, version suffixed -bench" OFF)
if(NUCLEUS_BENCH_NO_MOTOR)
    idf_build_set_property(COMPILE_DEFINITIONS "NUCLEUS_BENCH_NO_MOTOR=1" APPEND)
    message(WARNING "NUCLEUS_BENCH_NO_MOTOR: bench profile, the motor-power gate is advisory")
endif()
# PROJECT_VER (esp_app_desc.version) is the label git_label.py writes on every
# pio run. IDF's fallback, git describe at configure time, goes stale under
# pio (bd val-73g); it applies only to a build that bypasses pio.
if(EXISTS "${CMAKE_BINARY_DIR}/git_label.txt")
    file(READ "${CMAKE_BINARY_DIR}/git_label.txt" PROJECT_VER)
endif()
project(flagship_p4)

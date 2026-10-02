cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Catalog capacities for EVERY component, before project(): one Catalog32 type
# in the whole image (valence_capacity.cmake says why).
include(${CMAKE_CURRENT_LIST_DIR}/valence_capacity.cmake)
foreach(_def ${VALENCE_CAPACITY_DEFINITIONS})
    idf_build_set_property(COMPILE_DEFINITIONS "${_def}" APPEND)
endforeach()
project(flagship_p4)

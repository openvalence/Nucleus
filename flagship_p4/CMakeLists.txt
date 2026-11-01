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
option(NUCLEUS_BENCH_NO_MOTOR "devkit bench: motor-power gate advisory, version suffixed -bench, OTA bench token in place of the PAIR press" OFF)
if(NUCLEUS_BENCH_NO_MOTOR)
    idf_build_set_property(COMPILE_DEFINITIONS "NUCLEUS_BENCH_NO_MOTOR=1" APPEND)
    message(WARNING "NUCLEUS_BENCH_NO_MOTOR: bench profile, the motor-power gate is advisory")
endif()
# PROJECT_VER (esp_app_desc.version) is "<product>/<git label>": the board,
# then the label git_label.py writes on every pio run (IDF's configure-time git
# describe goes stale under pio, bd val-73g). The OTA board gate reads the
# product from an incoming image (system/OtaArm.h imageFit(), bd val-3yls), so
# it leads: IDF cuts the field to 31 characters from the end. The board's home
# is its Kconfig overlay in SDKCONFIG_DEFAULTS; NUCLEUS_IMAGE_BARNACLE lets
# ValenceOta.cpp refuse to build when the sdkconfig names another board.
set(NUCLEUS_PRODUCT "nucleus-p4")
foreach(_defaults ${SDKCONFIG_DEFAULTS})
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${_defaults}")
        file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/${_defaults}" _barnacle REGEX "^CONFIG_NUCLEUS_BOARD_BARNACLE=y")
        if(_barnacle)
            set(NUCLEUS_PRODUCT "nucleus-p4b")
            idf_build_set_property(COMPILE_DEFINITIONS "NUCLEUS_IMAGE_BARNACLE=1" APPEND)
        endif()
    endif()
endforeach()
set(_label "unknown")
if(EXISTS "${CMAKE_BINARY_DIR}/git_label.txt")
    file(READ "${CMAKE_BINARY_DIR}/git_label.txt" _label)
endif()
set(PROJECT_VER "${NUCLEUS_PRODUCT}/${_label}")
project(flagship_p4)

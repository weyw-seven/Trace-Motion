cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tm_chassis"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tm_sensors"
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trace-motion-line-infrared)

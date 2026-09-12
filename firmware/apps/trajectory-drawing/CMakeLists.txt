cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tm_chassis"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tm_trajectory"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tm_n3"
)

set(N3_ENABLE_HARDWARE 0 CACHE STRING "Enable hardware initialization (0 or 1)")
set(N3_ENABLE_MOTION 0 CACHE STRING "Enable trajectory motion commands (0 or 1)")
set(N3_ENABLE_CIRCLE 0 CACHE STRING "Enable guarded CIRCLE records (0 or 1)")
set(N3_ENABLE_ROTATE_REL 0 CACHE STRING "Enable ROTATE_REL acceptance profile (0 or 1)")
set(N3_ENABLE_PEN 0 CACHE STRING "Enable physical pen commands (0 or 1)")
set(N3_ENABLE_SIMULATION 0 CACHE STRING "Enable hardware-free simulation (0 or 1)")
set(N3_ENABLE_WIFI 0 CACHE STRING "Enable Wi-Fi SoftAP and N3 TCP (0 or 1)")
set(N3_WIFI_AP_SSID "TraceMotion" CACHE STRING "SoftAP name when Wi-Fi is enabled")
set(N3_WIFI_AP_PASSWORD "change-me-before-flashing" CACHE STRING "Set a private password before flashing")

set_property(CACHE N3_ENABLE_HARDWARE PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_MOTION PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_CIRCLE PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_ROTATE_REL PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_PEN PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_SIMULATION PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_WIFI PROPERTY STRINGS 0 1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trace-motion-trajectory-drawing)

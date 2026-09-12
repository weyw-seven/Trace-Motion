set(N3_ENABLE_HARDWARE 0 CACHE STRING
    "Enable hardware initialization in the N3 runtime (0 or 1)")
set(N3_ENABLE_MOTION 0 CACHE STRING
    "Enable trajectory motion commands in the N3 runtime (0 or 1)")
set(N3_ENABLE_CIRCLE 0 CACHE STRING
    "Enable guarded CIRCLE records in the N3 motion runtime (0 or 1)")
set(N3_ENABLE_ROTATE_REL 0 CACHE STRING
    "Enable the guarded ROTATE_REL hardware acceptance profile (0 or 1)")
set(N3_ENABLE_PEN 0 CACHE STRING
    "Enable physical pen commands in the N3 runtime (0 or 1)")
set(N3_ENABLE_SIMULATION 0 CACHE STRING
    "Enable hardware-free LINE/CIRCLE trajectory simulation (0 or 1)")
set(N3_ENABLE_WIFI 0 CACHE STRING
    "Enable protected Wi-Fi SoftAP and N3 TCP transport (0 or 1)")
set(N3_WIFI_AP_SSID "TraceMotion" CACHE STRING
    "SoftAP name used when N3_ENABLE_WIFI=1")
set(N3_WIFI_AP_PASSWORD "change-me-before-flashing" CACHE STRING
    "SoftAP password used when N3_ENABLE_WIFI=1; set a private value before flashing")

set_property(CACHE N3_ENABLE_HARDWARE PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_MOTION PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_CIRCLE PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_ROTATE_REL PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_PEN PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_SIMULATION PROPERTY STRINGS 0 1)
set_property(CACHE N3_ENABLE_WIFI PROPERTY STRINGS 0 1)

idf_component_register(SRCS "main.c"
                            "n3_service.c"
                            "n3_hardware.c"
                            "n3_runner_bridge.c"
                            "n3_motion.c"
                            "n3_transport.c"
                            "robot_stack_impl.c"
                            "trajectory_stack_impl.c"
                       INCLUDE_DIRS "."
                       REQUIRES spiffs json esp_driver_pcnt esp_driver_gpio led_strip
                               esp_driver_ledc esp_driver_i2c esp_driver_uart
                               esp_driver_usb_serial_jtag esp_timer esp_wifi esp_netif nvs_flash lwip)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    N3_ENABLE_HARDWARE=${N3_ENABLE_HARDWARE}
    N3_ENABLE_MOTION=${N3_ENABLE_MOTION}
    N3_ENABLE_CIRCLE=${N3_ENABLE_CIRCLE}
    N3_ENABLE_ROTATE_REL=${N3_ENABLE_ROTATE_REL}
    N3_ENABLE_PEN=${N3_ENABLE_PEN}
    N3_ENABLE_SIMULATION=${N3_ENABLE_SIMULATION}
    N3_ENABLE_WIFI=${N3_ENABLE_WIFI}
    N3_WIFI_AP_SSID=\"${N3_WIFI_AP_SSID}\"
    N3_WIFI_AP_PASSWORD=\"${N3_WIFI_AP_PASSWORD}\"
)

if(N3_ENABLE_ROTATE_REL)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        CHASSIS_MOTION_ROTATE_TOL_DEG=1.5
    )
endif()

if(N3_ENABLE_MOTION)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS=20000
    )
endif()

spiffs_create_partition_image(
    storage
    ../spiffs
    FLASH_IN_PROJECT
)

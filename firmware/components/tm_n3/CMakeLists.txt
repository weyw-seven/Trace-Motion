idf_component_register(
    SRCS "n3_service.c"
         "n3_hardware.c"
         "n3_runner_bridge.c"
         "n3_motion.c"
         "n3_transport.c"
         "trajectory_stack_impl.c"
    INCLUDE_DIRS "include"
    REQUIRES tm_chassis tm_trajectory spiffs json esp_driver_pcnt esp_driver_gpio led_strip
             esp_driver_ledc esp_driver_i2c esp_driver_uart esp_driver_usb_serial_jtag
             esp_timer esp_wifi esp_netif nvs_flash lwip
)

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
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CHASSIS_MOTION_ROTATE_TOL_DEG=1.5)
endif()

if(N3_ENABLE_MOTION)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TRAJECTORY_RUNNER_DEFAULT_RUN_TIMEOUT_MS=20000)
endif()

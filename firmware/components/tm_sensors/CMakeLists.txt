idf_component_register(
    SRCS "tm_sensors.c"
    INCLUDE_DIRS "include"
    REQUIRES tm_chassis esp_driver_gpio esp_driver_i2c esp_driver_rmt esp_timer
)

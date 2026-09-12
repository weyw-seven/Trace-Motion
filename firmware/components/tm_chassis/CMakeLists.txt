idf_component_register(
    SRCS "tm_chassis.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_pcnt esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_timer
)

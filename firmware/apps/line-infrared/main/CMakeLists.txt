idf_component_register(
    SRCS "main.c" "line_infrared_app.c"
    INCLUDE_DIRS "."
    REQUIRES tm_chassis tm_sensors esp_timer
)

idf_component_register(
    SRCS "tm_line_ir.c"
    INCLUDE_DIRS "include"
    REQUIRES tm_chassis tm_sensors esp_timer
)

idf_component_register(SRCS "main.c" REQUIRES tm_n3)

spiffs_create_partition_image(
    storage
    ../spiffs
    FLASH_IN_PROJECT
)

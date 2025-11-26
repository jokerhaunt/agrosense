idf_component_register(
    SRCS "mqtt_manager.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt json freertos
)

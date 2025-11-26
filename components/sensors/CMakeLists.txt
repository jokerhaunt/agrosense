idf_component_register(
    SRCS "sensors.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc_cal json
)

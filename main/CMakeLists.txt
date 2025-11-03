idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        sensors
        wifi_manager
        mqtt_manager
        storage
        power
        nvs_flash
        esp_https_ota
        esp_task_wdt
        freertos
)

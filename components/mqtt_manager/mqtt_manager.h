#pragma once
#include "freertos/queue.h"
#include "esp_err.h"
#include <stddef.h>

esp_err_t mqtt_manager_init(QueueHandle_t q);
esp_err_t mqtt_manager_start(void);
int       mqtt_manager_publish(const char *topic, const char *payload, size_t len);
void      mqtt_manager_stop(void);

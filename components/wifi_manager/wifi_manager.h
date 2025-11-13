#pragma once
/**
 * @file wifi_manager.h
 *
 * FIX 4: s_retry_count managed ONLY in retry task — event handler never
 *        touches it. Prevents double-increment from multiple disconnect events.
 * FIX 6: removed unused extern asm symbol for CONFIG_WIFI_SSID.
 */

#include "freertos/queue.h"
#include "esp_err.h"

esp_err_t wifi_manager_init(QueueHandle_t event_queue);
esp_err_t wifi_manager_connect(void);
void      wifi_manager_stop(void);
int       wifi_manager_get_rssi(void);

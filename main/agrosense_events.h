#pragma once
/**
 * @file agrosense_events.h
 * @brief System-wide FreeRTOS queue event definitions.
 *
 * FIX 2: EVENT_DATA_READY removed entirely. Sensor data is read
 * synchronously in app_main before any network activity. This avoids
 * the heap pointer leak that would occur if the event was queued but
 * the json pointer was never freed in unhandled branches.
 *
 * Event flow:
 *   wifi_manager  → EVENT_WIFI_CONNECTED / EVENT_WIFI_FAILED
 *   mqtt_manager  → EVENT_MQTT_READY / EVENT_MQTT_DISCONNECTED /
 *                   EVENT_MQTT_PUBLISH_OK / EVENT_MQTT_PUBLISH_FAIL /
 *                   EVENT_OTA_COMMAND / EVENT_REMOTE_CONFIG /
 *                   EVENT_OTA_RESULT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    EVENT_WIFI_CONNECTED     = 1,
    EVENT_WIFI_FAILED,
    EVENT_MQTT_READY,
    EVENT_MQTT_DISCONNECTED,
    EVENT_MQTT_PUBLISH_OK,
    EVENT_MQTT_PUBLISH_FAIL,
    EVENT_OTA_COMMAND,
    EVENT_OTA_RESULT,        /* FIX 1: OTA task reports success/failure */
    EVENT_REMOTE_CONFIG,
} agrosense_event_type_t;

typedef struct {
    agrosense_event_type_t type;
    union {
        struct { int msg_id; }   publish;
        struct { char url[256]; } ota_cmd;

        /* FIX 1: OTA result */
        struct { bool success; } ota_result;

        /* FIX 9: remote config — persisted to NVS by orchestrator */
        struct {
            uint32_t sleep_normal_min;
            uint32_t sleep_retry_min;
        } config;
    };
} agrosense_event_t;

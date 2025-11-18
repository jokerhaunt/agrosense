/**
 * @file main.c
 * @brief AgroSense v1.0 — main orchestrator.
 *
 * Fixes applied here:
 *   FIX 1:  OTA task posts EVENT_OTA_RESULT on success AND failure.
 *           wait_event handles it — no more portMAX_DELAY hang.
 *   FIX 2:  EVENT_DATA_READY removed from event system entirely.
 *           Sensor data read synchronously before any network code.
 *   FIX 5:  Hardware WDT enabled via esp_task_wdt; fed in wait_event loop.
 *   FIX 9:  Remote config persisted to NVS; loaded on every boot.
 *   FIX 12: vTaskDelay(portMAX_DELAY) replaced with proper event wait.
 */

#include "agrosense_events.h"
#include "sensors/sensors.h"
#include "wifi_manager/wifi_manager.h"
#include "mqtt_manager/mqtt_manager.h"
#include "storage/storage.h"
#include "power/power.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "main";

#define TOPIC_SENSORS    "agrosense/" CONFIG_DEVICE_ID "/sensors"
#define PUBACK_TIMEOUT_MS 8000
#define OTA_TIMEOUT_MS   120000   /* 2 min max for OTA download */
#define QUEUE_DEPTH      20
#define WDT_TIMEOUT_S    30

/* ── NVS keys for persistent config (FIX 9) ────────────────────────────── */
#define NVS_CFG_NS       "as_cfg"
#define NVS_KEY_SN       "sleep_n"
#define NVS_KEY_SR       "sleep_r"

/* ── Runtime config ─────────────────────────────────────────────────────── */
static uint64_t s_sleep_normal_us;
static uint64_t s_sleep_retry_us;

/* FIX 9: load config from NVS, fall back to Kconfig defaults */
static void cfg_load(void)
{
    nvs_handle_t h;
    uint32_t sn = CONFIG_SLEEP_NORMAL_MIN;
    uint32_t sr = CONFIG_SLEEP_RETRY_MIN;

    if (nvs_open(NVS_CFG_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_SN, &sn);
        nvs_get_u32(h, NVS_KEY_SR, &sr);
        nvs_close(h);
    }

    s_sleep_normal_us = (uint64_t)sn * 60 * 1000000;
    s_sleep_retry_us  = (uint64_t)sr * 60 * 1000000;
    ESP_LOGI(TAG, "Config: normal=%lum retry=%lum",
             (unsigned long)sn, (unsigned long)sr);
}

/* FIX 9: save config to NVS */
static void cfg_save(uint32_t sn_min, uint32_t sr_min)
{
    nvs_handle_t h;
    if (nvs_open(NVS_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    esp_err_t r = nvs_set_u32(h, NVS_KEY_SN, sn_min);
    if (r == ESP_OK) r = nvs_set_u32(h, NVS_KEY_SR, sr_min);
    if (r == ESP_OK) nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Config saved: normal=%lum retry=%lum",
             (unsigned long)sn_min, (unsigned long)sr_min);
}

/* ── OTA task — FIX 1: always posts EVENT_OTA_RESULT ───────────────────── */
typedef struct { char url[256]; QueueHandle_t q; } ota_arg_t;

static void ota_task(void *arg)
{
    ota_arg_t *a = (ota_arg_t *)arg;
    ESP_LOGI(TAG, "OTA from: %s", a->url);

    esp_http_client_config_t http = {
        .url = a->url,
        .skip_cert_common_name_check = true,
    };
    esp_https_ota_config_t cfg = { .http_config = &http };

    esp_err_t ret = esp_https_ota(&cfg);

    /* FIX 1: always report result — no silent hang */
    agrosense_event_t ev = {
        .type              = EVENT_OTA_RESULT,
        .ota_result.success = (ret == ESP_OK),
    };
    xQueueSend(a->q, &ev, pdMS_TO_TICKS(1000));

    free(a);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA success — rebooting");
        vTaskDelay(pdMS_TO_TICKS(500));
        power_reboot();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
    }
    vTaskDelete(NULL);
}

/* ── wait_event: deadline-bounded, feeds WDT, handles side events ────────── */
static bool wait_event(QueueHandle_t q, agrosense_event_type_t want,
                       agrosense_event_t *out, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        /* FIX 5: feed WDT in long waits */
        esp_task_wdt_reset();

        TickType_t left = deadline - xTaskGetTickCount();
        if (left == 0) break;
        /* Cap single wait to WDT feed interval */
        TickType_t wait = (left > pdMS_TO_TICKS(5000)) ? pdMS_TO_TICKS(5000) : left;

        agrosense_event_t ev;
        if (xQueueReceive(q, &ev, wait) != pdTRUE) continue;

        /* Handle side events regardless of what we're waiting for */
        if (ev.type == EVENT_REMOTE_CONFIG) {
            if (ev.config.sleep_normal_min > 0 || ev.config.sleep_retry_min > 0) {
                uint32_t sn = ev.config.sleep_normal_min ?: CONFIG_SLEEP_NORMAL_MIN;
                uint32_t sr = ev.config.sleep_retry_min  ?: CONFIG_SLEEP_RETRY_MIN;
                s_sleep_normal_us = (uint64_t)sn * 60 * 1000000;
                s_sleep_retry_us  = (uint64_t)sr * 60 * 1000000;
                cfg_save(sn, sr);  /* FIX 9: persist */
            }
            if (ev.type == want) { if (out) *out = ev; return true; }
            continue;
        }

        if (ev.type == EVENT_OTA_COMMAND) {
            ota_arg_t *a = malloc(sizeof(ota_arg_t));
            if (a) {
                strncpy(a->url, ev.ota_cmd.url, 255);
                a->q = q;
                xTaskCreate(ota_task, "ota", 8192, a, 5, NULL);
                /* FIX 1: wait for OTA result with timeout — no portMAX_DELAY */
                agrosense_event_t ota_ev;
                if (!wait_event(q, EVENT_OTA_RESULT, &ota_ev, OTA_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "OTA timed out");
                }
                /* After OTA (success reboots, failure falls through to sleep) */
            }
            /* FIX 12: no vTaskDelay(portMAX_DELAY) here */
            if (ev.type == want) { if (out) *out = ev; return true; }
            continue;
        }

        if (ev.type == want) {
            if (out) *out = ev;
            return true;
        }
    }
    return false;
}

/* ── Publish + PUBACK ───────────────────────────────────────────────────── */
static bool publish_confirmed(QueueHandle_t q, const char *json, size_t len)
{
    int msg_id = mqtt_manager_publish(TOPIC_SENSORS, json, len);
    if (msg_id < 0) return false;

    agrosense_event_t ev;
    if (!wait_event(q, EVENT_MQTT_PUBLISH_OK, &ev, PUBACK_TIMEOUT_MS))
        return false;
    return (ev.publish.msg_id == msg_id);
}

/* ── app_main ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== AgroSense v%s ===", CONFIG_FW_VERSION);
    ESP_LOGI(TAG, "Reset: %s", power_reset_reason());

    /* NVS */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    /* FIX 9: load persistent config */
    cfg_load();

    /* FIX 5: enable task WDT on current task */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    esp_task_wdt_add(NULL);

    /* Event queue — FIX 7: depth 20 to reduce drop risk */
    QueueHandle_t q = xQueueCreate(QUEUE_DEPTH, sizeof(agrosense_event_t));
    configASSERT(q);

    /* Init components */
    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(storage_init());
    ESP_ERROR_CHECK(wifi_manager_init(q));
    ESP_ERROR_CHECK(mqtt_manager_init(q));

    /* ── STEP 1: Read sensors (FIX 2: synchronous, no event) ── */
    sensor_reading_t reading;
    sensors_read(&reading);
    esp_task_wdt_reset();

    if (!reading.valid) {
        ESP_LOGE(TAG, "Sensor fail → retry sleep");
        mqtt_manager_stop();
        wifi_manager_stop();
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    size_t jlen = 0;
    char *json = sensors_to_json(&reading, 0,
                                  power_uptime_s(),
                                  power_reset_reason(), &jlen);
    if (!json) {
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    /* ── STEP 2: Wi-Fi ── */
    wifi_manager_connect();

    agrosense_event_t ev;
    uint32_t wifi_total_ms = (uint32_t)CONFIG_WIFI_MAX_RETRIES *
                             (10000 + 3000);
    if (!wait_event(q, EVENT_WIFI_CONNECTED, &ev, wifi_total_ms)) {
        ESP_LOGW(TAG, "Wi-Fi fail → NVS");
        storage_push(json, jlen);
        free(json);
        mqtt_manager_stop();
        wifi_manager_stop();
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    /* Update JSON with real RSSI */
    int rssi = wifi_manager_get_rssi();
    free(json);
    json = sensors_to_json(&reading, rssi,
                           power_uptime_s(), power_reset_reason(), &jlen);
    if (!json) {
        mqtt_manager_stop();
        wifi_manager_stop();
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    /* ── STEP 3: MQTT ── */
    mqtt_manager_start();
    esp_task_wdt_reset();

    if (!wait_event(q, EVENT_MQTT_READY, &ev, 8000)) {
        ESP_LOGW(TAG, "MQTT fail → NVS");
        storage_push(json, jlen);
        free(json);
        mqtt_manager_stop();
        wifi_manager_stop();
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    /* ── STEP 4: Flush stored packets ── */
    uint16_t pending = storage_count();
    if (pending > 0) {
        ESP_LOGI(TAG, "Flushing %u stored packets", pending);
        char buf[512];
        while (!storage_empty()) {
            esp_task_wdt_reset();
            if (storage_peek(buf, sizeof(buf)) != ESP_OK) break;
            if (publish_confirmed(q, buf, strlen(buf)))
                storage_pop();
            else
                break;
        }
    }

    /* ── STEP 5: Publish current reading ── */
    esp_task_wdt_reset();
    if (!publish_confirmed(q, json, jlen)) {
        ESP_LOGW(TAG, "Publish fail → NVS");
        storage_push(json, jlen);
        free(json);
        mqtt_manager_stop();
        wifi_manager_stop();
        power_deep_sleep(s_sleep_retry_us);
        return;
    }

    free(json);
    ESP_LOGI(TAG, "Done. Stored=%u", storage_count());

    /* ── STEP 6: Clean shutdown ── */
    mqtt_manager_stop();
    wifi_manager_stop();
    esp_task_wdt_delete(NULL);
    power_deep_sleep(s_sleep_normal_us);
}

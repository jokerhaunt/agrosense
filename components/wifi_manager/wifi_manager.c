/**
 * @file wifi_manager.c
 *
 * FIX 3 (delay not in handler): vTaskDelay lives in wifi_retry_task only.
 * FIX 4 (counter): s_retry_count incremented ONLY in wifi_retry_task.
 *        Event handler sets s_disconnected flag; task reads it and decides.
 * FIX 6: removed unused `extern const char wifi_ssid[]` declaration.
 */

#include "wifi_manager.h"
#include "../../main/agrosense_events.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/atomic.h"
#include <string.h>

static const char *TAG = "wifi";

#define CONNECT_TIMEOUT_MS  10000
#define RETRY_DELAY_MS      3000

static QueueHandle_t  s_q           = NULL;
static volatile bool  s_connected   = false;
static volatile bool  s_disconnected = false;  /* set by handler, read by task */
static TaskHandle_t   s_retry_task  = NULL;

/* ── Event handler: sets flags only, NO delays, NO counter changes ───────── */
static void on_wifi_event(void *a, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected   = false;
        s_disconnected = true;   /* FIX 4: flag only — task manages counter */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_connected   = true;
        s_disconnected = false;
        agrosense_event_t ev = { .type = EVENT_WIFI_CONNECTED };
        /* FIX 7: check queue send result */
        if (xQueueSend(s_q, &ev, 0) != pdTRUE)
            ESP_LOGW(TAG, "Event queue full — WIFI_CONNECTED dropped");
    }
}

/* ── Retry task: owns counter, safe to delay ────────────────────────────── */
static void wifi_retry_task(void *arg)
{
    int retries = 0;

    /* Absolute deadline: FIX 3 — no infinite wait */
    TickType_t deadline = xTaskGetTickCount() +
        pdMS_TO_TICKS((uint32_t)CONFIG_WIFI_MAX_RETRIES *
                      (CONNECT_TIMEOUT_MS + RETRY_DELAY_MS));

    while (xTaskGetTickCount() < deadline) {
        if (s_connected) {
            vTaskDelete(NULL);
            return;
        }

        if (s_disconnected) {
            s_disconnected = false;
            retries++;                  /* FIX 4: only here */

            if (retries >= CONFIG_WIFI_MAX_RETRIES) {
                ESP_LOGE(TAG, "Max retries (%d) reached", CONFIG_WIFI_MAX_RETRIES);
                agrosense_event_t ev = { .type = EVENT_WIFI_FAILED };
                if (xQueueSend(s_q, &ev, 0) != pdTRUE)
                    ESP_LOGW(TAG, "Queue full — WIFI_FAILED dropped");
                vTaskDelete(NULL);
                return;
            }

            ESP_LOGW(TAG, "Retry %d/%d", retries, CONFIG_WIFI_MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));  /* safe here */
            esp_wifi_connect();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGE(TAG, "Deadline exceeded");
    agrosense_event_t ev = { .type = EVENT_WIFI_FAILED };
    xQueueSend(s_q, &ev, 0);
    vTaskDelete(NULL);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t wifi_manager_init(QueueHandle_t q)
{
    s_q = q;
    s_connected    = false;
    s_disconnected = false;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL));

    return ESP_OK;
}

esp_err_t wifi_manager_connect(void)
{
    wifi_config_t wc = {0};
    /* FIX 6: use CONFIG_ macros directly — no extern asm hack */
    strncpy((char *)wc.sta.ssid,     CONFIG_WIFI_SSID,     31);
    strncpy((char *)wc.sta.password, CONFIG_WIFI_PASSWORD, 63);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(wifi_retry_task, "wifi_retry", 2560, NULL, 5, &s_retry_task);
    return ESP_OK;
}

void wifi_manager_stop(void)
{
    if (s_retry_task) {
        vTaskDelete(s_retry_task);
        s_retry_task = NULL;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    s_connected = false;
}

int wifi_manager_get_rssi(void)
{
    if (!s_connected) return 0;
    wifi_ap_record_t ap;
    return (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
}

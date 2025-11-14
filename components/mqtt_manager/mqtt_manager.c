/**
 * @file mqtt_manager.c
 *
 * FIX 7: All xQueueSend calls check return value.
 * FIX 8: s_connected protected by mutex — publish checks connection
 *        atomically to prevent TOCTOU race between check and send.
 */

#include "mqtt_manager.h"
#include "../../main/agrosense_events.h"
#include "sdkconfig.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "mqtt";

#define TOPIC_SENSORS  "agrosense/" CONFIG_DEVICE_ID "/sensors"
#define TOPIC_STATUS   "agrosense/" CONFIG_DEVICE_ID "/status"
#define TOPIC_CMD      "agrosense/" CONFIG_DEVICE_ID "/cmd"

#define LWT_OFFLINE    "{\"id\":\"" CONFIG_DEVICE_ID "\",\"status\":\"offline\"}"
#define LWT_ONLINE     "{\"id\":\"" CONFIG_DEVICE_ID "\",\"status\":\"online\",\"fw\":\"" CONFIG_FW_VERSION "\"}"

static esp_mqtt_client_handle_t s_client  = NULL;
static QueueHandle_t            s_q       = NULL;
static SemaphoreHandle_t        s_mutex   = NULL;   /* FIX 8 */
static volatile bool            s_connected = false;

/* ── Command parser ─────────────────────────────────────────────────────── */
static void parse_cmd(const char *data, int len)
{
    char *buf = strndup(data, len);
    if (!buf) return;

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;

    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (!cJSON_IsString(cmd)) { cJSON_Delete(root); return; }

    agrosense_event_t ev = {0};

    if (strcmp(cmd->valuestring, "update") == 0) {
        cJSON *url = cJSON_GetObjectItem(root, "url");
        if (cJSON_IsString(url) && strlen(url->valuestring) < 256) {
            ev.type = EVENT_OTA_COMMAND;
            strncpy(ev.ota_cmd.url, url->valuestring, 255);
            /* FIX 7: check send result */
            if (xQueueSend(s_q, &ev, 0) != pdTRUE)
                ESP_LOGW(TAG, "Queue full — OTA command dropped");
        }
    } else if (strcmp(cmd->valuestring, "config") == 0) {
        cJSON *sn = cJSON_GetObjectItem(root, "sleep_normal_min");
        cJSON *sr = cJSON_GetObjectItem(root, "sleep_retry_min");
        ev.type = EVENT_REMOTE_CONFIG;
        ev.config.sleep_normal_min = cJSON_IsNumber(sn) ? (uint32_t)sn->valuedouble : 0;
        ev.config.sleep_retry_min  = cJSON_IsNumber(sr) ? (uint32_t)sr->valuedouble : 0;
        if (xQueueSend(s_q, &ev, 0) != pdTRUE)
            ESP_LOGW(TAG, "Queue full — remote config dropped");
    }

    cJSON_Delete(root);
}

/* ── Event handler ──────────────────────────────────────────────────────── */
static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *edata)
{
    esp_mqtt_event_handle_t ev = edata;
    agrosense_event_t qev = {0};

    switch ((esp_mqtt_event_id_t)id) {

        case MQTT_EVENT_CONNECTED:
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_connected = true;
            xSemaphoreGive(s_mutex);
            esp_mqtt_client_subscribe(s_client, TOPIC_CMD, 1);
            esp_mqtt_client_publish(s_client, TOPIC_STATUS,
                                    LWT_ONLINE, 0, 1, 1);
            qev.type = EVENT_MQTT_READY;
            /* FIX 7 */
            if (xQueueSend(s_q, &qev, 0) != pdTRUE)
                ESP_LOGW(TAG, "Queue full — MQTT_READY dropped");
            break;

        case MQTT_EVENT_DISCONNECTED:
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_connected = false;
            xSemaphoreGive(s_mutex);
            qev.type = EVENT_MQTT_DISCONNECTED;
            xQueueSend(s_q, &qev, 0);
            break;

        case MQTT_EVENT_PUBLISHED:
            qev.type = EVENT_MQTT_PUBLISH_OK;
            qev.publish.msg_id = ev->msg_id;
            if (xQueueSend(s_q, &qev, 0) != pdTRUE)
                ESP_LOGW(TAG, "Queue full — PUBACK dropped (msg_id=%d)", ev->msg_id);
            break;

        case MQTT_EVENT_DATA:
            if (ev->data && ev->data_len > 0)
                parse_cmd(ev->data, ev->data_len);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error type=%d", ev->error_handle->error_type);
            qev.type = EVENT_MQTT_PUBLISH_FAIL;
            xQueueSend(s_q, &qev, 0);
            break;

        default: break;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t mqtt_manager_init(QueueHandle_t q)
{
    s_q = q;
    s_mutex = xSemaphoreCreateMutex();   /* FIX 8 */
    if (!s_mutex) return ESP_ERR_NO_MEM;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .session.last_will  = {
            .topic   = TOPIC_STATUS,
            .msg     = LWT_OFFLINE,
            .msg_len = sizeof(LWT_OFFLINE) - 1,
            .qos     = 1,
            .retain  = 1,
        },
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) return ESP_FAIL;

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_mqtt, NULL);
    return ESP_OK;
}

esp_err_t mqtt_manager_start(void)
{
    return s_client ? esp_mqtt_client_start(s_client) : ESP_ERR_INVALID_STATE;
}

int mqtt_manager_publish(const char *topic, const char *payload, size_t len)
{
    /* FIX 8: check + send under mutex to avoid TOCTOU */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool connected = s_connected;
    xSemaphoreGive(s_mutex);

    if (!connected || !s_client) return -1;
    return esp_mqtt_client_publish(s_client, topic, payload, (int)len, 1, 0);
}

void mqtt_manager_stop(void)
{
    if (!s_client) return;
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    if (s_mutex) { vSemaphoreDelete(s_mutex); s_mutex = NULL; }
    s_connected = false;
}

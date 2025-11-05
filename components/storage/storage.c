/**
 * @file storage.c
 * @brief NVS circular buffer — 24 packets, store-and-forward.
 *
 * FIX 3: All nvs_set_* calls are checked before nvs_commit.
 *        If any set fails, the handle is closed without commit,
 *        leaving NVS in its previous consistent state.
 */

#include "storage.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "storage";

#define NVS_NS          "as_queue"
#define KEY_HEAD        "head"
#define KEY_TAIL        "tail"
#define KEY_COUNT       "count"
#define MAX_PKT         CONFIG_STORAGE_MAX_PACKETS
#define MAX_PKT_BYTES   512

static void slot_key(uint16_t idx, char *buf, size_t sz)
{
    snprintf(buf, sz, "p%02u", (unsigned)(idx % MAX_PKT));
}

static uint16_t nvs_u16(nvs_handle_t h, const char *k, uint16_t def)
{
    uint16_t v = def;
    nvs_get_u16(h, k, &v);
    return v;
}

esp_err_t storage_init(void)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    nvs_close(h);
    ESP_LOGI(TAG, "Ready — %u packets stored", storage_count());
    return ESP_OK;
}

esp_err_t storage_push(const char *json, size_t len)
{
    if (!json || len == 0 || len >= MAX_PKT_BYTES)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;

    uint16_t head  = nvs_u16(h, KEY_HEAD,  0);
    uint16_t tail  = nvs_u16(h, KEY_TAIL,  0);
    uint16_t count = nvs_u16(h, KEY_COUNT, 0);

    char key[6];
    slot_key(head, key, sizeof(key));

    /* FIX 3: check every set before commit */
    r = nvs_set_str(h, key, json);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "push: set_str failed: %s", esp_err_to_name(r));
        nvs_close(h);
        return r;
    }

    uint16_t new_head  = (head + 1) % MAX_PKT;
    uint16_t new_tail  = tail;
    uint16_t new_count = count;

    if (count >= MAX_PKT) {
        new_tail  = (tail + 1) % MAX_PKT;
        ESP_LOGW(TAG, "Buffer full — oldest dropped");
    } else {
        new_count = count + 1;
    }

    r = nvs_set_u16(h, KEY_HEAD, new_head);
    if (r != ESP_OK) { nvs_close(h); return r; }
    r = nvs_set_u16(h, KEY_TAIL, new_tail);
    if (r != ESP_OK) { nvs_close(h); return r; }
    r = nvs_set_u16(h, KEY_COUNT, new_count);
    if (r != ESP_OK) { nvs_close(h); return r; }

    /* FIX 3: commit only when ALL sets succeeded */
    r = nvs_commit(h);
    nvs_close(h);

    if (r == ESP_OK)
        ESP_LOGI(TAG, "Pushed (count=%u/%d)", new_count, MAX_PKT);
    return r;
}

esp_err_t storage_peek(char *buf, size_t sz)
{
    if (!buf || sz == 0) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (r != ESP_OK) return r;

    uint16_t count = nvs_u16(h, KEY_COUNT, 0);
    if (count == 0) { nvs_close(h); return ESP_ERR_NOT_FOUND; }

    uint16_t tail = nvs_u16(h, KEY_TAIL, 0);
    char key[6];
    slot_key(tail, key, sizeof(key));

    size_t needed = 0;
    r = nvs_get_str(h, key, NULL, &needed);
    if (r != ESP_OK) { nvs_close(h); return r; }
    if (needed > sz) { nvs_close(h); return ESP_ERR_NO_MEM; }

    r = nvs_get_str(h, key, buf, &needed);
    nvs_close(h);
    return r;
}

esp_err_t storage_pop(void)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;

    uint16_t count = nvs_u16(h, KEY_COUNT, 0);
    if (count == 0) { nvs_close(h); return ESP_ERR_NOT_FOUND; }

    uint16_t tail = nvs_u16(h, KEY_TAIL, 0);
    char key[6];
    slot_key(tail, key, sizeof(key));
    nvs_erase_key(h, key);

    uint16_t new_tail  = (tail + 1) % MAX_PKT;
    uint16_t new_count = count - 1;

    /* FIX 3: check sets before commit */
    r = nvs_set_u16(h, KEY_TAIL,  new_tail);
    if (r != ESP_OK) { nvs_close(h); return r; }
    r = nvs_set_u16(h, KEY_COUNT, new_count);
    if (r != ESP_OK) { nvs_close(h); return r; }

    r = nvs_commit(h);
    nvs_close(h);
    return r;
}

uint16_t storage_count(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    uint16_t c = nvs_u16(h, KEY_COUNT, 0);
    nvs_close(h);
    return c;
}

bool storage_empty(void) { return storage_count() == 0; }

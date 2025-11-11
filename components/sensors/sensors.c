/**
 * @file sensors.c
 */

#include "sensors.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "sensors";

#define GPIO_PWR_SHT31      GPIO_NUM_25
#define GPIO_PWR_MQ135      GPIO_NUM_26
#define GPIO_PWR_MQ7        GPIO_NUM_27

#define I2C_PORT            I2C_NUM_0
#define I2C_SDA             21
#define I2C_SCL             22
#define I2C_FREQ_HZ         100000
#define SHT31_ADDR          0x44

#define ADC_NH3             ADC1_CHANNEL_6
#define ADC_CO              ADC1_CHANNEL_7

#define SHT31_WARMUP_MS     15
#define MQ_WARMUP_MS        120
#define SHT31_MEASURE_MS    20
#define SENSOR_TIMEOUT_MS   500
#define MEDIAN_N            7
#define MAX_JSON_BYTES      512

static esp_adc_cal_characteristics_t s_cal_nh3;
static esp_adc_cal_characteristics_t s_cal_co;

/* ── CRC-8 (Sensirion poly=0x31 init=0xFF) ─────────────────────────────── */
static uint8_t crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}

/* ── Power gate ─────────────────────────────────────────────────────────── */
static void gate_on(gpio_num_t pin, uint32_t ms)
{
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
}
static void gate_off(gpio_num_t pin) { gpio_set_level(pin, 0); }

/* ── Median filter ──────────────────────────────────────────────────────── */
static uint32_t median(uint32_t *a, int n)
{
    uint32_t buf[MEDIAN_N];
    memcpy(buf, a, n * sizeof(uint32_t));
    for (int i = 1; i < n; i++) {
        uint32_t k = buf[i]; int j = i - 1;
        while (j >= 0 && buf[j] > k) { buf[j+1] = buf[j]; j--; }
        buf[j+1] = k;
    }
    return buf[n / 2];
}

static uint32_t adc_mv(adc1_channel_t ch, const esp_adc_cal_characteristics_t *c)
{
    uint32_t s[MEDIAN_N];
    for (int i = 0; i < MEDIAN_N; i++) {
        s[i] = adc1_get_raw(ch);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return esp_adc_cal_raw_to_voltage(median(s, MEDIAN_N), c);
}

/* ── SHT31 read with per-operation timeout ──────────────────────────────── */
static esp_err_t sht31_read(float *t, float *h, bool *timedout)
{
    *timedout = false;
    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t data[6] = {0};

    esp_err_t r = i2c_master_write_to_device(I2C_PORT, SHT31_ADDR,
        cmd, 2, pdMS_TO_TICKS(SENSOR_TIMEOUT_MS));
    if (r == ESP_ERR_TIMEOUT) { *timedout = true; return r; }
    if (r != ESP_OK) return r;

    vTaskDelay(pdMS_TO_TICKS(SHT31_MEASURE_MS));

    r = i2c_master_read_from_device(I2C_PORT, SHT31_ADDR,
        data, 6, pdMS_TO_TICKS(SENSOR_TIMEOUT_MS));
    if (r == ESP_ERR_TIMEOUT) { *timedout = true; return r; }
    if (r != ESP_OK) return r;

    if (crc8(data, 2) != data[2]) return ESP_ERR_INVALID_CRC;
    if (crc8(data + 3, 2) != data[5]) return ESP_ERR_INVALID_CRC;

    uint16_t rt = ((uint16_t)data[0] << 8) | data[1];
    uint16_t rh = ((uint16_t)data[3] << 8) | data[4];
    *t = -45.0f + 175.0f * ((float)rt / 65535.0f);
    *h = 100.0f * ((float)rh / 65535.0f);
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t sensors_init(void)
{
    /* Power gates */
    gpio_config_t gc = {
        .pin_bit_mask = (1ULL << GPIO_PWR_SHT31) |
                        (1ULL << GPIO_PWR_MQ135)  |
                        (1ULL << GPIO_PWR_MQ7),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gc);
    gate_off(GPIO_PWR_SHT31);
    gate_off(GPIO_PWR_MQ135);
    gate_off(GPIO_PWR_MQ7);

    /* I2C */
    i2c_config_t ic = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &ic));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* ADC */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_NH3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CO,  ADC_ATTEN_DB_11);

    /* FIX 11: check eFuse calibration availability before characterize */
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "ADC: eFuse Two-Point cal available");
    } else if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "ADC: eFuse Vref cal available");
    } else {
        ESP_LOGW(TAG, "ADC: no eFuse cal — using default Vref=1100mV, "
                      "readings may have up to ±6%% error");
    }

    esp_adc_cal_value_t t;
    t = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                                 ADC_WIDTH_BIT_12, 1100, &s_cal_nh3);
    t = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                                 ADC_WIDTH_BIT_12, 1100, &s_cal_co);
    (void)t;
    return ESP_OK;
}

esp_err_t sensors_read(sensor_reading_t *out)
{
    memset(out, 0, sizeof(*out));

    /* SHT31 */
    gate_on(GPIO_PWR_SHT31, SHT31_WARMUP_MS);
    bool to = false;
    esp_err_t r = sht31_read(&out->temperature, &out->humidity, &to);
    gate_off(GPIO_PWR_SHT31);

    if (to)                          out->errors.sht31_timeout    = true;
    else if (r == ESP_ERR_INVALID_CRC) out->errors.sht31_crc_error = true;
    else if (r == ESP_OK) {
        if (out->temperature < -40.0f || out->temperature > 85.0f ||
            out->humidity    <   0.0f || out->humidity    > 100.0f)
            out->errors.sht31_range_error = true;
    }

    bool sht31_ok = (r == ESP_OK && !out->errors.sht31_crc_error &&
                     !out->errors.sht31_range_error && !out->errors.sht31_timeout);

    /* MQ-135 */
    TickType_t t0 = xTaskGetTickCount();
    gate_on(GPIO_PWR_MQ135, MQ_WARMUP_MS);
    if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(SENSOR_TIMEOUT_MS))
        out->errors.mq135_timeout = true;
    else
        out->nh3_mv = adc_mv(ADC_NH3, &s_cal_nh3);
    gate_off(GPIO_PWR_MQ135);

    /* MQ-7 */
    t0 = xTaskGetTickCount();
    gate_on(GPIO_PWR_MQ7, MQ_WARMUP_MS);
    if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(SENSOR_TIMEOUT_MS))
        out->errors.mq7_timeout = true;
    else
        out->co_mv = adc_mv(ADC_CO, &s_cal_co);
    gate_off(GPIO_PWR_MQ7);

    out->valid = sht31_ok;

    ESP_LOGI(TAG, "T=%.1f°C H=%.1f%% NH3=%lumV CO=%lumV valid=%d",
             out->temperature, out->humidity,
             (unsigned long)out->nh3_mv, (unsigned long)out->co_mv,
             out->valid);
    return ESP_OK;
}

char *sensors_to_json(const sensor_reading_t *r, int rssi,
                      uint32_t uptime_s, const char *reset_reason,
                      size_t *out_len)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddStringToObject(root, "device_id",     CONFIG_DEVICE_ID);
    cJSON_AddStringToObject(root, "fw",            CONFIG_FW_VERSION);
    cJSON_AddNumberToObject(root, "temperature",   r->temperature);
    cJSON_AddNumberToObject(root, "humidity",      r->humidity);
    cJSON_AddNumberToObject(root, "nh3_mv",        r->nh3_mv);
    cJSON_AddNumberToObject(root, "co_mv",         r->co_mv);
    cJSON_AddNumberToObject(root, "rssi_dbm",      rssi);
    cJSON_AddNumberToObject(root, "uptime_s",      uptime_s);
    cJSON_AddStringToObject(root, "reset_reason",  reset_reason ? reset_reason : "unknown");

    uint8_t flags = (r->errors.sht31_crc_error   ? 0x01 : 0) |
                    (r->errors.sht31_timeout      ? 0x02 : 0) |
                    (r->errors.sht31_range_error  ? 0x04 : 0) |
                    (r->errors.mq135_timeout      ? 0x08 : 0) |
                    (r->errors.mq7_timeout        ? 0x10 : 0);
    cJSON_AddNumberToObject(root, "err_flags", flags);

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!str) return NULL;

    /* FIX 10: validate size after generation */
    size_t len = strlen(str);
    if (len >= MAX_JSON_BYTES) {
        ESP_LOGE(TAG, "JSON oversized: %zu >= %d", len, MAX_JSON_BYTES);
        free(str);
        return NULL;
    }

    if (out_len) *out_len = len;
    return str;
}

#pragma once
/**
 * @file sensors.h
 * @brief Sensor component: SHT31 + MQ-135 + MQ-7 with power gating.
 *
 * FIX 11: esp_adc_cal_check_efuse() called at init — logs warning if
 *         eFuse calibration data is unavailable so the operator knows
 *         ADC readings may be less accurate on that specific chip.
 */

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    float    temperature;
    float    humidity;
    uint32_t nh3_mv;
    uint32_t co_mv;
    bool     valid;
    struct {
        bool sht31_crc_error;
        bool sht31_timeout;
        bool sht31_range_error;
        bool mq135_timeout;
        bool mq7_timeout;
    } errors;
} sensor_reading_t;

esp_err_t sensors_init(void);
esp_err_t sensors_read(sensor_reading_t *out);

/**
 * @brief Serialize reading to heap-allocated JSON.
 *        FIX 10: size validated after generation; returns NULL if oversized.
 *        Caller must free() the result.
 */
char *sensors_to_json(const sensor_reading_t *r, int rssi,
                      uint32_t uptime_s, const char *reset_reason,
                      size_t *out_len);

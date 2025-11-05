#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

esp_err_t storage_init(void);
esp_err_t storage_push(const char *json, size_t len);
esp_err_t storage_peek(char *buf, size_t buf_sz);
esp_err_t storage_pop(void);
uint16_t  storage_count(void);
bool      storage_empty(void);

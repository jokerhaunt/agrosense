# AgroSense

Autonomous ESP32 firmware for continuous environmental monitoring in agricultural storage facilities. Measures NH₃, CO, temperature and humidity. Publishes via MQTT QoS-1, survives power cuts via NVS store-and-forward.

---

## Architecture

```
components/
├── sensors/       SHT31 + MQ-135 + MQ-7, optocoupler power gating,
│                  CRC-8, median filter, ADC eFuse calibration
├── wifi_manager/  Connect with retry, no delay in event handler
├── mqtt_manager/  QoS-1 publish, LWT, OTA command, remote config
├── storage/       NVS circular buffer, 24 packets, transactional
└── power/         Deep sleep, WDT, reboot, reset reason
```

**Event model:** FreeRTOS queue between components and orchestrator.

---

## Fixes vs previous version

| # | Fix |
|---|---|
| 1 | OTA task always posts `EVENT_OTA_RESULT` — no `portMAX_DELAY` hang |
| 2 | `EVENT_DATA_READY` removed — sensor data read synchronously, no pointer leak |
| 3 | NVS transactions: all `nvs_set_*` checked before `nvs_commit` |
| 4 | Wi-Fi retry counter managed only in task — no double-increment from handler |
| 5 | Hardware WDT enabled; fed in `wait_event` loop |
| 6 | Removed unused `extern asm` symbol |
| 7 | All `xQueueSend` return values checked; queue depth 20 |
| 8 | `s_connected` protected by mutex — no TOCTOU in publish |
| 9 | Remote config persisted to NVS; loaded on every boot |
| 10 | JSON size validated after `cJSON_PrintUnformatted` |
| 11 | `esp_adc_cal_check_efuse()` called at init; logs warning if uncalibrated |
| 12 | `vTaskDelay(portMAX_DELAY)` replaced with `wait_event` + timeout |

---

## Hardware

| Component | Interface | Power gate GPIO |
|---|---|---|
| SHT31 | I2C GPIO21/22 | GPIO25 via optocoupler |
| MQ-135 (NH₃) | ADC GPIO34 | GPIO26 via optocoupler |
| MQ-7 (CO) | ADC GPIO35 | GPIO27 via optocoupler |

All sensors powered sequentially — one at a time — to minimize peak current.

---

## MQTT

**Data:** `agrosense/<device_id>/sensors`
```json
{
  "device_id": "agrosense_01", "fw": "1.0.0",
  "temperature": 4.2, "humidity": 87.5,
  "nh3_mv": 412, "co_mv": 198,
  "rssi_dbm": -62, "uptime_s": 14,
  "reset_reason": "deep_sleep", "err_flags": 0
}
```

**Status (LWT):** `agrosense/<device_id>/status`, retain=1

**Commands:** `agrosense/<device_id>/cmd`
```json
{ "cmd": "update", "url": "http://server/fw.bin" }
{ "cmd": "config", "sleep_normal_min": 60, "sleep_retry_min": 10 }
```

---

## Fault tolerance

```
Boot → sensors → Wi-Fi → MQTT → flush NVS → publish → sleep 60 min
         ↓          ↓       ↓        ↓          ↓
       save NVS → sleep 10 min (retry on next wake)
```

Up to 24 packets stored in NVS. Survives full power loss.

---

## Build

```bash
idf.py menuconfig   # AgroSense Configuration
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Author

**Ivan Shulik** — Embedded Firmware Engineer
[linkedin.com/in/ivan-shulik](https://www.linkedin.com/in/ivan-shulik/)

---

## License

MIT

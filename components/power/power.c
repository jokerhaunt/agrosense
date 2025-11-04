#include "power.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"

void power_deep_sleep(uint64_t us)
{
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
}

void power_reboot(void) { esp_restart(); }

const char *power_reset_reason(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power_on";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "panic";
        default:                return "other";
    }
}

uint32_t power_uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

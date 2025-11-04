#pragma once
#include <stdint.h>
void        power_deep_sleep(uint64_t us);
void        power_reboot(void);
const char *power_reset_reason(void);
uint32_t    power_uptime_s(void);

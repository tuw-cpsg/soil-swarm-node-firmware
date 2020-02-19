#include <zephyr/types.h>

s16_t util_temperature_to_ble(s16_t temperature);
u16_t util_humidity_to_ble(u32_t value);
u32_t util_sys_time_to_unix();
u16_t util_battery_to_v(u16_t value);
u8_t util_battery_to_ble(u16_t value);

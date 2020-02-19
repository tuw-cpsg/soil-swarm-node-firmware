#include "util.h"

#define HUMI_VAL_MAX 0x02d2
#define HUMI_VAL_MIN 0x016a

#define BATT_VAL_MAX 0x034e	/* 4.2V */
#define BATT_VAL_MIN 0x02c1	/* 3.5V */

s16_t util_temperature_to_ble(s16_t temperature) {
	return (temperature * 100) / 16;
}

u16_t util_humidity_to_ble(u32_t moisture) {
	if (moisture > HUMI_VAL_MAX) {
		moisture = HUMI_VAL_MAX;
	} else if (moisture < HUMI_VAL_MIN) {
		moisture = HUMI_VAL_MIN;
	}

	moisture  = moisture - HUMI_VAL_MIN;
	moisture  = (HUMI_VAL_MAX - HUMI_VAL_MIN) - moisture;
	moisture *= 10000;
	moisture /= (HUMI_VAL_MAX - HUMI_VAL_MIN);

	return (u16_t) moisture;
}

u16_t util_battery_to_v(u16_t value)
{
	if (value > BATT_VAL_MAX) {
		value = BATT_VAL_MAX;
	}

	value *= 42;
	value /= BATT_VAL_MAX;

	return value * 10;
}

u8_t util_battery_to_ble(u16_t value)
{
	if (value > BATT_VAL_MAX) {
		value = BATT_VAL_MAX;
	} else if (value < BATT_VAL_MIN) {
		value = BATT_VAL_MIN;
	}

	value -= BATT_VAL_MIN;
	value *= 100;
	value /= (BATT_VAL_MAX - BATT_VAL_MIN);

	return (u8_t) value;
}

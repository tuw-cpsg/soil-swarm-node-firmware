#include <zephyr.h>

s8_t  ds18b20_init(void);
s8_t  ds18b20_enable(void);
s8_t  ds18b20_disable(void);
s16_t ds18b20_measure_temp(void);
s16_t ds18b20_read_temp(void);

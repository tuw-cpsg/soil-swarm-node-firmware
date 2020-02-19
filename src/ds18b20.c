#include "ds18b20.h"
#include "onewire.h"

#include "config.h"

#include <zephyr.h>
#include <device.h>
#include <gpio.h>

struct device *ds18b20_dev;

K_SEM_DEFINE(ds18b20_guard, 1, 1);

s8_t ds18b20_init(void) {
    ds18b20_dev = device_get_binding(DS18B20_EN_PORT);
    if(ds18b20_dev == NULL)
        return -1;

    s8_t ret = gpio_pin_configure(ds18b20_dev, DS18B20_EN_PIN, GPIO_DIR_OUT);
    if(ret != 0)
        return ret;

    SetSpeed(1);
	
    ds18b20_disable();

    return 0;
}

int ds18b20_enable(s32_t timeout)
{
	int err = k_sem_take(&ds18b20_guard, timeout);
	if (err != 0)
		return err;

    OWPower();

    return gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 1);
}

int ds18b20_disable(void)
{
	OWDepower();

	int ret = gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 0);

    k_sem_give(&ds18b20_guard);

    if (ret != 0) {
        return ret;
    }

	return 0;
}

s16_t ds18b20_measure_temp(void)
{
    if(OWTouchReset())
        return -1000;

    OWWriteByte(0xCC, 0);  // Send Skip ROM command to select single device
    OWWriteByte(0x44, 1);  // start conversion, parasite power on

	return 0;
}

s16_t ds18b20_read_temp(void)
{
	u8_t data[9];

    if(OWTouchReset())
        return -1000;

    OWWriteByte(0xCC, 0);  // Send Skip ROM command to select single device
    OWWriteByte(0xBE, 0);  // read 9 bytes of scratchpad

    for (u8_t i=0; i<9; i++)
    {
        data[i] = OWReadByte();
    }

	OWTouchReset();

    return ((data[1] & 0xff) << 8) | (data[0] & 0xff);
}

#include "ds18b20.h"
#include "onewire.h"

#include "config.h"
#include <device.h>
#include <gpio.h>

struct device *ds18b20_dev;

s8_t ds18b20_init(void) {
    ds18b20_dev = device_get_binding(DS18B20_EN_PORT);
    if(ds18b20_dev == NULL)
        return -1;

    s8_t ret = gpio_pin_configure(ds18b20_dev, DS18B20_EN_PIN, GPIO_DIR_OUT);
    if(ret != NULL)
        return ret;

    SetSpeed(1);
	
    ds18b20_disable();

    return 0;
}

s8_t ds18b20_enable(void)
{
    OWPower();
    return gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 1);
}

s8_t ds18b20_disable(void)
{
	OWDepower();
    s8_t ret = gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 0);
    if(ret != 0)
        return ret;

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
    s8_t ret;
	u8_t data[9];

    if(OWTouchReset())
        return -1000;

    OWWriteByte(0xCC, 0);  // Send Skip ROM command to select single device
    OWWriteByte(0xBE, 0);  // read 9 bytes of scratchpad

    for(u8_t i=0; i<9; i++)
        data[i] = OWReadByte();

	OWTouchReset();

    return ((data[1] & 0xff) << 8) | (data[0] & 0xff);
}

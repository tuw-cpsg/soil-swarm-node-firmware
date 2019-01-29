#include "ds18b20.h"
#include "onewire.h"

#include "settings.h"
#include <device.h>
#include <gpio.h>

struct device *ds18b20_dev;

int ds18b20_init(void) {
    ds18b20_dev = device_get_binding(DS18B20_EN_PORT);
    if(ds18b20_dev == NULL)
        return -1;

    int ret = gpio_pin_configure(ds18b20_dev, DS18B20_EN_PIN, GPIO_DIR_OUT);
    if(ret != NULL)
        return ret;

    SetSpeed(1);
    //ds18b20_disable();
    ds18b20_enable();

    return 0;
}

int ds18b20_enable(void)
{
    OWPower();
    return gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 1);
}

int ds18b20_disable(void)
{
    int ret = gpio_pin_write(ds18b20_dev, DS18B20_EN_PIN, 0);
    if(ret != 0)
        return ret;

    OWDepower();
}

int ds18b20_measure_temp(void)
{
    if(OWTouchReset())
        return -1000;

    OWWriteByte(0xCC, 0);  // Send Skip ROM command to select single device
    OWWriteByte(0x44, 1);  // start conversion, parasite power on
}

int ds18b20_read_temp(void)
{
    int8_t ret, data[9];

    if(OWTouchReset())
        return -1000;

    OWWriteByte(0xCC, 0);  // Send Skip ROM command to select single device
    OWWriteByte(0xBE, 0);  // read 9 bytes of scratchpad
    for(uint8_t i=0; i<9; i++)
        data[i] = OWReadByte();

    return ((data[1] & 0xff) << 8) | (data[0] & 0xff);
}

#include "led.h"

#include "settings.h"
#include <device.h>
#include <gpio.h>

struct device *led;
uint8_t led_status = 0;

void led_init() {

    led = device_get_binding(LED_PORT);

    gpio_pin_configure(led, LED_N, GPIO_DIR_OUT);
    gpio_pin_configure(led, LED_P, GPIO_DIR_OUT);
    gpio_pin_write(led, LED_N, 0);
    led_off();
}

void led_on() {
    led_status = 1;
    gpio_pin_write(led, LED_P, led_status);
}

void led_off() {
    led_status = 0;
    gpio_pin_write(led, LED_P, led_status);
}

void led_toggle() {
    led_status = !led_status;
    gpio_pin_write(led, LED_P, led_status);
}

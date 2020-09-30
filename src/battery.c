//#include "battery.h"

//#include "settings.h"
#include <device.h>
#include <drivers/gpio.h>

#include <drivers/adc.h>
#include <hal/nrf_adc.h>

#define BATTERY_EN_PIN 17

struct device *battery_en;

#define BUFFER_SIZE 1
static int32_t m_sample_buffer[BUFFER_SIZE];

struct device *adc_dev;

static const struct adc_channel_cfg adc_cfg = {
    .gain             = ADC_GAIN_1_3,
    .reference        = ADC_REF_VDD_1_3,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//    .channel_id       = ADC_1ST_CHANNEL_ID,
//    .channel_id       = BATTERY_SENSE_PIN,
    .input_positive   = NRF_ADC_CONFIG_INPUT_6,
};

struct adc_sequence adc_sequence = {
//    .channels    = BIT(ADC_1ST_CHANNEL_ID),
    .buffer      = m_sample_buffer,
    .buffer_size = sizeof(m_sample_buffer),
    .resolution  = 10,
};

int battery_init(struct device *a_dev) {
    battery_en = device_get_binding(DT_LABEL(DT_ALIAS(gpio_0)));

    gpio_pin_configure(battery_en, BATTERY_EN_PIN, GPIO_OUTPUT);
    gpio_pin_set(battery_en, BATTERY_EN_PIN, 0);

	adc_dev = device_get_binding(DT_LABEL(DT_ALIAS(adc_0)));

    if(adc_dev == 0)
        return -1;

    return adc_channel_setup(adc_dev, &adc_cfg);
}

int16_t battery_sample() {
	int err = 0;

    if(adc_channel_setup(adc_dev, &adc_cfg) != 0)
        return -1;
    adc_sequence.channels = BIT(adc_cfg.channel_id);

    gpio_pin_set(battery_en, BATTERY_EN_PIN, 1);
    err = adc_read(adc_dev, &adc_sequence);
    gpio_pin_set(battery_en, BATTERY_EN_PIN, 0);

    err = adc_raw_to_millivolts(1000, adc_cfg.gain, adc_sequence.resolution, &m_sample_buffer[0]);
    if (err == 0) {
    	m_sample_buffer[0] *= (82+31);
    	m_sample_buffer[0] /= (82);
    }

    if(err < 0)
        return -EIO;

    return m_sample_buffer[0];
}

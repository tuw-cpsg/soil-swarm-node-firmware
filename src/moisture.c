#include "moisture.h"

#include "config.h"
#include <device.h>
#include <gpio.h>
#include <logging/log.h>

#include <adc.h>
#include <hal/nrf_adc.h>
#include <pwm.h>

#define PWM_DRIVER DT_INST_0_NORDIC_NRF_SW_PWM_LABEL
//#define PWM_CHANNEL LED_P
#define PWM_CHANNEL SENSE_EXCITATION_PIN
#define SENSE_TIME 800    // 4usec necessary

struct device *sense_en;
struct device *sense_pwm;

LOG_MODULE_REGISTER(moisture);

#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];

struct device *adc_moisture_dev;

static const struct adc_channel_cfg adc_low_cfg = {
    .gain             = ADC_GAIN,
    .reference        = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//    .channel_id       = ADC_2ND_CHANNEL_ID,
    .input_positive   = SENSE_LOW_PIN,
};

static const struct adc_channel_cfg adc_high_cfg = {
    .gain             = ADC_GAIN,
    .reference        = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//    .channel_id       = ADC_2ND_CHANNEL_ID,
    .input_positive   = SENSE_HIGH_PIN,
};

struct adc_sequence adc_moisture_sequence = {
//    .channels    = BIT(ADC_2ND_CHANNEL_ID) | BIT(ADC_3RD_CHANNEL_ID),
    .buffer      = m_sample_buffer,
    .buffer_size = sizeof(m_sample_buffer),
    .resolution  = ADC_RESOLUTION,
};

void init_sense()
{
    sense_pwm = device_get_binding(PWM_DRIVER);
    if (!sense_pwm) {
        LOG_INF("Cannot find %s!\n", PWM_DRIVER);
        return;
    }
}

void start_sense()
{
    gpio_pin_write(sense_en, SENSE_EN_PIN, 1);
    pwm_pin_set_cycles(sense_pwm, PWM_CHANNEL,
        4, 3);
}

void stop_sense()
{
    pwm_pin_set_cycles(sense_pwm, PWM_CHANNEL,
        4, 0);
    gpio_pin_write(sense_en, SENSE_EN_PIN, 0);
}

int moisture_init(struct device *a_dev) {
    sense_en = device_get_binding(SENSE_EN_PORT);

    gpio_pin_configure(sense_en, SENSE_EN_PIN, GPIO_DIR_OUT);
    gpio_pin_write(sense_en, SENSE_EN_PIN, 0);
    
    if(a_dev == 0)
        adc_moisture_dev = device_get_binding(ADC_DEVICE_NAME);
    else
        adc_moisture_dev = a_dev;

    if(adc_moisture_dev == 0)
        return -1;

    init_sense();
    stop_sense();

    return 0;
}

int moisture_read_value() {
    int value_low = moisture_read_low_value();
    if(value_low < 0)
        return -1;

    int value_high = moisture_read_high_value();
    if(value_high < 0)
        return -2;

    return value_high - value_low;
}

int moisture_read_low_value() {
    int value = 0;

    start_sense();
    if(adc_channel_setup(adc_moisture_dev, &adc_low_cfg) < 0)
        return -1;
    adc_moisture_sequence.channels = BIT(adc_low_cfg.channel_id);

    k_busy_wait(SENSE_TIME);
    value = adc_read(adc_moisture_dev, &adc_moisture_sequence);

    stop_sense();
    if(value < 0)
        return -2;

    return m_sample_buffer[0];
}

int moisture_read_high_value() {
    int value = 0;

    start_sense();
    if(adc_channel_setup(adc_moisture_dev, &adc_high_cfg) < 0)
        return -1;
    adc_moisture_sequence.channels = BIT(adc_high_cfg.channel_id);

    k_busy_wait(SENSE_TIME);
    value = adc_read(adc_moisture_dev, &adc_moisture_sequence);

    stop_sense();
    if(value < 0)
        return -2;

    return m_sample_buffer[0];
}

#ifdef CONFIG_SENSE_INTERVAL
#define SENSE_INTERVAL K_SECONDS(CONFIG_SENSE_INTERVAL)
#else
#define SENSE_INTERVAL K_SECONDS(10)
#endif

#ifdef CONFIG_MEASUREMENTS_SIZE
#define MEASUREMENTS_SIZE CONFIG_MEASUREMENTS_SIZE
#else
#define MEASUREMENTS_SIZE  480
#endif

#define LED_PORT "GPIO_0"
#define LED_P    6
#define LED_N    4

#define BATTERY_EN_PORT "GPIO_0"
#define BATTERY_EN_PIN  17

#define ONEWIRE_PORT "GPIO_0"
#define ONEWIRE_PIN  14

#define DS18B20_EN_PORT "GPIO_0"
#define DS18B20_EN_PIN  13

#define SENSE_EN_PORT "GPIO_0"
#define SENSE_EN_PIN  7

#define SENSE_EXCITATION_PORT "GPIO_0"
#define SENSE_EXCITATION_PIN  10

#define SENSE_LOW_PIN   NRF_ADC_CONFIG_INPUT_3   //P0.02/AIN3
#define SENSE_HIGH_PIN  NRF_ADC_CONFIG_INPUT_4   //P0.03/AIN4

#define ADC_DEVICE_NAME         DT_ADC_0_NAME
#define ADC_RESOLUTION          10
#define ADC_GAIN                ADC_GAIN_1_3
#define ADC_REFERENCE           ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME    ADC_ACQ_TIME_DEFAULT
#define BATTERY_SENSE_PIN       NRF_ADC_CONFIG_INPUT_6  // P0.05/GPIO5/ANALOG6

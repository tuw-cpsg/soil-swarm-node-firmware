#include "onewire.h"

#include <zephyr.h>

#include "settings.h"
#include <device.h>
#include <gpio.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>

#define WRITE_WAIT 1

struct device *onewire_dev;

struct gpio_nrfx_data {
	sys_slist_t callbacks;

	/* Mask holding information about which pins have been configured to
	 * trigger interrupts using gpio_nrfx_config function.
	 */
	u32_t pin_int_en;

	/* Mask holding information about which pins have enabled callbacks
	 * using gpio_nrfx_enable_callback function.
	 */
	u32_t int_en;

	u32_t active_level;
	u32_t trig_edge;
	u32_t double_edge;
	u32_t inverted;
};

struct gpio_nrfx_cfg {
	NRF_GPIO_Type *port;
	u8_t port_num;
};

static u8_t pin;

static struct gpio_nrfx_data *data;
static struct gpio_nrfx_cfg *cfg;

static nrf_gpio_pin_pull_t pull;
static nrf_gpio_pin_dir_t dir;
static nrf_gpio_pin_input_t input;
static nrf_gpio_pin_drive_t drive;

static inline struct gpio_nrfx_data *get_port_data(struct device *port)
{
	return port->driver_data;
}

static inline const struct gpio_nrfx_cfg *get_port_cfg(struct device *port)
{
	return port->config->config_info;
}

static inline void clear_bit()
{
	//gpio_pin_configure(onewire_dev, ONEWIRE_PIN, GPIO_DIR_OUT);
	//gpio_pin_write(onewire_dev, ONEWIRE_PIN, 0);
	nrf_gpio_cfg_output(ONEWIRE_PIN);
	nrf_gpio_pin_clear(ONEWIRE_PIN);
}

static inline void set_bit()
{
	//gpio_pin_configure(onewire_dev, ONEWIRE_PIN, GPIO_DIR_IN | GPIO_PUD_PULL_UP);
	//gpio_pin_write(onewire_dev, ONEWIRE_PIN, 1);

	nrf_gpio_pin_set(ONEWIRE_PIN);
}

static inline s8_t get_bit()
{
	int ret = 0;
	u32_t value = 0;
	if((ret = gpio_pin_read(onewire_dev, ONEWIRE_PIN, &value)) != 0)
		return (s8_t) ret;
	
	return value != 0;
}

__STATIC_INLINE void set_input(void)
{
	//nrf_gpio_cfg(NRF_GPIO_PIN_MAP(cfg->port_num, ONEWIRE_PIN),
	//        dir, input, pull, drive, NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_cfg(
			ONEWIRE_PIN,
			NRF_GPIO_PIN_DIR_INPUT,
			NRF_GPIO_PIN_INPUT_CONNECT,
			NRF_GPIO_PIN_NOPULL,
			//NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_S0S1,
			NRF_GPIO_PIN_NOSENSE);
}

__STATIC_INLINE void set_output_high(void)
{
	nrf_gpio_pin_set(ONEWIRE_PIN);
	nrf_gpio_cfg_output(ONEWIRE_PIN);
}

s8_t onewire_init(void)
{
	onewire_dev = device_get_binding(ONEWIRE_PORT);
	if(onewire_dev == NULL)
		return -1;

	s8_t ret = gpio_pin_configure(onewire_dev, ONEWIRE_PIN, GPIO_DIR_OUT);
	if(ret != 0)
		return ret;

	pin = ONEWIRE_PIN;

	data = get_port_data(onewire_dev);
	cfg = get_port_cfg(onewire_dev);

	pull = NRF_GPIO_PIN_PULLUP;
	dir = NRF_GPIO_PIN_DIR_INPUT;
	input = NRF_GPIO_PIN_INPUT_DISCONNECT;

	switch ((GPIO_DIR_IN | GPIO_PUD_PULL_UP) & (GPIO_DS_LOW_MASK | GPIO_DS_HIGH_MASK)) {
		case GPIO_DS_DFLT_LOW | GPIO_DS_DFLT_HIGH:
			drive = NRF_GPIO_PIN_S0S1;
			break;
		case GPIO_DS_DFLT_LOW | GPIO_DS_ALT_HIGH:
			drive = NRF_GPIO_PIN_S0H1;
			break;
		case GPIO_DS_DFLT_LOW | GPIO_DS_DISCONNECT_HIGH:
			drive = NRF_GPIO_PIN_S0D1;
			break;

		case GPIO_DS_ALT_LOW | GPIO_DS_DFLT_HIGH:
			drive = NRF_GPIO_PIN_H0S1;
			break;
		case GPIO_DS_ALT_LOW | GPIO_DS_ALT_HIGH:
			drive = NRF_GPIO_PIN_H0H1;
			break;
		case GPIO_DS_ALT_LOW | GPIO_DS_DISCONNECT_HIGH:
			drive = NRF_GPIO_PIN_H0D1;
			break;

		case GPIO_DS_DISCONNECT_LOW | GPIO_DS_DFLT_HIGH:
			drive = NRF_GPIO_PIN_D0S1;
			break;
		case GPIO_DS_DISCONNECT_LOW | GPIO_DS_ALT_HIGH:
			drive = NRF_GPIO_PIN_D0H1;
			break;

		default:
			return -EINVAL;
	}

	SetSpeed(1);
	set_input();
	gpio_pin_write(onewire_dev, ONEWIRE_PIN, 0);

	while(0)
	{
		set_bit();
		k_busy_wait(5);
		clear_bit();
		k_busy_wait(0);
	}

	return 0;
}

void OWDepower(void)
{
	clear_bit();
}

void OWPower(void)
{
	set_input();
}

// 'tick' values
int A,B,C,D,E,F,G,H,I,J;

//-----------------------------------------------------------------------------
// Set the 1-Wire timing to 'standard' (standard=1) or 'overdrive' (standard=0).
//
void SetSpeed(u8_t standard)
{
	// Adjust tick values depending on speed
	if (standard)
	{
		// Standard Speed
		A =   6;
		B =  64;
		C =  60;
		D =  10;
		E =   9;
		F =  55;
		G =   0;// - 4;
		H = 480;
		I =  70;
		J = 410;
	}
	else
	{
		// Overdrive Speed
		A = 1.5;
		B = 7.5;
		C = 7.5;
		D = 2.5;
		E = 0.75;
		F = 7;
		G = 2.5;
		H = 70;
		I = 8.5;
		J = 40;
	}
}


//-----------------------------------------------------------------------------
// Generate a 1-Wire reset, return 1 if no presence detect was found,
// return 0 otherwise.
// (NOTE: Does not handle alarm presence from DS2404/DS1994)
//
s16_t OWTouchReset(void)
{
	s16_t result;

	gpio_pin_configure(onewire_dev, ONEWIRE_PIN, GPIO_DIR_IN | GPIO_PUD_PULL_UP);
	u8_t i=0;
	while(get_bit() != 1)
	{
		i++;
		if(i > 125)
			return -1;
		k_busy_wait(2);
	}


	gpio_pin_configure(onewire_dev, ONEWIRE_PIN, GPIO_DIR_OUT);
	k_busy_wait(G);
	// Drives DQ low
	clear_bit();
	k_busy_wait(H);

	// Releases the bus
	set_input();
	k_busy_wait(I - 4);
	result = get_bit() & 0x01; // Sample the bit value from the slave

	k_busy_wait(J - 24); // Complete the time slot and 10us recovery
	set_bit();

	return result; // Return sample presence pulse result
}

//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit(u8_t bit)
{
	u32_t key;
	if (bit != 0)
	{
		// Write '1' bit
		key = irq_lock();
		k_busy_wait(WRITE_WAIT);
		clear_bit(); // Drives DQ low
		k_busy_wait(A);
		set_bit(); // Releases the bus
		irq_unlock(key);
		k_busy_wait(B); // Complete the time slot and 10us recovery
	}
	else
	{
		// Write '0' bit
		key = irq_lock();
		k_busy_wait(WRITE_WAIT);
		clear_bit(); // Drives DQ low
		k_busy_wait(C);
		set_bit(); // Releases the bus
		irq_unlock(key);
		k_busy_wait(D);
	}
}

//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
u8_t OWReadBit(void)
{
	u8_t result;
	u32_t key;

	key = irq_lock();
	clear_bit(); // Drives DQ low
	k_busy_wait(A); // 2.2us
	set_input();
	k_busy_wait(E);
	result = get_bit(); // Sample the bit value from the slave
	irq_unlock(key);
	k_busy_wait(F); // Complete the time slot and 10us recovery

	return result;
}

//-----------------------------------------------------------------------------
// Write 1-Wire data byte
//
void OWWriteByte(u8_t data, u8_t power)
{
	// Loop to write each bit in the byte, LS-bit first
	for (u8_t loop = 0; loop < 8; loop++) {
		OWWriteBit(data & 0x01);

		// shift the data byte for the next bit
		data >>= 1;

	}

	if (power) {
		set_output_high();
	}
}

//-----------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
u8_t OWReadByte(void)
{
	u8_t result=0;

	for (u8_t loop = 0; loop < 8; loop++) {
		// shift the result to get it ready for the next bit
		result >>= 1;

		// if result is one, then set MS bit
		if (OWReadBit())
			result |= 0x80;
	}

	return result;
}

//-----------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
u8_t OWTouchByte(u8_t data)
{
	u8_t result=0;

	for (u8_t loop = 0; loop < 8; loop++) {
		// shift the result to get it ready for the next bit
		result >>= 1;

		// If sending a '1' then read a bit else write a '0'
		if (data & 0x01) {
			if (OWReadBit())
				result |= 0x80;
		}
		else
			OWWriteBit(0);

		// shift the data byte for the next bit
		data >>= 1;
	}
	return result;
}

//-----------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void OWBlock(u8_t *data, u16_t data_len)
{
	for (u16_t loop = 0; loop < data_len; loop++)
	{
		data[loop] = OWTouchByte(data[loop]);
	}
}

//-----------------------------------------------------------------------------
// Set all devices on 1-Wire to overdrive speed. Return '1' if at least one
// overdrive capable device is detected.
//
s16_t OWOverdriveSkip(u8_t *data, u16_t data_len)
{
	// set the speed to 'standard'
	SetSpeed(1);

	// reset all devices
	if (OWTouchReset()) // Reset the 1-Wire bus
		return 0; // Return if no devices found

	// overdrive skip command
	OWWriteByte(0x3C, 0);

	// set the speed to 'overdrive'
	SetSpeed(0);

	// do a 1-Wire reset in 'overdrive' and return presence result
	return OWTouchReset();
}


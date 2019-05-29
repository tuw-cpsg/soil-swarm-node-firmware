/* main.c - Application main entry point */

/*
 *
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <misc/byteorder.h>
#include <logging/log.h>
#include <settings/settings.h>
#include <device.h>
#include <gpio.h>
#include <soc.h>

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "bas.h"
#include "cts.h"
#include "ess.h"
#include "led.h"
#include "battery.h"
#include "moisture.h"
#include "onewire.h"
#include "ds18b20.h"

#include "settings.h"

#include <nrfx/hal/nrf_radio.h>

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
#endif

#ifdef CONFIG_DEBUG
#define SENSE_INTERVAL 15		// 900 ^= 15min
#define MEASUREMENTS_SIZE  5	// one measurement per 15 min -> 96 / day
#else
#define SENSE_INTERVAL 900		// 900 ^= 15min
#define MEASUREMENTS_SIZE 960	//1344 	// one measurement per 15 min -> 96 / day
#endif

#define BT_LE_ADV_CONN_NAME_ID BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
                         BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_IDENTITY, \
                         BT_GAP_ADV_SLOW_INT_MIN, \
                         BT_GAP_ADV_SLOW_INT_MAX)

// BT_GAP_ADV_SLOW_INT_MIN = 0x0640 ^= 1s
// BT_GAP_ADV_SLOW_INT_MAX = 0x0780 ^= 1.2s
#define BT_LE_ADV_NCONN_NAME_ID BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_NAME | \
						 BT_LE_ADV_OPT_USE_IDENTITY, \
                         BT_GAP_ADV_SLOW_INT_MIN, \
                         BT_GAP_ADV_SLOW_INT_MAX)

LOG_MODULE_REGISTER(main);

void create_device_list(void);

struct {
	u32_t timestamp;
	u16_t battery;
	u16_t moisture;
	s16_t temperature;
} measurements[MEASUREMENTS_SIZE];

s16_t buf_start = 0;
s16_t buf_end   = 0;
static bool is_client_connected  = false;
bool is_sync_enabled = false;

extern u8_t bas_notify_enabled;
extern u8_t cts_notify_enabled;
extern u8_t ess_temp_notify_enabled;
extern u8_t ess_humi_notify_enabled;

//struct bt_le_adv_param *adv_params_conn = BT_LE_ADV_CONN_NAME_ID;
//struct bt_le_adv_param *adv_params_nconn = BT_LE_ADV_NCONN_NAME_ID;
struct bt_le_adv_param *adv_params = BT_LE_ADV_CONN_NAME_ID;

enum {
	NO_DATA_AVAILABLE = 0,
	DATA_AVAILABLE = BIT(0)
};

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  ec60de83-4b7e-4c75-96c9-2f4e76617a7e
 * Major: 102
 * Minor[1]: Status Flags
 * Minor[0]: SW Version
 * RSSI:  -56 dBm
 */
static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
                      0x4c, 0x00, /* Apple */
                      0x02, 0x15, /* iBeacon */
                      0xec, 0x60, 0xde, 0x83, /* UUID[15..12] */
                      0x4b, 0x7e, /* UUID[11..10] */
                      0x4c, 0x75, /* UUID[9..8] */
                      0x96, 0xc9, /* UUID[7..6] */
                      0x2f, 0x4e, 0x76, 0x61, 0x7a, 0x7e, /* UUID[5..0] */
                      0x00, 0x66, /* Major */
                      0x00, 0x66, /* Minor */
                      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};

void set_beacon_flags(u8_t flags) {
	*((u8_t *)(ad[1].data + 22)) = flags;
}

u8_t get_beacon_flags(void) {
	return *(ad[1].data + 22);
}

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}

	is_client_connected = true;
	bt_le_adv_stop();
	LOG_INF("Connected\n");
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	is_client_connected = false;
	bas_notify_enabled = 0;
	cts_notify_enabled = 0;
	ess_temp_notify_enabled = 0;
	ess_humi_notify_enabled = 0;

	//adv_params = adv_params_nconn;
	if (buf_start == buf_end) {
		set_beacon_flags(NO_DATA_AVAILABLE);
	}
	int err = bt_le_adv_start(adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
    LOG_INF("Advertising successfully started");
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");

	bas_init();
	cts_init();
    ess_init();
    
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	set_beacon_flags(NO_DATA_AVAILABLE);
	err = bt_le_adv_start(adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
    LOG_INF("Advertising successfully started");
}

void simblee_init(void)
{
    // initialize proximityMode
    NRF_GPIO->PIN_CNF[31] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                          | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                          | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                          | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                          | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    NRF_GPIO->OUTCLR = (1UL << 31);

    // disable dc to dc
    NRF_POWER->DCDCEN = 0;
    NRF_POWER->TASKS_CONSTLAT = 0;
	NRF_POWER->TASKS_LOWPWR = 1;
}

void sync_data(struct bt_conn *conn)
{
	if (!bas_notify_enabled
			|| !cts_notify_enabled
			|| !ess_temp_notify_enabled
			|| !ess_humi_notify_enabled
			|| !is_client_connected)
		return;

	if (buf_start == buf_end) {
		//bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	/* timestamp */
	cts_notify(measurements[buf_start].timestamp);
	/* battery level */
	bas_notify(measurements[buf_start].battery);
	/* environment notification */
	ess_notify(measurements[buf_start].temperature,
			measurements[buf_start].moisture);

	buf_start = (buf_start + 1) % MEASUREMENTS_SIZE;
}

#ifdef CONFIG_DEBUG
void print_device_list(void) {
	static struct device *pm_device_list;
	int count;
	u32_t power_state;

	device_list_get(&pm_device_list, &count);

	for(int i = 0; i < count; i++)
	{
		device_pm_enable(&pm_device_list[i]);
		device_get_power_state(&pm_device_list[i], &power_state);
		LOG_INF("device: %s / status: %u", pm_device_list[i].config->name, power_state);
	}
}
#endif

void main(void)
{
    simblee_init();

#ifdef CONFIG_DEBUG
	print_device_list();
#endif
   
	int err = 0;

	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

    bt_conn_cb_register(&conn_callbacks);

    struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

    int ret = 0;

    //led_init();
    //led_off();

	/*  */
    if(battery_init(adc_dev) < 0)
        LOG_ERR("Battery init failed");
    if((ret = moisture_init(adc_dev)) < 0)
        LOG_ERR("Moisture init failed: %i", ret);
    if((ret = onewire_init()) < 0)
        LOG_ERR("Cannot initialize 1-wire: %i", ret);
    else if((ret = ds18b20_init()) < 0)
        LOG_ERR("Cannot initialize DS18B20: %i", ret);
	/*  */

    u32_t entry = k_uptime_get_32();
    u32_t exit = entry;

	while (1) {

		k_sleep(SENSE_INTERVAL * MSEC_PER_SEC - (exit - entry));

#ifdef CONFIG_DEBUG
		print_device_list();
#endif

        entry = k_uptime_get_32();

        //led_toggle(); //continue;
        //exit = k_uptime_get_32(); continue;

		/* measure temperature */
        ds18b20_enable();
		k_sleep(1);
        s16_t temp = ds18b20_measure_temp();
        if(temp == 0) {
			k_sleep(750);
            temp = ds18b20_read_temp();
		}
        else
            temp = -1001;
        ds18b20_disable();

		/* measure moisture */
        s16_t moisture = moisture_read_value();

		measurements[buf_end].timestamp		= entry + 750;
		measurements[buf_end].battery		= (u16_t)battery_read_value();
		measurements[buf_end].moisture		= (u16_t)moisture;
		measurements[buf_end].temperature	= (s16_t)temp;

		LOG_INF("Timestamp:   %u", measurements[buf_end].timestamp);
        LOG_INF("Battery:     %u", measurements[buf_end].battery);
        LOG_INF("Temperature: %i (%x)", measurements[buf_end].temperature / 16,
				 temp);
        LOG_INF("Moisture:    %u", measurements[buf_end].moisture);

		buf_end = (buf_end + 1) % MEASUREMENTS_SIZE;

		if (get_beacon_flags() == NO_DATA_AVAILABLE) {
			bt_le_adv_stop();

			set_beacon_flags(DATA_AVAILABLE);
			int err = bt_le_adv_start(adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
			if (err) {
				//adv_params = adv_params_nconn;
				set_beacon_flags(NO_DATA_AVAILABLE);
				LOG_ERR("Advertising failed to start (err %d)", err);
			}
		}

		if (buf_end == buf_start) {
			/* TODO resort buffer */
			buf_start = (buf_start + 1) % MEASUREMENTS_SIZE;
		}

        exit = k_uptime_get_32();

#ifdef CONFIG_DEBUG
		LOG_ERR("loop end");
		print_device_list();
#endif
	}
}

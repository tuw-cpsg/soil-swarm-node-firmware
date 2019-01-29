/* main.c - Application main entry point */

/*
 *
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <logging/sys_log.h>
#include <settings/settings.h>
#include <device.h>
#include <gpio.h>

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "bas.h"
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

#define BT_LE_ADV_CONN_NAME_ID BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
                         BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_IDENTITY, \
                         BT_GAP_ADV_FAST_INT_MIN_1, \
                         BT_GAP_ADV_FAST_INT_MAX_2)

void create_device_list(void);

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  ec60de83-4b7e-4c75-96c9-2f4e76617a7e
 * Major: 102
 * Minor: 101
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
                      0x00, 0x65, /* Minor */
                      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		SYS_LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	SYS_LOG_INF("Bluetooth initialized");

	bas_init();
    ess_init();
    
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME_ID, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		SYS_LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
    SYS_LOG_INF("Advertising successfully started");
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
}

void main(void)
{
    //simblee_init();
    
	int err = 0;
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

    create_device_list();

    struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

    int ret = 0;

    led_init();
    SYS_LOG_DBG("CREAT_DEVICE_LIST");
    led_off();

    if(battery_init(adc_dev) < 0)
        printk("Battery init failed\n");
    if((ret = moisture_init(adc_dev)) < 0)
        printk("Moisture init failed: %i\n", ret);
    if((ret = onewire_init()) < 0)
        printk("Cannot initialize 1-wire: %i\n", ret);
    else if((ret = ds18b20_init()) < 0)
        printk("Cannot initialize DS18B20: %i\n", ret);

    s64_t entry = k_uptime_get();
    s64_t exit = entry;

	while (1) {
        printk("Entering service loop\n");

		k_sleep(5 * MSEC_PER_SEC - (exit - entry));

        entry = k_uptime_get();

        //led_toggle(); continue;

		/* battery level */
		bas_notify();
        printk("Battery: %i\n", battery_read_value());

		/* measure temperature */
        ds18b20_enable();
		k_sleep(10);
        int temp = ds18b20_measure_temp();
		k_sleep(900);
        if(temp == 0)
            temp = ds18b20_read_temp();
        else
            temp = -1001;
        ds18b20_disable();

		/* measure moisture */
        int16_t moisture = moisture_read_value();

        /* environment notification */
        ess_notify(temp, moisture);

        printk("Temperature: %i (%x)\n", temp/16, temp);
        printk("Moisture:  %i\n", moisture);

        exit = k_uptime_get();
    }
}

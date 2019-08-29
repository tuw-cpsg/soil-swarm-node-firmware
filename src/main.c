/* main.c - Application main for soil-swarm-node */

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

#include "config.h"

#include <nrfx/hal/nrf_radio.h>

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
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
u8_t prepare_beacon_flags(void);

#define STACK_SIZE 1024
#define PRIORITY 5

K_THREAD_STACK_DEFINE(measure_loop_stack, STACK_SIZE);
struct k_thread measure_loop_data;
k_tid_t measure_loop_tid;

K_THREAD_STACK_DEFINE(disconnect_timeout_stack, 32);
struct k_thread disconnect_timeout_data;
k_tid_t disconnect_timeout_tid;

void disconnect_timeout(void);

extern volatile struct time_sync remote_ts;

struct {
	u32_t timestamp;
	u16_t battery;
	u16_t moisture;
	s16_t temperature;
} measurements[MEASUREMENTS_SIZE];

s16_t buf_start = 0;
s16_t buf_end   = 0;
u32_t sense_interval_mult = 1;

struct k_sem measurements_sem;

static bool is_client_connected  = false;
bool is_sync_enabled = false;
struct bt_conn *client_connection = 0;
u32_t time_connected = 0;

u16_t num_connected = 0;

extern u8_t bas_notify_enabled;
extern u8_t cts_notify_enabled;
extern u8_t ess_temp_notify_enabled;
extern u8_t ess_humi_notify_enabled;

//struct bt_le_adv_param *adv_params_conn = BT_LE_ADV_CONN_NAME_ID;
//struct bt_le_adv_param *adv_params_nconn = BT_LE_ADV_NCONN_NAME_ID;
struct bt_le_adv_param *adv_params = BT_LE_ADV_CONN_NAME_ID;

enum {
	NO_DATA_AVAILABLE = 0,
	DATA_AVAILABLE = BIT(0),
	TIME_SYNC_NEEDED = BIT(1)
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

void start_advertising() {
	int err = 0, err_cnt = 0;
	do {
		err = bt_le_adv_start(adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
		if(err == 0 || err_cnt > 3)
			break;
		err_cnt++;
		k_sleep(15 * MSEC_PER_SEC);
	} while(err);

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
	LOG_INF("Advertising successfully started");
}

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}

	is_client_connected = true;
	client_connection = conn;
	bt_le_adv_stop();
	disconnect_timeout_tid = k_thread_create(&disconnect_timeout_data,
			disconnect_timeout_stack,
			K_THREAD_STACK_SIZEOF(disconnect_timeout_stack),
			disconnect_timeout,
			NULL, NULL, NULL, 1, 0, 300000);
	num_connected++;
	LOG_INF("Connected (%u times)", num_connected);
}

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

	set_beacon_flags(prepare_beacon_flags());
	start_advertising();
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);

	if (disconnect_timeout_tid)
	{
		k_wakeup(disconnect_timeout_tid);
	}

	if (client_connection)
	{
		bt_conn_unref(client_connection);
		client_connection = 0;
	}

	bas_notify_enabled = 0;
	cts_notify_enabled = 0;
	ess_temp_notify_enabled = 0;
	ess_humi_notify_enabled = 0;

	set_beacon_flags(prepare_beacon_flags());
	start_advertising();
}

static struct bt_conn_cb conn_callbacks = {
		.connected = connected,
		.disconnected = disconnected,
};

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
	//NRF_POWER->TASKS_CONSTLAT = 0;
	//NRF_POWER->TASKS_LOWPWR = 1;
}

void sync_data(struct bt_conn *conn)
{
	if(!is_client_connected)
	{
		bt_conn_disconnect(client_connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	if (buf_start == buf_end) {
		sense_interval_mult = 1;
		//bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	/* timestamp */
	if(cts_notify_enabled)
	{
		cts_notify(measurements[buf_start].timestamp);
		k_sleep(100);
	}

	/* battery level */
	if(bas_notify_enabled)
	{
		bas_notify(measurements[buf_start].battery);
		k_sleep(100);
	}

	/* environment notification */
	if(ess_temp_notify_enabled) {
		ess_notify_temp(measurements[buf_start].temperature);
		k_sleep(100);
	}
	if(ess_humi_notify_enabled) {
		ess_notify_humi(measurements[buf_start].moisture);
		k_sleep(100);
	}
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

u8_t prepare_beacon_flags(void)
{
	u8_t flags = 0;

	if(remote_ts.is_remotely_synced == 0)
		flags |= TIME_SYNC_NEEDED;
	if(buf_start != buf_end)
		flags |= DATA_AVAILABLE;

	return flags;
}

void check_and_reset_beacon_flags(void)
{
	u8_t flags = prepare_beacon_flags();
	if (get_beacon_flags() != flags) {
		bt_le_adv_stop();

		set_beacon_flags(flags);
		start_advertising();
	}
}

u32_t time_until(struct time_sync time_sync) {
	u32_t now = (u32_t)(k_uptime_get() % (SENSE_INTERVAL * MSEC_PER_SEC));
	u32_t synced = time_sync.time_synced % (SENSE_INTERVAL * MSEC_PER_SEC);
	u32_t ret = 0;

	if((now + time_sync.remote_time) > ((SENSE_INTERVAL * MSEC_PER_SEC) + synced))
		ret = (SENSE_INTERVAL * MSEC_PER_SEC * (1 + sense_interval_mult)) + synced - now - time_sync.remote_time;
	else
		ret = (SENSE_INTERVAL * MSEC_PER_SEC * sense_interval_mult) + synced - now - time_sync.remote_time;

	LOG_DBG("time_until: %u = (%u) - %u + (%u - %u)",
			ret, (SENSE_INTERVAL * MSEC_PER_SEC * sense_interval_mult), time_sync.remote_time, synced, now);

	return ret;
}

void disconnect_timeout(void)
{
	/* if connection is still up after 15min, (auto-)disconnect */
	if (is_client_connected == true)
	{
		bt_conn_disconnect(client_connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

	disconnect_timeout_tid = 0;
}

void measure_loop(void)
{
	u8_t buf_full = 0;

	while(1) {
		u32_t time_to_sleep = time_until(remote_ts);
		while(time_to_sleep > 300000)
		{
			k_sleep(300000);
			time_to_sleep = time_until(remote_ts);
		}
		k_sleep(time_to_sleep);

		LOG_INF("enter loop %x %x", buf_start, buf_end);

		/* get temperature */
		ds18b20_enable();
		k_sleep(1);
		s16_t temp = -1;
		temp = ds18b20_measure_temp();
		if(temp == 0) {
			k_sleep(750);
			temp = ds18b20_read_temp();
		}
		else
			temp = -1001;

		ds18b20_disable();
		k_sleep(1);

		/* get moisture */
		s16_t moisture = moisture_read_value();

		/* get timestamp */
		u64_t timestamp = k_uptime_get();

		if (buf_full) {
			k_sem_take(&measurements_sem, K_FOREVER);
			for (s16_t i = 0; i < MEASUREMENTS_SIZE / 2; i++) {
				s16_t buf_dst = (buf_start + i) % MEASUREMENTS_SIZE;
				s16_t buf_src = (buf_start + i * 2) % MEASUREMENTS_SIZE;
				measurements[buf_dst].timestamp		= measurements[buf_src].timestamp;
				measurements[buf_dst].battery		= measurements[buf_src].battery;
				measurements[buf_dst].moisture		= measurements[buf_src].moisture;
				measurements[buf_dst].temperature	= measurements[buf_src].temperature;
			}
			buf_end = (buf_start + MEASUREMENTS_SIZE / 2 + 1) % MEASUREMENTS_SIZE;
			k_sem_give(&measurements_sem);
			sense_interval_mult *= 2;
			buf_full = 0;
		}

		k_sem_take(&measurements_sem, K_FOREVER);
		measurements[buf_end].timestamp		= (u32_t)timestamp;
		measurements[buf_end].battery		= (u16_t)battery_read_value();
		measurements[buf_end].moisture		= (u16_t)moisture;
		measurements[buf_end].temperature	= (s16_t)temp;

		LOG_INF("Timestamp:   %u", measurements[buf_end].timestamp);
		LOG_INF("Battery:     %u", measurements[buf_end].battery);
		LOG_INF("Temperature: %i (%x)", measurements[buf_end].temperature / 16,
				temp);
		LOG_INF("Moisture:    %u", measurements[buf_end].moisture);

		buf_end = (buf_end + 1) % MEASUREMENTS_SIZE;
		k_sem_give(&measurements_sem);

		buf_full = (buf_end == buf_start);

		check_and_reset_beacon_flags();
	}
}

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

	led_init();
	led_off();

	if(battery_init(adc_dev) < 0)
		LOG_ERR("Battery init failed");
	if((ret = moisture_init(adc_dev)) < 0)
		LOG_ERR("Moisture init failed: %i", ret);
	if((ret = onewire_init()) < 0)
		LOG_ERR("Cannot initialize 1-wire: %i", ret);
	else if((ret = ds18b20_init()) < 0)
		LOG_ERR("Cannot initialize DS18B20: %i", ret);

	while (remote_ts.is_remotely_synced == 0) {
		check_and_reset_beacon_flags();
		k_sleep(100);
	}

	k_sem_init(&measurements_sem, 1, 1);

	measure_loop_tid = k_thread_create(&measure_loop_data, measure_loop_stack,
			K_THREAD_STACK_SIZEOF(measure_loop_stack),
			measure_loop,
			NULL, NULL, NULL,
			PRIORITY + 1, 0, K_NO_WAIT);
}

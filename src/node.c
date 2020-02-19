#include "node.h"

#include <zephyr.h>
#include <logging/log.h>
#include <device.h>

#include <kernel.h>

#include <drivers/counter.h>
#include <drivers/sensor.h>

#include "config.h"
#include "util.h"

#include "battery.h"
#include "moisture.h"
#include "onewire.h"
#include "ds18b20.h"

static struct measurement measurements[MEASUREMENTS_SIZE];

static s16_t buf_start = 0;
static s16_t buf_end   = 0;
static u32_t sense_interval_mult = 1;

extern u32_t cts_uptime_synced_s;
extern u32_t cts_ref_unix_sec;

K_SEM_DEFINE(measurements_sem, 1, 1);
K_SEM_DEFINE(loop_guard, 1, 1);

static void node_loop(struct k_timer *timer);

K_THREAD_STACK_DEFINE(node_loop_worker_stack_area, 1024);
struct k_thread node_loop_worker_data;
k_tid_t node_loop_worker_tid;

static void node_loop_worker(void *, void *, void *);

LOG_MODULE_REGISTER(node);

K_TIMER_DEFINE(loop_timer, node_loop, NULL);

u8_t node_initialize(void)
{
	int ret = 0;

	struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	if (battery_init(adc_dev) < 0) {
		LOG_ERR("Battery init failed");
	}
	if ((ret = moisture_init(adc_dev)) < 0) {
		LOG_ERR("Moisture init failed: %i", ret);
	}
	if ((ret = onewire_init()) < 0) {
		LOG_ERR("Cannot initialize 1-wire: %i", ret);
	}
	else if ((ret = ds18b20_init()) < 0) {
		LOG_ERR("Cannot initialize DS18B20: %i", ret);
	}

#ifdef CONFIG_DEBUG
	k_timer_start(&loop_timer, SENSE_INTERVAL, SENSE_INTERVAL);
#endif

	return ret;
}

void node_start_sensing(s32_t delay)
{
	k_timer_start(&loop_timer, delay, SENSE_INTERVAL);
}

static void node_loop(struct k_timer *timer)
{
	if (k_sem_take(&loop_guard, K_NO_WAIT) != 0)
		return;

	node_loop_worker_tid = k_thread_create(&node_loop_worker_data, node_loop_worker_stack_area,
			K_THREAD_STACK_SIZEOF(node_loop_worker_stack_area),
			node_loop_worker,
			NULL, NULL, NULL,
			5, 0, K_NO_WAIT);
}

static void node_loop_worker(void *p1, void *p2, void *p3)
{
	static u8_t buf_full = 0;

	LOG_INF("enter loop %x %x", buf_start, buf_end);

	/* get temperature */
	ds18b20_enable(K_FOREVER);
	k_sleep(1);
	s16_t temp = ds18b20_measure_temp();
	if(temp == 0) {
		k_sleep(750);
		temp = ds18b20_read_temp();
	}
	else {
		temp = -1001;
	}
	ds18b20_disable();

	s64_t ts = k_uptime_get();
	ts /= 1000;

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
	measurements[buf_end].timestamp		= (u32_t)ts;
	measurements[buf_end].battery		= util_battery_to_v(battery_read_value());
	measurements[buf_end].moisture		= util_humidity_to_ble(moisture_read_value());
	measurements[buf_end].temperature	= util_temperature_to_ble(temp);

	LOG_INF("Timestamp:   %u", measurements[buf_end].timestamp);
	LOG_INF("Battery:     %u", measurements[buf_end].battery);
	LOG_INF("Temperature: %3i.%02iC", measurements[buf_end].temperature / 100,
			measurements[buf_end].temperature % 100);
	LOG_INF("Moisture:    %3i.%02i%%", measurements[buf_end].moisture / 100,
			measurements[buf_end].moisture % 100);

	buf_end = (buf_end + 1) % MEASUREMENTS_SIZE;
	k_sem_give(&measurements_sem);

	buf_full = (buf_end == buf_start);

	k_sem_give(&loop_guard);
}

size_t node_buf_get_elem_size(void)
{
	struct measurement *meas = measurements;
	return sizeof(meas->timestamp)
			+ sizeof(meas->battery)
			+ sizeof(meas->moisture)
			+ sizeof(meas->temperature);
}

size_t node_buf_cpy_data(void *dst, size_t cnt)
{
	size_t i;
	u8_t *bdst = (u8_t *)dst;

	struct measurement *meas;

	k_sem_take(&measurements_sem, K_FOREVER);
	for (i = 0; i < cnt; i++)
	{
		if (((buf_start + i) % MEASUREMENTS_SIZE) == buf_end)
			break;

		meas = &measurements[buf_start + i];

		u32_t timestamp = meas->timestamp
				- cts_uptime_synced_s
				+ cts_ref_unix_sec;

		memcpy(bdst, &timestamp, sizeof(timestamp));
		bdst = bdst + sizeof(meas->timestamp);

		memcpy(bdst, &meas->battery, sizeof(meas->battery));
		bdst = bdst + sizeof(meas->battery);

		memcpy(bdst, &meas->temperature, sizeof(meas->temperature));
		bdst = bdst + sizeof(meas->temperature);

		memcpy(bdst, &meas->moisture, sizeof(meas->moisture));
		bdst = bdst + sizeof(meas->moisture);
	}
	k_sem_give(&measurements_sem);

	return i;
}

u8_t node_buf_get_finish(size_t cnt)
{
	k_sem_take(&measurements_sem, K_FOREVER);
	for(size_t i = 0; i < cnt; i++)
	{
		buf_start = (buf_start + 1) % MEASUREMENTS_SIZE;
		if (buf_start == buf_end) {
			k_sem_give(&measurements_sem);
			return 0;
		}
	}
	k_sem_give(&measurements_sem);

	LOG_INF("buf_start = %u", buf_start);

	return 1;
}

/** @file
 *  @brief CTS Service for soil-swarm-node
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "cts.h"
#include "config.h"

extern u16_t buf_start;
extern u16_t buf_end;
extern u32_t sense_interval_mult;
extern struct k_sem measurements_sem;
extern k_tid_t measure_loop_tid;

static struct bt_gatt_ccc_cfg  cts_ccc_cfg[BT_GATT_CCC_MAX] = {};
u8_t   cts_notify_enabled = 0;
u8_t   cts_time_synced = 0;
static u32_t time;

volatile struct time_sync remote_ts;

static void cts_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	cts_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	cts_time_synced = 0;
}

static ssize_t read_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	time = (u32_t)k_uptime_get();

	if(cts_time_synced)
	{
		k_sem_take(&measurements_sem, K_FOREVER);
		buf_start = (buf_start + 1) % MEASUREMENTS_SIZE;
		k_sem_give(&measurements_sem);
	}

	cts_time_synced = 1;

	k_sem_take(&measurements_sem, K_FOREVER);
	if(buf_start == buf_end)
	{
		sense_interval_mult = 1;
		time = 0;
	}
	k_sem_give(&measurements_sem);

	int err = bt_gatt_attr_read(conn, attr, buf, len, offset, &time,
				 sizeof(u32_t));

	if(time != 0)
	{
		k_sem_take(&measurements_sem, K_FOREVER);
		sync_data(conn);
		k_sem_give(&measurements_sem);
	}

	return err;
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, u16_t len, u16_t offset,
			u8_t flags)
{
	remote_ts.time_synced = (u32_t)k_uptime_get();
	u8_t *value = (u8_t *) &remote_ts.remote_time;

	if (offset + len > sizeof(remote_ts.time_synced)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	// get remote time (minutes in milliseconds of hour) relative to SENSE_INTERVAL
	remote_ts.remote_time = remote_ts.remote_time % (SENSE_INTERVAL * MSEC_PER_SEC);

	remote_ts.is_remotely_synced = 1;

	if (measure_loop_tid != 0)
	{
		k_wakeup(measure_loop_tid);
	}

	return len;
}

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_svc,
//static struct bt_gatt_attr cts_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
				   read_ct, write_ct, &time),
	BT_GATT_CCC(cts_ccc_cfg, cts_ccc_cfg_changed),
//};
);

//static struct bt_gatt_service cts_svc = BT_GATT_SERVICE(cts_attrs);

void cts_init(void)
{
	remote_ts.is_remotely_synced = 0;
	remote_ts.remote_time = 0;

	//bt_gatt_service_register(&cts_svc);
}

int cts_notify(u32_t _timestamp)
{
	if (!cts_notify_enabled) {
		return 0;
	}

	return bt_gatt_notify(NULL, &cts_svc.attrs[1], &_timestamp, sizeof(u32_t));
}

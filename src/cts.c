/** @file
 *  @brief BAS Service sample
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
	time = k_uptime_get_32();

	if(cts_time_synced)
		buf_start = (buf_start + 1) % MEASUREMENTS_SIZE;
	cts_time_synced = 1;

	if(buf_start == buf_end)
	{
		sense_interval_mult = 1;
		time = 0;
	}

	int err = bt_gatt_attr_read(conn, attr, buf, len, offset, &time,
				 sizeof(u32_t));

	if(time != 0)
		sync_data(conn);

	return err;
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, u16_t len, u16_t offset,
			u8_t flags)
{
	remote_ts.time_synced = k_uptime_get_32();
	u8_t *value = (u8_t *) &remote_ts.remote_time;

	if (offset + len > sizeof(remote_ts.time_synced)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	// get remote time (minutes in milliseconds of hour) relative to SENSE_INTERVAL
	remote_ts.remote_time = remote_ts.remote_time % (SENSE_INTERVAL * MSEC_PER_SEC);

	remote_ts.is_remotely_synced = 1;

	return len;
}

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
				   read_ct, write_ct, &time),
	BT_GATT_CCC(cts_ccc_cfg, cts_ccc_cfg_changed),
);

void cts_init(void)
{
	remote_ts.is_remotely_synced = 0;
	remote_ts.remote_time = 0;
}

int cts_notify(u32_t _timestamp)
{
	if (!cts_notify_enabled) {
		return 0;
	}

	return bt_gatt_notify(NULL, &cts_svc.attrs[1], &_timestamp, sizeof(u32_t));
}

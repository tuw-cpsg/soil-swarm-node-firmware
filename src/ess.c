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

static struct bt_gatt_ccc_cfg  temp_ccc_cfg[BT_GATT_CCC_MAX] = {};
static struct bt_gatt_ccc_cfg  humi_ccc_cfg[BT_GATT_CCC_MAX] = {};

u8_t ess_temp_notify_enabled, ess_humi_notify_enabled;
static int16_t temperature;
static int16_t humidity;

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	ess_temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static void humi_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	ess_humi_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
        void *buf, u16_t len, u16_t offset)
{
    ds18b20_enable();
	k_sleep(1);
    int16_t temperature = ds18b20_measure_temp();
    if (temperature == 0) {
    	k_sleep(750);
        temperature = ds18b20_read_temp();
	}
    else
        temperature = -1001;
    ds18b20_disable();

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature,
            sizeof(temperature));
}

static ssize_t read_humi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
    int16_t moisture = moisture_read_value();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &moisture,
				 sizeof(moisture));
}

/* ESS Declaration */
BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_temp, NULL, &temperature),
	BT_GATT_CCC(temp_ccc_cfg, temp_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_humi, NULL, &humidity),
	BT_GATT_CCC(humi_ccc_cfg, humi_ccc_cfg_changed),
);

void ess_init(void)
{
}

int ess_notify(s16_t temperature, u16_t humidity)
{
	int err = 0;
	if (ess_temp_notify_enabled) {
	    err = bt_gatt_notify(NULL, &ess_svc.attrs[2], &temperature, sizeof(temperature));
	}
	if(err != 0)
		return err;

	if (ess_humi_notify_enabled) {
	    err = bt_gatt_notify(NULL, &ess_svc.attrs[5], &humidity, sizeof(humidity));
	}

	return err;
}

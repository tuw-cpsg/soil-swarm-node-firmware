/** @file
 *  @brief BAS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "ds18b20.h"
#include "moisture.h"
#include "util.h"

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
        void *buf, u16_t len, u16_t offset);
static ssize_t read_humi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset);

/* ESS Declaration */
BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_temp, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_NONE),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_humi, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_NONE),
);

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
        void *buf, u16_t len, u16_t offset)
{
    ds18b20_enable(K_FOREVER);
    k_sleep(K_MSEC(1));
    s16_t temperature = ds18b20_measure_temp();
    if (temperature == 0) {
    	k_sleep(K_MSEC(750));
        temperature = ds18b20_read_temp();
	}
    else {
        temperature = -1001;
    }
    ds18b20_disable();

    temperature = util_temperature_to_ble(temperature);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temperature,
            sizeof(temperature));
}

static ssize_t read_humi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
    u16_t humidity = util_humidity_to_ble(moisture_read_value());

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &humidity,
				 sizeof(humidity));
}

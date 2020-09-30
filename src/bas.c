#include <zephyr.h>
#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "battery.h"

static ssize_t read_blvl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		void *buf, u16_t len, u16_t offset)
{
	u16_t value = (u16_t) battery_sample();// battery_level_pptt(battery_sample());

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
			sizeof(value));
}

/* Battery Service Declaration */
BT_GATT_SERVICE_DEFINE(bas_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_READ, read_blvl, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_NONE)
);

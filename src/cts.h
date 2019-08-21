/** @file
 *  @brief BAS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

volatile struct time_sync {
	u32_t remote_time;
	u8_t  is_remotely_synced;
	u32_t time_synced;
};

void cts_init(void);
int  cts_notify(u32_t _timestamp);

#ifdef __cplusplus
}
#endif

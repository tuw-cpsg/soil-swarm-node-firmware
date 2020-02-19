/*
 * Copyright (c) 2020 Christian Hirsch
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <hal/nrf_power.h>

static int rfd77101_soil_swarm_node_hardware_init(struct device *dev)
{
	ARG_UNUSED(dev);

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

	return 0;
}

SYS_INIT(rfd77101_soil_swarm_node_hardware_init, PRE_KERNEL_1,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

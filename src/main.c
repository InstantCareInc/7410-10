/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

int main(void)
{
	printk("Hello World via RTT\n");
	LOG_INF("Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}

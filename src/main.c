/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(main);

/* Get the node identifiers from the devicetree */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define BUZZER0_NODE DT_ALIAS(buzzer0)
#define BUZZER1_NODE DT_ALIAS(buzzer1)
#define BUZZER2_NODE DT_ALIAS(buzzer2)

/* Pull out the device from the node */
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec buzzer0_spec = GPIO_DT_SPEC_GET(BUZZER0_NODE, gpios);
static const struct gpio_dt_spec buzzer1_spec = GPIO_DT_SPEC_GET(BUZZER1_NODE, gpios);
static const struct gpio_dt_spec buzzer2_spec = GPIO_DT_SPEC_GET(BUZZER2_NODE, gpios);

int setup_gpio(void)
{
	if (!gpio_is_ready_dt(&led0_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&led1_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&buzzer0_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer0_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer1_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer1_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer2_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer2_spec, GPIO_OUTPUT_INACTIVE);

	return 0;
}

int main(void)
{
	LOG_INF("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (setup_gpio() < 0)
	{
		LOG_ERR("Failed to setup GPIO");
		return 0;
	}
	LOG_INF("LED0 device: %s ready=%d", led0_spec.port->name, device_is_ready(led0_spec.port));
	LOG_INF("LED1 device: %s ready=%d", led1_spec.port->name, device_is_ready(led1_spec.port));
	LOG_INF("SOUNDER0 device: %s ready=%d", buzzer0_spec.port->name, device_is_ready(buzzer0_spec.port));
	LOG_INF("SOUNDER1 device: %s ready=%d", buzzer1_spec.port->name, device_is_ready(buzzer1_spec.port));
	LOG_INF("SOUNDER2 device: %s ready=%d", buzzer2_spec.port->name, device_is_ready(buzzer2_spec.port));
	gpio_pin_set_dt(&led1_spec, 0);

	while (1)
	{
		// Wait here
	}

	return 0;
}

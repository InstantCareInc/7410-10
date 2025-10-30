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

/* Pull out the device from the node */
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

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

	return 0;
}

int main(void)
{
	LOG_INF("Hello World! %s\n", CONFIG_BOARD_TARGET);

	bool led_state = true;

	if (setup_gpio() < 0)
	{
		LOG_ERR("Failed to setup GPIO");
		return 0;
	}
	printk("LED0 GPIO: %s pin %d flags %x\n", led0_spec.port->name, led0_spec.pin, led0_spec.dt_flags);
	printk("LED1 GPIO: %s pin %d flags %x\n", led1_spec.port->name, led1_spec.pin, led1_spec.dt_flags);
	LOG_INF("LED0 device: %s ready=%d", led0_spec.port->name, device_is_ready(led0_spec.port));
	LOG_INF("LED1 device: %s ready=%d", led1_spec.port->name, device_is_ready(led1_spec.port));

	while (1)
	{
		if (gpio_pin_toggle_dt(&led0_spec) < 0)
		{
			return 0;
		}
		if (gpio_pin_toggle_dt(&led1_spec) < 0)
		{
			return 0;
		}
		led_state = !led_state;
		printk("LEDs are %s\n", led_state ? "ON" : "OFF");
		k_msleep(1000);
	}

	return 0;
}

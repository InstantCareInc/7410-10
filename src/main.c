#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>
#include <zephyr/app_version.h>
#include <stdio.h>

#include "leds.h"
#include "button.h"
#include "sounders.h"
#include "pressure_sensor.h"
#include "accelerometer.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("Button pressed!");
	led_red_toggle();
}

int main(void)
{
	int err = 0;
	err |= leds_init();
	err |= button_init(button_pressed);
	err |= sounders_init();
	err |= pressure_sensor_init();
	err |= accelerometer_init();

	if (err == 0) {
		LOG_INF("All I/O initialized successfully");
	} else {
		LOG_WRN("Some I/O failed to initialize. System may be in a degraded state.");
	}

	led_red_set(0); // Turn off red LED
	led_grn_set(0); // Turn off green LED

	err = data_ready_init();
	err = free_fall_init();

	while (1)
	{
		k_sleep(K_FOREVER);
	}

	return 0;
}

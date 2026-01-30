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
	printf("Button pressed! pins=0x%x\n", pins);
	led_red_toggle();
}

int main(void)
{
	int err;
	if (leds_init() < 0) {
		LOG_ERR("Failed to initalize LEDs");
		return 0;
	}

	if (button_init(button_pressed) < 0) {
		LOG_ERR("Failed to initalize button");
		return 0;
	}

	if (sounders_init() < 0) {
		LOG_ERR("Failed to initalize sounders");
		return 0;
	}

	if (pressure_sensor_init() < 0) {
		LOG_ERR("Failed to initialize pressure sensor");
		return 0;
	}

	if (accelerometer_init() < 0) {
		LOG_ERR("Failed to initialize accelerometer");
		// return 0;
	}

	err = led_red_set(0);

	printk("All I/O initalized successfully");
	while (1)
	{
		k_sleep(K_FOREVER);
	}

	return 0;
}

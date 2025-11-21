/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/sensor.h>

#include "ble_manager.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Get the node identifiers from the devicetree */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define BUZZER0_NODE DT_ALIAS(buzzer0)
#define BUZZER1_NODE DT_ALIAS(buzzer1)
#define BUZZER2_NODE DT_ALIAS(buzzer2)
#define BUTTON0_NODE DT_ALIAS(sw0)

#define ACCEL_NODE DT_ALIAS(accel0)
#define PRESS_NODE DT_ALIAS(press0)

#define BMP5_SEA_LEVEL_PRESSURE_PA 101325
/* ODR settings */
#define BMP5_ODR_240_HZ 0x00
#define BMP5_ODR_218_5_HZ 0x01
#define BMP5_ODR_199_1_HZ 0x02
#define BMP5_ODR_179_2_HZ 0x03
#define BMP5_ODR_160_HZ 0x04
#define BMP5_ODR_149_3_HZ 0x05
#define BMP5_ODR_140_HZ 0x06
#define BMP5_ODR_129_8_HZ 0x07
#define BMP5_ODR_120_HZ 0x08
#define BMP5_ODR_110_1_HZ 0x09
#define BMP5_ODR_100_2_HZ 0x0A
#define BMP5_ODR_89_6_HZ 0x0B
#define BMP5_ODR_80_HZ 0x0C
#define BMP5_ODR_70_HZ 0x0D
#define BMP5_ODR_60_HZ 0x0E
#define BMP5_ODR_50_HZ 0x0F
#define BMP5_ODR_45_HZ 0x10
#define BMP5_ODR_40_HZ 0x11
#define BMP5_ODR_35_HZ 0x12
#define BMP5_ODR_30_HZ 0x13
#define BMP5_ODR_25_HZ 0x14
#define BMP5_ODR_20_HZ 0x15
#define BMP5_ODR_15_HZ 0x16
#define BMP5_ODR_10_HZ 0x17
#define BMP5_ODR_05_HZ 0x18
#define BMP5_ODR_04_HZ 0x19
#define BMP5_ODR_03_HZ 0x1A
#define BMP5_ODR_02_HZ 0x1B
#define BMP5_ODR_01_HZ 0x1C
#define BMP5_ODR_0_5_HZ 0x1D
#define BMP5_ODR_0_250_HZ 0x1E
#define BMP5_ODR_0_125_HZ 0x1F

/* Oversampling for temperature and pressure */
#define BMP5_OVERSAMPLING_1X 0x00
#define BMP5_OVERSAMPLING_2X 0x01
#define BMP5_OVERSAMPLING_4X 0x02
#define BMP5_OVERSAMPLING_8X 0x03
#define BMP5_OVERSAMPLING_16X 0x04
#define BMP5_OVERSAMPLING_32X 0x05
#define BMP5_OVERSAMPLING_64X 0x06
#define BMP5_OVERSAMPLING_128X 0x07

/* IIR filter for temperature and pressure */
#define BMP5_IIR_FILTER_BYPASS 0x00
#define BMP5_IIR_FILTER_COEFF_1 0x01
#define BMP5_IIR_FILTER_COEFF_3 0x02
#define BMP5_IIR_FILTER_COEFF_7 0x03
#define BMP5_IIR_FILTER_COEFF_15 0x04
#define BMP5_IIR_FILTER_COEFF_31 0x05
#define BMP5_IIR_FILTER_COEFF_63 0x06
#define BMP5_IIR_FILTER_COEFF_127 0x07

/* This is used to enable IIR config, keep in mind that disabling IIR back in runtime is not supported yet */
#define BMP5_ATTR_IIR_CONFIG (SENSOR_ATTR_PRIV_START + 1u)
#define BMP5_ATTR_POWER_MODE (SENSOR_ATTR_PRIV_START + 2u)
#define BMP5_ATTR_ALTITUDE (SENSOR_ATTR_PRIV_START + 3u)
enum bmp5_powermode
{
	/*! Standby powermode */
	BMP5_POWERMODE_STANDBY,
	/*! Normal powermode */
	BMP5_POWERMODE_NORMAL,
	/*! Forced powermode */
	BMP5_POWERMODE_FORCED,
	/*! Continous powermode */
	BMP5_POWERMODE_CONTINOUS,
	/*! Deep standby powermode */
	BMP5_POWERMODE_DEEP_STANDBY
};

/* Pull out the device from the node */
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec buzzer0_spec = GPIO_DT_SPEC_GET(BUZZER0_NODE, gpios);
static const struct gpio_dt_spec buzzer1_spec = GPIO_DT_SPEC_GET(BUZZER1_NODE, gpios);
static const struct gpio_dt_spec buzzer2_spec = GPIO_DT_SPEC_GET(BUZZER2_NODE, gpios);
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET_OR(BUTTON0_NODE, gpios, {0});

const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);
const struct device *press_dev = DEVICE_DT_GET(PRESS_NODE);

static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printf("Button pressed! pins=0x%x\n", pins);
	gpio_pin_toggle_dt(&led1_spec);
}

int setup_gpio(void)
{
	if (!gpio_is_ready_dt(&led0_spec))
	{
		return -ENODEV;
	}
	LOG_INF("LED0 device: %s ready=%d\n", led0_spec.port->name, device_is_ready(led0_spec.port));
	gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&led1_spec))
	{
		return -ENODEV;
	}
	LOG_INF("LED1 device: %s ready=%d\n", led1_spec.port->name, device_is_ready(led1_spec.port));
	gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&buzzer0_spec))
	{
		return -ENODEV;
	}
	LOG_INF("SOUNDER0 device: %s ready=%d\n", buzzer0_spec.port->name, device_is_ready(buzzer0_spec.port));
	gpio_pin_configure_dt(&buzzer0_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer1_spec))
	{
		return -ENODEV;
	}
	LOG_INF("SOUNDER1 device: %s ready=%d\n", buzzer1_spec.port->name, device_is_ready(buzzer1_spec.port));
	gpio_pin_configure_dt(&buzzer1_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer2_spec))
	{
		return -ENODEV;
	}
	LOG_INF("SOUNDER2 device: %s ready=%d\n", buzzer2_spec.port->name, device_is_ready(buzzer2_spec.port));
	gpio_pin_configure_dt(&buzzer2_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&button0_spec))
	{
		return -ENODEV;
	}
	LOG_INF("BUTTON0 device: %s ready=%d\n", button0_spec.port->name, device_is_ready(button0_spec.port));
	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button_cb_data);

	if (!device_is_ready(press_dev))
	{
		LOG_INF("Pressure sensor device not ready\n");
		// return -ENODEV;
	}
	LOG_INF("Pressure sensor device: %s ready=%d\n", press_dev->name, device_is_ready(press_dev));

	if (!device_is_ready(accel_dev))
	{
		LOG_INF("Accelerometer device not ready\n");
		// return -ENODEV;
	}
	LOG_INF("Accelerometer device: %s ready=%d\n", accel_dev->name, device_is_ready(accel_dev));

	return 0;
}

int main(void)
{
	int err;
	printf("\n\r\n\rHello World! %s\n\r\n\r", CONFIG_BOARD_TARGET);
	k_msleep(1000);

	if (setup_gpio() < 0)
	{
		LOG_ERR("Failed to setup GPIO");
		return 0;
	}

	gpio_pin_set_dt(&led1_spec, 0);

	err = ble_manager_init();

	if (err)
	{
		LOG_ERR("BLE Manager init failed (err %d)\n", err);
		return -1;
	}

	struct sensor_value pressure_oversampling_rate = {BMP5_OVERSAMPLING_16X, 1};
	if (sensor_attr_set(press_dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &pressure_oversampling_rate) != 0)
	{
		printf("Could not set pressure oversampling rate of %d", pressure_oversampling_rate.val1);
		return 0;
	}
	struct sensor_value temperature_oversampling_rate = {BMP5_OVERSAMPLING_2X, 1};
	if (sensor_attr_set(press_dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &temperature_oversampling_rate) != 0)
	{
		printf("Could not set temperature oversampling rate of %d", pressure_oversampling_rate.val1);
		return 0;
	}

	struct sensor_value pressure_odr = {BMP5_ODR_25_HZ, 0};
	if (sensor_attr_set(press_dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_SAMPLING_FREQUENCY, &pressure_odr) != 0)
	{
		printf("Could not set pressure odr of %d", pressure_oversampling_rate.val1);
		return 0;
	}

	struct sensor_value power_mode = {BMP5_POWERMODE_NORMAL, 0};
	if (sensor_attr_set(press_dev, SENSOR_CHAN_PRESS, BMP5_ATTR_POWER_MODE, &power_mode) != 0)
	{
		printf("Could not set powermode  of %d", power_mode.val1);
		return 0;
	}

	k_msleep(1000);
	printf("Starting pressure& temperature polling.\n");
	struct sensor_value pressure;
	struct sensor_value temperature;

	while (1)
	{
		k_msleep(1000);
		if (sensor_sample_fetch_chan(press_dev, SENSOR_CHAN_ALL) == 0)
		{
			sensor_channel_get(press_dev, SENSOR_CHAN_PRESS, &pressure);
			sensor_channel_get(press_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);

			printf("temp %f Cel, pressure %f kPa \n",
				   sensor_value_to_double(&temperature), sensor_value_to_double(&pressure));
		}
	}

	return 0;
}

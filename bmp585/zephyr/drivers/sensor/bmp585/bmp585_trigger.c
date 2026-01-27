/* Bosch BMP585 pressure sensor
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "bmp585.h"


LOG_MODULE_REGISTER(bmp585_trigger, CONFIG_SENSOR_LOG_LEVEL);

int bmp585_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	int ret = -ENOTSUP;
	struct bmp585_data* drv = (struct bmp585_data *) dev->data;
	uint8_t int_source = 0;
	LOG_ERR("trig->type %d", trig->type);
	switch(trig->type)
	{
		case SENSOR_TRIG_DATA_READY:
		ret = bmp585_reg_read(dev, BMP5_REG_INT_SOURCE, &int_source, 1);
		int_source = BMP5_SET_BITS_POS_0(int_source, BMP5_INT_DRDY_EN, BMP5_ENABLE);
		ret = bmp585_reg_write(dev, BMP5_REG_INT_SOURCE, &int_source, 1);
		ret = bmp585_reg_read(dev, BMP5_REG_INT_SOURCE, &int_source, 1);
		LOG_ERR("reg 0x%x value 0x%x", BMP5_REG_INT_SOURCE, int_source);
		drv->drdy_handler = handler;
		break;
		default:
		break;
	}

	return ret;
}

static void bmp585_handle_int(struct bmp585_data* drv)
{
	int ret = 0;
	uint8_t int_status = 0;

	ret = bmp585_reg_read(drv->dev, BMP5_REG_INT_STATUS, &int_status, 1);
	if (ret != BMP5_OK)
	{
		LOG_ERR("Failed to read interrupt status.");
		return;
	}

	if ( (int_status & BMP5_INT_ASSERTED_DRDY) && drv->drdy_handler)
	{
		struct sensor_trigger drdy_trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		drv->drdy_handler(drv->dev, &drdy_trig);
	}
}

static void bmp585_work_cb(struct k_work *work)
{
	/*lint -e26 -e10 -e124 -e40 -e413 -e30 -e578 -e514 -e516*/
	struct bmp585_data *drv = CONTAINER_OF(work, struct bmp585_data, work);
	/*lint +e26 +e10 +e124 +e40 +e413 +e30 +e578 +e514 +e516*/
	bmp585_handle_int(drv);
}

static void bmp585_gpio_callback(const struct device *port,
				 struct gpio_callback *cb,
				 uint32_t pin)
{
	/*lint -e26 -e10 -e124 -e40 -e413 -e30 -e578 -e514 -e516*/
	struct bmp585_data *drv = CONTAINER_OF(cb, struct bmp585_data, gpio_cb);
	/*lint +e26 +e10 +e124 +e40 +e413 +e30 +e578 +e514 +e516*/

	ARG_UNUSED(port);
	ARG_UNUSED(pin);

	k_work_submit(&drv->work);
}


int bmp585_trigger_init(const struct device *dev)
{
	const struct bmp585_config* cfg = (const struct bmp585_config*) dev->config;
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;
	int ret = BMP5_OK;

	uint8_t int_source = 0;
	uint8_t int_status = 0;
	uint8_t int_config = 0;

	ret = bmp585_reg_read(dev, BMP5_REG_INT_CONFIG, &int_config, 1);
	if (ret == BMP5_OK)
	{
		/* Any change between latched/pulsed mode has to be applied while interrupt is disabled */
		/* Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00) */
		ret = bmp585_reg_write(dev, BMP5_REG_INT_SOURCE, &int_source, 1);
		if (ret == BMP5_OK)
		{
			/* Step 2 : Read the INT_STATUS register to clear the status */
			ret = bmp585_reg_read(dev, BMP5_REG_INT_STATUS, &int_status, 1);

			if (ret == BMP5_OK)
			{
				/* Step 3 : Set the desired mode in INT_CONFIG.int_mode */
				int_config = BMP5_SET_BITS_POS_0(int_config, BMP5_INT_MODE, BMP5_INT_MODE_PULSED);
				int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_POL, BMP5_INT_POL_ACTIVE_HIGH);
				int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_OD, BMP5_INT_OD_PUSHPULL);
				int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_EN, BMP5_ENABLE);
				/* Finally transfer the interrupt configurations */
				ret = bmp585_reg_write(dev, BMP5_REG_INT_CONFIG, &int_config, 1);
				ret = bmp585_reg_read(dev, BMP5_REG_INT_CONFIG, &int_config, 1);
				LOG_ERR("reg 0x%x value 0x%x", BMP5_REG_INT_CONFIG, int_config);
			}
		}
	}

	if (ret != BMP5_OK)
	{
		LOG_ERR("Unable to configure interrupts for BMP5xx.");
		return ret;
	}

	drv->gpio = cfg->input;
	if (!drv->gpio.port) {
		LOG_ERR("Gpio controller not found");
		return -EINVAL;
	}

	drv->dev = dev;
	drv->work.handler = bmp585_work_cb;
	ret = gpio_pin_configure_dt(&drv->gpio, (drv->gpio.dt_flags | GPIO_INPUT));
	ret = gpio_pin_interrupt_configure_dt(&drv->gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}
	/*lint -e64 -e119*/
	gpio_init_callback(&drv->gpio_cb, bmp585_gpio_callback, BIT(cfg->input.pin));
	/*lint +e64 +e119*/
	ret = gpio_add_callback(drv->gpio.port, &drv->gpio_cb);
	if (ret < 0) {
		return ret;
	}

    return ret;
}

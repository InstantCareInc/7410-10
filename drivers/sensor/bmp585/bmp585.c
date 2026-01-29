/* Bosch BMP585 pressure sensor
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * version 1.0.0.
 */
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "bmp585.h"


/*lint -e10 -e551 -e752 -e413*/
LOG_MODULE_REGISTER(BMP585, CONFIG_SENSOR_LOG_LEVEL);
/*lint -e10 -e551 -e752 -e413*/

int bmp585_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size)
{
	const struct bmp585_config *cfg = dev->config;
	return cfg->bus_io->read(&cfg->bus, start, buf, size);
}
int bmp585_reg_write(const struct device *dev, uint8_t reg, const uint8_t *data, uint16_t length)
{
	const struct bmp585_config *cfg = dev->config;
	return cfg->bus_io->write(&cfg->bus, reg, data, length);
}

static int power_up_check(const struct device *dev);
static int get_nvm_status(uint8_t* nvm_status, const struct device *dev);
static int validate_chip_id(const struct device *dev);
static int get_osr_odr_press_config(struct bmp585_osr_odr_press_config* osr_odr_press_cfg, const struct device *dev);
static int set_osr_config(const struct sensor_value* osr, enum sensor_channel chan, const struct device *dev);
static int set_odr_config(const struct sensor_value* odr, const struct device *dev);
static int soft_reset(const struct device *dev);
static int set_iir_config(const struct sensor_value* iir, const struct device *dev);
static int get_power_mode(enum bmp5_powermode *powermode, const struct device *dev);
static int set_power_mode(enum bmp5_powermode powermode, const struct device *dev);
static int set_power_mode(enum bmp5_powermode powermode, const struct device *dev)
{
	int ret = BMP5_OK;
	uint8_t odr = 0;
	enum bmp5_powermode current_powermode;

	ret = get_power_mode(&current_powermode, dev);
	if (ret != BMP5_OK)
	{
		LOG_DBG("Couldnt set the power mode because something went wrong when getting the current power mode.");
		return ret;
	}

	if (current_powermode != BMP5_POWERMODE_STANDBY)
	{
		/* Device should be set to standby before transitioning to forced mode or normal mode or continous mode. */

		ret = bmp585_reg_read(dev, BMP5_REG_ODR_CONFIG, &odr, 1);
		if (ret == BMP5_OK)
		{
			/* Setting deep_dis = 1(BMP5_DEEP_DISABLED) disables the deep standby mode */
			odr = BMP5_SET_BITSLICE(odr, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
			odr = BMP5_SET_BITS_POS_0(odr, BMP5_POWERMODE, BMP5_POWERMODE_STANDBY);
			ret = bmp585_reg_write(dev, BMP5_REG_ODR_CONFIG, &odr, 1);
			if (ret != BMP5_OK)
			{
				LOG_DBG("Failed to set power mode to BMP5_POWERMODE_STANDBY.");
				return ret;
			}
		}
	}

	/* lets update the power mode */
	switch(powermode)
	{
		case BMP5_POWERMODE_STANDBY:
			// this change is already done so we can just return
			ret = BMP5_OK;
			break;
		case BMP5_POWERMODE_DEEP_STANDBY:
			LOG_DBG("Setting power mode to DEEP STANDBY is not supported, current power mode is BMP5_POWERMODE_STANDBY.");
			ret = -ENOTSUP;
			break;
		case BMP5_POWERMODE_NORMAL:
		case BMP5_POWERMODE_FORCED:
		case BMP5_POWERMODE_CONTINUOUS:
			odr = BMP5_SET_BITSLICE(odr, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
			odr = BMP5_SET_BITS_POS_0(odr, BMP5_POWERMODE, powermode);
			ret = bmp585_reg_write(dev, BMP5_REG_ODR_CONFIG, &odr, 1);
			break;
		default:
			ret = BMP5_E_INVALID_POWERMODE;
			break;
	}
	ret = get_power_mode(&current_powermode, dev);
	LOG_DBG("current power mode %d", current_powermode);

	return ret;
}

static int get_power_mode(enum bmp5_powermode *powermode, const struct device *dev)
{
	int ret = BMP5_OK;

	if (powermode != NULL)
	{
		uint8_t reg = 0;
		uint8_t raw_power_mode = 0;
		ret = bmp585_reg_read(dev, BMP5_REG_ODR_CONFIG, &reg, 1);
		if (ret != BMP5_OK)
		{
			LOG_DBG("Failed to read odr config to get power mode!");
			return ret;
		}

		raw_power_mode = BMP5_GET_BITS_POS_0(reg, BMP5_POWERMODE);

		switch(raw_power_mode)
		{
			case BMP5_POWERMODE_STANDBY:
			{
				/* Getting deep disable status */
				uint8_t deep_dis = BMP5_GET_BITSLICE(reg, BMP5_DEEP_DISABLE);

				/* Checking deepstandby status only when powermode is in standby mode */

				/* If deep_dis = 0(BMP5_DEEP_ENABLED) then deepstandby mode is enabled.
					* If deep_dis = 1(BMP5_DEEP_DISABLED) then deepstandby mode is disabled
					*/
				if (deep_dis == BMP5_DEEP_ENABLED)
				{
					*powermode = BMP5_POWERMODE_DEEP_STANDBY;
				}
				else
				{
					*powermode = BMP5_POWERMODE_STANDBY;
				}

				break;
			}
			case BMP5_POWERMODE_NORMAL:
				*powermode = BMP5_POWERMODE_NORMAL;
				break;
			case BMP5_POWERMODE_FORCED:
				*powermode = BMP5_POWERMODE_FORCED;
				break;
			case BMP5_POWERMODE_CONTINUOUS:
				*powermode = BMP5_POWERMODE_CONTINUOUS;
				break;
			default:
				ret = BMP5_E_INVALID_POWERMODE;
				LOG_DBG("Something went wrong invalid powermode!");
				break;
		}
	}
	else
	{
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int power_up_check(const struct device *dev) {
	int8_t rslt = 0;
	uint8_t nvm_status = 0;

	rslt = get_nvm_status(&nvm_status, dev);

	if (rslt == BMP5_OK) {
		/* Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed */
		if ((nvm_status & BMP5_INT_NVM_RDY) != 0 && (nvm_status & BMP5_INT_NVM_ERR) == 0)
		{
			rslt = BMP5_OK;
		}
		else
		{
			rslt = BMP5_E_POWER_UP;
		}
	}

	return rslt;
}

static int get_nvm_status(uint8_t* nvm_status, const struct device *dev)
{
	int8_t rslt = 0;

	if (nvm_status != NULL)
	{
		rslt = bmp585_reg_read(dev, BMP5_REG_STATUS, nvm_status, 1);
	}
	else
	{
		rslt = BMP5_E_NULL_PTR;
	}

	return rslt;
}

static int validate_chip_id(const struct device *dev)
{
	int8_t rslt = 0;
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;

	if ( (drv->chip_id == BMP5_CHIP_ID_PRIM) || (drv->chip_id == BMP5_CHIP_ID_SEC) )
	{
		rslt = BMP5_OK;
	}
	else
	{
		drv->chip_id = 0;
		rslt = BMP5_E_DEV_NOT_FOUND;
	}

	return rslt;
}

/*!
 *  This API gets the configuration for oversampling of temperature, oversampling of
 *  pressure and ODR configuration along with pressure enable.
 */
static int get_osr_odr_press_config(struct bmp585_osr_odr_press_config* osr_odr_press_cfg, const struct device *dev)
{
	/* Variable to store the function result */
	int8_t rslt = 0;

	/* Variable to store OSR and ODR config */
	uint8_t reg_data[2] = {0};

	if (osr_odr_press_cfg != NULL)
	{
		/* Get OSR and ODR configuration in burst read */
		rslt = bmp585_reg_read(dev, BMP5_REG_OSR_CONFIG, reg_data, 2);

		if (rslt == BMP5_OK)
		{
		osr_odr_press_cfg->osr_t = BMP5_GET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS);
		osr_odr_press_cfg->osr_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_OS);
		osr_odr_press_cfg->press_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_EN);
		osr_odr_press_cfg->odr = BMP5_GET_BITSLICE(reg_data[1], BMP5_ODR);
		}
	}
	else
	{
		rslt = BMP5_E_NULL_PTR;
	}
	LOG_DBG("osr_d 0x%x osr_p 0x%x press_en x%x odr 0x%x", 
	osr_odr_press_cfg->osr_t, osr_odr_press_cfg->osr_t, osr_odr_press_cfg->press_en, osr_odr_press_cfg->odr);
	return rslt;
}

static int set_osr_config(const struct sensor_value* osr, enum sensor_channel chan, const struct device *dev)
{
	int ret = 0;
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;

	if (osr != NULL)
	{
		uint8_t oversampling = osr->val1;
		uint8_t press_en = osr->val2 != 0; // if it is not 0 then pressure is enabled
		uint8_t osr_val = 0;

		ret = bmp585_reg_read(dev, BMP5_REG_OSR_CONFIG, &osr_val, 1);
		if (ret == BMP5_OK)
		{
			switch(chan)
			{
				case SENSOR_CHAN_ALL:
					osr_val = BMP5_SET_BITS_POS_0(osr_val, BMP5_TEMP_OS, oversampling);
					osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_OS, oversampling);
					osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_EN, press_en);
					break;
				case SENSOR_CHAN_PRESS:
					osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_OS, oversampling);
					osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_EN, press_en);
					break;
				case SENSOR_CHAN_AMBIENT_TEMP:
					osr_val = BMP5_SET_BITS_POS_0(osr_val, BMP5_TEMP_OS, oversampling);
					break;
				default:
					ret = -ENOTSUP;
					break;
			}

			if (ret == BMP5_OK)
			{
				ret = bmp585_reg_write(dev, BMP5_REG_OSR_CONFIG, &osr_val, 1);
				ret += get_osr_odr_press_config(&drv->osr_odr_press_config, dev);
			}
		}
	}
	else
	{
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int set_odr_config(const struct sensor_value* odr, const struct device *dev)
{
	int ret = 0;
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;

	if (odr != NULL)
	{
		uint8_t odr_val = 0;
		ret = bmp585_reg_read(dev, BMP5_REG_ODR_CONFIG, &odr_val, 1);
		if (ret != BMP5_OK)
		{
			return ret;
		}
		odr_val = BMP5_SET_BITSLICE(odr_val, BMP5_ODR, odr->val1);
		ret = bmp585_reg_write(dev, BMP5_REG_ODR_CONFIG, (const uint8_t*) &odr_val, 1);
		ret += get_osr_odr_press_config(&drv->osr_odr_press_config, dev);
	}
	else
	{
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int soft_reset(const struct device *dev)
{
	int ret = 0;
	const uint8_t reset_cmd = BMP5_SOFT_RESET_CMD;
	uint8_t int_status = 0;

	ret = bmp585_reg_write(dev, BMP5_REG_CMD, &reset_cmd, 1);

	if (ret == BMP5_OK)
	{
		k_usleep(BMP5_DELAY_US_SOFT_RESET);
		if (ret == BMP5_OK)
		{
			ret = bmp585_reg_read(dev, BMP5_REG_INT_STATUS, &int_status, 1);
			if (ret == BMP5_OK)
			{
				if (int_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE)
				{
					ret = BMP5_OK;
				}
				else
				{
					ret = BMP5_E_POR_SOFTRESET;
				}
			}
			else
			{
				ret = BMP5_E_NULL_PTR;
			}
		}
	}
	else
	{
		LOG_DBG("Failed perform soft-reset.");
	}

	return ret;
}

static int bmp585_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;
	uint8_t data[6];
	int ret = 0;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	ret = bmp585_reg_read(dev, BMP5_REG_TEMP_DATA_XLSB, data, 6);
	if (ret == BMP5_OK)
	{
		int32_t raw_temperature = (int32_t) ((int32_t) ((uint32_t)(((uint32_t)data[2] << 16) | ((uint16_t)data[1] << 8) |data[0]) <<8) >> 8);

		/* Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C */
		drv->last_sample.temperature = (float)(raw_temperature / 65536.0);
		if (drv->osr_odr_press_config.press_en == BMP5_ENABLE)
		{
			uint32_t raw_pressure = (uint32_t)((uint32_t)(data[5] << 16) | (uint16_t)(data[4] << 8) | data[3]);
			/* Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa */
			drv->last_sample.pressure = (float)(raw_pressure / 64.0);
			drv->last_sample.pressure = drv->last_sample.pressure / 1000.0;
		}
		else
		{
			drv->last_sample.pressure = 0;
		}
	}
	//LOG_DBG("pressure %f temperature %f \n", drv->last_sample.pressure, drv->last_sample.temperature);

	return ret;
}

static int bmp585_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;
	int ret = BMP5_OK;

	if (val == NULL)
	{
		return BMP5_E_NULL_PTR;
	}
	switch(chan)
	{
		case SENSOR_CHAN_PRESS:
			// returns pressure in Pa
			ret = sensor_value_from_double(val, drv->last_sample.pressure);
			return ret;
		case SENSOR_CHAN_AMBIENT_TEMP:
			// returns temperature in Celcius
			ret = sensor_value_from_double(val, drv->last_sample.temperature);
			return ret;
		default:
			return -ENOTSUP;
	}
}

static int set_iir_config(const struct sensor_value* iir, const struct device *dev)
{
	int ret = BMP5_OK;

	if (iir != NULL)
	{
		/* Variable to store existing powermode */
		enum bmp5_powermode prev_powermode;

		ret = get_power_mode(&prev_powermode, dev);
		if (ret != BMP5_OK)
		{
			LOG_DBG("Not able to get current power mode.");
			return ret;
		}
		/* IIR configuration is writable only during STANDBY mode(as per datasheet) */
		ret = set_power_mode(BMP5_POWERMODE_STANDBY, dev);

		// update IIR config
		uint8_t dsp_config[2];

		ret = bmp585_reg_read(dev, BMP5_REG_DSP_CONFIG, dsp_config, 2);
		if (ret != BMP5_OK)
		{
			LOG_DBG("Failed to read dsp config register.");
			return ret;
		}
		// Put IIR filtered values in data registers
		dsp_config[0] = BMP5_SET_BITSLICE(dsp_config[0], BMP5_SHDW_SET_IIR_TEMP, BMP5_ENABLE);
		dsp_config[0] = BMP5_SET_BITSLICE(dsp_config[0], BMP5_SHDW_SET_IIR_PRESS, BMP5_ENABLE);

		// Configure IIR filter
		dsp_config[1] = iir->val1;
		dsp_config[1] = BMP5_SET_BITSLICE(dsp_config[1], BMP5_SET_IIR_PRESS, iir->val2);

		/* Set IIR configuration */
		ret = bmp585_reg_write(dev, BMP5_REG_DSP_CONFIG, dsp_config, 2);

		if (ret != BMP5_OK)
		{
			LOG_DBG("Failed to configure IIR filter.");
			return ret;
		}

		// Restore previous power mode if it is not standby already
		if (prev_powermode != BMP5_POWERMODE_STANDBY)
		{
			ret = set_power_mode(prev_powermode, dev);
		}
	}
	else
	{
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int bmp585_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = -ENOTSUP;
	LOG_DBG("attr %d val1 %d val2 %d", (int) attr, val->val1, val->val2);

	switch((int) attr)
	{
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			ret = set_odr_config(val, dev);
			break;
		case SENSOR_ATTR_OVERSAMPLING:
			ret = set_osr_config(val, chan, dev);
			break;
		case BMP5_ATTR_POWER_MODE:
			{
				enum bmp5_powermode powermode = (enum bmp5_powermode) val->val1;
				ret = set_power_mode(powermode, dev);
			}
			break;
		case BMP5_ATTR_IIR_CONFIG:
			ret = set_iir_config(val, dev);
			break;
		default:
			ret = -ENOTSUP;
			break;
	}
	return ret;
}



static int bmp585_init(const struct device *dev)
{
	//const struct bmp585_config* cfg = (struct bmp585_config*) dev->config;
	struct bmp585_data* drv = (struct bmp585_data*) dev->data;
	int ret = -1;

	// Reset the chip id.
	drv->chip_id = 0;
	memset(&drv->osr_odr_press_config, 0, sizeof(drv->osr_odr_press_config));
	memset(&drv->last_sample, 0, sizeof(drv->last_sample));

	ret = soft_reset(dev);

	ret = bmp585_reg_read(dev, BMP5_REG_CHIP_ID, &drv->chip_id, 1);
	if (ret != BMP5_OK)
	{
		return ret;
	}

	if (drv->chip_id != 0)
	{
		ret = power_up_check(dev);
		if (ret == BMP5_OK)
		{
			ret = validate_chip_id(dev);
			if (ret != BMP5_OK)
			{
				LOG_ERR("Unexpected chip id (%x). Expected (%x or %x)",
					drv->chip_id, BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC);
			}
			else
			{
				#ifdef CONFIG_BMP585_TRIGGER
				ret = bmp585_trigger_init(dev);
				if (ret != BMP5_OK)
				{
					LOG_ERR("Unable to initialize trigger for BMP585");
					return ret;
				}
				#endif
			}
		}
	} else {
		// that means something went wrong
		LOG_ERR("Unexpected chip id (%x). Expected (%x or %x)",
			drv->chip_id, BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC);
		return BMP5_E_INVALID_CHIP_ID;
	}
	LOG_ERR("read chip id (%x). Expected (%x or %x)",
			drv->chip_id, BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC);
	return ret;
}

static const struct sensor_driver_api bmp585_driver_api = {
	.sample_fetch = bmp585_sample_fetch,
	.channel_get = bmp585_channel_get,
#ifdef CONFIG_BMP585_TRIGGER
	.trigger_set = bmp585_trigger_set,
#endif
	.attr_set = bmp585_attr_set
};

/* Initializes a struct bmp585_config for an instance on a SPI bus. */
#define BMP585_CONFIG_SPI(inst)				\
	.bus.spi = SPI_DT_SPEC_INST_GET(inst, BMP585_SPI_OPERATION, 0),	\
	.bus_io = &bmp585_bus_io_spi,

/* Initializes a struct bmp585_config for an instance on an I2C bus. */
#define BMP585_CONFIG_I2C(inst)			       \
	.bus.i2c = I2C_DT_SPEC_INST_GET(inst),	       \
	.bus_io = &bmp585_bus_io_i2c,

#define BMP585_BUS_CFG(inst)			\
	COND_CODE_1(DT_INST_ON_BUS(inst, i2c),	\
		    (BMP585_CONFIG_I2C(inst)),	\
		    (BMP585_CONFIG_SPI(inst)))

#if defined(CONFIG_BMP585_TRIGGER)
#define BMP585_INT_CFG(inst) \
	.input = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),
#else
#define BMP585_INT_CFG(inst)
#endif

#define BMP585_INST(inst)						   \
	static struct bmp585_data bmp585_data_##inst = {		   \
		.odr = DT_INST_ENUM_IDX(inst, odr),			   \
		.osr_pressure = DT_INST_ENUM_IDX(inst, osr_press),	   \
		.osr_temp = DT_INST_ENUM_IDX(inst, osr_temp),		   \
	};								   \
	static const struct bmp585_config bmp585_config_##inst = {	   \
		BMP585_BUS_CFG(inst)					   \
		BMP585_INT_CFG(inst)					   \
		.iir_filter = DT_INST_ENUM_IDX(inst, iir_filter),	   \
	};								   \
	SENSOR_DEVICE_DT_INST_DEFINE(					   \
		inst,							   \
		bmp585_init,						   \
		PM_DEVICE_DT_INST_GET(inst),				   \
		&bmp585_data_##inst,					   \
		&bmp585_config_##inst,					   \
		POST_KERNEL,						   \
		CONFIG_SENSOR_INIT_PRIORITY,				   \
		&bmp585_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BMP585_INST)

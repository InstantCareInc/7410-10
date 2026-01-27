/*
 * Copyright (c) 2025 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
/*
 * Get a device structure from a devicetree node from alias
 * "pressure_sensor".
 */
#define BMP5_SEA_LEVEL_PRESSURE_PA                101325
/* ODR settings */
#define BMP5_ODR_240_HZ                           0x00
#define BMP5_ODR_218_5_HZ                         0x01
#define BMP5_ODR_199_1_HZ                         0x02
#define BMP5_ODR_179_2_HZ                         0x03
#define BMP5_ODR_160_HZ                           0x04
#define BMP5_ODR_149_3_HZ                         0x05
#define BMP5_ODR_140_HZ                           0x06
#define BMP5_ODR_129_8_HZ                         0x07
#define BMP5_ODR_120_HZ                           0x08
#define BMP5_ODR_110_1_HZ                         0x09
#define BMP5_ODR_100_2_HZ                         0x0A
#define BMP5_ODR_89_6_HZ                          0x0B
#define BMP5_ODR_80_HZ                            0x0C
#define BMP5_ODR_70_HZ                            0x0D
#define BMP5_ODR_60_HZ                            0x0E
#define BMP5_ODR_50_HZ                            0x0F
#define BMP5_ODR_45_HZ                            0x10
#define BMP5_ODR_40_HZ                            0x11
#define BMP5_ODR_35_HZ                            0x12
#define BMP5_ODR_30_HZ                            0x13
#define BMP5_ODR_25_HZ                            0x14
#define BMP5_ODR_20_HZ                            0x15
#define BMP5_ODR_15_HZ                            0x16
#define BMP5_ODR_10_HZ                            0x17
#define BMP5_ODR_05_HZ                            0x18
#define BMP5_ODR_04_HZ                            0x19
#define BMP5_ODR_03_HZ                            0x1A
#define BMP5_ODR_02_HZ                            0x1B
#define BMP5_ODR_01_HZ                            0x1C
#define BMP5_ODR_0_5_HZ                           0x1D
#define BMP5_ODR_0_250_HZ                         0x1E
#define BMP5_ODR_0_125_HZ                         0x1F

/* Oversampling for temperature and pressure */
#define BMP5_OVERSAMPLING_1X                      0x00
#define BMP5_OVERSAMPLING_2X                      0x01
#define BMP5_OVERSAMPLING_4X                      0x02
#define BMP5_OVERSAMPLING_8X                      0x03
#define BMP5_OVERSAMPLING_16X                     0x04
#define BMP5_OVERSAMPLING_32X                     0x05
#define BMP5_OVERSAMPLING_64X                     0x06
#define BMP5_OVERSAMPLING_128X                    0x07

/* IIR filter for temperature and pressure */
#define BMP5_IIR_FILTER_BYPASS                    0x00
#define BMP5_IIR_FILTER_COEFF_1                   0x01
#define BMP5_IIR_FILTER_COEFF_3                   0x02
#define BMP5_IIR_FILTER_COEFF_7                   0x03
#define BMP5_IIR_FILTER_COEFF_15                  0x04
#define BMP5_IIR_FILTER_COEFF_31                  0x05
#define BMP5_IIR_FILTER_COEFF_63                  0x06
#define BMP5_IIR_FILTER_COEFF_127                 0x07
/* Custom ATTR values */

/* This is used to enable IIR config, keep in mind that disabling IIR back in runtime is not supported yet */
#define BMP5_ATTR_IIR_CONFIG                      (SENSOR_ATTR_PRIV_START + 1u)
#define BMP5_ATTR_POWER_MODE                      (SENSOR_ATTR_PRIV_START + 2u)
#define BMP5_ATTR_ALTITUDE                      (SENSOR_ATTR_PRIV_START + 3u)
enum bmp5_powermode {
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

#define BMP5_DATA_READY 1
#define BMP5_POLLING    0
#if BMP5_DATA_READY 
static void trigger_handler(const struct device *dev,
			const struct sensor_trigger *trig)
{
	struct sensor_value pressure;
	struct sensor_value temperature;
	int64_t current_time = k_uptime_get();
	if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		printk("Data_ready TIME(ms) %lld temp %f Cel, pressure %f kPa\n", current_time,
		sensor_value_to_double(&temperature), sensor_value_to_double(&pressure));
	}
}
#endif

int main(void)
{

	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmp585);

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return 0;
	}

	if (dev == NULL) {
		return 0;
	}
	
	struct sensor_value pressure_oversampling_rate = { BMP5_OVERSAMPLING_16X, 1 };
	if (sensor_attr_set(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &pressure_oversampling_rate) != 0) {
		printk("Could not set oversampling rate of %d", pressure_oversampling_rate.val1);
		return 0;
	}
	struct sensor_value temperature_oversampling_rate = { BMP5_OVERSAMPLING_2X, 1 };
	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &temperature_oversampling_rate) != 0) {
		printk("Could not set oversampling rate of %d", pressure_oversampling_rate.val1);
		return 0;
	}

	struct sensor_value pressure_odr = { BMP5_ODR_25_HZ, 0 };
	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_SAMPLING_FREQUENCY, &pressure_odr) != 0) {
		printk("Could not pressure_odrof %d", pressure_oversampling_rate.val1);
		return 0;
	}


	struct sensor_value power_mode = { BMP5_POWERMODE_NORMAL, 0 };
	if (sensor_attr_set(dev, SENSOR_CHAN_PRESS, BMP5_ATTR_POWER_MODE, &power_mode) != 0) {
		printk("Could not set oversampling rate of %d", pressure_oversampling_rate.val1);
		return 0;
	}


#if BMP5_DATA_READY
	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_PRESS;

	if (sensor_trigger_set(dev, &trig, trigger_handler) != 0) {
		printf("Could not set sensor type and channel\n");
		return 0;
	}
#endif

#if BMP5_POLLING
	int32_t sampling_freq = 25;
	if (pressure_odr.val1 == BMP5_ODR_25_HZ) {
		sampling_freq = 25;
	} else if (pressure_odr.val1 == BMP5_ODR_10_HZ) {
		sampling_freq = 10;
	} else if (pressure_odr.val1 == BMP5_ODR_05_HZ) {
		sampling_freq = 5;
	}
	k_msleep(1000);	
	printk("Starting pressure, temperature and altitude polling sample.\n");
	struct sensor_value pressure;
	struct sensor_value temperature;
	while (1) {
		int64_t current_time = k_uptime_get();
		k_sleep(K_MSEC(1000 / sampling_freq));
		if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
			sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
			sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	
		printk("Polling TIME(ms) %lld temp %f Cel, pressure %f kPa \n", current_time,
		sensor_value_to_double(&temperature), sensor_value_to_double(&pressure));
		}
	}
#endif

	return 0;
}

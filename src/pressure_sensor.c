#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "pressure_sensor.h"

LOG_MODULE_REGISTER(pressure_sensor, LOG_LEVEL_INF);

#define PRESS_NODE DT_ALIAS(press0)

static const struct device *press_dev = DEVICE_DT_GET(PRESS_NODE);

int pressure_sensor_init(void)
{
	if (!device_is_ready(press_dev)) {
		LOG_ERR("Pressure sensor device not ready");
		return -ENODEV;
	}

	return 0;
}

const struct device *pressure_sensor_get_device(void)
{
	return press_dev;
}

static int cmd_set_press_oversampling(const struct shell *sh, size_t argc, char **argv, void *data) {
	ARG_UNUSED(argc);

	struct sensor_value pressure_oversampling_rate = {
		.val1 = (int)data,
		.val2 = 1 // Enable pressure measurements in bmp585.c
	};

	int err = sensor_attr_set(pressure_sensor_get_device(), SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &pressure_oversampling_rate);
	if (err < 0) {
		shell_error(sh, "Failed to set pressure OSR (err %d)", err);
		return err;
	}
	
	shell_print(sh, "Pressure OSR set to: %s", argv[0]);
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_set_press_oversampling, cmd_set_press_oversampling,
	(1X,	BMP5_OVERSAMPLING_1X,	"1x oversampling"),
	(2X,	BMP5_OVERSAMPLING_2X, 	"2x oversampling"),
	(4X,   	BMP5_OVERSAMPLING_4X,	"4x oversampling"),
	(8X,   	BMP5_OVERSAMPLING_8X,   "8x oversampling"),
	(16X, 	BMP5_OVERSAMPLING_16X,  "16x oversampling"),
	(32X,  	BMP5_OVERSAMPLING_32X,  "32x oversampling"),
	(64X,  	BMP5_OVERSAMPLING_64X,  "64x oversampling"),
	(128X, 	BMP5_OVERSAMPLING_128X,	"128x oversampling")
);

static int cmd_set_temp_oversampling(const struct shell *sh, size_t argc, char **argv, void *data) {
	ARG_UNUSED(argc);

	struct sensor_value temp_oversampling_rate = {
		.val1 = (int)data,
		.val2 = 0 // unused in bmp585.c
	};

	int err = sensor_attr_set(pressure_sensor_get_device(), SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &temp_oversampling_rate);
	if (err < 0) {
		shell_error(sh, "Failed to set temp OSR (err %d)", err);
		return err;
	}
	
	shell_print(sh, "Temperature OSR set to: %s", argv[0]);
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_set_temp_oversampling, cmd_set_temp_oversampling,
	(1X,	BMP5_OVERSAMPLING_1X,	"1x oversampling"),
	(2X,	BMP5_OVERSAMPLING_2X, 	"2x oversampling"),
	(4X,   	BMP5_OVERSAMPLING_4X,	"4x oversampling"),
	(8X,   	BMP5_OVERSAMPLING_8X,   "8x oversampling"),
	(16X, 	BMP5_OVERSAMPLING_16X,  "16x oversampling"),
	(32X,  	BMP5_OVERSAMPLING_32X,  "32x oversampling"),
	(64X,  	BMP5_OVERSAMPLING_64X,  "64x oversampling"),
	(128X, 	BMP5_OVERSAMPLING_128X,	"128x oversampling")
);

static int cmd_set_odr(const struct shell *sh, size_t argc, char **argv, void *data) {
	ARG_UNUSED(argc);

	struct sensor_value press_odr = {
		.val1 = (int)data,
		.val2 = 0 // Unused in bmp585.c
	};

	int err = sensor_attr_set(pressure_sensor_get_device(), SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &press_odr);
	if (err < 0) {
		shell_error(sh, "Could not set ODR (err %d)", err);
		return err;
	}
	
	shell_print(sh, "ODR set to: %s", argv[0]);
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_set_odr, cmd_set_odr,
	(240HZ,		BMP5_ODR_240_HZ,	"240 Hz"),
	(218HZ,		BMP5_ODR_218_5_HZ,	"218 Hz"),
	(199_1HZ,	BMP5_ODR_199_1_HZ,	"199.1 Hz"),
	(179_2HZ,   BMP5_ODR_179_2_HZ,	"179.2 Hz"),
	(160HZ,  	BMP5_ODR_160_HZ,	"160 Hz"),
	(149_3HZ,  	BMP5_ODR_149_3_HZ,  "149.3Hz"),
	(140HZ,  	BMP5_ODR_140_HZ,  	"140 Hz"),
	(129_8HZ, 	BMP5_ODR_129_8_HZ, 	"129.8 Hz"),
	(120HZ,		BMP5_ODR_120_HZ, 	"120 Hz"),
	(110_1HZ,	BMP5_ODR_110_1_HZ, 	"110.1 Hz"),
	(100_2HZ,   BMP5_ODR_100_2_HZ,  "100.2 Hz"),
	(89_6HZ,   	BMP5_ODR_89_6_HZ,   "89.6 Hz"),
	(80HZ,  	BMP5_ODR_80_HZ,  	"80 Hz"),
	(70HZ,  	BMP5_ODR_70_HZ,  	"70 Hz"),
	(60HZ,  	BMP5_ODR_60_HZ,  	"60 Hz"),
	(50HZ, 		BMP5_ODR_50_HZ, 	"50 Hz"),
	(45HZ,		BMP5_ODR_45_HZ, 	"45 Hz"),
	(40HZ,		BMP5_ODR_40_HZ, 	"40 Hz"),
	(35HZ,   	BMP5_ODR_35_HZ,   	"35 Hz"),
	(30HZ,   	BMP5_ODR_30_HZ,   	"30 Hz"),
	(25HZ,  	BMP5_ODR_25_HZ,  	"25 Hz"),
	(20HZ,  	BMP5_ODR_20_HZ,  	"20 Hz"),
	(15HZ,  	BMP5_ODR_15_HZ,  	"15 Hz"),
	(10HZ, 		BMP5_ODR_10_HZ, 	"10 Hz"),
	(5HZ,		BMP5_ODR_05_HZ, 	"5 Hz"),
	(4HZ,		BMP5_ODR_04_HZ, 	"4 Hz"),
	(3HZ,  		BMP5_ODR_03_HZ,   	"3 Hz"),
	(2HZ,   	BMP5_ODR_02_HZ,   	"2 Hz"),
	(1HZ,  		BMP5_ODR_01_HZ,  	"1 Hz"),
	(0_5HZ,  	BMP5_ODR_0_5_HZ,  	"0.5 Hz"),
	(0_25HZ,  	BMP5_ODR_0_250_HZ,  "0.25 Hz"),
	(0_125HZ, 	BMP5_ODR_0_125_HZ, 	"0.123 Hz")
);

static int cmd_set_powermode(const struct shell *sh, size_t argc, char **argv, void *data) {
	ARG_UNUSED(argc);

	struct sensor_value power_mode = {
		.val1 = (int)data,
		.val2 = 0 // Unused in bmp585.c
	};

	int err = sensor_attr_set(pressure_sensor_get_device(), SENSOR_CHAN_ALL, BMP5_ATTR_POWER_MODE, &power_mode);
	if (err < 0) {
		shell_error(sh, "Power mode change failed (err %d)", err);
		return err;
	}
	
	shell_print(sh, "Power mode set to: %s", argv[0]);
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_set_powermode, cmd_set_powermode,
	(STANDBY,		BMP5_POWERMODE_STANDBY,			"Standby powermode"),
	(NORMAL,		BMP5_POWERMODE_NORMAL, 			"Normal powermode"),
	(FORCED,   		BMP5_POWERMODE_FORCED,   		"Forced powermode"),
	(CONTINUOUS,   	BMP5_POWERMODE_CONTINUOUS,		"Continuous powermode"),
	(DEEP_STANDBY,  BMP5_POWERMODE_DEEP_STANDBY,	"Deep Standby powemode")
);

static int cmd_get_press(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct sensor_value pressure;
	int err = sensor_sample_fetch(pressure_sensor_get_device());
	if (err != 0) {
		shell_error(sh, "Sensor fetch failed (err %d)", err);
	}

	err = sensor_channel_get(pressure_sensor_get_device(), SENSOR_CHAN_PRESS, &pressure);
	if (err < 0) {
		shell_error(sh, "Could not get pressure (err %d)", err);
	}
	
	shell_print(sh, "Pressure = %f kPa", sensor_value_to_double(&pressure));
	return 0;
}

static int cmd_get_temp(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct sensor_value temperature;
	int err = sensor_sample_fetch(pressure_sensor_get_device());
	if (err != 0) {
		shell_error(sh, "Sensor fetch failed (err %d)", err);
	}

	err = sensor_channel_get(pressure_sensor_get_device(), SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	if (err < 0) {
		shell_error(sh, "Could not get ambient temp (err %d)", err);
	}
	
	shell_print(sh, "Ambient Temperature = %f Cel", sensor_value_to_double(&temperature));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(oversample_subcmds,
				SHELL_CMD(pressure, &sub_set_press_oversampling, "Set pressure oversampling rate", NULL),
				SHELL_CMD(temperature, &sub_set_temp_oversampling, "Set temperature oversampling rate", NULL),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(set_subcmds,
				SHELL_CMD(oversample, &oversample_subcmds, "Set oversampling rates", NULL),
				SHELL_CMD(odr, &sub_set_odr, "Set ODR", NULL),
				SHELL_CMD(powermode, &sub_set_powermode, "Set Power Mode", NULL),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
				SHELL_CMD(pressure, NULL, "Get pressure reading (kPa)", &cmd_get_press),
				SHELL_CMD(temperature, NULL, "Get ambient temperature reading (Celsius)", &cmd_get_temp),
				SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(press_subcmds,
				SHELL_CMD(get, &get_subcmds, "Get BMP585 values", NULL),
				SHELL_CMD(set, &set_subcmds, "Set BMP585 values", NULL),
				SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(BMP585, &press_subcmds, "BMP585 sensor control commands", NULL);

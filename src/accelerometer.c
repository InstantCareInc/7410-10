#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "accelerometer.h"

LOG_MODULE_REGISTER(accelerometer, LOG_LEVEL_INF);

#define ACCEL_NODE DT_ALIAS(accel0)

static const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

int accelerometer_init(void)
{
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device not ready");
		return -ENODEV;
	}

	return 0;
}

const struct device *accelerometer_get_device(void)
{
	return accel_dev;
}


static int cmd_get_accel(const struct shell *sh, size_t argc, char **argv) 
{
    struct sensor_value accel[3];
    
    int err = sensor_sample_fetch(accelerometer_get_device());
    if (err != 0) {
        shell_error(sh, "Sensor fetch failed (err %d)", err);
        return err;
    }

    sensor_channel_get(accelerometer_get_device(), SENSOR_CHAN_ACCEL_XYZ, accel);

    shell_print(sh, "X: %.2f, Y: %.2f, Z: %.2f (m/s^2)\n",
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
    SHELL_CMD(data, NULL, "Read X, Y, Z axis", cmd_get_accel),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(accel_subcmds,
    SHELL_CMD(get, &get_subcmds, "Get ADXL367 values", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(adxl367, &accel_subcmds, "ADXL367 sensor control commands", NULL);
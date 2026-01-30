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
    
    if (sensor_sample_fetch(accelerometer_get_device()) < 0) {
        shell_error(sh, "Failed to fetch sample");
        return -EIO;
    }

    sensor_channel_get(accelerometer_get_device(), SENSOR_CHAN_ACCEL_XYZ, accel);

    shell_print(sh, "X: %d.%06d, Y: %d.%06d, Z: %d.%06d m/s^2",
                accel[0].val1, abs(accel[0].val2),
                accel[1].val1, abs(accel[1].val2),
                accel[2].val1, abs(accel[2].val2));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
    SHELL_CMD(data, NULL, "Read X, Y, Z axes", cmd_get_accel),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(accel_subcmds,
    SHELL_CMD(get, &get_subcmds, "Get accelerometer data", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ADXL367, &accel_subcmds, "ADXL367 control commands", NULL);
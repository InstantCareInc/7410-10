#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "accelerometer.h"
#include "leds.h"

LOG_MODULE_REGISTER(accelerometer, LOG_LEVEL_INF);

#define ACCEL_NODE DT_ALIAS(accel0)

static const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

static void free_fall_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    LOG_INF("!!! Free fall detected !!!");
    led_grn_set(1);
}

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
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

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

static int cmd_accel_freefall(const struct shell *sh, size_t argc, char **argv) 
{
    int err;
    bool enable = shell_strtobool(argv[1], 10, &err);
    if (err < 0) {
        shell_error(sh, "Cannot parse %s. Use 'on' or 'off' to set free fall trigger.", argv[1]);
		return -EINVAL;
    }

    if (enable) {
        
        double thresh_g = 0.6; // default to 600mg
        if (argc >= 3) {
            thresh_g = (double)shell_strtol(argv[2], 10, &err);
            if (err < 0 || thresh_g <= 0) {
                shell_error(sh, "Cannot parse %s as threshold g value.", argv[2]);
                return -EINVAL;
            }
        }

        uint32_t dur_ms = 100; // default to 100ms
        if (argc >= 4) {
            dur_ms = (uint32_t)shell_strtol(argv[3], 10, &err);
            if (err < 0 || dur_ms <= 0) {
                shell_error(sh, "Cannot parse %s as duration in ms.", argv[3]);
                return -EINVAL;
            }
        }

        /* Set threshold */
        struct sensor_value thresh;
        sensor_g_to_ms2(thresh_g, &thresh);
        sensor_attr_set(accelerometer_get_device(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_UPPER_THRESH, &thresh);

        /* Set duration to 100ms */
        struct sensor_value dur = {.val1 = dur_ms};
        sensor_attr_set(accelerometer_get_device(), SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_HYSTERESIS, &dur);

        /* Register Trigger */
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_FREEFALL,
            .chan = SENSOR_CHAN_ACCEL_XYZ,
        };

        if (sensor_trigger_set(accelerometer_get_device(), &trig, free_fall_handler) < 0) {
            shell_error(sh, "Failed to enable free-fall interrupt");
            return -EIO;
        }
    } else {
        /* Disable trigger */
        struct sensor_trigger trig = {.type = SENSOR_TRIG_FREEFALL};
        sensor_trigger_set(accelerometer_get_device(), &trig, NULL);
        shell_print(sh, "Free-fall detection disarmed.");
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
    SHELL_CMD(data, NULL, "Read X, Y, Z axis (Usage: adxl367 get data)", cmd_get_accel),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(set_subcmds,
    SHELL_CMD_ARG(freefall, NULL, "Enable/Disable free-fall detection (Usage: adxl367 set freefall <on|off> [thresh_g] [dur_ms])", cmd_accel_freefall, 2, 2),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(accel_subcmds,
    SHELL_CMD(get, &get_subcmds, "Get ADXL367 values", NULL),
    SHELL_CMD(set, &set_subcmds, "Set ADXL367 values", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(adxl367, &accel_subcmds, "ADXL367 sensor control commands", NULL);
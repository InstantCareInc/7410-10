#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "leds.h"

LOG_MODULE_REGISTER(leds, LOG_LEVEL_INF);

#define LED_GRN_NODE DT_ALIAS(led0)
#define LED_RED_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led_grn_spec = GPIO_DT_SPEC_GET(LED_GRN_NODE, gpios);
static const struct gpio_dt_spec led_red_spec = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);

int leds_init(void)
{
	if (!gpio_is_ready_dt(&led_grn_spec)) {
		LOG_ERR("Green LED device not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&led_grn_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&led_red_spec)) {
		LOG_ERR("Red LED device not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&led_red_spec, GPIO_OUTPUT_ACTIVE);

	return 0;
}

int led_grn_toggle(void) {
	return gpio_pin_toggle_dt(&led_grn_spec);
}

int led_grn_set(int value) {
	return gpio_pin_set_dt(&led_grn_spec, value);
}

int led_red_toggle(void) {
	return gpio_pin_toggle_dt(&led_red_spec);
}

int led_red_set(int value) {
	return gpio_pin_set_dt(&led_red_spec, value);
}

static int cmd_toggle_led(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);

	int err;

	if (strcmp(argv[1], "green") == 0) {
		err = led_grn_toggle();
	} else if (strcmp(argv[1], "red") == 0) {
		err = led_red_toggle();
	} else {
		shell_error(sh, "Unrecognized LED (%s).  Use 'red' or 'green' instead.", argv[1]);
		return -EINVAL;
	}

	if (err < 0) {
		shell_error(sh, "Issue toggling %s LED. Error code: %d", argv[1], err);
		return err;
	}

	shell_print(sh, "%s LED toggled", argv[1]);
	return 0;
}

static int cmd_set_led(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);

	int err;
	int val = shell_strtobool(argv[2], 10, &err) ? 1 : 0;
	if (err < 0) {
		shell_error(sh, "Cannot parse %s. Use 'on' or 'off' to set LED state.", argv[2]);
		return -EINVAL;
	}

	if (strcmp(argv[1], "green") == 0) {
		err = led_grn_set(val);
	} else if (strcmp(argv[1], "red") == 0) {
		err = led_red_set(val);
	} else {
		shell_error(sh, "Unrecognized LED (%s).  Use 'red' or 'green' instead.", argv[1]);
		return -EINVAL;
	}

	if (err < 0) {
		shell_error(sh, "Issue setting %s LED. Error code: %d", argv[1], err);
		return err;
	}

	shell_print(sh, "%s LED turned %s", argv[1], val ? "ON" : "OFF");
	return 0;
}

/* Register shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(led_subcmds,
								SHELL_CMD_ARG(toggle, NULL, "Toggle LED (Usage: led toggle <green | red>)", cmd_toggle_led, 2, 0),
								SHELL_CMD_ARG(set, NULL, "Set LED ON/OFF (Usage: led set <green|red> <on|off>)", cmd_set_led, 3, 0),
								SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(led, &led_subcmds, "LED control commands", NULL);

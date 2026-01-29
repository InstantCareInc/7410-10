#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "button.h"

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

#define BUTTON0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET_OR(BUTTON0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

int button_init(button_callback_t callback)
{
	if (!gpio_is_ready_dt(&button0_spec)) {
		LOG_ERR("Button device not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb_data, callback, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button_cb_data);

	return 0;
}

SHELL_CMD_REGISTER(button, NULL, "Not implemented yet (Sorry)", NULL);
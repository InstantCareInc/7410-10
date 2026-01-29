#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>
#include <zephyr/app_version.h>
#include <stdio.h>

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
	gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&led1_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT_ACTIVE);

	if (!gpio_is_ready_dt(&buzzer0_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer0_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer1_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer1_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&buzzer2_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&buzzer2_spec, GPIO_OUTPUT_INACTIVE);

	if (!gpio_is_ready_dt(&button0_spec))
	{
		return -ENODEV;
	}
	gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0_spec.pin));
	gpio_add_callback(button0_spec.port, &button_cb_data);

	if (!device_is_ready(press_dev))
	{
		return -ENODEV;
	}

	if (!device_is_ready(accel_dev))
	{
		return -ENODEV;
	}

	return 0;
}

int main(void)
{
	int err;
	if (setup_gpio() < 0)
	{
		LOG_ERR("Failed to setup GPIO");
		return 0;
	}

	err = gpio_pin_set_dt(&led1_spec, 0);

	while (1)
	{
		k_sleep(K_FOREVER);
	}

	return 0;
}

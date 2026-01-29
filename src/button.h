#ifndef BUTTON_H
#define BUTTON_H

#include <zephyr/drivers/gpio.h>

/**
 * @brief Button callback function type
 * @param dev GPIO device that triggered the callback
 * @param cb Callback structure
 * @param pins Pin mask that triggered the interrupt
 */
typedef void (*button_callback_t)(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/**
 * @brief Initialize button GPIO and interrupt
 * @param callback Function to call when button is pressed
 * @return 0 on success, negative errno on failure
 */
int button_init(button_callback_t callback);

#endif /* BUTTON_H */
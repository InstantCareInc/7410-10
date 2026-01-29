#ifndef LEDS_H
#define LEDS_H

/**
 * @brief Initialize LED GPIOs
 * @return 0 on success, negative errno on failure
 */
int leds_init(void);

/**
 * @brief Toggle Green LED
 * @return 0 on success, negative errno on failure
 */
int led_grn_toggle(void);

/**
 * @brief Set Green LED state
 * @param value 1 for on, 0 for off
 * @return 0 on success, negative errno on failure
 */
int led_grn_set(int value);

/**
 * @brief Toggle Red LED
 * @return 0 on success, negative errno on failure
 */
int led_red_toggle(void);

/**
 * @brief Set Red LED state
 * @param value 1 for on, 0 for off
 * @return 0 on success, negative errno on failure
 */
int led_red_set(int value);

#endif /* LEDS_H */
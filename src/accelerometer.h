#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <zephyr/drivers/sensor.h>

/**
 * @brief Initialize accelerometer
 * @return 0 on success, negative errno on failure
 */
int accelerometer_init(void);

/**
 * @brief Get accelerometer device pointer
 * @return Pointer to accelerometer device
 */
const struct device *accelerometer_get_device(void);

#endif /* ACCELEROMETER_H */
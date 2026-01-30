#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <zephyr/drivers/sensor.h>
#include <drivers/sensor/bmp585/bmp585.h>

/**
 * @brief Initialize pressure sensor
 * @return 0 on success, negative errno on failure
 */
int pressure_sensor_init(void);

/**
 * @brief Get pressure sensor device pointer
 * @return Pointer to pressure sensor device
 */
const struct device *pressure_sensor_get_device(void);

#endif /* PRESSURE_SENSOR_H */
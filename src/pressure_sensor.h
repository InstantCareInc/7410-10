#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <zephyr/drivers/sensor.h>
#include <drivers/sensor/bmp585/bmp585.h>

#define BMP5_SEA_LEVEL_PRESSURE_PA                101325
#define VELOCITY_SAMPLES 3


/**
 * @brief Get pressure sensor device pointer
 * @return Pointer to pressure sensor device
 */
const struct device *pressure_sensor_get_device(void);

/**
 * @brief Initialize pressure sensor
 * @return 0 on success, negative errno on failure
 */
int pressure_sensor_init(void);

/**
 * @brief Enable data ready trigger and work handler
 * @return 0 on success, negative errno on failure
 */
int data_ready_init(void);
#endif /* PRESSURE_SENSOR_H */
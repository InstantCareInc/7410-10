/* Bosch BMP585 pressure sensor
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BMP585s accessed via I2C.
 */

#include "bmp585.h"

#if BMP585_BUS_I2C
static int bmp585_bus_check_i2c(const union bmp585_bus *bus)
{
	return i2c_is_ready_dt(&bus->i2c) ? 0 : -ENODEV;
}

static int bmp585_reg_read_i2c(const union bmp585_bus *bus,
			       uint8_t start, uint8_t *buf, int size)
{
	return i2c_burst_read_dt(&bus->i2c, start, buf, size);
}

static int bmp585_reg_write_i2c(const union bmp585_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	return i2c_burst_write_dt(&bus->i2c, start, data, len);
}


const struct bmp585_bus_io bmp585_bus_io_i2c = {
	.check = bmp585_bus_check_i2c,
	.read = bmp585_reg_read_i2c,
	.write = bmp585_reg_write_i2c,
};
#endif /* BMP585_BUS_I2C */

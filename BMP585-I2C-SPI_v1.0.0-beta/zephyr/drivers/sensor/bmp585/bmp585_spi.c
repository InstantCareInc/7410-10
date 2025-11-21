/* Bosch BMP585 pressure sensor
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BMP585 accessed via SPI.
 */

#include <zephyr/logging/log.h>
#include "bmp585.h"

#if BMP585_BUS_SPI

LOG_MODULE_DECLARE(BMP585, CONFIG_SENSOR_LOG_LEVEL);

static int bmp585_bus_check_spi(const union bmp585_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static int bmp585_reg_read_spi(const union bmp585_bus *bus,
			       uint8_t start, uint8_t *buf, int size)
{
	uint8_t addr;
	const struct spi_buf tx_buf = {
		.buf = &addr,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};
	int i;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].len = 1;

	for (i = 0; i < size; i++) {
		int ret;

		addr = (start + i) | 0x80;
		rx_buf[1].buf = &buf[i];

		ret = spi_transceive_dt(&bus->spi, &tx, &rx);
		if (ret) {
			LOG_DBG("spi_transceive FAIL %d\n", ret);
			return ret;
		}
	}

	return 0;
}

#define BMP585_REG_MASK            GENMASK(6, 0)

static int bmp585_reg_write_spi(const union bmp585_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	int ret;
	uint8_t addr;
	const struct spi_buf tx_buf[2] = {
		{.buf = &addr, .len = sizeof(addr)},
		{.buf = (uint8_t *)data, .len = len}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};

	addr = start & BMP585_REG_MASK;

	ret = spi_write_dt(&bus->spi, &tx);
	if (ret < 0) {
		LOG_ERR("spi_write_dt failed %i", ret);
		return ret;
	}

	return 0;
}


const struct bmp585_bus_io bmp585_bus_io_spi = {
	.check = bmp585_bus_check_spi,
	.read = bmp585_reg_read_spi,
	.write = bmp585_reg_write_spi,
};
#endif /* BMP585_BUS_SPI */

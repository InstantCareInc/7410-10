
zephyr_library()

zephyr_library_sources(
    drivers/sensor/bmp585/bmp585.c
    drivers/sensor/bmp585/bmp585_spi.c
    drivers/sensor/bmp585/bmp585_trigger.c
    drivers/sensor/bmp585/bmp585_i2c.c
)

zephyr_library_include_directories(include/zephyr/drivers)
zephyr_library_include_directories(drivers/sensor/bmp585)
#
# Copyright (c) 2020 Facebook, Inc. and its affiliates
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
zephyr_library_sources(bmp585.c bmp585_i2c.c bmp585_spi.c)
zephyr_library_sources_ifdef(CONFIG_BMP585_TRIGGER bmp585_trigger.c)

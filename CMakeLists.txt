# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND EXTRA_ZEPHYR_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/bmp585
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(7410-10)

target_sources(app PRIVATE
    src/main.c
    src/ble_manager.c
)

zephyr_library_include_directories(.)
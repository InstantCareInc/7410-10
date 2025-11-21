========================
7410-10 Evaluation Board
========================

Overview
********

This is a Zephyr project meant to evaluate the usage of an ADXL367 and BMP585 in conjunction with an nRF52832 microcontroller for the use of a wrist worn fall detection device.

Building and Running
********************

This application was built using the nRF Connect Extension for Visual Studio Code. To build the application ensure you have the nRF Connect Extension installed and folliw these steps:

#. Select or Install nRF Connect SDK version 3.1.1
#. Select Install nRF Connect SDK Toolchain version 3.1.1
#. Add Bosch Beta BMP585 driver to your SDK installtion (See instructions in next section)
#. Open this project folder in Visual Studio Code using the NRF Connect Extension option "Open an existing application"
#. Select the build target correct For this application.
   - For the 7410-10 board select: `PCB_7410_10_006`
   - For the nRF52840DK select: `nrf52840dk_nrf52840` (make sure to add the nrf52840dk_nrf52840.overlay file to the build configuration)
#. Click the build button in the bottom Actions bar  of NRF Connect Extension.

Adding Bosch Beta BMP585 Driver
*******************************
This application uses a beta version of the Bosch BMP585 driver which is not included in the standard nRF Connect SDK installation. To add this driver to your SDK installation follow these steps:

#. Navigate to your nRF Connect SDK installation folder.
#. Open the `\ncs\zephyr\drivers\sensor` folder & copy the contents from `7410-10\BMP585-I2C-SPI_v1.0.0-beta\zephyr\drivers\sensor` into this folder.
#. In the CMakeLists.txt file located in the `\ncs\zephyr\drivers\sensor` folder, add the following line to the list of source files:
   
   .. code-block:: none

      add_subdirectory_ifdef(CONFIG_BMP585 bmp585)

#. In the Kconfig file located in the `\ncs\zephyr\drivers\sensor` folder, add the following lines to the list of configuration options:
   
   .. code-block:: none

      source "drivers/sensor/bmp585/Kconfig"

#. Open the `\ncs\zephyr\dts\bindings` folder & copy the contents from `7410-10\BMP585-I2C-SPI_v1.0.0-beta\zephyr\dts\bindings` into this folder.
#. Open the `\ncs\zephyr\samples\sensor` folder & copy the contents from `7410-10\BMP585-I2C-SPI_v1.0.0-beta\zephyr\samples\sensor` into this folder.

Custom Board Support
********************

The schematic for the 7410-10 board can be found here: `7410-10 Schematic <SCH_7410_10_006.pdf>`_

nRF52840DK Pin Mapping
++++++++++++++++++++++++++
The following table shows the pin mapping for the nRF52840DK when used to evaluate the 7410-10 board.

+----------------+----------------+---------------------+
| 7410-10 Signal | nRF52840DK Pin | Description         |
+================+================+=====================+
| SCLK           | 0.26           | SPI CLOCK           |
+----------------+----------------+---------------------+
| MOSI           | 0.27           | SPI MOSI            |
+----------------+----------------+---------------------+
| MISO           | 0.28           | SPI MISO            |
+----------------+----------------+---------------------+
| CS_A           | 0.29           | ADXL367 Chip Select |
+----------------+----------------+---------------------+
| CS_P           | 0.30           | BMP585 Chip Select  |
+----------------+----------------+---------------------+
| SN1            | 0.02           | Sounder 1           |
+----------------+----------------+---------------------+
| SN2            | 0.03           | Sounder 2           |
+----------------+----------------+---------------------+
| SN3            | 0.04           | Sounder 3           |
+----------------+----------------+---------------------+




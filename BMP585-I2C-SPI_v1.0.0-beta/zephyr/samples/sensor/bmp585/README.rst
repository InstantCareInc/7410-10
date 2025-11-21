.. _bmp585:

BMP585: pressure sensor
########################################

Description
***********

This sample application configures the pressure to
measure data at 25Hz. The result is written to the console.


This sample uses the BMP585 sensor controlled using the I2C interface.
Connect Supply: **VDD**, **VDDIO**, **GND** and Interface: **SDA**, **SCL**.
The supply voltage can be in the 1.8V to 3.6V range.
Depending on the baseboard used, the **SDA** and **SCL** lines require Pull-Up
resistors.

Building and Running
********************

This sample supports pressure sensor devices. Device needs
to be aliased as ``pressure-sensor``. For example:
/* 
&arduino_i2c {
	status = "okay";
	bmp585@46{
		compatible = "bosch,bmp585";
		int-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;
		status = "okay";
		reg = <0x46>;
	};
};
*/
&arduino_spi {
	status = "okay";
	bmp585@0{
		compatible = "bosch,bmp585";
		int-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;
		reg = <0>;
		spi-max-frequency = <DT_FREQ_M(10)>;
	};
};
Make sure the aliases are in devicetree, then build and run with:

.. zephyr-app-commands::
   :zephyr-app: zephyr\samples\sensor\bmp585
   :board: nrf52dk/nrf52832
   :goals: build flash

Sample Output
=============

.. code-block:: console

[00:00:00.417,785] <err> bmp581_trigger: reg 0x15 value 0x1
Data_ready TIME(ms) 430 temp 24.485397 Cel, pressure 101876.703125 kPa
Data_ready TIME(ms) 470 temp 24.437362 Cel, pressure 101874.250000 kPa
Data_ready TIME(ms) 510 temp 24.430145 Cel, pressure 101872.375000 kPa
Data_ready TIME(ms) 549 temp 24.413925 Cel, pressure 101870.781250 kPa
Data_ready TIME(ms) 589 temp 24.409835 Cel, pressure 101869.421875 kPa
Data_ready TIME(ms) 629 temp 24.409393 Cel, pressure 101868.562500 kPa
Data_ready TIME(ms) 669 temp 24.402572 Cel, pressure 101867.484375 kPa
Data_ready TIME(ms) 709 temp 24.414016 Cel, pressure 101866.906250 kPa
Data_ready TIME(ms) 749 temp 24.406265 Cel, pressure 101866.046875 kPa
Data_ready TIME(ms) 789 temp 24.399047 Cel, pressure 101865.140625 kPa
Data_ready TIME(ms) 829 temp 24.401550 Cel, pressure 101864.218750 kPa



polling TIME(ms) 8796 temp 24.361984 Cel, pressure 101852.468750 kPa
Polling TIME(ms) 8837 temp 24.367141 Cel, pressure 101852.687500 kPa
Polling TIME(ms) 8877 temp 24.360931 Cel, pressure 101852.640625 kPa
Polling TIME(ms) 8918 temp 24.357711 Cel, pressure 101852.593750 kPa
Polling TIME(ms) 8958 temp 24.355300 Cel, pressure 101852.781250 kPa
Polling TIME(ms) 8999 temp 24.365753 Cel, pressure 101852.796875 kPa
Polling TIME(ms) 9039 temp 24.354675 Cel, pressure 101853.062500 kPa
Polling TIME(ms) 9080 temp 24.363479 Cel, pressure 101852.796875 kPa
Polling TIME(ms) 9120 temp 24.353286 Cel, pressure 101853.078125 kPa
Polling TIME(ms) 9161 temp 24.366653 Cel, pressure 101852.781250 kPa
Polling TIME(ms) 9201 temp 24.361801 Cel, pressure 101852.875000 kPa
Polling TIME(ms) 9242 temp 24.355255 Cel, pressure 101852.734375 kPa
Polling TIME(ms) 9282 temp 24.369888 Cel, pressure 101852.875000 kPa
Polling TIME(ms) 9323 temp 24.361755 Cel, pressure 101852.765625 kPa

   <repeats endlessly>

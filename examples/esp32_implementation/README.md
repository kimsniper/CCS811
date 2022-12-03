# CCS811 - ESP-IDF

CCS811 i2c and SPI library for ESP-IDF.
ESP-IDF template used for this project: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_simple

## Overview

This example demonstrates usage of CCS811 for reading gas sensor values.

### Hardware Required

The CCS811 is an ultra-low power digital gas sensor solution which integrates a metal oxide (MOX) gas sensor to detect a wide range of Volatile Organic Compounds (VOCs) for indoor air quality monitoring with a microcontroller unit (MCU), which includes an Analog-to-Digital converter (ADC), and an I²C interface. It is easy to operate via a simple I2C command, you can read the datasheet [here](https://cdn.sparkfun.com/assets/2/4/2/9/6/CCS811_Datasheet.pdf).

#### Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| CCS811           | SDA            | SCL            |


For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash

```

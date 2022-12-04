# CCS811 - ESP-IDF

CCS811 i2c and SPI library for ESP-IDF.
ESP-IDF template used for this project: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_simple

## Overview

This example demonstrates usage of CCS811 for reading eCO2 & TVOC values.

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
I (322) example_usage: Hardware id: 0x81
I (332) example_usage: Hardware version: 0x12
I (332) example_usage: Boot version: 1.0.0
I (342) example_usage: App version: 1.1.0
I (572) example_usage: Drive mode setting successful
I (572) example_usage: CCS811 initialization successful
I (572) example_usage: Waiting for new data. . . .
I (2552) example_usage: eCO2: 0 ppm, TVOC: 0 ppb
I (3552) example_usage: eCO2: 0 ppm, TVOC: 0 ppb
I (4552) example_usage: eCO2: 0 ppm, TVOC: 0 ppb
I (5552) example_usage: eCO2: 400 ppm, TVOC: 0 ppb
I (6552) example_usage: eCO2: 406 ppm, TVOC: 0 ppb
I (7552) example_usage: eCO2: 406 ppm, TVOC: 0 ppb
I (8552) example_usage: eCO2: 409 ppm, TVOC: 1 ppb
I (9552) example_usage: eCO2: 409 ppm, TVOC: 1 ppb
I (10552) example_usage: eCO2: 406 ppm, TVOC: 0 ppb
I (11552) example_usage: eCO2: 409 ppm, TVOC: 1 ppb
I (12552) example_usage: eCO2: 414 ppm, TVOC: 2 ppb
I (13552) example_usage: eCO2: 409 ppm, TVOC: 1 ppb
I (14552) example_usage: eCO2: 409 ppm, TVOC: 1 ppb
I (15552) example_usage: eCO2: 414 ppm, TVOC: 2 ppb
I (16552) example_usage: eCO2: 414 ppm, TVOC: 2 ppb
I (17552) example_usage: eCO2: 414 ppm, TVOC: 2 ppb
I (18552) example_usage: eCO2: 417 ppm, TVOC: 2 ppb
I (19552) example_usage: eCO2: 426 ppm, TVOC: 3 ppb

```

/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAIN_CCS811_I2C
#define MAIN_CCS811_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include "ccs811_i2c_hal.h" 

typedef enum{
    WRITE_REG_INVALID = 0x00,
    READ_REG_INVALID = 0x01,
    MEASMODE_INVALID = 0x02,
    MAX_RESISTANCE = 0x03,
    HEATER_FAULT = 0x04,
    HEATER_SUPPLY = 0x05,
} ccs811_error_code_t;

typedef enum{
    FW_MODE_BOOT = 0x00,
    FW_MODE_APP = 0x01,
} ccs811_fw_mode_t;

typedef enum{
    DRV_MODE_IDLE = 0x00,
    DRV_MODE_CONST_POWER_IAQ = 0x01,
    DRV_MODE_PULSE_HEATING = 0x02,
    DRV_MODE_LOW_PULSE_HEATING = 0x03,
    DRV_MODE_CONST_POWER_SEN = 0x04,
} ccs811_drv_mode_t;

typedef struct{
    uint8_t datardy;
    uint8_t thresh;
} ccs811_int_pin_t;

typedef struct{
    uint16_t eco2;
    uint16_t tvoc;
} ccs811_alg_res_dt_t;

typedef struct{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} ccs811_calib_t;

typedef enum{
    T_SB_0_5 = 0x00,
    T_SB_62_5 = 0x01,
    T_SB_125 = 0x02,
    T_SB_250 = 0x03,
    T_SB_5000 = 0x04,
    T_SB_1000 = 0x05,
    T_SB_2000 = 0x06,
    T_SB_4000 = 0x07,
} ccs811_sb_time_t;

typedef enum{
    FILTER_OFF = 0x00,
    FILTER_2 = 0x01,
    FILTER_4 = 0x02,
    FILTER_8 = 0x03,
    FILTER_16 = 0x04,
} ccs811_filter_t;

typedef struct{
    ccs811_sb_time_t t_sb : 3;
    ccs811_filter_t filter : 3;
} ccs811_config_t;

typedef struct{
    uint8_t measuring : 1;
    uint8_t im_update : 1;
} ccs811_status_t;

typedef enum{
    OSRS_x0 = 0x00,
    OSRS_x1 = 0x01,
    OSRS_x2 = 0x02,
    OSRS_x4 = 0x03,
    OSRS_x8 = 0x04,
    OSRS_x16 = 0x05,
} ccs811_osrs_t;

typedef struct{
    ccs811_osrs_t osrs_tmp : 3;
    ccs811_osrs_t osrs_press : 3;
    ccs811_pwr_mode_t pmode : 2;
} ccs811_ctrl_meas_t;

typedef struct{
    uint32_t pressure;
    int32_t temperature;   
} ccs811_data_t;

/**
 * @brief CCS811 I2C slave address
 */
#define I2C_ADDRESS_CCS811              0x5A

/**
 * @brief CCS811 command code registers
 */
#define REG_STATUS                      0x00
#define REG_MEAS_MODE                   0x01
#define REG_ALG_RESULT_DATA             0x02
#define REG_RAW_DATA                    0x03
#define REG_ENV_DATA                    0x05
#define REG_NTC                         0x06
#define REG_THRESHOLDS                  0x10
#define REG_BASELINE                    0x11
#define REG_HW_ID                       0x20
#define REG_HW_VERSION                  0x21
#define REG_FW_BOOT_VERSION             0x23
#define REG_FW_APP_VERSION              0x24
#define REG_ERROR_ID                    0xE0
#define REG_SW_RESET                    0xFF

/**
 * @brief CCS811 interrupt pin macros
 */
#define REG_INTPIN_DATARDY_EN           0x01
#define REG_INTPIN_DATARDY_DIS          0x00
#define REG_INTPIN_THRESH_EN            0x01
#define REG_INTPIN_THRESH_DIS           0x00

/**
 * @brief CCS811 reset sequence macros
 */
#define RESET_SEQ_VAL_1                 0x11
#define RESET_SEQ_VAL_2                 0xE5
#define RESET_SEQ_VAL_3                 0x72
#define RESET_SEQ_VAL_4                 0x8A)

/**
 * @brief Other CCS811 macros
 */
#define REG_HW_ID_VAL                   0x81
#define REG_HW_VERSION                  0x11
#define REG_INTPIN_DATARDY_EN           0x01
#define REG_INTPIN_DATARDY_DIS          0x00

/**
 * @brief CCS811 error bit shift macros
 */
#define WRITE_REG_INVALID_SHIFT         ~(1 << 0)
#define READ_REG_INVALID_SHIFT          ~(1 << 1)
#define MEASMODE_INVALID_SHIFT          ~(1 << 2)
#define MAX_RESISTANCE_SHIFT            ~(1 << 3)
#define HEATER_FAULT_SHIFT              ~(1 << 4)
#define HEATER_SUPPLY_SHIFT             ~(1 << 5)

/**
 * @brief Read CCS811 calibration data
 */
int16_t ccs811_i2c_read_calib(ccs811_calib_t *clb);

/**
 * @brief Setting CCS811 calibration data
 */
int16_t ccs811_i2c_set_calib();

/**
 * @brief Read CCS811 configuration setting
 */
int16_t ccs811_i2c_read_config(uint8_t *cfg);

/**
 * @brief Set CCS811 filter setting
 */
int16_t ccs811_i2c_write_config_filter(ccs811_filter_t fltr);

/**
 * @brief Set CCS811 standby time setting
 */
int16_t ccs811_i2c_write_config_standby_time(ccs811_sb_time_t t_sb);

/**
 * @brief Read CCS811 ctrl_meas register
 */
int16_t ccs811_i2c_read_ctrl_meas(uint8_t *cfg);

/**
 * @brief Set CCS811 power mode setting
 */
int16_t ccs811_i2c_write_power_mode(ccs811_pwr_mode_t pmode);

/**
 * @brief Set CCS811 oversampling setting for temp/presusure
 */
int16_t ccs811_i2c_write_osrs(ccs811_ctrl_meas_t cfg);

/**
 * @brief Read CCS811 status (measuring/updating)
 */
int16_t ccs811_i2c_read_status(ccs811_status_t *sts);

/**
 * @brief Perform CCS811 device reset
 */
int16_t ccs811_i2c_reset();

/**
 * @brief Read CCS811 raw pressure data
 */
int16_t ccs811_i2c_read_pressure_r(int32_t *dt);

/**
 * @brief Read CCS811 raw temperature data
 */
int16_t ccs811_i2c_read_temperature_r(int32_t *dt);

/**
 * @brief Read CCS811 pressure and temperature data
 */
int16_t ccs811_i2c_read_data(ccs811_data_t *dt);

/**
 * @brief Read CCS811 part number
 */
int16_t ccs811_i2c_read_part_number(uint8_t *dt);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CCS811_I2C */

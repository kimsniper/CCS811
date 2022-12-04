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
    uint16_t co2_ltm_thr;
    uint16_t co2_mth_thr;
    uint8_t hysteresis;
} ccs811_threshold_t;

typedef struct{
    uint16_t eco2;
    uint16_t tvoc;
} ccs811_alg_res_dt_t;

typedef struct{
    float humidity;
    float temperature;   
} ccs811_env_data_t;

typedef struct{
    uint16_t V_RREF;
    uint16_t V_RNTC;   
} ccs811_ntc_t;

typedef struct{
    uint16_t sens_volt;
    uint8_t sens_amp;   
} ccs811_raw_t;

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
#define REG_APP_START                   0xF4
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
#define RESET_SEQ_VAL_4                 0x8A

/**
 * @brief Other CCS811 macros
 */
#define REG_HW_ID_VAL                   0x81
#define REG_INTPIN_DATARDY_EN           0x01
#define REG_INTPIN_DATARDY_DIS          0x00

/**
 * @brief CCS811 error bit shift macros
 */
#define WRITE_REG_INVALID_SHIFT         (1 << 0)
#define READ_REG_INVALID_SHIFT          (1 << 1)
#define MEASMODE_INVALID_SHIFT          (1 << 2)
#define MAX_RESISTANCE_SHIFT            (1 << 3)
#define HEATER_FAULT_SHIFT              (1 << 4)
#define HEATER_SUPPLY_SHIFT             (1 << 5)

/**
 * @brief Read CCS811 status
 */
int16_t ccs811_i2c_read_status(uint8_t *sts);

int16_t ccs811_i2c_read_meas_mode(uint8_t *meas_mode);

int8_t ccs811_i2c_fw_mode();

int8_t ccs811_i2c_fw_app_valid();

int16_t ccs811_i2c_start_app();

int8_t ccs811_i2c_data_ready();

int8_t ccs811_i2c_error();

int16_t ccs811_i2c_read_drive_mode(ccs811_drv_mode_t *drv_mode);

int16_t ccs811_i2c_write_drive_mode(ccs811_drv_mode_t drv_mode);

int16_t ccs811_i2c_intpin_init(ccs811_int_pin_t intpin);

int16_t ccs811_i2c_write_threshold(ccs811_threshold_t val);

void ccs811_error_decode(uint8_t error);

int16_t ccs811_i2c_read_alg_result_data(ccs811_alg_res_dt_t *alg_data);

int16_t ccs811_i2c_read_env_data(ccs811_env_data_t *env_data);

int16_t ccs811_i2c_read_raw_data(ccs811_raw_t *raw_data);

int16_t ccs811_i2c_read_ntc(ccs811_ntc_t *ntc);

int16_t ccs811_i2c_reset();

int16_t ccs811_i2c_read_hw_id(uint8_t *id);

int16_t ccs811_i2c_read_hw_version(uint8_t *ver);

int16_t ccs811_i2c_read_boot_version(uint8_t *ver);

int16_t ccs811_i2c_read_app_version(uint8_t *ver);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CCS811_I2C */

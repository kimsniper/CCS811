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

#include "ccs811_i2c.h" 
#include "ccs811_i2c_hal.h" 

#include "stdio.h"

ccs811_calib_t calib_params;

int16_t ccs811_i2c_read_status(uint8_t *sts)
{
    uint8_t reg = REG_STATUS;
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, sts, 1);
    return err;
}

int16_t ccs811_i2c_read_meas_mode(uint8_t *meas_mode)
{
    uint8_t reg = REG_MEAS_MODE;
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, meas_mode, 1);
    return err;
}

int8_t ccs811_i2c_fw_mode()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x80) >> 0x07 : CCS811_ERR;
}

int8_t ccs811_i2c_fw_app_valid()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x10) >> 0x04 : CCS811_ERR;
}

int8_t ccs811_i2c_data_ready()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x8) >> 0x03 : CCS811_ERR;
}

int8_t ccs811_i2c_read_error_sts()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x01) : CCS811_ERR;
}

int16_t ccs811_i2c_read_drive_mode(ccs811_drv_mode_t *drv_mode)
{
    uint8_t reg = REG_MEAS_MODE;
    uint8_t meas_mode;
    int16_t err = ccs811_i2c_read_meas_mode(&meas_mode);
    *drv_mode = (meas_mode & 0x70) >> 4;
    return err;
}

int16_t ccs811_i2c_drive_mode(ccs811_drv_mode_t drv_mode)
{
    uint8_t reg = REG_MEAS_MODE;
    uint8_t data[2], meas_mode;
    int16_t err = ccs811_i2c_read_meas_mode(&meas_mode);
    data[0] = reg;
    data[1] = (meas_mode & 0x8F) | (drv_mode << 4);
    err += ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data));
    return err;
}

int16_t ccs811_i2c_intpin_init(ccs811_int_pin_t intpin)
{
    uint8_t reg = REG_MEAS_MODE;
    uint8_t data[2], meas_mode;
    int16_t err = ccs811_i2c_read_meas_mode(&meas_mode);
    data[0] = reg;
    data[1] = (meas_mode & 0xF3) | (intpin.datardy << 3) | (intpin.thresh << 2);
    err += ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data));
    return err;
}

void ccs811_error_decode(uint8_t error){

    if(error & WRITE_REG_INVALID_SHIFT)
    {
        printf("WRITE_REG_INVALID");
    }
    if(error & READ_REG_INVALID_SHIFT)
    {
        printf("READ_REG_INVALID");
    }
    if(error & MEASMODE_INVALID_SHIFT)
    {
        printf("MEASMODE_INVALID");
    }
    if(error & MAX_RESISTANCE_SHIFT)
    {
        printf("MAX_RESISTANCE");
    }
    if(error & HEATER_FAULT_SHIFT)
    {
        printf("HEATER_FAULT");
    }
    if(error & HEATER_SUPPLY_SHIFT)
    {
        printf("HEATER_SUPPLY");
    }
}

int16_t ccs811_i2c_read_alg_result_data(ccs811_alg_res_dt_t *alg_data)
{
    uint8_t reg = REG_ALG_RESULT_DATA;
    uint8_t data[6];
    int16_t err = htu21d_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));

    if(data[5]) 
    {
        /* Uncomment this to decode and print error */
        //ccs811_error_decode(data[5]);
        return data[5];
    }        

    alg_data->eco2 = (data[0] << 8) | data[1];
    alg_data->tvoc = (data[2] << 8) | data[3];

    return err;
}

int16_t ccs811_i2c_reset()
{
    uint8_t reg = REG_SW_RESET;
    uint8_t data[5];
    
    data[0] = reg;
    data[1] = RESET_SEQ_VAL_1;
    data[2] = RESET_SEQ_VAL_2;
    data[3] = RESET_SEQ_VAL_3;
    data[4] = RESET_SEQ_VAL_4;
    
    return err = ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data));;
}
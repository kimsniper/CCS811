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

int16_t ccs811_i2c_read_status(uint8_t *sts)
{
    uint8_t reg = REG_STATUS;
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, sts, 1);
    printf("ccs811_i2c_read_status: %d", *sts);
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

int16_t ccs811_i2c_start_app()
{
    uint8_t reg = REG_APP_START;
    uint8_t data[1] = {reg};
    uint8_t sts;

    if(ccs811_i2c_read_status(&sts) == CCS811_ERR)  
        return CCS811_ERR;

    if(!(sts & (1<<7)))
    {
        if(!(sts & (1<<4)))
            return CCS811_ERR;
    }

    if(ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data)) == CCS811_ERR)
        return CCS811_ERR;

    ccs811_i2c_hal_ms_delay(100);

    return ccs811_i2c_fw_mode() ? CCS811_OK : CCS811_ERR;
}

int8_t ccs811_i2c_data_ready()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x08) >> 0x03 : CCS811_ERR;
}

int8_t ccs811_i2c_error()
{
    uint8_t sts;
    return ccs811_i2c_read_status(&sts) == CCS811_OK ? (sts & 0x01) : CCS811_ERR;
}

int16_t ccs811_i2c_read_drive_mode(ccs811_drv_mode_t *drv_mode)
{
    uint8_t meas_mode;
    int16_t err = ccs811_i2c_read_meas_mode(&meas_mode);
    *drv_mode = (meas_mode & 0x70) >> 4;
    return err;
}

int16_t ccs811_i2c_write_drive_mode(ccs811_drv_mode_t drv_mode)
{
    uint8_t reg = REG_MEAS_MODE;
    uint8_t data[2], meas_mode;
    int16_t err = ccs811_i2c_read_meas_mode(&meas_mode);
    data[0] = reg;
    data[1] = (meas_mode & 0x0F) | (drv_mode << 4);
    printf("data[1]: %d", data[1]);
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

int16_t ccs811_i2c_write_threshold(ccs811_threshold_t val)
{
    uint8_t reg = REG_THRESHOLDS;
    uint8_t data[6];

    data[0] = reg;
    data[1] = val.co2_ltm_thr >> 8;
    data[2] = val.co2_ltm_thr & 0xFF;
    data[3] = val.co2_mth_thr >> 8;
    data[4] = val.co2_mth_thr & 0xFF;
    data[5] = val.hysteresis;

    return ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data));
}

void ccs811_error_decode(uint8_t error){

    if(error & WRITE_REG_INVALID_SHIFT)
    {
        printf("\nWRITE_REG_INVALID");
    }
    if(error & READ_REG_INVALID_SHIFT)
    {
        printf("\nREAD_REG_INVALID");
    }
    if(error & MEASMODE_INVALID_SHIFT)
    {
        printf("\nMEASMODE_INVALID");
    }
    if(error & MAX_RESISTANCE_SHIFT)
    {
        printf("\nMAX_RESISTANCE");
    }
    if(error & HEATER_FAULT_SHIFT)
    {
        printf("\nHEATER_FAULT");
    }
    if(error & HEATER_SUPPLY_SHIFT)
    {
        printf("\nHEATER_SUPPLY");
    }
    printf("\n");
}

int16_t ccs811_i2c_read_alg_result_data(ccs811_alg_res_dt_t *alg_data)
{
    uint8_t reg = REG_ALG_RESULT_DATA;
    uint8_t data[6];
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));

    if(data[5]) 
    {
        /* Uncomment this to decode and print error */
        //ccs811_error_decode(data[5]);
        return data[5];
    }    

    if(!(data[4] & (1<<3))) /* Check if new data is ready */
        return CCS811_ERR;
   
    alg_data->eco2 = (data[0] << 8) | data[1];
    alg_data->tvoc = (data[2] << 8) | data[3];

    return err;
}

int16_t ccs811_i2c_read_env_data(ccs811_env_data_t *env_data)
{
    uint8_t reg = REG_ENV_DATA;
    uint8_t data[4];
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));    

    uint16_t hum_16bit = (data[0] << 8) | data[1];
    uint16_t temp_16bit = (data[2] << 8) | data[3];
    float init_mul = 128;

    for(int i=15; i>=0;i--)
    {
        init_mul /= 2;
        if(temp_16bit & (1 << i))
            env_data->temperature += init_mul;
        if(hum_16bit & (1 << i))
            env_data->humidity += init_mul;
    }

    env_data->temperature -= 25;

    return err;
}

int16_t ccs811_i2c_read_ntc(ccs811_ntc_t *ntc)
{
    uint8_t reg = REG_NTC;
    uint8_t data[4];
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));    

    ntc->V_RREF = (data[0] << 8) | data[1];
    ntc->V_RNTC = (data[2] << 8) | data[3];

    return err;
}

int16_t ccs811_i2c_reset()
{
    uint8_t reg = REG_SW_RESET;
    uint8_t data[5] = {reg, RESET_SEQ_VAL_1, RESET_SEQ_VAL_2, RESET_SEQ_VAL_3, RESET_SEQ_VAL_4};
    
    return ccs811_i2c_hal_write(I2C_ADDRESS_CCS811, data, sizeof(data));;
}

int16_t ccs811_i2c_read_hw_id(uint8_t *id)
{
    uint8_t reg = REG_HW_ID;
    return ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, id, 1);
}

int16_t ccs811_i2c_read_hw_version(uint8_t *ver)
{
    uint8_t reg = REG_HW_VERSION;
    return ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, ver, 1);
}

int16_t ccs811_i2c_read_boot_version(uint8_t *ver)
{
    uint8_t reg = REG_FW_BOOT_VERSION;
    uint8_t data[2];
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));

    ver[0] = data[0] >> 4;
    ver[1] = data[0] & 0x0F;
    ver[2] = data[1];
    return err;
}

int16_t ccs811_i2c_read_app_version(uint8_t *ver)
{
    uint8_t reg = REG_FW_APP_VERSION;
    uint8_t data[2];
    int16_t err = ccs811_i2c_hal_read(I2C_ADDRESS_CCS811, &reg, data, sizeof(data));
    ver[0] = data[0] >> 4;
    ver[1] = data[0] & 0x0F;
    ver[2] = data[1];
    return err;
}
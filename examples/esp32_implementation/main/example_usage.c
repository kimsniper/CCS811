#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//CCS811 components
#include "ccs811_i2c.h"
#include "ccs811_i2c_hal.h"

static const char *TAG = "example_usage";

#include "driver/i2c.h"

void app_main(void)
{
    esp_err_t err = CCS811_OK;
    uint8_t id = 0, ver, boot_version[3], app_version[3];

    ccs811_i2c_hal_init();

    err = ccs811_i2c_reset();
    if(err != CCS811_OK) 
        ESP_LOGE(TAG, "Error resetting the device!");

    ccs811_i2c_hal_ms_delay(100);
    if(err != CCS811_OK) 
        ESP_LOGE(TAG, "Unable to start app!");

    err += ccs811_i2c_read_hw_id(&id);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Hardware id: 0x%02x", id);
    }
    else{
        ESP_LOGE(TAG, "Unable to read hardware id!");
    }

    err += ccs811_i2c_read_hw_version(&ver);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Hardware version: 0x%02x", ver);
    }
    else{
        ESP_LOGE(TAG, "Unable to read hardware version!");
    }

    err += ccs811_i2c_read_boot_version(boot_version);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Boot version: %d.%d.%d", boot_version[0],boot_version[1],boot_version[2]);
    }
    else{
        ESP_LOGE(TAG, "Unable to read boot version!");
    }

    err += ccs811_i2c_read_app_version(app_version);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "App version: %d.%d.%d", app_version[0],app_version[1],app_version[2]);
    }
    else{
        ESP_LOGE(TAG, "Unable to read app version!");
    }

    /* Device setting */
    err += ccs811_i2c_write_drive_mode(DRV_MODE_CONST_POWER_IAQ);
    ESP_LOGI(TAG, "Drive mode setting %s", err == ESP_OK ? "successful" : "failed");

    ccs811_i2c_start_app();

    uint8_t cfg;
    ccs811_i2c_read_meas_mode(&cfg);
    ESP_LOGW(TAG, "meas_mode: %d", cfg);

    //ccs811_env_data_t val;
    //ccs811_i2c_read_env_data(&val);
    //ESP_LOGW(TAG, "hum: %.2f, temp: %.2f", val.humidity, val.temperature);

    if (err == CCS811_OK)
    {
        ESP_LOGI(TAG, "CCS811 initialization successful");
        ccs811_alg_res_dt_t ccs811_dt;
        while(1)
        {
            //Reading here
            while(!ccs811_i2c_data_ready()){
                ESP_LOGE(TAG, "Dev not ready!");
                ccs811_i2c_hal_ms_delay(1);
            }
                
            if(ccs811_i2c_read_alg_result_data(&ccs811_dt) == CCS811_OK)
            {
                ESP_LOGI(TAG, "eCO2: %d ppm", ccs811_dt.eco2);
                ESP_LOGI(TAG, "TVOC: %d ppb", ccs811_dt.tvoc);
            }
            else{
                ESP_LOGE(TAG, "Error reading data!");
            }
            
            ccs811_i2c_hal_ms_delay(1000);
        }
    }
    else{
        ESP_LOGE(TAG, "CCS811 initialization failed!");
    }
}

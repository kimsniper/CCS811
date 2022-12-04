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
    esp_err_t err;
    uint8_t id = 0, ver, boot_version[3], app_version[3];

    ccs811_i2c_hal_init();

    err = ccs811_i2c_read_hw_id(&id);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Hardware id: 0x%02x", id);
    }
    else{
        ESP_LOGE(TAG, "Unable to read hardware id!");
        return;
    }

    err = ccs811_i2c_read_hw_version(&ver);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Hardware version: 0x%02x", ver);
    }
    else{
        ESP_LOGE(TAG, "Unable to read hardware version!");
        return;
    }

    err = ccs811_i2c_read_boot_version(boot_version);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Boot version: %d.%d.%d", boot_version[0],boot_version[1],boot_version[2]);
    }
    else{
        ESP_LOGE(TAG, "Unable to read boot version!");
        return;
    }

    err = ccs811_i2c_read_app_version(app_version);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "App version: %d.%d.%d", app_version[0],app_version[1],app_version[2]);
    }
    else{
        ESP_LOGE(TAG, "Unable to read app version!");
        return;
    }

    err = ccs811_i2c_reset();
    if(err != CCS811_OK) 
    {
        ESP_LOGE(TAG, "Error resetting the device!");
        return;
    }

    ccs811_i2c_hal_ms_delay(100);

    err = ccs811_i2c_start_app();
    if(err != CCS811_OK) 
    {
        ESP_LOGE(TAG, "Unable to start app!");
        return;
    }

    /* Device setting */
    err += ccs811_i2c_write_drive_mode(DRV_MODE_CONST_POWER_IAQ);
    ESP_LOGI(TAG, "Drive mode setting %s", err == ESP_OK ? "successful" : "failed");

    if (err == CCS811_OK)
    {
        ESP_LOGI(TAG, "CCS811 initialization successful");
        ESP_LOGI(TAG, "Waiting for new data. . . .");
        ccs811_alg_res_dt_t ccs811_dt;
        ccs811_env_data_t env_data;
        env_data.temperature = 23.5;
        env_data.humidity = 48.5;
        ccs811_i2c_write_env_data(env_data);

        while(1)
        {
            //Reading here
            while(!ccs811_i2c_data_ready())
                ccs811_i2c_hal_ms_delay(1);
               
            if(ccs811_i2c_read_alg_result_data(&ccs811_dt) == CCS811_OK)
                ESP_LOGI(TAG, "eCO2: %d ppm, TVOC: %d ppb", ccs811_dt.eco2, ccs811_dt.tvoc);
            else
                ESP_LOGE(TAG, "Error reading alg result data!");

            ccs811_i2c_hal_ms_delay(1000);
        }
    }
    else{
        ESP_LOGE(TAG, "CCS811 initialization failed!");
    }
}

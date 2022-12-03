#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//CCS811 components
#include "ccs811_i2c.h"
#include "ccs811_i2c_hal.h"

static const char *TAG = "example_usage";

void app_main(void)
{
    esp_err_t err;
    uint8_t id = 0;

    ccs811_i2c_hal_init();

    err = ccs811_i2c_reset();
    if(err != CCS811_OK) ESP_LOGE(TAG, "Error setting the device!");

    err += ccs811_i2c_read_part_number(&id);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "Part number: 0x%02x", id);
    } 
    else{
        ESP_LOGE(TAG, "Unable to read part number!");
    }

    err += ccs811_i2c_set_calib();
    ESP_LOGI(TAG, "Calibration data setting: %s", err == CCS811_OK ? "Successful" : "Failed");

    //uint8_t cfg;
    //ccs811_i2c_read_config(&cfg);
    //ESP_LOGW(TAG, "read_config: %d", cfg);

    if (err == CCS811_OK && id == REG_HW_ID_VAL)
    {
        ESP_LOGI(TAG, "CCS811 initialization successful");
        ccs811_data_t ccs811_dt;
        while(1)
        {
            //Reading here
            if(ccs811_i2c_read_data(&ccs811_dt) == CCS811_OK)
            {
                ESP_LOGI(TAG, "Pressure: %.01f Pa", (float)ccs811_dt.pressure/256);
                ESP_LOGI(TAG, "Temperature: %.01f °C", (float)ccs811_dt.temperature/100);
            }
            else{
                ESP_LOGE(TAG, "Error reading data!");
            }
            
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    else{
        ESP_LOGE(TAG, "CCS811 initialization failed!");
    }
}

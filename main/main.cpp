#include <stdio.h>
#include "BTHome.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "secrets.hpp"

BTHome bthome;

extern "C" void app_main(void)
{

    ESP_LOGI("main", "Starting up...");

    bthome.begin(DEVICE_NAME, true, BIND_KEY);
    vTaskDelay(250 / portTICK_PERIOD_MS);

    ESP_LOGI("main", "BTHome initialization complete; looping...");


    for(;;)
    {
        ESP_LOGI("main", "Loop start!");
        bthome.addMeasurement(ID_TEMPERATURE_PRECISE, 35.00f);
        bthome.addMeasurement(ID_HUMIDITY_PRECISE, 50.12f);
        bthome.sendPacket();

        ESP_LOGI("main", "Loop end; waiting 2500ms!");
        vTaskDelay(2500 / portTICK_PERIOD_MS);

    }
    bthome.stop();

}
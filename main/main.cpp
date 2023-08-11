#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <bmp280.h>

#include "BTHome.h"
#include "secrets.hpp"
#include "NimBLEDevice.h"

static const char* LOG_TAG = "BTHomeMain";

static constexpr float TEMPERATURE_THRESHOLD = 0.2f;
static constexpr float HUMIDITY_THRESHOLD = 0.75f;
static constexpr TickType_t BROADCAST_TIME_MS = (TickType_t)(CONFIG_BTHOME_BROADCAST_TIME_MS);
static constexpr TickType_t BROADCAST_TIME_TICKS = BROADCAST_TIME_MS / portTICK_PERIOD_MS;
static constexpr TickType_t SLEEP_TIME_MS = (TickType_t)(CONFIG_BTHOME_SLEEP_TIME_MS);
static constexpr TickType_t SLEEP_TIME_US = (TickType_t)(CONFIG_BTHOME_SLEEP_TIME_MS * 1000);
static constexpr TickType_t SLEEP_TIME_TICKS = SLEEP_TIME_MS / portTICK_PERIOD_MS;
#ifdef CONFIG_BTHOME_DO_THRESHOLD_TEST
static constexpr bool DO_THRESHOLD_TEST = true;
#else
static constexpr bool DO_THRESHOLD_TEST = false;
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR float lastSentTemp = -1;
RTC_DATA_ATTR float lastSentHumidity = -1;
RTC_DATA_ATTR bmp280_params_t bme280_params;
RTC_DATA_ATTR bmp280_t bme280_dev;

volatile bool runningLEDCFade = false;

extern "C" void app_main(void)
{
    uint16_t ledc_loops = 0;
    TaskHandle_t fade_task_handle = NULL;
    BTHome bthome;
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    bootCount += 1;
    switch(wakeup_reason)
    {
        // case ESP_SLEEP_WAKEUP_EXT0 : ESP_LOGI(LOG_TAG, "Wakeup caused by external signal using RTC_IO; boot count: %d", bootCount); break;
        // case ESP_SLEEP_WAKEUP_EXT1 : ESP_LOGI(LOG_TAG, "Wakeup caused by external signal using RTC_CNTL; boot count: %d", bootCount); break;
        case ESP_SLEEP_WAKEUP_GPIO : 
            ESP_LOGI(LOG_TAG, "Wakeup caused by external signal using GPIO; boot count: %d", bootCount); 
            runningLEDCFade = true;
            break;
        case ESP_SLEEP_WAKEUP_TIMER : 
            ESP_LOGI(LOG_TAG, "Wakeup caused by timer; boot count: %d", bootCount); 
            break;
        // case ESP_SLEEP_WAKEUP_TOUCHPAD : ESP_LOGI(LOG_TAG, "Wakeup caused by touchpad; boot count: %d", bootCount); break;
        // case ESP_SLEEP_WAKEUP_ULP : ESP_LOGI(LOG_TAG, "Wakeup caused by ULP program; boot count: %d", bootCount); break;
        // default : ESP_LOGI(LOG_TAG, "Wakeup was not caused by deep sleep: %d, boot count: %d", wakeup_reason, bootCount); break;
        default: 
            ESP_LOGI(LOG_TAG, "Wakeup was not caused by deep sleep: %d, boot count: %d", wakeup_reason, bootCount); 
            // let's see if this even works... here we're gonna only initialize the BME280 on
            // first wakeup, and keep the necessary data around in RTC.........
            bme280_params = {
                .mode = BMP280_MODE_FORCED,
                .filter = BMP280_FILTER_OFF,
                .oversampling_pressure = BMP280_ULTRA_LOW_POWER,
                .oversampling_temperature = BMP280_ULTRA_LOW_POWER,
                .oversampling_humidity = BMP280_ULTRA_LOW_POWER,
                .standby = BMP280_STANDBY_125, // unused in forced mode
            };
            memset(&bme280_dev, 0, sizeof(bmp280_t));
            bmp280_init_desc(&bme280_dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
            if(bmp280_init(&bme280_dev, &bme280_params) != ESP_OK) {
                ESP_LOGE(LOG_TAG, "Failed to initialize BME; failing.");
                abort();
            }
            break;       
    }


    for(;;)
    {
        bool btInitialized = false;
        float temperature, humidity, pressure;
        esp_err_t bmp_err;
        if((bmp_err = bmp280_force_measurement(&bme280_dev)) == ESP_OK) {
            bool busy = true;
            while(busy && bmp_err == ESP_OK) {
                bmp_err = bmp280_is_measuring(&bme280_dev, &busy);
                if(busy && bmp_err == ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));
            }
            if(bmp_err == ESP_OK) {
                bmp_err = bmp280_read_float(&bme280_dev, &temperature, &pressure, &humidity);
            }
        }
        
        // esp_err_t aht_err = aht_get_and_print(&aht_device, &temperature, &humidity);
        if(bmp_err == ESP_OK) {
            if(
                !DO_THRESHOLD_TEST || (
                    fade_task_handle != NULL || // always send if we're in broadcast pairing mode
                    fabs(temperature - lastSentTemp) > TEMPERATURE_THRESHOLD ||
                    fabs(humidity - lastSentHumidity) > HUMIDITY_THRESHOLD
                )
            ) {
                if(!btInitialized) {
                    ESP_LOGI(LOG_TAG, "Beginning BTHome initialization...");
                    bthome.begin(DEVICE_NAME, true, BIND_KEY);
                    btInitialized = true;
                    ESP_LOGI(LOG_TAG, "BTHome initialization complete!");
                }

                // led_set_color(led_device, rgb_from_values(0, 255, 0));
                lastSentTemp = temperature;
                lastSentHumidity = humidity;
                
                bthome.addMeasurement(ID_TEMPERATURE_PRECISE, temperature);
                bthome.addMeasurement(ID_HUMIDITY_PRECISE, humidity);
                // convert to hPa for upload...
                bthome.addMeasurement(ID_PRESSURE, pressure * 0.01f);
                ESP_LOGV(LOG_TAG, "Broadcasting...");
                bthome.sendPacket(BROADCAST_TIME_MS);
                ESP_LOGV(LOG_TAG, "Broadcast complete!");
            } else {
                ESP_LOGD(LOG_TAG, "Not enough difference, bypassing broadcast");
            }
        } else {
            ESP_LOGE(LOG_TAG, "Bypassing sensor update on failed read.");
        }

        if(runningLEDCFade) {
            ledc_loops += 1;
            bthome.stop();
            vTaskDelay(pdMS_TO_TICKS(2500));
            if(eTaskGetState(fade_task_handle) == eDeleted) {
                fade_task_handle = NULL;
            }
        } else {
            if(SLEEP_TIME_MS > 0) {
                ESP_LOGI(LOG_TAG, "Heading for bed; sleep time is %ld ms", SLEEP_TIME_MS);
                // led_set_color(led_device, rgb_from_values(0,0,0));

                if(btInitialized) {
                    bthome.stop();
                    bthome.end();
                }
                /*
                auto res = esp_deep_sleep_enable_gpio_wakeup(1<<3,ESP_GPIO_WAKEUP_GPIO_LOW);
                if(res != ESP_OK) 
                {
                    ESP_LOGE(LOG_TAG, "Error setting sleep GPIO: %d (%s)", res, esp_err_to_name(res));
                }
                */
                esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
                esp_deep_sleep_start();
            }
        }
    }
}
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

#include <aht.h>
#include <led_strip.h>
#include <rgb.h>
#include <tweeny/tweeny.h>
#include <tweeny/easing.h>

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

static void led_set_color(led_strip_t& strip, rgb_t color) {
    ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, color));
    ESP_ERROR_CHECK(led_strip_flush(&strip));
}

static inline led_strip_t led_initialize() {
    led_strip_install();
    led_strip_t strip = {
        .type = LED_STRIP_WS2812,
        .is_rgbw = false,
        .brightness = 0,
        .length = 1,
        .gpio = GPIO_NUM_2,
        .channel = RMT_CHANNEL_0,
        .buf = NULL,
    };

    ESP_ERROR_CHECK(led_strip_init(&strip));
    return strip;
}

static inline aht_t aht_initialize() {
    ESP_ERROR_CHECK(i2cdev_init());
    aht_t dev = { 
        .i2c_dev = I2C_NUM_0,
        .type = AHT_TYPE_AHT20,
        .mode = AHT_MODE_NORMAL,
    };

    ESP_ERROR_CHECK(aht_init_desc(&dev, AHT_I2C_ADDRESS_GND, I2C_NUM_0, GPIO_NUM_7, GPIO_NUM_6));
    ESP_ERROR_CHECK(aht_init(&dev));
    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated) {
        ESP_LOGD(LOG_TAG, "AHT20 Sensor calibrated; nice!");
    } else {
        ESP_LOGW(LOG_TAG, "AHT20 Sensor not calibrated, better figure that out...");
    }

    return dev;
}

static esp_err_t aht_get_and_print(aht_t *dev, float *temperature, float *humidity)
{
    esp_err_t res = aht_get_data(dev, temperature, humidity);
    if (res == ESP_OK)
        ESP_LOGI(LOG_TAG, "Temperature: %.1f°C, Humidity: %.2f%%", *temperature, *humidity);
    else
        ESP_LOGE(LOG_TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));
    return res;
}

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR float lastSentTemp = -1;
RTC_DATA_ATTR float lastSentHumidity = -1;

volatile bool fade_task_running = false;
static void fade_task_entry(void *pvParams) {
    ESP_LOGV(LOG_TAG, "Hello from fade task!"); 
    fade_task_running = true;
    led_strip_t led = led_initialize();
    led_set_color(led, rgb_from_values(0,255,0));
    TickType_t dt = 0;
    auto tween = tweeny::from((uint8_t)0)
                        .to((uint8_t)255).during((TickType_t)1000).via(tweeny::easing::sinusoidalIn)
                        .to((uint8_t)0).during((TickType_t)1000).via(tweeny::easing::sinusoidalIn);
    
    int loopCount = 0;
    
    for(;loopCount < 20;) {
        tween.step(dt);
        if(tween.progress() >= 1.0f) {
            tween.seek(0.0f); 
            loopCount += 1;
        }
        led.brightness = tween.peek();
        ESP_ERROR_CHECK(led_strip_flush(&led));
        dt = 33; // ms
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }

    led_set_color(led, rgb_from_values(0,0,0));
    fade_task_running = false;
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
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
            fade_task_running = true;
            xTaskCreate(
                fade_task_entry, 
                "led-task", 
                configMINIMAL_STACK_SIZE * 8, 
                NULL, 
                tskIDLE_PRIORITY, 
                &fade_task_handle);
            break;
        // case ESP_SLEEP_WAKEUP_TIMER : ESP_LOGI(LOG_TAG, "Wakeup caused by timer; boot count: %d", bootCount); break;
        // case ESP_SLEEP_WAKEUP_TOUCHPAD : ESP_LOGI(LOG_TAG, "Wakeup caused by touchpad; boot count: %d", bootCount); break;
        // case ESP_SLEEP_WAKEUP_ULP : ESP_LOGI(LOG_TAG, "Wakeup caused by ULP program; boot count: %d", bootCount); break;
        // default : ESP_LOGI(LOG_TAG, "Wakeup was not caused by deep sleep: %d, boot count: %d", wakeup_reason, bootCount); break;
        default: 
            led_strip_t led_device = led_initialize();
            led_set_color(led_device, rgb_from_values(0,0,0));
            break;
    }

    aht_t aht_device = aht_initialize();
    // led_strip_t led_device = led_initialize();
    // led_set_color(led_device, rgb_from_values(255, 255, 255));

    for(;;)
    {
        bool btInitialized = false;
        float temperature, humidity;
        esp_err_t aht_err = aht_get_and_print(&aht_device, &temperature, &humidity);
        if(aht_err == ESP_OK) {
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
                ESP_LOGV(LOG_TAG, "Broadcasting...");
                bthome.sendPacket(BROADCAST_TIME_MS);
                ESP_LOGV(LOG_TAG, "Broadcast complete!");
                // led_set_color(led_device, rgb_from_values(0,0,0));
            } else {
                ESP_LOGD(LOG_TAG, "Not enough difference, bypassing broadcast");
            }
        } else {
            ESP_LOGE(LOG_TAG, "Bypassing sensor update on failed read.");
        }

        if(fade_task_handle != NULL) {
            bthome.stop();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
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
                auto res = esp_deep_sleep_enable_gpio_wakeup(1<<3,ESP_GPIO_WAKEUP_GPIO_LOW);
                if(res != ESP_OK) 
                {
                    ESP_LOGE(LOG_TAG, "Error setting sleep GPIO: %d (%s)", res, esp_err_to_name(res));
                }
                esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
                esp_deep_sleep_start();
            }
        }
    }
}
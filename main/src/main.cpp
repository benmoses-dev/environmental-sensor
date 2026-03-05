#include "bme280.hpp"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "network.hpp"

static const char *TAG = "MAIN";

BME280 bme;
QueueHandle_t temp_queue;
QueueHandle_t hum_queue;
QueueHandle_t pres_queue;

void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    while (true) {
        const Event tEvent = {bme.readTemperature(), getTime()};
        const Event hEvent = {bme.readHumidity(), getTime()};
        const Event pEvent = {bme.readPressure(), getTime()};
        xQueueSend(temp_queue, &tEvent, portMAX_DELAY);
        xQueueSend(hum_queue, &hEvent, portMAX_DELAY);
        xQueueSend(pres_queue, &pEvent, portMAX_DELAY);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}

void tempTask(void *pvParameters) {
    Event temp;
    while (true) {
        if (xQueueReceive(temp_queue, &temp, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Temp: %.2f °C", temp.val);
        }
    }
}

void humTask(void *pvParameters) {
    Event hum;
    while (true) {
        if (xQueueReceive(hum_queue, &hum, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Humidity: %.2f %%", hum.val);
        }
    }
}

void presTask(void *pvParameters) {
    Event pres;
    while (true) {
        if (xQueueReceive(pres_queue, &pres, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Pressure: %.2f hPa", pres.val / 100.0f);
        }
    }
}

extern "C" void app_main() {
    if (!bme.init()) {
        ESP_LOGE(TAG, "BME280 failed to initialise!");
        return;
    }
    if (!wifiConnect()) {
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
        return;
    }
    if (!obtainTime()) {
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
        return;
    }
    temp_queue = xQueueCreate(10, sizeof(Event));
    hum_queue = xQueueCreate(10, sizeof(Event));
    pres_queue = xQueueCreate(10, sizeof(Event));
    xTaskCreate(readTask, "ReadTask", 4096, NULL, 5, NULL);
    xTaskCreate(tempTask, "TemperatureTask", 4096, NULL, 5, NULL);
    xTaskCreate(humTask, "HumidityTask", 4096, NULL, 5, NULL);
    xTaskCreate(presTask, "PressureTask", 4096, NULL, 5, NULL);
}

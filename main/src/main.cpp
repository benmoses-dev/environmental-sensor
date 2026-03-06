#include "bme280.hpp"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "utils.hpp"
#include "wifi.hpp"

static const char *TAG = "MAIN";

WIFI wifi;
BME280 bme;
QueueHandle_t eventQueue;

void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    while (true) {
        const auto t = wifi.getTime();
        const Event tEvent = {bme.readTemperature(), t, EventType::TEMP};
        const Event hEvent = {bme.readHumidity(), t, EventType::HUM};
        const Event pEvent = {bme.readPressure(), t, EventType::PRES};
        xQueueSend(eventQueue, &tEvent, portMAX_DELAY);
        xQueueSend(eventQueue, &hEvent, portMAX_DELAY);
        xQueueSend(eventQueue, &pEvent, portMAX_DELAY);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}

void logTask(void *pvParameters) {
    Event event;
    while (true) {
        if (!WIFI::connected) {
            delay_ms(1000);
            continue;
        }
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY)) {
            if (event.type == EventType::TEMP) {
                ESP_LOGI(TAG, "Temp: %.2f °C", event.val);
            } else if (event.type == EventType::HUM) {
                ESP_LOGI(TAG, "Humidity: %.2f %%", event.val);
            } else {
                ESP_LOGI(TAG, "Pressure: %.2f hPa", event.val / 100.0f);
            }
        }
    }
}

extern "C" void app_main() {
    if (!bme.init()) {
        ESP_LOGE(TAG, "BME280 failed to initialise!");
        return;
    }
    if (!wifi.init()) {
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
        return;
    }
    if (!wifi.initTime()) {
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
        return;
    }
    eventQueue = xQueueCreate(100, sizeof(Event));
    xTaskCreate(readTask, "ReadTask", 4096, NULL, 6, NULL);
    xTaskCreate(logTask, "LogTask", 4096, NULL, 3, NULL);
}

#include "bme280.hpp"
#include "bme680.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt.hpp"
#include "sensor.hpp"
#include "utils.hpp"
#include "wifi.hpp"

static const char *TAG = "MAIN";

WIFI wifi;
MQTT mqtt;
QueueHandle_t eventQueue;

#if READ_BME280
BME280::Device bme280;
#endif

#if READ_BME680
BME680::Device bme680;
#endif

ISensor *sensors[] = {
#if READ_BME280
    &bme280,
#endif
#if READ_BME680
    &bme680,
#endif
};

void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    while (true) {
        for (ISensor *s : sensors) {
            const auto t = wifi.getTime();
            s->logReadings(eventQueue, t);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}

void toJson(const Event &event, char *buf, const std::size_t size) {
    const std::int32_t n = snprintf(buf, size, "{\"time\":%lld,\"val\":%.2f}",
                                    static_cast<long long>(event.timestamp), event.val);
    if (n < 0 || n >= size) {
        ESP_LOGW("MQTT", "JSON string truncated!");
    }
}

void logTask(void *pvParameters) {
    Event event;
    while (true) {
        if (!wifi.connected || !mqtt.connected) {
            delay_ms(1000);
            continue;
        }
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY)) {
            const std::size_t len = 64;
            char buf[len];
            toJson(event, buf, len);
            if (event.type == EventType::TEMP) {
                ESP_LOGI(TAG, "Temp: %.2f °C", event.val);
                mqtt.publish("temperature", buf);
            } else if (event.type == EventType::HUM) {
                ESP_LOGI(TAG, "Humidity: %.2f %%", event.val);
                mqtt.publish("humidity", buf);
            } else if (event.type == EventType::PRES) {
                ESP_LOGI(TAG, "Pressure: %.2f hPa", event.val / 100.0f);
                mqtt.publish("pressure", buf);
            } else {
                ESP_LOGI(TAG, "Gas: %.2f", event.val);
                mqtt.publish("gas", buf);
            }
        }
    }
}

extern "C" void app_main() {
    for (ISensor *s : sensors) {
        if (!s->init()) {
            ESP_LOGE(TAG, "Sensor failed to initialise!");
            return;
        }
    }
    if (!wifi.init()) {
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
        return;
    }
    if (!wifi.initTime()) {
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
        return;
    }
    if (!mqtt.init()) {
        ESP_LOGE(TAG, "Could not initialise MQTT, exiting...");
        return;
    }
    eventQueue = xQueueCreate(100, sizeof(Event));
    xTaskCreate(readTask, "ReadTask", 4096, NULL, 6, NULL);
    xTaskCreate(logTask, "LogTask", 4096, NULL, 3, NULL);
}

#include "bme280.hpp"
#include "bme680.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt.hpp"
#include "scd41.hpp"
#include "sds011.hpp"
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

#if READ_SDS011
SDS011::Device sds;
#endif

#if READ_SCD41
SCD41::Device scd;
#endif

ISensor *sensors[] = {
#if READ_BME280
    &bme280,
#endif
#if READ_BME680
    &bme680,
#endif
#if READ_SDS011
    &sds,
#endif
#if READ_SCD41
    &scd,
#endif
};

void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    while (true) {
        for (ISensor *s : sensors) {
            s->logReadings(eventQueue);
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
            switch (event.type) {
            case EventType::TEMP:
                ESP_LOGI(TAG, "Temp: %.2f °C", event.val);
                mqtt.publish("temperature", buf);
                break;

            case EventType::HUM:
                ESP_LOGI(TAG, "Humidity: %.2f %%", event.val);
                mqtt.publish("humidity", buf);
                break;

            case EventType::PRES:
                ESP_LOGI(TAG, "Pressure: %.2f hPa", event.val / 100.0f);
                mqtt.publish("pressure", buf);
                break;

            case EventType::GAS:
                ESP_LOGI(TAG, "Gas: %.2f kΩ", event.val / 1000.0f);
                mqtt.publish("gas", buf);
                break;

            case EventType::PM2_5:
                ESP_LOGI(TAG, "PM2.5: %.2f ug/m3", event.val);
                mqtt.publish("pm2_5", buf);
                break;

            case EventType::PM10:
                ESP_LOGI(TAG, "PM10 : %.2f ug/m3", event.val);
                mqtt.publish("pm10", buf);
                break;

            case EventType::CO2:
                ESP_LOGI(TAG, "CO2 : %.2f ppm", event.val);
                mqtt.publish("co2", buf);
                break;

            default:
                ESP_LOGE(TAG, "Unknown event type");
                break;
            }
        }
    }
}

extern "C" void app_main() {
    if (INITIALISE_I2C && !initialiseI2C(I2C_MASTER_NUM)) {
        ESP_LOGE(TAG, "Failed to initialise i2c!");
        return;
    }
    std::uint32_t count = 0;
    for (ISensor *s : sensors) {
        count++;
        if (!s->init()) {
            ESP_LOGE(TAG, "Sensor failed to initialise!");
            return;
        }
    }
    ESP_LOGI(TAG, "Initialised %d Sensors...", count);
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

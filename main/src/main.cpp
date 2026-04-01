#include "bme280.hpp"
#include "bme680.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
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

const bool INITIALISE_I2C = READ_BME280 || READ_BME680 || READ_SCD41;
const std::uint32_t SLEEP_PERIOD_S = MEASUREMENT_PERIOD_S - WARMUP_TIME_S;

void toJson(const Event &event, char *buf, const std::size_t size) {
    const std::int32_t n = snprintf(buf, size, "{\"time\":%lld,\"val\":%.2f}",
                                    static_cast<long long>(event.timestamp), event.val);
    if (n < 0 || n >= size) {
        ESP_LOGW("MQTT", "JSON string truncated!");
    }
}

void goToSleep() {
    if (OPERATING_MODE > 0 && SLEEP_PERIOD_S > 0) {
        ESP_LOGI(TAG, "Shutting down sensors and entering deep sleep...");
        const std::uint64_t sleepDur =
            static_cast<std::uint64_t>(SLEEP_PERIOD_S) * 1'000'000ULL;
        esp_sleep_enable_timer_wakeup(sleepDur);
        esp_deep_sleep_start();
    }
    ESP_LOGI(TAG, "Restarting...");
    esp_restart();
}

void shutdownSensors() {
    for (ISensor *s : sensors) {
        if (s->isInitialised()) {
            s->sleep();
        }
    }
}

void powerOff() {
    shutdownSensors();
    goToSleep();
}

void takeReadings() {
    for (ISensor *s : sensors) {
        if (s->isInitialised()) {
            s->logReadings(eventQueue);
        }
    }
}

void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    while (true) {
        takeReadings();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}

void publishReadings(const std::uint32_t portDelay = 0) {
    if (!wifi.connected || !mqtt.connected) {
        ESP_LOGE(TAG, "WiFi or MQTT disconnected, returning from publishReadings...");
        return;
    }
    Event event;
    while (xQueueReceive(eventQueue, &event, pdMS_TO_TICKS(portDelay)) == pdTRUE) {
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

void logTask(void *pvParameters) {
    while (true) {
        publishReadings(2000);
    }
}

extern "C" void app_main() {
    TickType_t start = xTaskGetTickCount();
    if (!wifi.init()) {
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
        goToSleep();
    }
    if (!wifi.initTime()) {
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
        goToSleep();
    }
    if (!mqtt.init()) {
        ESP_LOGE(TAG, "Could not initialise MQTT, exiting...");
        goToSleep();
    }
    if (INITIALISE_I2C && !initialiseI2C(I2C_MASTER_NUM)) {
        ESP_LOGE(TAG, "Failed to initialise i2c!");
        goToSleep();
    }
    std::uint32_t count = 0;
    for (ISensor *s : sensors) {
        if (!s->init()) {
            ESP_LOGE(TAG, "Sensor failed to initialise!");
            continue;
        }
        count++;
    }
    if (count == 0) {
        ESP_LOGE(TAG, "No Sensors initialised!");
        goToSleep();
    }
    ESP_LOGI(TAG, "Initialised %d Sensors...", count);
    eventQueue = xQueueCreate(1000, sizeof(Event));
    if (OPERATING_MODE == 0) { // Continuous
        ESP_LOGI(TAG, "Continuous mode - starting tasks and returning from main...");
        xTaskCreate(readTask, "ReadTask", 4096, NULL, 6, NULL);
        xTaskCreate(logTask, "LogTask", 4096, NULL, 3, NULL);
        return;
    }
    ESP_LOGI(TAG, "Periodic mode - waiting until end of measurement period...");
    xTaskDelayUntil(&start, pdMS_TO_TICKS(WARMUP_TIME_S * 1000UL));
    takeReadings();
    shutdownSensors();
    publishReadings(0);
    const TickType_t timeoutTicks = pdMS_TO_TICKS(5000);
    if (!mqtt.waitForPublishes(timeoutTicks)) {
        ESP_LOGW(TAG, "Not all MQTT messages confirmed within timeout!");
    } else {
        ESP_LOGI(TAG, "All MQTT messages confirmed");
    }
    goToSleep();
}

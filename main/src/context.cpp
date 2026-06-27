#include "context.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "events.hpp"
#include <string>

static const char *TAG = "MAIN";

static void toJson(const Event &event, char *buf, const std::size_t size) {
    const std::int32_t n = snprintf(buf, size, "{\"time\":%lld,\"val\":%.2f}",
                                    static_cast<long long>(event.timestamp), event.val);
    if (n < 0 || n >= size) {
#if MAIN_DEBUG
        ESP_LOGW("MQTT", "JSON string truncated!");
#endif
    }
}

MainContext::MainContext(const std::uint32_t ml, ISensor **s, const std::size_t sc,
                         const QueueHandle_t eq, const WIFI &w, MQTT &m, Logger &l)
    : mainLoopMS(ml), sensors(s), sensorCount(sc), eventQueue(eq), wifi(w), mqtt(m),
      logger(l) {};

std::uint32_t MainContext::getWarmupTime() {
    std::uint32_t warmupTime = 0;
    std::uint32_t initSoFar = 0;
    ISensor *s;
    for (std::size_t i = 0; i < sensorCount; ++i) {
        s = sensors[i];
        const std::uint32_t at = s->getInitTime() + s->getDataReadyTime();
        warmupTime = std::max(warmupTime, at + initSoFar);
        initSoFar += s->getInitTime();
    }
    const std::string message = "Warmup time: " + std::to_string(warmupTime);
    logger.logInfo(TAG, message.c_str());
    return warmupTime;
}

void MainContext::shutdownSensors() {
    const std::string message = "Shutting down sensors...";
    logger.logInfo(TAG, message.c_str());
    ISensor *s;
    for (std::size_t i = 0; i < sensorCount; ++i) {
        s = sensors[i];
        if (s->isInitialised()) {
            s->sleep();
        }
    }
}

void MainContext::takeReadings() const {
    ISensor *s;
    for (std::size_t i = 0; i < sensorCount; ++i) {
        s = sensors[i];
        if (s->isInitialised()) {
            s->logReadings(eventQueue);
        }
    }
}

void MainContext::publishReadings(const std::uint32_t portDelay) const {
    if (!wifi.connected || !mqtt.connected) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "WiFi or MQTT disconnected, returning from publishReadings...");
#endif
        return;
    }
    Event event;
    while (xQueueReceive(eventQueue, &event, pdMS_TO_TICKS(portDelay)) == pdTRUE) {
        const std::size_t len = 64;
        char buf[len];
        toJson(event, buf, len);
        switch (event.type) {
        case EventType::TEMP:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Temp: %.2f °C", event.val);
#endif
            mqtt.publish("temperature", buf);
            break;

        case EventType::HUM:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Humidity: %.2f %%", event.val);
#endif
            mqtt.publish("humidity", buf);
            break;

        case EventType::PRES:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Pressure: %.2f hPa", event.val / 100.0f);
#endif
            mqtt.publish("pressure", buf);
            break;

        case EventType::GAS:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Gas: %.2f kΩ", event.val / 1000.0f);
#endif
            mqtt.publish("gas", buf);
            break;

        case EventType::PM2_5:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "PM2.5: %.2f ug/m3", event.val);
#endif
            mqtt.publish("pm2_5", buf);
            break;

        case EventType::PM10:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "PM10 : %.2f ug/m3", event.val);
#endif
            mqtt.publish("pm10", buf);
            break;

        case EventType::CO2:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "CO2 : %.2f ppm", event.val);
#endif
            mqtt.publish("co2", buf);
            break;

        case EventType::VOC:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "VOC : %.2f", event.val);
#endif
            mqtt.publish("voc", buf);
            break;

        case EventType::NOX:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "NOX : %.2f", event.val);
#endif
            mqtt.publish("nox", buf);
            break;

        case EventType::DEW:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Dew Point : %.2f °C", event.val);
#endif
            mqtt.publish("dew_point", buf);
            break;

        case EventType::VPD:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Vapour Pressure Deficit : %.2f kPa", event.val);
#endif
            mqtt.publish("vpd", buf);
            break;

        case EventType::AH:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Absolute Humidity : %.2f g/m³", event.val);
#endif
            mqtt.publish("abs_humidity", buf);
            break;

        default:
#if MAIN_DEBUG
            ESP_LOGE(TAG, "Unknown event type");
#endif
            break;
        }
    }
}
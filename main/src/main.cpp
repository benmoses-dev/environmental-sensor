#include "bme280.hpp"
#include "bme680.hpp"
#include "config.hpp"
#include "environment.hpp"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "events.hpp"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "i2c.hpp"
#include "mqtt.hpp"
#include "scd41.hpp"
#include "sds011.hpp"
#include "sensor.hpp"
#include "sgp40.hpp"
#include "sgp41.hpp"
#include "sht45.hpp"
#include "utils.hpp"
#include "wifi.hpp"
#include <algorithm>
#include <iterator>
#include <limits>

static const char *TAG = "MAIN";

static constexpr bool shouldInitI2C() {
    return READ_BME280 || READ_BME680 || READ_SCD41 || READ_SHT45 || READ_SGP40 ||
           READ_SGP41;
}

struct MainContext {
    ISensor **sensors;
    const std::size_t sensorCount;
    const std::uint32_t mainLoopMS;
    const QueueHandle_t eventQueue;
    const WIFI &wifi;
    MQTT &mqtt;
};

SemaphoreHandle_t i2cMutex = nullptr;

static std::uint32_t getWarmupTime(const MainContext *context) {
    std::uint32_t warmupTime = 0;
    std::uint32_t initSoFar = 0;
    ISensor *s;
    for (std::size_t i = 0; i < context->sensorCount; ++i) {
        s = context->sensors[i];
        const std::uint32_t at = s->getInitTime() + s->getDataReadyTime();
        warmupTime = std::max(warmupTime, at + initSoFar);
        initSoFar += s->getInitTime();
    }
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Warmup time: %u", warmupTime);
#endif
    return warmupTime;
}

static std::uint32_t getMainLoopTime(ISensor **sensors, const std::size_t sensorCount) {
    std::uint32_t mainLoopTime = std::numeric_limits<std::uint32_t>::max();
    ISensor *s;
    for (std::size_t i = 0; i < sensorCount; ++i) {
        s = sensors[i];
        mainLoopTime = std::min(mainLoopTime, s->getLoopTime());
    }
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Main loop time: %u", mainLoopTime);
#endif
    return mainLoopTime;
}

static void toJson(const Event &event, char *buf, const std::size_t size) {
    const std::int32_t n = snprintf(buf, size, "{\"time\":%lld,\"val\":%.2f}",
                                    static_cast<long long>(event.timestamp), event.val);
    if (n < 0 || n >= size) {
#if MAIN_DEBUG
        ESP_LOGW("MQTT", "JSON string truncated!");
#endif
    }
}

static void goToSleep(const std::uint32_t sleepPeriod) {
    if (OPERATING_MODE > 0 && sleepPeriod > 0) {
#if MAIN_DEBUG
        ESP_LOGI(TAG, "Entering deep sleep for %u ms...", sleepPeriod);
#endif
        const std::uint64_t sleepDur = static_cast<std::uint64_t>(sleepPeriod) * 1'000ULL;
        esp_sleep_enable_timer_wakeup(sleepDur);
        esp_deep_sleep_start();
    }
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Restarting...");
#endif
    esp_restart();
}

static void shutdownSensors(const MainContext *context) {
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Shutting down sensors...");
#endif
    ISensor *s;
    for (std::size_t i = 0; i < context->sensorCount; ++i) {
        s = context->sensors[i];
        if (s->isInitialised()) {
            s->sleep();
        }
    }
}

static void takeReadings(const MainContext *context) {
    ISensor *s;
    for (std::size_t i = 0; i < context->sensorCount; ++i) {
        s = context->sensors[i];
        if (s->isInitialised()) {
            s->logReadings(context->eventQueue);
        }
    }
}

static void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    const MainContext *context = static_cast<const MainContext *>(pvParameters);
    while (true) {
        takeReadings(context);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(context->mainLoopMS));
    }
}

static void publishReadings(const MainContext *context, const std::uint32_t portDelay) {
    if (!context->wifi.connected || !context->mqtt.connected) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "WiFi or MQTT disconnected, returning from publishReadings...");
#endif
        return;
    }
    Event event;
    while (xQueueReceive(context->eventQueue, &event, pdMS_TO_TICKS(portDelay)) ==
           pdTRUE) {
        const std::size_t len = 64;
        char buf[len];
        toJson(event, buf, len);
        switch (event.type) {
        case EventType::TEMP:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Temp: %.2f °C", event.val);
#endif
            context->mqtt.publish("temperature", buf);
            break;

        case EventType::HUM:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Humidity: %.2f %%", event.val);
#endif
            context->mqtt.publish("humidity", buf);
            break;

        case EventType::PRES:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Pressure: %.2f hPa", event.val / 100.0f);
#endif
            context->mqtt.publish("pressure", buf);
            break;

        case EventType::GAS:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "Gas: %.2f kΩ", event.val / 1000.0f);
#endif
            context->mqtt.publish("gas", buf);
            break;

        case EventType::PM2_5:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "PM2.5: %.2f ug/m3", event.val);
#endif
            context->mqtt.publish("pm2_5", buf);
            break;

        case EventType::PM10:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "PM10 : %.2f ug/m3", event.val);
#endif
            context->mqtt.publish("pm10", buf);
            break;

        case EventType::CO2:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "CO2 : %.2f ppm", event.val);
#endif
            context->mqtt.publish("co2", buf);
            break;

        case EventType::VOC:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "VOC : %.2f", event.val);
#endif
            context->mqtt.publish("voc", buf);
            break;

        case EventType::NOX:
#if MAIN_DEBUG
            ESP_LOGI(TAG, "NOX : %.2f", event.val);
#endif
            context->mqtt.publish("nox", buf);
            break;

        default:
#if MAIN_DEBUG
            ESP_LOGE(TAG, "Unknown event type");
#endif
            break;
        }
    }
}

static void logTask(void *pvParameters) {
    const MainContext *context = static_cast<const MainContext *>(pvParameters);
    while (true) {
        publishReadings(context, 1500);
    }
}

static void printResetReason(const esp_reset_reason_t reason) {
    switch (reason) {
    case ESP_RST_POWERON:
        ESP_LOGI("RESET", "Power-on reset");
        break;

    case ESP_RST_EXT:
        ESP_LOGI("RESET", "External reset (reset pin)");
        break;

    case ESP_RST_SW:
        ESP_LOGI("RESET", "Software reset (esp_restart)");
        break;

    case ESP_RST_PANIC:
        ESP_LOGI("RESET", "Panic / crash reset");
        break;

    case ESP_RST_INT_WDT:
        ESP_LOGI("RESET", "Interrupt watchdog reset");
        break;

    case ESP_RST_TASK_WDT:
        ESP_LOGI("RESET", "Task watchdog reset");
        break;

    case ESP_RST_WDT:
        ESP_LOGI("RESET", "Other watchdog reset");
        break;

    case ESP_RST_DEEPSLEEP:
        ESP_LOGI("RESET", "Wake from deep sleep");
        break;

    case ESP_RST_BROWNOUT:
        ESP_LOGI("RESET", "Brownout reset");
        break;

    case ESP_RST_SDIO:
        ESP_LOGI("RESET", "SDIO reset");
        break;

    default:
        ESP_LOGI("RESET", "Unknown reset reason: %d", reason);
        break;
    }
}

extern "C" void app_main() {
    const esp_reset_reason_t reason = esp_reset_reason();
#if MAIN_DEBUG
    printResetReason(reason);
#endif

    TickType_t startWifi = xTaskGetTickCount();

#if READ_BME280
    static BME280::Device bme280;
#endif
#if READ_BME680
    static BME680::Device bme680;
#endif
#if READ_SDS011
    static SDS011::Device sds;
#endif
#if READ_SCD41
    static SCD41::Device scd;
#endif
#if READ_SHT45
    static SHT45::Device sht45;
#endif
#if READ_SGP40
    static SGP40::Device sgp40;
#endif
#if READ_SGP41
    static SGP41::Device sgp41;
#endif
    static ISensor *sensors[] = {
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
#if READ_SHT45
        &sht45,
#endif
#if READ_SGP40
        &sgp40,
#endif
#if READ_SGP41
        &sgp41,
#endif
    };
    constexpr std::size_t SENSOR_COUNT = std::size(sensors);
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        for (std::size_t j = i + 1; j < SENSOR_COUNT; ++j) {
            if (sensors[j]->getDataReadyTime() > sensors[i]->getDataReadyTime()) {
                ISensor *tmp = sensors[i];
                sensors[i] = sensors[j];
                sensors[j] = tmp;
            }
        }
    }

    const std::uint32_t mainLoopPeriod = getMainLoopTime(sensors, SENSOR_COUNT);
    static const QueueHandle_t eventQueue = xQueueCreate(1000, sizeof(Event));
    static WIFI wifi;
    static MQTT mqtt;
    static MainContext context = {
        sensors, SENSOR_COUNT, mainLoopPeriod, eventQueue, wifi, mqtt,
    };

    const std::uint32_t warmupPeriod = getWarmupTime(&context);
    constexpr std::uint32_t WIFI_CONNECT_TIME_MS = 10'000;
    const std::uint32_t sleepPeriod = static_cast<std::uint32_t>(std::max(
        static_cast<int>(MEASUREMENT_PERIOD_MS) - static_cast<int>(warmupPeriod) -
            static_cast<int>(WIFI_CONNECT_TIME_MS),
        0));

#if MAIN_DEBUG
    const auto sTime = millis();
#endif
    if (!wifi.init()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
#endif
        goToSleep(sleepPeriod);
    }
    if (!wifi.initTime()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
#endif
        goToSleep(sleepPeriod);
    }
    if (!mqtt.init()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not initialise MQTT, exiting...");
#endif
        goToSleep(sleepPeriod);
    }
#if MAIN_DEBUG
    const auto eTime = millis();
    const auto wifiTime = eTime - sTime;
    ESP_LOGI(TAG, "WiFi time: %u", wifiTime);
#endif

    vTaskDelayUntil(&startWifi, pdMS_TO_TICKS(WIFI_CONNECT_TIME_MS));

    TickType_t startInit = xTaskGetTickCount();

    constexpr bool INITIALISE_I2C = shouldInitI2C();
    if (INITIALISE_I2C && !initialiseI2C(I2C_MASTER_NUM)) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Failed to initialise i2c!");
#endif
        goToSleep(sleepPeriod);
    }
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not create i2c mutex, aborting...");
#endif
        goToSleep(sleepPeriod);
    }

    std::uint32_t count = 0;
    for (ISensor *s : sensors) {
        if (!s->init()) {
#if MAIN_DEBUG
            ESP_LOGE(TAG, "Sensor failed to initialise!");
#endif
            continue;
        }
        count++;
    }
    if (count == 0) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "No Sensors initialised!");
#endif
        goToSleep(sleepPeriod);
    }
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Initialised %d Sensors...", count);
    ESP_LOGI(TAG, "Adjustments: temp = %.2f, hum = %.2f, pres = %.2f", TEMP_ADJUST,
             HUM_ADJUST, PRES_ADJUST);
#endif
#if MAIN_DEBUG
    ESP_LOGI(TAG, "Waiting until end of warmup period...");
#endif

    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(warmupPeriod));

    if (OPERATING_MODE == 0) {
#if MAIN_DEBUG
        ESP_LOGI(TAG, "Continuous mode - starting tasks and returning from main...");
#endif
        xTaskCreate(readTask, "ReadTask", 4096, &context, 6, NULL);
        xTaskCreate(logTask, "LogTask", 4096, &context, 3, NULL);
        return;
    }

    takeReadings(&context);
    shutdownSensors(&context);
    publishReadings(&context, 0);
    const TickType_t timeoutTicks = pdMS_TO_TICKS(5000);
    if (!mqtt.waitForPublishes(timeoutTicks)) {
#if MAIN_DEBUG
        ESP_LOGW(TAG, "Not all MQTT messages confirmed within timeout!");
#endif
    } else {
#if MAIN_DEBUG
        ESP_LOGI(TAG, "All MQTT messages confirmed");
#endif
    }

    goToSleep(sleepPeriod);
}

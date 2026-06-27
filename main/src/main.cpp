#include "bme280.hpp"
#include "bme680.hpp"
#include "config.hpp"
#include "context.hpp"
#include "environment.hpp"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "events.hpp"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "i2c.hpp"
#include "scd41.hpp"
#include "sds011.hpp"
#include "sgp40.hpp"
#include "sgp41.hpp"
#include "sht45.hpp"
#include "utils.hpp"
#include <algorithm>
#include <iterator>
#include <limits>

static const char *TAG = "MAIN";

static constexpr bool shouldInitI2C() {
    return READ_BME280 || READ_BME680 || READ_SCD41 || READ_SHT45 || READ_SGP40 ||
           READ_SGP41;
}

SemaphoreHandle_t i2cMutex = nullptr;

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

static void readTask(void *pvParameters) {
    TickType_t last = xTaskGetTickCount();
    const MainContext *context = static_cast<const MainContext *>(pvParameters);
    while (true) {
        context->takeReadings();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(context->mainLoopMS));
    }
}

static void logTask(void *pvParameters) {
    const MainContext *context = static_cast<const MainContext *>(pvParameters);
    while (true) {
        context->publishReadings(1500);
    }
}

static void publishResetReason(const esp_reset_reason_t reason, Logger &logger) {
    std::string r;
    switch (reason) {
    case ESP_RST_POWERON:
        r = "Power-on reset";
        break;

    case ESP_RST_EXT:
        r = "External reset (reset pin)";
        break;

    case ESP_RST_SW:
        r = "Software reset (esp_restart)";
        break;

    case ESP_RST_PANIC:
        r = "Panic / crash reset";
        break;

    case ESP_RST_INT_WDT:
        r = "Interrupt watchdog reset";
        break;

    case ESP_RST_TASK_WDT:
        r = "Task watchdog reset";
        break;

    case ESP_RST_WDT:
        r = "Other watchdog reset";
        break;

    case ESP_RST_DEEPSLEEP:
        r = "Wake from deep sleep";
        break;

    case ESP_RST_BROWNOUT:
        r = "Brownout reset";
        break;

    case ESP_RST_SDIO:
        r = "SDIO reset";
        break;

    default:
        r = "Unknown reset reason: " + std::to_string(reason);
        break;
    }

    logger.logInfo(TAG, r.c_str());
}

static bool initWifi(WIFI &wifi) {
    if (!wifi.init()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "WiFi initialisation failed, exiting...");
#endif
        return false;
    }
    if (!wifi.initTime()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not synchronise NTP, exiting...");
#endif
        return false;
    }
    return true;
}

static bool initMqtt(MQTT &mqtt) {
    if (!mqtt.init()) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not initialise MQTT, exiting...");
#endif
        return false;
    }
    return true;
}

extern "C" void app_main() {
    TickType_t startWifi = xTaskGetTickCount();
    static Environment env;

#if READ_BME280
    static BME280::Device bme280(env);
#endif
#if READ_BME680
    static BME680::Device bme680(env);
#endif
#if READ_SDS011
    static SDS011::Device sds;
#endif
#if READ_SCD41
    static SCD41::Device scd(env);
#endif
#if READ_SHT45
    static SHT45::Device sht45(env);
#endif
#if READ_SGP40
    static SGP40::Device sgp40(env);
#endif
#if READ_SGP41
    static SGP41::Device sgp41(env);
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

#if MAIN_DEBUG
    const auto sTime = millis();
#endif

    static WIFI wifi;
    static MQTT mqtt;

    bool wifiInitialised = initWifi(wifi);
    bool mqttInitialised = wifiInitialised && initMqtt(mqtt);
    if (!mqttInitialised) {
#if MAIN_DEBUG
        ESP_LOGI(TAG, "Restarting...");
#endif
        esp_restart();
    }

    static Logger logger{mqtt};
    const esp_reset_reason_t reason = esp_reset_reason();
    publishResetReason(reason, logger);
#if MAIN_DEBUG
    const auto eTime = millis();
    const auto wifiTime = eTime - sTime;
    ESP_LOGI(TAG, "WiFi time: %u", wifiTime);
#endif

    static const QueueHandle_t eventQueue = xQueueCreate(1000, sizeof(Event));
    static MainContext context{sensors, SENSOR_COUNT, eventQueue, wifi, mqtt, logger};

    const std::uint32_t warmupPeriod = context.getWarmupTime();
    constexpr std::uint32_t WIFI_CONNECT_TIME_MS = 10'000;
    const std::uint32_t sleepPeriod = static_cast<std::uint32_t>(std::max(
        static_cast<int>(MEASUREMENT_PERIOD_MS) - static_cast<int>(warmupPeriod) -
            static_cast<int>(WIFI_CONNECT_TIME_MS),
        0));

    vTaskDelayUntil(&startWifi, pdMS_TO_TICKS(WIFI_CONNECT_TIME_MS));

    TickType_t startInit = xTaskGetTickCount();

    constexpr bool INITIALISE_I2C = shouldInitI2C();
    if (INITIALISE_I2C && !initialiseI2C(I2C_MASTER_NUM)) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Failed to initialise i2c!");
#endif
        mqtt.publish("info", "Failed to initialise i2c!");
        goToSleep(sleepPeriod);
    }
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) {
#if MAIN_DEBUG
        ESP_LOGE(TAG, "Could not create i2c mutex, aborting...");
#endif
        mqtt.publish("info", "Could not create i2c mutex, aborting...");
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
        mqtt.publish("info", "No Sensors initialised!");
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
        xTaskCreate(readTask, "ReadTask", 4096, &context, 4, NULL);
        xTaskCreate(logTask, "LogTask", 4096, &context, 3, NULL);
        return;
    }

    context.takeReadings();
    context.shutdownSensors();
    context.publishReadings(0);
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

#include "sht45.hpp"
#include "config.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "i2c.hpp"
#include "portmacro.h"
#include "utils.hpp"
#include <cmath>
#include <ctime>

namespace SHT45 {

static const char *TAG = "SHT45";

Device::Device(Environment &e, const i2c_port_t port, const std::uint8_t addr)
    : env(e), i2c_port(port), i2c_addr(addr), initialised(false), shutdown(false) {}

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
}

void sht45Task(void *pvParameters) {
    Device *sht45 = static_cast<Device *>(pvParameters);
    sht45->start();
}

bool Device::init() {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
#if SHT45_DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
#endif
        return false;
    }
    TickType_t startInit = xTaskGetTickCount();
#if SHT45_DEBUG
    const auto startTime = millis();
    ESP_LOGI(TAG, "Initialising SHT45...");
#endif
    if (!reset()) {
#if SHT45_DEBUG
        ESP_LOGE(TAG, "Failed to soft reset device!");
#endif
        return false;
    }
#if SHT45_DEBUG
    ESP_LOGI(TAG, "SHT45 initialised successfully!");
#endif
    initialised = true;
#if SHT45_DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u, predicted: %u", totalTime, getInitTime());
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(sht45Task, "SHT45Task", 4096, this, 5, &taskHandle);
    return true;
}

bool Device::getShutdown() {
    taskENTER_CRITICAL(&shutdownMux);
    const bool shouldStop = shutdown;
    taskEXIT_CRITICAL(&shutdownMux);
    return shouldStop;
}

void Device::start() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
#if SHT45_DEBUG
        const auto sTime = millis();
#endif
        if (getShutdown()) {
#if SHT45_DEBUG
            ESP_LOGI(TAG, "SHT45 read-loop task stopping...");
#endif
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        if (!storeReading()) {
#if SHT45_DEBUG
            ESP_LOGW(TAG, "Failed to take reading in read-loop!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(SHT45_READ_FREQ_MS));
            continue;
        }
#if SHT45_DEBUG
        const auto eTime = millis();
        const auto totalTime = eTime - sTime;
        ESP_LOGI(TAG, "Total reading time: %u, predicted: %u", totalTime,
                 READING_DURATION_MS);
#endif
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SHT45_READ_FREQ_MS));
    }
}

bool Device::isInitialised() { return initialised; }

Reading Device::getReading() {
    taskENTER_CRITICAL(&readingMux);
    const Reading res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    return res;
}

void Device::logReadings(QueueHandle_t q) {
    const Reading res = getReading();
    if (res.read) {
#if SHT45_DEBUG
        ESP_LOGW(TAG, "Reading has already been read...");
#endif
        return;
    }
    if (!res.valid) {
#if SHT45_DEBUG
        ESP_LOGW(TAG, "Reading is invalid...");
#endif
        return;
    }
    sendCombinedMetrics(res.temperature, res.humidity, res.tval, q);
}

void Device::setShutdown() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
}

bool Device::sleep() {
    setShutdown();
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
#if SHT45_DEBUG
    ESP_LOGI(TAG, "Got shutdown semaphore, sleeping...");
#endif
    return true;
}

bool Device::reset() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write8(SOFT_RESET, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (!res) {
        return false;
    }
    delay_ms(1);
    return true;
}

bool Device::storeReading() {
    Reading r{};
    r.valid = false;
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (!write8(NO_HEAT_HIGH_PRECISION, i2c_port, i2c_addr)) {
#if SHT45_DEBUG
        ESP_LOGE(TAG, "Failed to start reading!");
#endif
        xSemaphoreGive(i2cMutex);
        return false;
    }
    delay_ms(10);
    std::uint8_t buffer[6];
    const bool res = readBytes(buffer, 6, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if SHT45_DEBUG
        ESP_LOGE(TAG, "Failed to read response!");
#endif
        return false;
    }
    r.tval = time(NULL);
    if (buffer[2] != getCRC8(buffer) || buffer[5] != getCRC8(buffer + 3)) {
#if SHT45_DEBUG
        ESP_LOGE(TAG, "CRC checksum failed!");
#endif
        return false;
    }
    const std::uint16_t tempRaw = (buffer[0] << 8) | buffer[1];
    const float tempNorm = static_cast<float>(tempRaw) / 65535.0f;
    const float temperature = (175.0 * tempNorm) - 45.0;
    const std::uint16_t humRaw = (buffer[3] << 8) | buffer[4];
    const float humNorm = static_cast<float>(humRaw) / 65535.0f;
    const float humidity = -6.0f + 125.0f * humNorm;
    r.temperature = temperature + TEMP_ADJUST;
    r.humidity = std::min(std::max(humidity + HUM_ADJUST, 0.0f), 100.0f);
    r.read = false;
    r.valid = true;
    taskENTER_CRITICAL(&readingMux);
    reading = r;
    taskEXIT_CRITICAL(&readingMux);
    env.setTemperature(r.temperature, r.tval);
    env.setHumidity(r.humidity, r.tval);
    return true;
}

} // namespace SHT45

#include "sgp41.hpp"
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

namespace SGP41 {

static const char *TAG = "SGP41";

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr), initialised(false), shutdown(false) {}

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
}

void sgp41Task(void *pvParameters) {
    Device *sgp41 = static_cast<Device *>(pvParameters);
    sgp41->start();
}

bool Device::init() {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
#if SGP41_DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
#endif
        return false;
    }
    TickType_t startInit = xTaskGetTickCount();
#if SGP41_DEBUG
    const auto startTime = millis();
    ESP_LOGI(TAG, "Initialising SGP41...");
#endif
#if SGP41_DEBUG
    ESP_LOGI(TAG, "Configuring SGP41...");
#endif
    ESP_LOGI(TAG, "SGP41 configured successfully!");
    initialised = true;
#if SGP41_DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u, predicted: %u", totalTime, getInitTime());
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(sgp41Task, "SGP41Task", 4096, this, 5, &taskHandle);
    return true;
}

void Device::start() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
#if SGP41_DEBUG
        const auto sTime = millis();
#endif
        bool shouldStop;
        taskENTER_CRITICAL(&shutdownMux);
        shouldStop = shutdown;
        taskEXIT_CRITICAL(&shutdownMux);
        if (shouldStop) {
#if SGP41_DEBUG
            ESP_LOGI(TAG, "SGP41 read-loop task stopping...");
#endif
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        if (!performReading()) {
#if SGP41_DEBUG
            ESP_LOGW(TAG, "Failed to take reading in read-loop!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(SGP41_READ_FREQ_MS));
            continue;
        }
#if SGP41_DEBUG
        const auto eTime = millis();
        const auto totalTime = eTime - sTime;
        ESP_LOGI(TAG, "Total reading time: %u, predicted: %u", totalTime,
                 READING_DURATION_MS);
#endif
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SGP41_READ_FREQ_MS));
    }
}

bool Device::isInitialised() { return initialised; }

void Device::logReadings(QueueHandle_t q) {
    Reading res{};
    taskENTER_CRITICAL(&readingMux);
    res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    if (res.read) {
#if SGP41_DEBUG
        ESP_LOGW(TAG, "Reading has already been read...");
#endif
        return;
    }
    if (!res.valid) {
#if SGP41_DEBUG
        ESP_LOGW(TAG, "Reading is invalid...");
#endif
        return;
    }
    const Event vEvent = {res.voc, res.tval, EventType::VOC};
    const Event nEvent = {res.nox, res.tval, EventType::NOX};
    xQueueSend(q, &vEvent, portMAX_DELAY);
    xQueueSend(q, &nEvent, portMAX_DELAY);
}

bool Device::sleep() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
#if SGP41_DEBUG
    ESP_LOGI(TAG, "Got shutdown semaphore, sleeping...");
#endif
    return true;
}

bool Device::performReading() {
    Reading r{};
    r.valid = false;
    // Take reading.
    r.tval = time(NULL);
    r.read = false;
    taskENTER_CRITICAL(&readingMux);
    reading = r;
    taskEXIT_CRITICAL(&readingMux);
    return true;
}

} // namespace SGP41

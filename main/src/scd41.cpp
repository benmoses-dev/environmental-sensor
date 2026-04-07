#include "scd41.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/idf_additions.h"
#include "i2c.hpp"
#include "portmacro.h"
#include "utils.hpp"

namespace SCD41 {

static const char *TAG = "SCD41";

#define DEBUG 1

void scdTask(void *pvParameters) {
    Device *scd = static_cast<Device *>(pvParameters);
    scd->start();
}

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr), initialised(false), shutdown(false) {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
        ESP_LOGE(TAG, "Failed to create semaphore");
    }
}

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
}

bool Device::init() {
    TickType_t startInit = xTaskGetTickCount();
#if DEBUG
    const auto startTime = millis();
#endif
    if (!wake()) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to wake device!");
#endif
    }
    if (!stopPeriodicMeasurement()) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to stop periodic measurement");
#endif
        return false;
    }
    ESP_LOGI(TAG, "SCD41 initialised successfully!");
    initialised = true;
#if DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u", totalTime);
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(scdTask, "SCDTask", 4096, this, 5, &taskHandle);
    return true;
}

void Device::start() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
#if DEBUG
        const auto sTime = millis();
#endif
        bool shouldStop;
        taskENTER_CRITICAL(&shutdownMux);
        shouldStop = shutdown;
        taskEXIT_CRITICAL(&shutdownMux);
        if (shouldStop) {
            ESP_LOGI(TAG, "SCD41 read-loop task stopping...");
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        const bool res = singleShot();
        if (!res) {
#if DEBUG
            ESP_LOGE(TAG, "Failed to start single shot!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(SINGLE_SHOT_FREQ_MS));
            continue;
        }
        if (!getReading()) {
#if DEBUG
            ESP_LOGE(TAG, "Failed to get reading!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(SINGLE_SHOT_FREQ_MS));
            continue;
        }
#if DEBUG
        const auto eTime = millis();
        const auto totalTime = eTime - sTime;
        ESP_LOGI(TAG, "Total loop time: %u", totalTime);
#endif
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SINGLE_SHOT_FREQ_MS));
    }
}

bool Device::isInitialised() { return initialised; }

bool Device::sleep() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
    const bool stopped = stopPeriodicMeasurement();
#if DEBUG
    if (!stopped) {
        ESP_LOGE(TAG, "Could not stop device");
    }
#endif
    const bool res = powerDown();
#if DEBUG
    if (!res) {
        ESP_LOGE(TAG, "Could not sleep device");
    }
#endif
    ESP_LOGI(TAG, "Device going to sleep...");
    return stopped && res;
}

void Device::logReadings(QueueHandle_t q) {
    Reading res{};
    taskENTER_CRITICAL(&readingMux);
    res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    if (res.read) {
#if DEBUG
        ESP_LOGW(TAG, "Reading has already been read...");
#endif
        return;
    }
    if (!res.valid) {
#if DEBUG
        ESP_LOGW(TAG, "Reading is invalid...");
#endif
        return;
    }
    const Event co2Event = {res.co2, res.t, EventType::CO2};
    xQueueSend(q, &co2Event, portMAX_DELAY);
}

bool Device::getReading() {
    const bool ready = isDataReady();
    if (!ready) {
#if DEBUG
        ESP_LOGI(TAG, "Data not ready yet");
#endif
        return false;
    }
#if DEBUG
    ESP_LOGI(TAG, "Reading data");
#endif
    std::uint8_t raw[9];
    if (!readMeasurement(raw)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to read SCD41");
#endif
        return false;
    }
    const time_t t = time(NULL);
    const std::uint16_t co2Raw = (raw[0] << 8) | raw[1];
    const std::uint16_t tempRaw = (raw[3] << 8) | raw[4];
    const std::uint16_t humRaw = (raw[6] << 8) | raw[7];
    const float tempNorm = static_cast<float>(tempRaw) / 65535.0f; // 2^16 - 1
    const float temperature = (175.0 * tempNorm) - 45.0;           // -45...130
    const float humNorm = static_cast<float>(humRaw) / 65535.0f;
    const float humidity = 100.0 * humNorm; // 0..1 -> 0..100
    const float co2PPM = static_cast<float>(co2Raw);
#if DEBUG
    ESP_LOGI(TAG, "Temp: %.2f °C, Hum: %.2f %%", temperature, humidity);
#endif
    Reading res{};
    res.co2 = co2PPM;
    res.temp = temperature;
    res.hum = humidity;
    res.t = t;
    res.valid = true;
    res.read = false;
    taskENTER_CRITICAL(&readingMux);
    reading = res;
    taskEXIT_CRITICAL(&readingMux);
    return true;
}

bool Device::startPeriodicMeasurement() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_START_PERIODIC_MEASUREMENT);
    xSemaphoreGive(i2cMutex);
    return res;
}

bool Device::startLowPowerPeriodicMeasurement() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_START_LP_PERIODIC_MEASUREMENT);
    xSemaphoreGive(i2cMutex);
    return res;
}

bool Device::singleShot() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_MEASURE_SH);
    xSemaphoreGive(i2cMutex);
    if (!res) {
        return false;
    }
    delay_ms(5'000);
    return true;
}

bool Device::singleShotRHT() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_MEASURE_SH_RHT);
    xSemaphoreGive(i2cMutex);
    if (!res) {
        return false;
    }
    delay_ms(50);
    return true;
}

bool Device::stopPeriodicMeasurement() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_STOP_PERIODIC_MEASUREMENT);
    xSemaphoreGive(i2cMutex);
    delay_ms(500);
    return res;
}

bool Device::powerDown() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_POWER_DOWN);
    xSemaphoreGive(i2cMutex);
    delay_ms(1);
    return res;
}

bool Device::wake() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write16(CMD_WAKE_UP);
    xSemaphoreGive(i2cMutex);
    delay_ms(30);
    return res;
}

bool Device::readMeasurement(std::uint8_t *buffer) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (!write16(CMD_READ_MEASUREMENT)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to send read measurement command");
#endif
        xSemaphoreGive(i2cMutex);
        return false;
    }
    delay_ms(1);
    const bool res = readBytes(buffer, 9);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to read measurement bytes");
#endif
        return false;
    }
    for (std::uint32_t i = 0; i < 9; i += 3) {
        if (getCRC8(&buffer[i]) != buffer[i + 2]) {
#if DEBUG
            ESP_LOGE(TAG, "CRC mismatch for measurement %d", i);
#endif
            return false;
        }
    }
    return true;
}

bool Device::isDataReady() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (!write16(CMD_GET_DATA_READY_STATUS)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to send data ready status command");
#endif
        xSemaphoreGive(i2cMutex);
        return false;
    }
    delay_ms(1);
    std::uint8_t raw[3];
    const bool res = readBytes(raw, 3);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to read data ready status");
#endif
        return false;
    }
    if (getCRC8(raw) != raw[2]) {
#if DEBUG
        ESP_LOGE(TAG, "CRC mismatch for data ready status");
#endif
        return false;
    }
    const std::uint16_t status = (raw[0] << 8) | raw[1];
    return (status & 0x07FF) != 0; // lower 11 bits non-zero
}

std::uint8_t Device::getCRC8(const std::uint8_t *data) {
    std::uint8_t crc = 0xFF;
    for (std::uint32_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (std::uint32_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}

bool Device::write16(const std::uint16_t value) {
    std::uint8_t data[2];
    data[0] = value >> 8;
    data[1] = value & 0xFF;
    const esp_err_t res =
        i2c_master_write_to_device(i2c_port, i2c_addr, data, 2, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

bool Device::write16WithArg(const std::uint16_t cmd, const std::uint16_t arg) {
    std::uint8_t data[5];
    data[0] = cmd >> 8;
    data[1] = cmd & 0xFF;
    data[2] = arg >> 8;
    data[3] = arg & 0xFF;
    data[4] = getCRC8(&data[2]);
    const esp_err_t res =
        i2c_master_write_to_device(i2c_port, i2c_addr, data, 5, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

bool Device::readBytes(std::uint8_t *buffer, std::size_t len) {
    const esp_err_t res =
        i2c_master_read_from_device(i2c_port, i2c_addr, buffer, len, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

} // namespace SCD41

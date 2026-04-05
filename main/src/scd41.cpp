#include "scd41.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "utils.hpp"
#include <algorithm>

namespace SCD41 {

static const char *TAG = "SCD41";

constexpr std::uint32_t SINGLE_SHOT_FREQ_MS =
    std::max(SCD41_SINGLE_SHOT_FREQ_MS, static_cast<std::uint32_t>(5'000));

#define DEBUG 0

void scdTask(void *pvParameters) {
    Device *scd = static_cast<Device *>(pvParameters);
    scd->start();
}

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr), initialised(false) {}

Device::~Device() {}

bool Device::init() {
    if (!wake()) {
        ESP_LOGE(TAG, "Failed to wake device!");
        return false;
    }
    if (!stopPeriodicMeasurement()) {
        ESP_LOGE(TAG, "Failed to stop periodic measurement");
        return false;
    }
    ESP_LOGI(TAG, "SCD41 initialised successfully!");
    taskENTER_CRITICAL(&initMux);
    initialised = true;
    taskEXIT_CRITICAL(&initMux);
    xTaskCreate(scdTask, "SCDTask", 4096, this, 5, NULL);
    return true;
}

void Device::start() {
    while (true) {
        TickType_t last = xTaskGetTickCount();
        if (!isInitialised()) {
            ESP_LOGI(TAG, "SCD41 read-loop task stopping...");
            vTaskDelete(NULL);
            return;
        }
        const bool res = singleShot();
        if (!res) {
            ESP_LOGE(TAG, "Failed to start single shot!");
            continue;
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SINGLE_SHOT_FREQ_MS - 5'000));
    }
}

bool Device::isInitialised() {
    bool res;
    taskENTER_CRITICAL(&initMux);
    res = initialised;
    taskEXIT_CRITICAL(&initMux);
    return res;
}

bool Device::sleep() {
    taskENTER_CRITICAL(&initMux);
    initialised = false;
    taskEXIT_CRITICAL(&initMux);
    // Todo: may need to either wait here or stop the periodic task.
    const bool stopped = stopPeriodicMeasurement();
    if (!stopped) {
        ESP_LOGE(TAG, "Could not stop device");
    }
    const bool res = powerDown();
    if (!res) {
        ESP_LOGE(TAG, "Could not sleep device");
    }
    return stopped && res;
}

void Device::logReadings(QueueHandle_t q) {
    const bool ready = isDataReady();
    if (!ready) {
        ESP_LOGI(TAG, "Data not ready yet");
        return;
    }
    ESP_LOGI(TAG, "Reading data");
    std::uint8_t raw[9];
    if (!readMeasurement(raw)) {
        ESP_LOGE(TAG, "Failed to read SCD41");
        return;
    }
    const std::uint16_t co2Raw = (raw[0] << 8) | raw[1];
    const std::uint16_t tempRaw = (raw[3] << 8) | raw[4];
    const std::uint16_t humRaw = (raw[6] << 8) | raw[7];
    const float tempNorm = static_cast<float>(tempRaw) / 65535.0f; // 2^16 - 1
    const float temperature = (175.0 * tempNorm) - 45.0;           // -45...130
    const float humNorm = static_cast<float>(humRaw) / 65535.0f;
    const float humidity = 100.0 * humNorm; // 0..1 -> 0..100
    const float co2PPM = static_cast<float>(co2Raw);
    ESP_LOGI(TAG, "Temp: %.2f °C, Hum: %.2f %%", temperature, humidity);
    const Event co2Event = {co2PPM, time(NULL), EventType::CO2};
    xQueueSend(q, &co2Event, portMAX_DELAY);
}

bool Device::startPeriodicMeasurement() {
    return write16(CMD_START_PERIODIC_MEASUREMENT);
}

bool Device::startLowPowerPeriodicMeasurement() {
    return write16(CMD_START_LP_PERIODIC_MEASUREMENT);
}

bool Device::singleShot() {
    const bool res = write16(CMD_MEASURE_SH);
    if (!res) {
        return false;
    }
    delay_ms(5'000);
    return true;
}

bool Device::singleShotRHT() {
    const bool res = write16(CMD_MEASURE_SH_RHT);
    if (!res) {
        return false;
    }
    delay_ms(50);
    return true;
}

bool Device::stopPeriodicMeasurement() {
    const bool res = write16(CMD_STOP_PERIODIC_MEASUREMENT);
    delay_ms(500);
    return res;
}

bool Device::powerDown() {
    const bool res = write16(CMD_POWER_DOWN);
    delay_ms(1);
    return res;
}

bool Device::wake() {
    const bool res = write16(CMD_WAKE_UP);
    delay_ms(30);
    return res;
}

bool Device::readBytes(std::uint8_t *buffer, std::size_t len) {
    const esp_err_t res =
        i2c_master_read_from_device(i2c_port, i2c_addr, buffer, len, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

bool Device::readMeasurement(std::uint8_t *buffer) {
    if (!write16(CMD_READ_MEASUREMENT)) {
        ESP_LOGE(TAG, "Failed to send read measurement command");
        return false;
    }
    delay_ms(1);
    if (!readBytes(buffer, 9)) {
        ESP_LOGE(TAG, "Failed to read measurement bytes");
        return false;
    }
    for (std::uint32_t i = 0; i < 9; i += 3) {
        if (getCRC8(&buffer[i]) != buffer[i + 2]) {
            ESP_LOGE(TAG, "CRC mismatch for measurement %d", i);
            return false;
        }
    }
    return true;
}

bool Device::isDataReady() {
    if (!write16(CMD_GET_DATA_READY_STATUS)) {
        ESP_LOGE(TAG, "Failed to send data ready status command");
        return false;
    }
    delay_ms(1);
    std::uint8_t raw[3];
    if (!readBytes(raw, 3)) {
        ESP_LOGE(TAG, "Failed to read data ready status");
        return false;
    }
    if (getCRC8(raw) != raw[2]) {
        ESP_LOGE(TAG, "CRC mismatch for data ready status");
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

} // namespace SCD41

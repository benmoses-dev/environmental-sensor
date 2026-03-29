#include "scd41.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "utils.hpp"

namespace SCD41 {

static const char *TAG = "SCD41";
static constexpr std::uint16_t CMD_START_PERIODIC_MEASUREMENT = 0x21B1;
static constexpr std::uint16_t CMD_STOP_PERIODIC_MEASUREMENT = 0x3F86;
static constexpr std::uint16_t CMD_READ_MEASUREMENT = 0xEC05;
static constexpr std::uint16_t CMD_GET_DATA_READY_STATUS = 0xE4B8;

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr) {}

Device::~Device() {}

bool Device::init() {
    if (!stopPeriodicMeasurement()) {
        ESP_LOGE(TAG, "Failed to stop periodic measurement");
        return false;
    }
    delay_ms(500);
    if (!startPeriodicMeasurement()) {
        ESP_LOGE(TAG, "Failed to start periodic measurement");
        return false;
    }
    ESP_LOGI(TAG, "SCD41 initialised successfully!");
    return true;
}

bool Device::startPeriodicMeasurement() {
    return write16(CMD_START_PERIODIC_MEASUREMENT);
}

bool Device::stopPeriodicMeasurement() { return write16(CMD_STOP_PERIODIC_MEASUREMENT); }

bool Device::readBytes(std::uint8_t *buffer, std::size_t len) {
    const esp_err_t res =
        i2c_master_read_from_device(i2c_port, i2c_addr, buffer, len, pdMS_TO_TICKS(200));
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
        i2c_master_write_to_device(i2c_port, i2c_addr, data, 2, pdMS_TO_TICKS(1000));
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
        i2c_master_write_to_device(i2c_port, i2c_addr, data, 5, pdMS_TO_TICKS(1000));
    return res == ESP_OK;
}

} // namespace SCD41

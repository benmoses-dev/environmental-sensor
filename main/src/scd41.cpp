#include "scd41.hpp"
#include "esp_log.h"
#include "events.hpp"

namespace SCD41 {

static const char *TAG = "SCD41";

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr) {}

Device::~Device() {}

bool Device::init() {
    if (!write16(0x21B1)) {
        ESP_LOGE(TAG, "Failed to start periodic measurement");
        return false;
    }
    ESP_LOGI(TAG, "SCD41 initialised successfully!");
    return true;
}

void Device::logReadings(QueueHandle_t q) {
    std::uint8_t raw[9];
    esp_err_t res =
        i2c_master_read_from_device(i2c_port, i2c_addr, raw, 9, pdMS_TO_TICKS(1000));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD41");
        return;
    }
    for (std::uint32_t i = 0; i < 9; i += 3) {
        if (getCRC8(&raw[i]) != raw[i + 2]) {
            ESP_LOGW(TAG, "CRC mismatch for measurement %d", i);
            return;
        }
    }
    std::uint16_t co2Raw = (raw[0] << 8) | raw[1];
    std::uint16_t tempRaw = (raw[3] << 8) | raw[4];
    std::uint16_t humRaw = (raw[6] << 8) | raw[7];
    float tempNorm = static_cast<float>(tempRaw) / 65535.0f; // 2^16 - 1
    float temperature = (175.0 * tempNorm) - 45.0;           // -45...130
    float humNorm = static_cast<float>(humRaw) / 65535.0f;
    float humidity = 100.0 * humNorm; // 0..1 -> 0..100
    float co2PPM = static_cast<float>(co2Raw);
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

bool Device::write16(std::uint16_t value) {
    std::uint8_t data[2];
    data[0] = value >> 8;
    data[1] = value & 0xFF;
    esp_err_t res =
        i2c_master_write_to_device(i2c_port, i2c_addr, data, 2, pdMS_TO_TICKS(1000));
    return res == ESP_OK;
}

} // namespace SCD41

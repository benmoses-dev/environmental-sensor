#include "scd41.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "utils.hpp"

namespace SCD41 {

static const char *TAG = "SCD41";

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr) {}

Device::~Device() {}

bool Device::init() {
    if (!i2cInitialised && !initialiseI2C(i2c_port, i2c_addr)) {
        return false;
    }
    ESP_LOGI(TAG, "SCD41 initialised successfully!");
    return true;
}

void Device::logReadings(QueueHandle_t q) {}
} // namespace SCD41

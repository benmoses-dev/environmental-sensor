#pragma once

#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor.hpp"

namespace SCD41 {

class Device : public ISensor {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = SCD41_ADDR);
    ~Device();

    bool init() override;
    void logReadings(QueueHandle_t q) override;
    bool sleep() override;
    bool isInitialised() override;

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    bool initialised;
    portMUX_TYPE initMux = portMUX_INITIALIZER_UNLOCKED;

    bool write16(std::uint16_t value);
    bool write16WithArg(const std::uint16_t cmd, const std::uint16_t arg);
    std::uint8_t getCRC8(const std::uint8_t *data);
    bool readBytes(std::uint8_t *buffer, std::size_t len);
    bool readMeasurement(std::uint8_t *buffer);
    bool isDataReady();
    bool startPeriodicMeasurement();
    bool stopPeriodicMeasurement();
};

} // namespace SCD41

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

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;

    bool write16(std::uint16_t value);
    std::uint8_t getCRC8(const std::uint8_t *data);
};

} // namespace SCD41

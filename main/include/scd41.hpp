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
    void start();
    bool isInitialised() override;
    bool sleep() override;
    void logReadings(QueueHandle_t q) override;

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    bool initialised;
    portMUX_TYPE initMux = portMUX_INITIALIZER_UNLOCKED;

    bool startPeriodicMeasurement();
    bool startLowPowerPeriodicMeasurement();
    bool singleShot();
    bool singleShotRHT();
    bool stopPeriodicMeasurement();
    bool powerDown();
    bool wake();
    bool readBytes(std::uint8_t *buffer, std::size_t len);
    bool readMeasurement(std::uint8_t *buffer);
    bool isDataReady();
    std::uint8_t getCRC8(const std::uint8_t *data);
    bool write16(std::uint16_t value);
    bool write16WithArg(const std::uint16_t cmd, const std::uint16_t arg);

    static constexpr std::uint16_t CMD_START_PERIODIC_MEASUREMENT = 0x21B1;
    static constexpr std::uint16_t CMD_STOP_PERIODIC_MEASUREMENT = 0x3F86;
    static constexpr std::uint16_t CMD_READ_MEASUREMENT = 0xEC05;
    static constexpr std::uint16_t CMD_GET_DATA_READY_STATUS = 0xE4B8;
    static constexpr std::uint16_t CMD_START_LP_PERIODIC_MEASUREMENT = 0x21AC;
    static constexpr std::uint16_t CMD_MEASURE_SH = 0x219D;
    static constexpr std::uint16_t CMD_MEASURE_SH_RHT = 0x2196;
    static constexpr std::uint16_t CMD_POWER_DOWN = 0x36E0;
    static constexpr std::uint16_t CMD_WAKE_UP = 0x36F6;
};

} // namespace SCD41

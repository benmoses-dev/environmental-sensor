#pragma once

#include "config.hpp"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstdint>
#include <cstring>
#include <ctime>

namespace SHT45 {

struct Reading {
    float temperature = 0.0f;
    float humidity = 0.0f;
    time_t tval = 0;
    bool read = false;
    bool valid = false;
};

class Device : public ISensor {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = SHT45_ADDR);
    ~Device();

    bool init() override;
    std::uint32_t getInitTime() override { return 5; };
    std::uint32_t getDataReadyTime() override { return READING_DURATION_MS; };
    std::uint32_t getLoopTime() override { return SHT45_READ_FREQ_MS; };
    void logReadings(QueueHandle_t q) override;
    bool sleep() override;
    bool isInitialised() override;
    void start();

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    volatile bool initialised;
    bool shutdown;
    portMUX_TYPE shutdownMux = portMUX_INITIALIZER_UNLOCKED;
    SemaphoreHandle_t shutdownAck = nullptr;
    TaskHandle_t taskHandle = nullptr;
    static constexpr std::uint32_t READING_DURATION_MS = 20;
    static constexpr std::uint8_t NO_HEAT_HIGH_PRECISION = 0xFD;
    static constexpr std::uint8_t NO_HEAT_MED_PRECISION = 0xF6;
    static constexpr std::uint8_t NO_HEAT_LOW_PRECISION = 0xE0;
    static constexpr std::uint8_t HIGH_HEAT_1_S = 0x39;
    static constexpr std::uint8_t HIGH_HEAT_100_MS = 0x32;
    static constexpr std::uint8_t MED_HEAT_1_S = 0x2F;
    static constexpr std::uint8_t MED_HEAT_100_MS = 0x24;
    static constexpr std::uint8_t LOW_HEAT_1_S = 0x1E;
    static constexpr std::uint8_t LOW_HEAT_100_MS = 0x15;
    static constexpr std::uint8_t READ_SERIAL = 0x89;
    static constexpr std::uint8_t SOFT_RESET = 0x94;

    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;

    bool storeReading();
    bool reset();
    void setShutdown();
    bool getShutdown();
    Reading getReading();
};

} // namespace SHT45

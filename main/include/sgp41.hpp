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

namespace SGP41 {

struct Reading {
    float voc = 0.0f;
    float nox = 0.0f;
    time_t tval = 0;
    bool read = false;
    bool valid = false;
};

class Device : public ISensor {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = SGP41_ADDR);
    ~Device();

    bool init() override;
    std::uint32_t getInitTime() override { return 200; };
    std::uint32_t getDataReadyTime() override { return READING_DURATION_MS; };
    std::uint32_t getLoopTime() override { return SGP41_READ_FREQ_MS; };
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
    static constexpr std::uint32_t READING_DURATION_MS = 200;

    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;

    bool performReading();
};

} // namespace SGP41

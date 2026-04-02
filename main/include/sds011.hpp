#pragma once

#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstring>
#include <ctime>

namespace SDS011 {

struct Reading {
    float pm2_5 = 0.0f;
    float pm10 = 0.0f;
    bool valid = false;
    time_t t = 0;
};

class Device : public ISensor {
  public:
    explicit Device(const uart_port_t p = UART_PORT);
    ~Device();

    bool init() override;
    void logReadings(QueueHandle_t q) override;
    void start();
    bool sleep() override;
    bool isInitialised() override;

  private:
    const uart_port_t port;
    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;
    bool initialised;
    portMUX_TYPE initMux = portMUX_INITIALIZER_UNLOCKED;

    static void parseFrame(const std::uint8_t frame[10], Reading &r);
    bool sendCommand(const std::uint8_t command, const std::uint8_t param);
};

} // namespace SDS011

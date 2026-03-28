#pragma once

#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstring>

namespace SDS011 {

struct Reading {
    float pm2_5 = 0.0f;
    float pm10 = 0.0f;
    bool valid = false;
};

class Device : public ISensor {
  public:
    explicit Device(const uart_port_t p = UART_PORT);
    ~Device();

    bool init() override;
    void logReadings(QueueHandle_t q, const time_t t) override;
    void start();

  private:
    const uart_port_t port;
    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;

    static void parseFrame(const std::uint8_t frame[10], Reading &r);
};

} // namespace SDS011

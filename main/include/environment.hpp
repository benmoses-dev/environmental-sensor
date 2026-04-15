#pragma once

#include "freertos/FreeRTOS.h"
#include <ctime>

struct EnvironmentState {
    float temperature, humidity;
    time_t last;
};

class Environment {
  public:
    explicit Environment();
    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;
    EnvironmentState getSnapshot();
    void setTemperature(const float temp, const time_t t);
    void setHumidity(const float hum, const time_t t);

  private:
    EnvironmentState state;
    portMUX_TYPE envMux = portMUX_INITIALIZER_UNLOCKED;
};

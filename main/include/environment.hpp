#pragma once

#include "portmacro.h"
#include <ctime>

struct EnvironmentState {
    float temperature, humidity;
    time_t last;
    float dewPoint() const;
};

class Environment {
  public:
    explicit Environment();
    EnvironmentState getSnapshot();
    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;

  private:
    EnvironmentState state;
    portMUX_TYPE envMux = portMUX_INITIALIZER_UNLOCKED;
    void setTemperature(const float temp);
    void setHumidity(const float hum);
};

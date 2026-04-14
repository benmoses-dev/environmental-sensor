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
    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;
    EnvironmentState getSnapshot();
    void setTemperature(const float temp);
    void setHumidity(const float hum);

  private:
    EnvironmentState state;
    portMUX_TYPE envMux = portMUX_INITIALIZER_UNLOCKED;
};

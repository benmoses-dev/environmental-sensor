#include "environment.hpp"
#include "freertos/idf_additions.h"
#include <atomic>
#include <cmath>
#include <ctime>

Environment::Environment() : state(20.0f, 55.0f, time(NULL)) {
    static std::atomic<bool> created = false;
    bool expected = false;
    if (!created.compare_exchange_strong(expected, true)) {
        abort(); // We can remove this if we want multiple environments...
    }
}

EnvironmentState Environment::getSnapshot() {
    taskENTER_CRITICAL(&envMux);
    const EnvironmentState res = state;
    taskEXIT_CRITICAL(&envMux);
    return res;
}
void Environment::setTemperature(const float temp) {
    taskENTER_CRITICAL(&envMux);
    state.temperature = temp;
    taskEXIT_CRITICAL(&envMux);
}
void Environment::setHumidity(const float hum) {
    taskENTER_CRITICAL(&envMux);
    state.humidity = hum;
    taskEXIT_CRITICAL(&envMux);
}

float EnvironmentState::dewPoint() const {
    constexpr float a_w = 17.62f;
    constexpr float b_w = 243.12f;
    constexpr float a_i = 22.46f;
    constexpr float b_i = 272.62f;
    const float a = (temperature >= 0.0f) ? a_w : a_i;
    const float b = (temperature >= 0.0f) ? b_w : b_i;
    const float rh = humidity / 100.0f;
    if (rh <= 0.0f) {
        return NAN;
    }
    const float gamma = (a * temperature) / (b + temperature) + std::log(rh);
    return (b * gamma) / (a - gamma);
}

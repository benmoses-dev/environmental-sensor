#include "environment.hpp"
#include "freertos/idf_additions.h"
#include <algorithm>
#include <atomic>
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
void Environment::setTemperature(const float temp, const time_t t) {
    taskENTER_CRITICAL(&envMux);
    if (t > state.last) {
        state.last = t;
        state.temperature = temp;
    }
    taskEXIT_CRITICAL(&envMux);
}
void Environment::setHumidity(const float hum, const time_t t) {
    taskENTER_CRITICAL(&envMux);
    if (t > state.last) {
        state.last = t;
        state.humidity = hum;
    }
    taskEXIT_CRITICAL(&envMux);
}

#pragma once

#include "mqtt.hpp"

class Logger {
    public:
    explicit Logger(MQTT &m);
    void logInfo(const char *tag, const char *message);

    private:
    MQTT &mqtt;
};
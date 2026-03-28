#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <ctime>

struct ISensor {
    virtual bool init() = 0;
    virtual void logReadings(QueueHandle_t q, const time_t t) = 0;
    virtual ~ISensor() = default;
};

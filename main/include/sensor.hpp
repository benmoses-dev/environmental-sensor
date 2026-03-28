#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct ISensor {
    virtual bool init() = 0;
    virtual void logReadings(QueueHandle_t q) = 0;
    virtual ~ISensor() = default;
};

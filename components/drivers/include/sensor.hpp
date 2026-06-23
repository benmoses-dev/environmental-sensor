#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

struct ISensor {
    virtual bool init() = 0;
    virtual void logReadings(QueueHandle_t q) = 0;
    virtual ~ISensor() = default;
    virtual bool sleep() = 0;
    virtual bool isInitialised() = 0;
    virtual std::uint32_t getInitTime() = 0;
    virtual std::uint32_t getDataReadyTime() = 0;
    virtual std::uint32_t getLoopTime() = 0;
};

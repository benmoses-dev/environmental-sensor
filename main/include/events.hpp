#pragma once

#include <ctime>

enum EventType {
    TEMP,
    HUM,
    PRES,
    GAS,
    PM2_5,
    PM10,
};

struct Event {
    float val;
    time_t timestamp;
    EventType type;
};

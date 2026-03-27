#pragma once

#include <ctime>

enum EventType {
    TEMP,
    HUM,
    PRES,
    GAS,
};

struct Event {
    float val;
    time_t timestamp;
    EventType type;
};

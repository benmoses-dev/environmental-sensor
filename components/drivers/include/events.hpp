#pragma once

#include <ctime>

enum EventType {
    TEMP,
    HUM,
    PRES,
    GAS,
    PM0_5,
    PM1,
    PM2_5,
    PM4,
    PM10,
    CO2,
    VOC,
    NOX,
    DEW,
    VPD,
    AH,
};

struct Event {
    float val;
    time_t timestamp;
    EventType type;
};

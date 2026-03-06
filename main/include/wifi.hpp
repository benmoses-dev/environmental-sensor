#pragma once

#include "esp_wifi.h"
#include <ctime>

class WIFI {
  public:
    static volatile bool connected;
    bool init(void);
    bool initTime(void) const;
    time_t getTime() const;

  private:
    esp_event_handler_instance_t wifiH;
    esp_event_handler_instance_t ipH;
    void initialiseSNTP(void) const;
};

#pragma once

#include "esp_wifi.h"
#include <cstdint>
#include <ctime>

class WIFI {
  public:
    explicit WIFI();
    ~WIFI();
    static volatile bool CONNECTED;
    static const std::int32_t CONNECTED_BIT = BIT0;
    bool init(void);
    bool initTime(void) const;
    time_t getTime() const;

  private:
    const char *ssid;
    const char *pass;
    esp_event_handler_instance_t wifiH;
    esp_event_handler_instance_t ipH;
    EventGroupHandle_t weg;
    void initialiseSNTP(void) const;
    static void wifiEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                                 void *data);
    static void ipEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                               void *data);
};

#pragma once

#include "esp_wifi.h"
#include <cstdint>
#include <ctime>
#include <atomic>

class WIFI {
  public:
    explicit WIFI();
    WIFI(const WIFI &) = delete;
    WIFI(const WIFI &&) = delete;
    ~WIFI() = default;
    WIFI &operator=(const WIFI &) = delete;
    WIFI &operator=(const WIFI &&) = delete;
    static const std::int32_t CONNECTED_BIT = BIT0;
    std::atomic<bool> connected = false;
    bool init();
    bool initTime() const;
    time_t getTime() const;

  private:
    const char *ssid;
    const char *pass;
    esp_event_handler_instance_t wifiH;
    esp_event_handler_instance_t ipH;
    EventGroupHandle_t weg;
    void initialiseSNTP() const;
    static void wifiEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                                 void *data);
    static void ipEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                               void *data);
};

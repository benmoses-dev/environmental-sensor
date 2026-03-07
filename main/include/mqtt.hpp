#pragma once

#include "mqtt_client.h"
#include <cstdint>

class MQTT {
  public:
    explicit MQTT();
    ~MQTT();
    bool init();
    static volatile bool CONNECTED;
    static const std::int32_t CONNECTED_BIT = BIT0;
    bool publish(const char *topic, const char *message);

  private:
    const char *endpoint;
    std::uint32_t port;
    const char *user;
    const char *pass;
    const char *clientID;
    esp_mqtt_client_handle_t client;
    EventGroupHandle_t meg;
    static void handler(void *args, esp_event_base_t base, std::int32_t id, void *data);
};

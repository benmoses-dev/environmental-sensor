#pragma once

#include "mqtt_client.h"
#include <atomic>
#include <cstdint>

class MQTT {
  public:
    explicit MQTT();
    ~MQTT();
    bool init();
    static constexpr EventBits_t CONNECTED_BIT = BIT0;
    volatile bool connected;
    bool publish(const char *topic, const char *message);
    bool waitForPublishes(TickType_t timeoutTicks) const;
    static constexpr EventBits_t ALL_PUBLISHED_BIT = BIT1;

  private:
    const char *endpoint;
    std::uint32_t port;
    const char *user;
    const char *pass;
    const char *clientID;
    const char *deviceID;
    esp_mqtt_client_handle_t client;
    EventGroupHandle_t meg;
    static void handler(void *args, esp_event_base_t base, std::int32_t id, void *data);
    void getTopic(const char *topic, char *buf, const std::size_t len) const;
    std::atomic<std::uint32_t> pendingPublishes{0};
};

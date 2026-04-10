#include "mqtt.hpp"
#include "config.hpp"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/event_groups.h"

static const char *TAG = "MQTT";

#define DEBUG 0

void MQTT::getTopic(const char *topic, char *buf, const std::size_t len) const {
    const std::int32_t n = snprintf(buf, len, "device/%s/%s", deviceID, topic);
    if (n < 0 || n >= len) {
#if DEBUG
        ESP_LOGW(TAG, "Topic truncated!");
#endif
    }
}

MQTT::MQTT()
    : connected(false), endpoint(CONFIG_MQTT_ENDPOINT), port(CONFIG_MQTT_PORT),
      user(CONFIG_MQTT_USER), pass(CONFIG_MQTT_PASS), clientID(CONFIG_MQTT_CLIENT_ID),
      deviceID(CONFIG_MQTT_DEVICE_ID) {};

MQTT::~MQTT() {};

void MQTT::handler(void *args, esp_event_base_t base, std::int32_t id, void *data) {
    MQTT *self = static_cast<MQTT *>(args);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;
    std::uint32_t pending;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
#if DEBUG
        ESP_LOGI(TAG, "MQTT connected");
#endif
        self->connected = true;
        xEventGroupSetBits(self->meg, MQTT::CONNECTED_BIT);
        self->publish("status", "online");
        break;

    case MQTT_EVENT_DISCONNECTED:
#if DEBUG
        ESP_LOGW(TAG, "MQTT disconnected");
#endif
        self->connected = false;
        xEventGroupClearBits(self->meg, MQTT::CONNECTED_BIT);
        break;

    case MQTT_EVENT_PUBLISHED:
        if (self->pendingPublishes.load() > 0) {
            self->pendingPublishes.fetch_sub(1);
        }
        pending = self->pendingPublishes.load();
#if DEBUG
        ESP_LOGI(TAG, "MQTT publish ACK received, pending=%lu",
                 static_cast<unsigned long>(pending));
#endif
        if (pending == 0) {
            xEventGroupSetBits(self->meg, MQTT::ALL_PUBLISHED_BIT);
        }
        break;

    default:
        break;
    }
}

bool MQTT::publish(const char *topic, const char *message) {
    if (!connected) {
#if DEBUG
        ESP_LOGW(TAG, "MQTT not connected, not sending message!");
#endif
        return false;
    }
    const std::size_t len = 64;
    char buf[len];
    getTopic(topic, buf, len);
    pendingPublishes.fetch_add(1);
    xEventGroupClearBits(meg, MQTT::ALL_PUBLISHED_BIT);
    const std::int32_t res = esp_mqtt_client_publish(client, buf, message, 0, 1, 0);
    if (res < 0) {
#if DEBUG
        ESP_LOGW(TAG, "Error publishing message!");
#endif
        pendingPublishes.fetch_sub(1);
        if (pendingPublishes.load() == 0) {
            xEventGroupSetBits(meg, MQTT::ALL_PUBLISHED_BIT);
        }
        return false;
    }
#if DEBUG
    const std::uint32_t pending = pendingPublishes.load();
    ESP_LOGI(TAG, "Queued publish to %s, pending=%lu", buf,
             static_cast<unsigned long>(pending));
#endif
    return true;
}

bool MQTT::waitForPublishes(TickType_t timeoutTicks) const {
    if (pendingPublishes.load() == 0) {
        return true;
    }
#if DEBUG
    ESP_LOGI(TAG, "Waiting for MQTT ACKs...");
#endif
    EventBits_t bits =
        xEventGroupWaitBits(meg, MQTT::ALL_PUBLISHED_BIT, pdFALSE, pdTRUE, timeoutTicks);
    return (bits & MQTT::ALL_PUBLISHED_BIT) != 0;
}

bool MQTT::init() {
#if DEBUG
    ESP_LOGI(TAG, "Initialising MQTT...");
#endif
    meg = xEventGroupCreate();
    esp_mqtt_client_config_t config = {};
    config.broker.address.hostname = endpoint;
    config.broker.address.port = port;
    config.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;
    config.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    config.credentials.username = user;
    config.credentials.client_id = clientID;
    config.credentials.authentication.password = pass;
    config.session.keepalive = 0;
    config.session.disable_keepalive = false;
    config.session.protocol_ver = MQTT_PROTOCOL_V_5;
    config.task.priority = 3;
    client = esp_mqtt_client_init(&config);
    if (client == NULL) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to initialise MQTT client!");
#endif
        return false;
    }
    esp_err_t res =
        esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, MQTT::handler, this);
    if (res != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to register MQTT client handler!");
#endif
        return false;
    }
    res = esp_mqtt_client_start(client);
    if (res != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to start MQTT client!");
#endif
        return false;
    }
    EventBits_t bits =
        xEventGroupWaitBits(meg, MQTT::CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if (bits & MQTT::CONNECTED_BIT) {
#if DEBUG
        ESP_LOGI(TAG, "MQTT initialised successfully!");
#endif
        return true;
    }
#if DEBUG
    ESP_LOGE(TAG, "Failed to initialise MQTT!");
#endif
    return false;
}

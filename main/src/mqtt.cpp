#include "mqtt.hpp"
#include "config.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/event_groups.h"

static const char *TAG = "MQTT";

MQTT::MQTT()
    : endpoint(CONFIG_MQTT_ENDPOINT), port(CONFIG_MQTT_PORT), user(CONFIG_MQTT_USER),
      pass(CONFIG_MQTT_PASS), clientID(CONFIG_MQTT_CLIENT_ID) {};

MQTT::~MQTT() {};

void MQTT::handler(void *args, esp_event_base_t base, std::int32_t id, void *data) {
    MQTT *self = static_cast<MQTT *>(args);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(self->meg, MQTT::CONNECTED_BIT);
        self->publish("device/status", "online");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(self->meg, MQTT::CONNECTED_BIT);
        break;
    default:
        break;
    }
}

bool MQTT::publish(const char *topic, const char *message) {
    const std::int32_t res = esp_mqtt_client_publish(client, topic, message, 0, 2, 1);
    if (res < 0) {
        return false;
    }
    return true;
}

bool MQTT::init() {
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
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, MQTT::handler, this);
    esp_mqtt_client_start(client);
    EventBits_t bits =
        xEventGroupWaitBits(meg, MQTT::CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    return (bits & MQTT::CONNECTED_BIT);
}

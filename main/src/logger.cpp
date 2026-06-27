#include "logger.hpp"
#include "config.hpp"
#include "esp_log.h"

Logger::Logger(MQTT &m) : mqtt(m) {}

void Logger::logInfo(const char *tag, const char *message) {
#if MAIN_DEBUG
    ESP_LOGI(tag, "%s", message);
#endif
    mqtt.publish("info", message);
}
#include "wifi.hpp"
#include "config.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cstring>
#include <ctime>

static const char *TAG = "NETWORK";

void WIFI::wifiEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                            void *data) {
#if WIFI_DEBUG
    ESP_LOGI(TAG, "WiFi handler fired! base=%s id=%d", base, id);
#endif
    WIFI *self = static_cast<WIFI *>(arg);
    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
#if WIFI_DEBUG
        ESP_LOGI(TAG, "Retrying connection...");
#endif
        xEventGroupClearBits(self->weg, WIFI::CONNECTED_BIT);
        self->connected = false;
        esp_wifi_connect();
        break;
    default:
        break;
    }
}

void WIFI::ipEventHandler(void *arg, esp_event_base_t base, std::int32_t id, void *data) {
#if WIFI_DEBUG
    ESP_LOGI(TAG, "IP handler fired! base=%s id=%d", base, id);
#endif
    WIFI *self = static_cast<WIFI *>(arg);
    ip_event_got_ip_t *event;
    switch (id) {
    case IP_EVENT_STA_GOT_IP:
        event = (ip_event_got_ip_t *)data;
#if WIFI_DEBUG
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
#endif
        self->connected = true;
        xEventGroupSetBits(self->weg, WIFI::CONNECTED_BIT);
        break;
    case IP_EVENT_STA_LOST_IP:
        xEventGroupClearBits(self->weg, WIFI::CONNECTED_BIT);
        self->connected = false;
        break;
    default:
        break;
    }
}

WIFI::WIFI() : connected(false), ssid(CONFIG_WIFI_SSID), pass(CONFIG_WIFI_PASS) {}

WIFI::~WIFI() {}

bool WIFI::init() {
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Initialising NVS...");
#endif
    esp_err_t res = nvs_flash_init();
    if (res != ESP_OK) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
#if WIFI_DEBUG
            ESP_LOGE(TAG, "No NVS partition");
#endif
            return false;
        }
        res = nvs_flash_init();
        if (res != ESP_OK) {
            switch (res) {
            case ESP_ERR_NVS_NO_FREE_PAGES:
#if WIFI_DEBUG
                ESP_LOGE(TAG, "No free pages");
#endif
                return false;
            case ESP_ERR_NOT_FOUND:
#if WIFI_DEBUG
                ESP_LOGE(TAG, "No NVS partition");
#endif
                return false;
            case ESP_ERR_NO_MEM:
#if WIFI_DEBUG
                ESP_LOGE(TAG, "No memory could be allocated");
#endif
                return false;
            default:
#if WIFI_DEBUG
                ESP_LOGE(TAG, "Error initialising flash");
#endif
                return false;
            }
        }
    }
#if WIFI_DEBUG
    ESP_LOGI(TAG, "NVS initialised successfully!");
    ESP_LOGI(TAG, "Creating event loop...");
#endif
    res = esp_event_loop_create_default();
    if (res != ESP_OK && res != ESP_ERR_INVALID_STATE) {
        switch (res) {
        case ESP_ERR_NO_MEM:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "No memory could be allocated");
#endif
            return false;
        case ESP_FAIL:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Failed to create task loop");
#endif
            return false;
        default:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Error creating event loop");
#endif
            return false;
        }
    }
    weg = xEventGroupCreate();
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &WIFI::wifiEventHandler, this, &wifiH);
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &WIFI::ipEventHandler,
                                        this, &ipH);
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Event loop created and handlers registered!");
    ESP_LOGI(TAG, "Initialising TCP/IP stack");
#endif
    res = esp_netif_init();
    if (res != ESP_OK) {
#if WIFI_DEBUG
        ESP_LOGE(TAG, "TCP/IP initialisation failed!");
#endif
        return false;
    }
    if (esp_netif_create_default_wifi_sta() == nullptr) {
#if WIFI_DEBUG
        ESP_LOGE(TAG, "Failed to create the default STA!");
#endif
        return false;
    }
#if WIFI_DEBUG
    ESP_LOGI(TAG, "TCP/IP stack initialised successfully!");
#endif
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#if WIFI_DEBUG
    ESP_LOGI(TAG, "WiFi init...");
#endif
    res = esp_wifi_init(&cfg);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_NO_MEM:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "No memory could be allocated");
#endif
            return false;
        default:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Error initialising WiFi");
#endif
            return false;
        }
    }
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Setting WiFi mode...");
#endif
    res = esp_wifi_set_mode(WIFI_MODE_STA);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_INVALID_ARG:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Invalid argument");
#endif
            return false;
        default:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Error setting WiFi mode");
#endif
            return false;
        }
    }
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Setting WiFi config...");
#endif
    res = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_WIFI_PASSWORD:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Invalid password");
#endif
            return false;
        case ESP_ERR_WIFI_STATE:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Still connecting...");
#endif
            return false;
        default:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Error setting WiFi config");
#endif
            return false;
        }
    }
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Starting WiFi...");
#endif
    res = esp_wifi_start();
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_NO_MEM:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Out of memory");
#endif
            return false;
        case ESP_ERR_WIFI_CONN:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "WiFi internal error");
#endif
            return false;
        default:
#if WIFI_DEBUG
            ESP_LOGE(TAG, "Error starting WiFi");
#endif
            return false;
        }
    }
    const EventBits_t bits =
        xEventGroupWaitBits(weg, WIFI::CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if (bits & WIFI::CONNECTED_BIT) {
#if WIFI_DEBUG
        ESP_LOGI(TAG, "WiFi connected!");
#endif
        return true;
    }
    return false;
}

void WIFI::initialiseSNTP() const {
#if WIFI_DEBUG
    ESP_LOGI(TAG, "Initialising SNTP...");
#endif
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();
#if WIFI_DEBUG
    ESP_LOGI(TAG, "SNTP Initialised successfully!");
#endif
}

bool WIFI::initTime() const {
    initialiseSNTP();
    time_t now = 0;
    struct tm timeinfo;
    timeinfo.tm_year = 0;
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry <= retry_count) {
#if WIFI_DEBUG
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
#endif
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    if (timeinfo.tm_year >= (2016 - 1900)) {
#if WIFI_DEBUG
        ESP_LOGI(TAG, "Time synchronised: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#endif
        return true;
    } else {
#if WIFI_DEBUG
        ESP_LOGE(TAG, "Failed to synchronise time");
#endif
        return false;
    }
}

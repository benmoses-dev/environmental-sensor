#include "wifi.hpp"
#include "config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cstring>

static const char *TAG = "NETWORK";

void WIFI::wifiEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                            void *data) {
    ESP_LOGI(TAG, "WiFi handler fired! base=%s id=%d", base, id);
    WIFI *self = static_cast<WIFI *>(arg);
    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "Retrying connection...");
        xEventGroupClearBits(self->weg, WIFI::CONNECTED_BIT);
        self->connected = false;
        esp_wifi_connect();
        break;
    default:
        break;
    }
}

void WIFI::ipEventHandler(void *arg, esp_event_base_t base, std::int32_t id, void *data) {
    ESP_LOGI(TAG, "IP handler fired! base=%s id=%d", base, id);
    WIFI *self = static_cast<WIFI *>(arg);
    ip_event_got_ip_t *event;
    switch (id) {
    case IP_EVENT_STA_GOT_IP:
        event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
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
    ESP_LOGI(TAG, "Initialising NVS...");
    esp_err_t res = nvs_flash_init();
    if (res != ESP_OK) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "No NVS partition");
            return false;
        }
        res = nvs_flash_init();
        if (res != ESP_OK) {
            switch (res) {
            case ESP_ERR_NVS_NO_FREE_PAGES:
                ESP_LOGE(TAG, "No free pages");
                return false;
            case ESP_ERR_NOT_FOUND:
                ESP_LOGE(TAG, "No NVS partition");
                return false;
            case ESP_ERR_NO_MEM:
                ESP_LOGE(TAG, "No memory could be allocated");
                return false;
            default:
                ESP_LOGE(TAG, "Error initialising flash");
                return false;
            }
        }
    }
    ESP_LOGI(TAG, "NVS initialised successfully!");
    ESP_LOGI(TAG, "Creating event loop...");
    res = esp_event_loop_create_default();
    if (res != ESP_OK && res != ESP_ERR_INVALID_STATE) {
        switch (res) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "No memory could be allocated");
            return false;
        case ESP_FAIL:
            ESP_LOGE(TAG, "Failed to create task loop");
            return false;
        default:
            ESP_LOGE(TAG, "Error creating event loop");
            return false;
        }
    }
    weg = xEventGroupCreate();
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &WIFI::wifiEventHandler, this, &wifiH);
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &WIFI::ipEventHandler,
                                        this, &ipH);
    ESP_LOGI(TAG, "Event loop created and handlers registered!");
    ESP_LOGI(TAG, "Initialising TCP/IP stack");
    res = esp_netif_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "TCP/IP initialisation failed!");
        return false;
    }
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        ESP_LOGE(TAG, "Failed to create the default STA!");
        return false;
    }
    ESP_LOGI(TAG, "TCP/IP stack initialised successfully!");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "WiFi init...");
    res = esp_wifi_init(&cfg);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "No memory could be allocated");
            return false;
        default:
            ESP_LOGE(TAG, "Error initialising WiFi");
            return false;
        }
    }
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    ESP_LOGI(TAG, "Setting WiFi mode...");
    res = esp_wifi_set_mode(WIFI_MODE_STA);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "Invalid argument");
            return false;
        default:
            ESP_LOGE(TAG, "Error setting WiFi mode");
            return false;
        }
    }
    ESP_LOGI(TAG, "Setting WiFi config...");
    res = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_WIFI_PASSWORD:
            ESP_LOGE(TAG, "Invalid password");
            return false;
        case ESP_ERR_WIFI_STATE:
            ESP_LOGE(TAG, "Still connecting...");
            return false;
        default:
            ESP_LOGE(TAG, "Error setting WiFi config");
            return false;
        }
    }
    ESP_LOGI(TAG, "Starting WiFi...");
    res = esp_wifi_start();
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "Out of memory");
            return false;
        case ESP_ERR_WIFI_CONN:
            ESP_LOGE(TAG, "WiFi internal error");
            return false;
        default:
            ESP_LOGE(TAG, "Error starting WiFi");
            return false;
        }
    }
    const EventBits_t bits =
        xEventGroupWaitBits(weg, WIFI::CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if (bits & WIFI::CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        return true;
    }
    return false;
}

void WIFI::initialiseSNTP() const {
    ESP_LOGI(TAG, "Initialising SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP Initialised successfully!");
}

bool WIFI::initTime() const {
    initialiseSNTP();
    time_t now = 0;
    struct tm timeinfo;
    timeinfo.tm_year = 0;
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry <= retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    if (timeinfo.tm_year >= (2016 - 1900)) {
        ESP_LOGI(TAG, "Time synchronised: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to synchronise time");
        return false;
    }
}

time_t WIFI::getTime() const { return time(NULL); }

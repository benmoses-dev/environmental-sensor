#include "network.hpp"
#include "config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "utils.hpp"
#include <cstdint>
#include <cstring>

static const char *TAG = "NETWORK";
static EventGroupHandle_t weg;
static const int WIFI_CONNECTED_BIT = BIT0;
static volatile bool wifi_connected = false;

static void wifiEventHandler(void *arg, esp_event_base_t base, std::int32_t id,
                             void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying connection...");
        delay_ms(1000);
        wifi_connected = false;
        esp_wifi_connect();
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        xEventGroupSetBits(weg, WIFI_CONNECTED_BIT);
    }
}

bool wifiConnect(void) {
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
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler,
                                        NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler,
                                        NULL, &instance_got_ip);
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
    strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));
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
    ESP_LOGI(TAG, "Connecting WiFi...");
    res = esp_wifi_connect();
    if (res != ESP_OK) {
        switch (res) {
        case ESP_ERR_WIFI_MODE:
            ESP_LOGE(TAG, "WiFi mode error");
            return false;
        case ESP_ERR_WIFI_CONN:
            ESP_LOGE(TAG, "WiFi internal error");
            return false;
        case ESP_ERR_WIFI_SSID:
            ESP_LOGE(TAG, "SSID error");
            return false;
        default:
            ESP_LOGE(TAG, "Error initialising WiFi connection...");
            return false;
        }
    }
    const EventBits_t bits =
        xEventGroupWaitBits(weg, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        return true;
    }
    return false;
}

void initialiseSNTP(void) {
    ESP_LOGI(TAG, "Initialising SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();
    ESP_LOGI(TAG, "Done");
}

bool obtainTime(void) {
    initialiseSNTP();
    time_t now = 0;
    struct tm timeinfo;
    timeinfo.tm_year = 0;
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
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
        ESP_LOGW(TAG, "Failed to synchronise time");
        return false;
    }
}

time_t getTime() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    return now;
}

#include "sds011.hpp"
#include "config.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "events.hpp"
#include "utils.hpp"
#include <ctime>

namespace SDS011 {

static const char *TAG = "SDS011";

Device::Device(const uart_port_t p) : port(p) {};

Device::~Device() {};

void sdsTask(void *pvParameters) {
    Device *sds = static_cast<Device *>(pvParameters);
    sds->start();
}

bool Device::init() {
    uart_config_t uartConfig = {};
    uartConfig.baud_rate = 9600;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_driver_install(port, UART_BUF_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return false;
    }
    err = uart_param_config(port, &uartConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART");
        return false;
    }
    err = uart_set_pin(port, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return false;
    }
    ESP_LOGI(TAG, "SDS011 UART initialised");
    xTaskCreate(sdsTask, "SDSTask", 4096, this, 5, NULL);
    return true;
}

void Device::logReadings(QueueHandle_t q) {
    Reading at{};
    taskENTER_CRITICAL(&readingMux);
    at = reading;
    taskEXIT_CRITICAL(&readingMux);
    const Event pm2_5Event = {at.pm2_5, at.t, EventType::PM2_5};
    const Event pm10Event = {at.pm10, at.t, EventType::PM10};
    xQueueSend(q, &pm2_5Event, portMAX_DELAY);
    xQueueSend(q, &pm10Event, portMAX_DELAY);
}

void Device::start() {
    std::uint8_t byte = 0;
    std::uint8_t frame[10];
    while (true) {
        std::int32_t len = uart_read_bytes(port, &byte, 1, pdMS_TO_TICKS(2000));
        if (len != 1) {
            ESP_LOGW(TAG, "Timed out waiting for SDS011 data");
            continue;
        }
        if (byte != 0xAA) {
            continue;
        }
        frame[0] = byte;
        len = uart_read_bytes(port, frame + 1, 9, pdMS_TO_TICKS(200));
        if (len != 9) {
            ESP_LOGW(TAG, "Incorrect number of bytes read from uart");
            continue;
        }
        Reading res{};
        parseFrame(frame, res);
        taskENTER_CRITICAL(&readingMux);
        reading = res;
        taskEXIT_CRITICAL(&readingMux);
        delay_ms(500);
    }
}

void Device::parseFrame(const std::uint8_t frame[10], Reading &reading) {
    reading.valid = false;
    reading.pm2_5 = 0.0f;
    reading.pm10 = 0.0f;
    reading.t = 0;
    if (frame[0] != 0xAA || frame[1] != 0xC0 || frame[9] != 0xAB) {
        ESP_LOGW(TAG, "Incorrect frame format");
        return;
    }
    std::uint8_t checksum = 0;
    for (std::uint32_t i = 2; i <= 7; ++i) {
        checksum += frame[i];
    }
    if (checksum != frame[8]) {
        ESP_LOGW(TAG, "Incorrect checksum");
        return;
    }
    reading.valid = true;
    reading.t = time(NULL);
    std::uint16_t pm2_5Raw = (static_cast<std::uint16_t>(frame[3]) << 8) | frame[2];
    std::uint16_t pm10Raw = (static_cast<std::uint16_t>(frame[5]) << 8) | frame[4];
    reading.pm2_5 = static_cast<float>(pm2_5Raw) / 10.0f;
    reading.pm10 = static_cast<float>(pm10Raw) / 10.0f;
}

} // namespace SDS011

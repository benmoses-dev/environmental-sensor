#include "sds011.hpp"
#include "config.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "events.hpp"
#include "utils.hpp"
#include <cstring>
#include <ctime>

namespace SDS011 {

static const char *TAG = "SDS011";

#define DEBUG 0

Device::Device(const uart_port_t p) : port(p), initialised(false) {};

Device::~Device() {};

void sdsTask(void *pvParameters) {
    Device *sds = static_cast<Device *>(pvParameters);
    sds->start();
}

bool Device::init() {
    TickType_t startInit = xTaskGetTickCount();
#if DEBUG
    const auto startTime = millis();
#endif
    uart_config_t uartConfig = {};
    uartConfig.baud_rate = 9600;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_driver_install(port, UART_BUF_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to install UART driver");
#endif
        return false;
    }
    err = uart_param_config(port, &uartConfig);
    if (err != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to configure UART");
#endif
        return false;
    }
    err = uart_set_pin(port, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to set UART pins");
#endif
        return false;
    }
#if DEBUG
    ESP_LOGI(TAG, "SDS011 UART initialised");
#endif
    if (!wake()) {
        return false;
    }
    taskENTER_CRITICAL(&initMux);
    initialised = true;
    taskEXIT_CRITICAL(&initMux);
#if DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u", totalTime);
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(sdsTask, "SDSTask", 4096, this, 5, NULL);
    return true;
}

bool Device::wake() {
    uart_flush_input(port);
    if (!sendCommand(SDS_WAKE_FRAME)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to working mode!");
#endif
        return false;
    }
    std::uint8_t res[SDS_RESPONSE_LENGTH];
    if (!readResponse(res, pdMS_TO_TICKS(1000))) {
#if DEBUG
        ESP_LOGW(TAG, "Failed to query SDS011 working mode!");
#endif
        return false;
    }
    if (res[2] != 0x06) {
#if DEBUG
        ESP_LOGW(TAG, "Working mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x01) { // Work
#if DEBUG
        ESP_LOGW(TAG, "Working mode query returned non-working result!");
#endif
        return false;
    }
#if DEBUG
    ESP_LOGI(TAG, "Set SDS011 to working state.");
#endif
    uart_flush_input(port);
    if (!sendCommand(SDS_SET_ACTIVE_MODE_FRAME)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to active mode!");
#endif
        return false;
    }
    if (!readResponse(res, pdMS_TO_TICKS(1000))) {
#if DEBUG
        ESP_LOGW(TAG, "Failed to query SDS011 active mode!");
#endif
        return false;
    }
    if (res[2] != 0x02) {
#if DEBUG
        ESP_LOGW(TAG, "Active mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x00) { // Active
#if DEBUG
        ESP_LOGW(TAG, "Active mode query returned non-active result!");
#endif
        return false;
    }
    ESP_LOGI(TAG, "Set SDS011 to active mode.");
    return true;
}

bool Device::sleep() {
    taskENTER_CRITICAL(&initMux);
    initialised = false;
    taskEXIT_CRITICAL(&initMux);
    delay_ms(300);
    uart_flush_input(port);
#if DEBUG
    ESP_LOGI(TAG, "Setting SDS011 to sleeping state...");
#endif
    if (!sendCommand(SDS_SLEEP_FRAME)) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to sleeping state!");
#endif
        return false;
    }
    std::uint8_t res[SDS_RESPONSE_LENGTH];
    if (!readResponse(res, pdMS_TO_TICKS(1000))) {
#if DEBUG
        ESP_LOGW(TAG, "Failed to query SDS011 working mode!");
#endif
        return false;
    }
    if (res[2] != 0x06) {
#if DEBUG
        ESP_LOGW(TAG, "Working mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x00) { // Sleep
#if DEBUG
        ESP_LOGW(TAG, "Working mode query returned non-sleeping result!");
#endif
        return false;
    }
    ESP_LOGI(TAG, "Set SDS011 to sleeping state.");
    taskENTER_CRITICAL(&readingMux);
    reading = {};
    reading.valid = false;
    taskEXIT_CRITICAL(&readingMux);
    return true;
}

bool Device::isInitialised() {
    bool res;
    taskENTER_CRITICAL(&initMux);
    res = initialised;
    taskEXIT_CRITICAL(&initMux);
    return res;
}

void Device::logReadings(QueueHandle_t q) {
    Reading res{};
    taskENTER_CRITICAL(&readingMux);
    res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    if (res.read || !res.valid) {
#if DEBUG
        ESP_LOGW(TAG, "Reading has already been read or is invalid...");
#endif
        return;
    }
    const Event pm2_5Event = {res.pm2_5, res.t, EventType::PM2_5};
    const Event pm10Event = {res.pm10, res.t, EventType::PM10};
    xQueueSend(q, &pm2_5Event, portMAX_DELAY);
    xQueueSend(q, &pm10Event, portMAX_DELAY);
}

bool Device::sendCommand(const std::uint8_t (&frame)[SDS_FRAME_LENGTH]) {
    std::uint8_t temp[SDS_FRAME_LENGTH];
    std::memcpy(temp, frame, SDS_FRAME_LENGTH);
    std::uint8_t checksum = 0;
    for (std::size_t i = 2; i <= 16; ++i) {
        checksum += temp[i];
    }
    temp[17] = checksum;
    const std::int32_t written =
        uart_write_bytes(port, reinterpret_cast<const char *>(temp), SDS_FRAME_LENGTH);
    if (written != SDS_FRAME_LENGTH) {
#if DEBUG
        ESP_LOGE(TAG, "Failed to send SDS011 frame");
#endif
        return false;
    }
    const esp_err_t err = uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
#if DEBUG
        ESP_LOGE(TAG, "UART TX did not complete!");
#endif
        return false;
    }
    return true;
}

void Device::start() {
    std::uint8_t frame[SDS_RESPONSE_LENGTH];
    uart_flush_input(port);
#if DEBUG
    const auto sTime = millis();
    std::uint32_t count = 0;
#endif
    while (true) {
        if (!isInitialised()) {
            ESP_LOGI(TAG, "SDS011 read-loop task stopping...");
            vTaskDelete(NULL);
            return;
        }
        if (!readResponse(frame, 100)) {
            continue;
        }
        Reading res{};
        parseFrame(frame, res);
        if (res.valid) {
            taskENTER_CRITICAL(&readingMux);
            reading = res;
            taskEXIT_CRITICAL(&readingMux);
#if DEBUG
            count++;
            const auto eTime = millis();
            const auto totalTime = eTime - sTime;
            ESP_LOGI(TAG, "Total loop time: %u", totalTime);
            ESP_LOGI(TAG, "Total loop count: %u", count);
#endif
        }
    }
}

bool Device::readResponse(std::uint8_t (&resp)[SDS_RESPONSE_LENGTH], TickType_t timeout) {
    std::uint8_t byte = 0;
    std::int32_t len = uart_read_bytes(port, &byte, 1, timeout);
    if (len != 1 || byte != 0xAA) {
        return false;
    }
    resp[0] = byte;
    len = uart_read_bytes(port, resp + 1, SDS_RESPONSE_LENGTH - 1, timeout);
    if (len != SDS_RESPONSE_LENGTH - 1) {
#if DEBUG
        ESP_LOGW(TAG, "Incorrect number of bytes read from uart");
#endif
        return false;
    }
    std::uint8_t checksum = 0;
    for (std::uint32_t i = 2; i <= 7; ++i) {
        checksum += resp[i];
    }
    if (checksum != resp[8]) {
#if DEBUG
        ESP_LOGW(TAG, "Incorrect checksum");
#endif
        return false;
    }
    return true;
}

void Device::parseFrame(const std::uint8_t frame[SDS_RESPONSE_LENGTH], Reading &reading) {
    reading.valid = false;
    reading.pm2_5 = 0.0f;
    reading.pm10 = 0.0f;
    reading.t = 0;
    reading.read = false;
    if (frame[0] != 0xAA || frame[1] != 0xC0 || frame[9] != 0xAB) {
#if DEBUG
        ESP_LOGW(TAG, "Incorrect frame format");
#endif
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

#include "sds011.hpp"
#include "config.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "utils.hpp"
#include <cstring>
#include <ctime>

namespace SDS011 {

static const char *TAG = "SDS011";

Device::Device(const uart_port_t p) : port(p), initialised(false), shutdown(false) {};

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
};

void sdsTask(void *pvParameters) {
    Device *sds = static_cast<Device *>(pvParameters);
    sds->start();
}

bool Device::init() {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
#endif
        return false;
    }
    uartMutex = xSemaphoreCreateMutex();
    if (uartMutex == nullptr) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to create uart mutex");
#endif
        return false;
    }
    TickType_t startInit = xTaskGetTickCount();
#if SDS011_DEBUG
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
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to install UART driver");
#endif
        return false;
    }
    err = uart_param_config(port, &uartConfig);
    if (err != ESP_OK) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to configure UART");
#endif
        return false;
    }
    err = uart_set_pin(port, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to set UART pins");
#endif
        return false;
    }
#if SDS011_DEBUG
    ESP_LOGI(TAG, "SDS011 UART initialised");
#endif
    xSemaphoreTake(uartMutex, portMAX_DELAY);
    const bool res = wake();
    xSemaphoreGive(uartMutex);
    if (!res) {
        return false;
    }
    initialised = true;
#if SDS011_DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u, predicted: %u", totalTime, getInitTime());
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(sdsTask, "SDSTask", 4096, this, 5, &taskHandle);
    return true;
}

bool Device::wake() {
    uart_flush_input(port);
    if (!sendCommand(SDS_WAKE_FRAME)) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to working mode!");
#endif
        return false;
    }
    std::uint8_t res[SDS_RESPONSE_LENGTH];
    if (!readResponse(res)) {
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Failed to query SDS011 working mode!");
#endif
        return false;
    }
    if (res[2] != 0x06) {
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Working mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x01) { // Work
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Working mode query returned non-working result!");
#endif
        return false;
    }
#if SDS011_DEBUG
    ESP_LOGI(TAG, "Set SDS011 to working state.");
#endif
    uart_flush_input(port);
    if (!sendCommand(SDS_SET_QUERY_MODE_FRAME)) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to query mode!");
#endif
        return false;
    }
    if (!readResponse(res)) {
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Failed to query SDS011 active mode!");
#endif
        return false;
    }
    if (res[1] != 0xC5 || res[2] != 0x02) {
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Active mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x01) { // Query
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Query mode query returned non-active result!");
#endif
        return false;
    }
    ESP_LOGI(TAG, "Set SDS011 to query mode.");
    return true;
}

bool Device::sleep() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
    xSemaphoreTake(uartMutex, portMAX_DELAY);
    uart_flush_input(port);
#if SDS011_DEBUG
    ESP_LOGI(TAG, "Setting SDS011 to sleeping state...");
#endif
    if (!sendCommand(SDS_SLEEP_FRAME)) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to set SDS011 to sleeping state!");
#endif
        xSemaphoreGive(uartMutex);
        return false;
    }
    std::uint8_t res[SDS_RESPONSE_LENGTH];
    const bool result = readResponse(res);
    xSemaphoreGive(uartMutex);
    if (!result) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to query SDS011 working mode!");
#endif
        return false;
    }
    if (res[2] != 0x06) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Working mode query incorrect command!");
#endif
        return false;
    }
    if (res[4] != 0x00) { // Sleep
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Working mode query returned non-sleeping result!");
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

bool Device::isInitialised() { return initialised; }

void Device::logReadings(QueueHandle_t q) {
    Reading res{};
    taskENTER_CRITICAL(&readingMux);
    res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    if (res.read || !res.valid) {
#if SDS011_DEBUG
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
#if SDS011_DEBUG
        ESP_LOGE(TAG, "Failed to send SDS011 frame");
#endif
        return false;
    }
    const esp_err_t err = uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
#if SDS011_DEBUG
        ESP_LOGE(TAG, "UART TX did not complete!");
#endif
        return false;
    }
    return true;
}

void Device::start() {
    std::uint8_t frame[SDS_RESPONSE_LENGTH];
    TickType_t startLoop = xTaskGetTickCount();
    while (true) {
#if SDS011_DEBUG
        const auto sTime = millis();
        ESP_LOGI(TAG, "Start loop time: %u", sTime);
#endif
        bool shouldStop;
        taskENTER_CRITICAL(&shutdownMux);
        shouldStop = shutdown;
        taskEXIT_CRITICAL(&shutdownMux);
        if (shouldStop) {
            ESP_LOGI(TAG, "SDS011 read-loop task stopping...");
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        xSemaphoreTake(uartMutex, portMAX_DELAY);
        uart_flush_input(port);
        if (!sendCommand(SDS_QUERY_DATA_FRAME)) {
#if SDS011_DEBUG
            ESP_LOGE(TAG, "Failed to send query data frame!");
#endif
            xSemaphoreGive(uartMutex);
            vTaskDelayUntil(&startLoop, pdMS_TO_TICKS(READING_FREQ_MS));
            continue;
        }
        const bool result = readResponse(frame, pdMS_TO_TICKS(200));
        xSemaphoreGive(uartMutex);
        if (!result) {
#if SDS011_DEBUG
            ESP_LOGE(TAG, "Failed to read data response frame!");
#endif
            vTaskDelayUntil(&startLoop, pdMS_TO_TICKS(READING_FREQ_MS));
            continue;
        }
        Reading res{};
        parseFrame(frame, res);
        if (res.valid) {
            taskENTER_CRITICAL(&readingMux);
            reading = res;
            taskEXIT_CRITICAL(&readingMux);
#if SDS011_DEBUG
            const auto eTime = millis();
            ESP_LOGI(TAG, "End loop time: %u", eTime);
            const auto totalTime = eTime - sTime;
            ESP_LOGI(TAG, "Total loop time: %u", totalTime);
#endif
        } else {
#if SDS011_DEBUG
            ESP_LOGW(TAG, "Reading is invalid!");
#endif
        }
        vTaskDelayUntil(&startLoop, pdMS_TO_TICKS(READING_FREQ_MS));
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
#if SDS011_DEBUG
        ESP_LOGW(TAG, "Incorrect number of bytes read from uart");
#endif
        return false;
    }
    std::uint8_t checksum = 0;
    for (std::uint32_t i = 2; i <= 7; ++i) {
        checksum += resp[i];
    }
    if (checksum != resp[8]) {
#if SDS011_DEBUG
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
#if SDS011_DEBUG
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

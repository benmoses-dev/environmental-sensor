#pragma once

#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstring>
#include <ctime>

namespace SDS011 {

struct Reading {
    float pm2_5 = 0.0f;
    float pm10 = 0.0f;
    bool valid = false;
    time_t t = 0;
    bool read = false;
};

class Device : public ISensor {
  public:
    explicit Device(const uart_port_t p = UART_PORT);
    ~Device();

    bool init() override;
    std::uint32_t getInitTime() override { return 500; };
    std::uint32_t getDataReadyTime() override { return 30'000; };
    void logReadings(QueueHandle_t q) override;
    void start();
    bool sleep() override;
    bool isInitialised() override;

  private:
    const uart_port_t port;
    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;
    bool initialised;
    portMUX_TYPE initMux = portMUX_INITIALIZER_UNLOCKED;
    static constexpr std::size_t SDS_FRAME_LENGTH = 19;
    static constexpr std::size_t SDS_RESPONSE_LENGTH = 10;
    static constexpr std::uint8_t SDS_WAKE_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x08, 0xAB};
    static constexpr std::uint8_t SDS_SLEEP_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB};
    static constexpr std::uint8_t SDS_QUERY_WORK_STATE_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
    static constexpr std::uint8_t SDS_SET_ACTIVE_MODE_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
    static constexpr std::uint8_t SDS_SET_QUERY_MODE_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0xAB};
    static constexpr std::uint8_t SDS_QUERY_REPORT_MODE_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0xAB};
    static constexpr std::uint8_t SDS_QUERY_DATA_FRAME[SDS_FRAME_LENGTH] = {
        0xAA, 0xB4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0xAB};

    static void parseFrame(const std::uint8_t frame[10], Reading &r);
    bool sendCommand(const std::uint8_t (&frame)[SDS_FRAME_LENGTH]);
    bool readResponse(std::uint8_t (&res)[SDS_RESPONSE_LENGTH],
                      TickType_t timeout = pdMS_TO_TICKS(100));
    bool wake();
};

} // namespace SDS011

#pragma once

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "events.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include <cmath>
#include <cstdint>
#include <ctime>

inline void delay_ms(std::uint32_t ms) {
    if (ms == 0) {
        return;
    }
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

inline void delay_us(std::uint32_t us, void *interface) { esp_rom_delay_us(us); }

inline std::uint32_t millis() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

inline std::uint8_t getCRC8(const std::uint8_t *data) {
    std::uint8_t crc = 0xFF;
    for (std::uint32_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (std::uint32_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}

/**
 * Calculate dew point, vapour pressure deficit, and absolute humidity using Magnus
 * formula. Uses constants for water or ice, depending on ambient temperature.
 */
inline void sendCombinedMetrics(const float temp, const float hum, const time_t tval,
                                const QueueHandle_t q) {
    if (!std::isfinite(temp) || !std::isfinite(hum)) {
        return;
    }
    constexpr float A_W = 17.62f;
    constexpr float B_W = 243.12f;
    constexpr float A_I = 22.46f;
    constexpr float B_I = 272.62f;
    const float a = (temp >= 0.0f) ? A_W : A_I;
    const float b = (temp >= 0.0f) ? B_W : B_I;
    const float gamma_s = (a * temp) / (b + temp);
    const float es = 6.112f * expf(gamma_s);
    const float rh = fminf(fmaxf(hum * 0.01f, 1e-4f), 1.0f);
    const float e = es * rh;
    const float vpd = (es - e) * 0.1f;
    const float gamma = logf(rh) + gamma_s;
    const float dewPoint = (b * gamma) / (a - gamma);
    const float absHumidity = (216.7f * e) / (temp + 273.15f);
    const Event tEvent = {temp, tval, EventType::TEMP};
    const Event hEvent = {hum, tval, EventType::HUM};
    const Event vpdEvent = {vpd, tval, EventType::VPD};
    const Event dpEvent = {dewPoint, tval, EventType::DEW};
    const Event ahEvent = {absHumidity, tval, EventType::AH};
    xQueueSend(q, &tEvent, portMAX_DELAY);
    xQueueSend(q, &hEvent, portMAX_DELAY);
    xQueueSend(q, &vpdEvent, portMAX_DELAY);
    xQueueSend(q, &dpEvent, portMAX_DELAY);
    xQueueSend(q, &ahEvent, portMAX_DELAY);
}

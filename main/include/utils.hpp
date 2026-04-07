#pragma once

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>

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

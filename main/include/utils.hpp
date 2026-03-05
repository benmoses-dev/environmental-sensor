#pragma once

#include "freertos/FreeRTOS.h"
#include <cstdint>

static void delay_ms(std::uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

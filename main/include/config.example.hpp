#pragma once

#include <cstdint>

/**
 * Sensors.
 */
#define READ_BME280 0
#define READ_BME680 1
#define READ_SDS011 1
#define READ_SCD41 0
#define READ_SHT45 0
#define READ_SGP40 0
#define READ_SGP41 0
#define READ_SPS30 0

/**
 * Debugging.
 */
#define MAIN_DEBUG 0

/**
 * Timing configuration.
 */

/**
 * Operating mode:
 * 0 = continuous
 * 1 or greater = periodic
 * I would not recommend periodic mode for the BME680 gas heater as it needs to stabilise.
 */
inline constexpr std::uint8_t OPERATING_MODE = 0;
/**
 * Periodic mode only.
 * This is how long to wait between measurement publishing.
 * This must be at least as long as the warmup time.
 * Longer durations greatly reduce power consumption.
 */
inline constexpr std::uint32_t MEASUREMENT_PERIOD_MS = 300'000;

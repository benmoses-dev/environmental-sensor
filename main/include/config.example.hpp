#pragma once

#include "driver/i2c.h"
#include "driver/uart.h"
#include <cstdint>

#define CONFIG_MQTT_ENDPOINT "broker.example"
#define CONFIG_MQTT_USER "username"
#define CONFIG_MQTT_PASS "mqttpass"
#define CONFIG_MQTT_PORT 8883
#define CONFIG_MQTT_CLIENT_ID "device1"
#define CONFIG_MQTT_DEVICE_ID "1"
#define CONFIG_WIFI_SSID "myssid"
#define CONFIG_WIFI_PASS "wifipassword"

#define READ_BME280 1
#define READ_BME680 0
#define READ_SDS011 1
#define READ_SCD41 0

inline constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_0;
inline constexpr std::uint32_t I2C_MASTER_FREQ_HZ = 100'000;
inline constexpr std::int32_t I2C_MASTER_SDA_IO = 22;
inline constexpr std::int32_t I2C_MASTER_SCL_IO = 23;

inline constexpr std::uint8_t BME280_ADDR = 0x76;
inline constexpr std::uint8_t BME680_ADDR = 0x77;
inline constexpr std::uint8_t BME680_CHIP_ID = 0x61;
/**
 * How often the BME680 should take a reading, including a heater cycle.
 * There is a trade-off between heater residual temperatures and measurement resolution.
 */
inline constexpr std::uint32_t BME680_HEATER_FREQ_MS = 60'000;
inline constexpr std::uint8_t SCD41_ADDR = 0x62;

inline constexpr uart_port_t UART_PORT = UART_NUM_2;
inline constexpr std::uint8_t TX_PIN = 19;
inline constexpr std::uint8_t RX_PIN = 18;
inline constexpr std::uint32_t UART_BUF_SIZE = 256;

inline constexpr float TEMP_ADJUST = 0.0;
inline constexpr float HUM_ADJUST = 0.0;
inline constexpr float PRES_ADJUST = 0.0;
inline constexpr float GAS_ADJUST = 0.0;

/**
 * Operating mode:
 * 0 = continuous
 * 1 or greater = periodic
 * I would not recommend periodic mode for the BME680 gas heater as it needs to stabilise.
 */
inline constexpr std::uint8_t OPERATING_MODE = 0;
/**
 * Periodic mode only.
 * This is the time to wait for the sensors to stabilise.
 * Typically, I have found that this is:
 * 30 seconds for the SDS011 fan
 * 5-10 seconds for the SCD41
 * If using the BME680, this must be at least as long as the BME680_HEATER_FREQ_MS
 * configuration above!
 */
inline constexpr std::uint32_t WARMUP_TIME_S = 10;
/**
 * Periodic mode only.
 * This is how long to wait between measurement publishing.
 * This must be at least as long as the warmup time.
 * Longer durations greatly reduce power consumption.
 */
inline constexpr std::uint32_t MEASUREMENT_PERIOD_S = 300;
/**
 * Continuous mode only.
 * This determines how long between reading/publishing in continuous mode.
 * Increasing this slightly reduces power consumption.
 */
inline constexpr std::uint32_t MAIN_LOOP_MS = 5'000; // Continuous loop frequency

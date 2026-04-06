#pragma once

#include "driver/i2c.h"
#include "driver/uart.h"
#include <cstdint>

/**
 * WiFi/MQTT configuration.
 */
#define CONFIG_MQTT_ENDPOINT "broker.example"
#define CONFIG_MQTT_USER "username"
#define CONFIG_MQTT_PASS "mqttpass"
#define CONFIG_MQTT_PORT 8883
#define CONFIG_MQTT_CLIENT_ID "device1"
#define CONFIG_MQTT_DEVICE_ID "1"
#define CONFIG_WIFI_SSID "myssid"
#define CONFIG_WIFI_PASS "wifipassword"

/**
 * Sensors.
 */
#define READ_BME280 0
#define READ_BME680 1
#define READ_SDS011 1
#define READ_SCD41 0

/**
 * I2C configuration.
 */
inline constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_0;
inline constexpr std::uint32_t I2C_MASTER_FREQ_HZ = 100'000;
inline constexpr std::int32_t I2C_MASTER_SDA_IO = 22;
inline constexpr std::int32_t I2C_MASTER_SCL_IO = 23;

/**
 * BME280 configuration.
 */
inline constexpr std::uint8_t BME280_ADDR = 0x76;
inline constexpr std::uint32_t BME280_READ_FREQ_MS = 1'000;

/**
 * BME680 configuration.
 */
inline constexpr std::uint8_t BME680_ADDR = 0x77;
inline constexpr std::uint8_t BME680_CHIP_ID = 0x61;
/**
 * How often the BME680 should take a reading, including a heater cycle.
 * There is a trade-off between heater residual temperatures and measurement resolution.
 */
inline constexpr std::uint32_t BME680_HEATER_FREQ_MS = 60'000;

/**
 * SCD41 configuration.
 */
inline constexpr std::uint8_t SCD41_ADDR = 0x62;
inline constexpr std::uint32_t SCD41_SINGLE_SHOT_FREQ_MS = 5'010; // Must be >= 5'010

/**
 * UART/SDS011 configuration.
 */
inline constexpr uart_port_t UART_PORT = UART_NUM_2;
inline constexpr std::uint8_t TX_PIN = 19;
inline constexpr std::uint8_t RX_PIN = 18;
inline constexpr std::uint32_t UART_BUF_SIZE = 256;
inline constexpr std::uint32_t SDS011_READING_FREQ_MS = 1'000;

/**
 * Sensor adjustments.
 */
inline constexpr float TEMP_ADJUST = 0.0;
inline constexpr float HUM_ADJUST = 0.0;
inline constexpr float PRES_ADJUST = 0.0;
inline constexpr float GAS_ADJUST = 0.0;

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

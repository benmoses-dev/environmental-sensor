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
 */
inline constexpr std::uint8_t OPERATING_MODE = 1;
inline constexpr std::uint32_t MEASUREMENT_PERIOD_S = 60; // Increase to save power
inline constexpr std::uint32_t WARMUP_TIME_S = 30;

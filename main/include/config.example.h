#pragma once

#include <cstdint>

#define CONFIG_MQTT_ENDPOINT "broker.example"
#define CONFIG_MQTT_USER "username"
#define CONFIG_MQTT_PASS "mqttpass"
#define CONFIG_MQTT_PORT 8883
#define CONFIG_MQTT_CLIENT_ID "device1"
#define CONFIG_MQTT_DEVICE_ID "1"
#define CONFIG_WIFI_SSID "myssid"
#define CONFIG_WIFI_PASS "wifipassword"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_SCL_IO 23

inline constexpr std::uint8_t READ_BME280 = true;
inline constexpr std::uint8_t READ_BME680 = false;
inline constexpr std::uint8_t BME280_ADDR = 0x76;
inline constexpr std::uint8_t BME680_ADDR = 0x77;

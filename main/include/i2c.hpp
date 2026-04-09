#pragma once

#include "config.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "hal/i2c_types.h"

extern SemaphoreHandle_t i2cMutex;

inline bool initialiseI2C(const i2c_port_t port) {
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t res = i2c_param_config(port, &conf);
    if (res != ESP_OK) {
        ESP_LOGE("I2C_SCAN", "Failed to configure i2c params!");
        return false;
    }
    res = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (res != ESP_OK) {
        ESP_LOGE("I2C_SCAN", "Failed to install I2C driver");
        return false;
    }
    return true;
}

inline bool readBytes(std::uint8_t *buffer, const std::size_t bytes,
                      const i2c_port_t port, const std::uint8_t addr) {
    const esp_err_t res =
        i2c_master_read_from_device(port, addr, buffer, bytes, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

inline bool writeBytes(const std::uint8_t *data, const std::size_t bytes,
                       const i2c_port_t port, const std::uint8_t addr) {
    const esp_err_t res =
        i2c_master_write_to_device(port, addr, data, bytes, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

inline bool writeBytesThenRead(const std::uint8_t *in, const std::size_t inBytes,
                               std::uint8_t *buffer, const std::size_t outBytes,
                               const i2c_port_t port, const std::uint8_t addr) {
    const esp_err_t res = i2c_master_write_read_device(port, addr, in, inBytes, buffer,
                                                       outBytes, pdMS_TO_TICKS(100));
    return res == ESP_OK;
}

inline bool write8(const std::uint8_t value, const i2c_port_t port,
                   const std::uint8_t addr) {
    return writeBytes(&value, 1, port, addr);
}

inline bool write16(const std::uint16_t value, const i2c_port_t port,
                    const std::uint8_t addr) {
    std::uint8_t data[2];
    data[0] = static_cast<std::uint8_t>(value >> 8);
    data[1] = static_cast<std::uint8_t>(value & 0xFF);
    return writeBytes(data, 2, port, addr);
}

inline bool write8read(const std::uint8_t value, std::uint8_t *buffer,
                       const std::size_t bytes, const i2c_port_t port,
                       const std::uint8_t addr) {
    return writeBytesThenRead(&value, 1, buffer, bytes, port, addr);
}

inline bool write16read(const std::uint16_t value, std::uint8_t *buffer,
                        const std::size_t bytes, const i2c_port_t port,
                        const std::uint8_t addr) {
    std::uint8_t data[2];
    data[0] = static_cast<std::uint8_t>(value >> 8);
    data[1] = static_cast<std::uint8_t>(value & 0xFF);
    return writeBytesThenRead(data, 2, buffer, bytes, port, addr);
}

inline std::uint8_t write8read8(const std::uint8_t value, const i2c_port_t port,
                                const std::uint8_t addr) {
    std::uint8_t buffer[1];
    write8read(value, buffer, 1, port, addr);
    return buffer[0];
}

inline std::int8_t write8readS8(const std::uint8_t value, const i2c_port_t port,
                                const std::uint8_t addr) {
    return static_cast<std::int8_t>(write8read8(value, port, addr));
}

inline std::uint16_t write8read16(const std::uint8_t value, const i2c_port_t port,
                                  const std::uint8_t addr) {
    std::uint8_t buffer[2];
    write8read(value, buffer, 2, port, addr);
    return (static_cast<std::uint16_t>(buffer[0]) << 8) | buffer[1];
}

inline std::int16_t write8readS16(const std::uint8_t value, const i2c_port_t port,
                                  const std::uint8_t addr) {
    return static_cast<std::int16_t>(write8read16(value, port, addr));
}

inline std::uint16_t write8read16LE(const std::uint8_t value, const i2c_port_t port,
                                    const std::uint8_t addr) {
    std::uint16_t val = write8read16(value, port, addr);
    return (val >> 8) | (val << 8);
}

inline std::int16_t write8readS16LE(const std::uint8_t value, const i2c_port_t port,
                                    const std::uint8_t addr) {
    return static_cast<std::int16_t>(write8read16LE(value, port, addr));
}

inline std::uint32_t write8read24(const std::uint8_t value, const i2c_port_t port,
                                  const std::uint8_t addr) {
    std::uint8_t buffer[3];
    write8read(value, buffer, 3, port, addr);
    return (static_cast<std::uint32_t>(buffer[0]) << 16) |
           (static_cast<std::uint32_t>(buffer[1]) << 8) | buffer[2];
}

inline bool write8WithArg(const std::uint8_t reg, const std::uint8_t arg,
                          const i2c_port_t port, const std::uint8_t addr) {
    std::uint8_t data[2] = {reg, arg};
    return writeBytes(data, 2, port, addr);
}

inline bool scanI2C(const i2c_port_t port) {
    if (!initialiseI2C(port)) {
        return false;
    }
    std::uint8_t addr = 0x00;
    ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");
    for (std::uint8_t a = 1; a < 0x78; a++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (a << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (res == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", a);
            addr = a;
        }
    }
    if (addr == 0x00) {
        ESP_LOGE("I2C_SCAN", "No device found!");
        return false;
    }
    ESP_LOGI("I2C_SCAN", "Scan complete");
    std::uint8_t chip_id = 0;
    esp_err_t test = i2c_master_write_read_device(port, addr, (std::uint8_t[]){0xD0}, 1,
                                                  &chip_id, 1, pdMS_TO_TICKS(100));
    if (test != ESP_OK) {
        ESP_LOGE("I2C_SCAN", "Manual chip ID read failed!");
        return false;
    }
    ESP_LOGI("I2C_SCAN", "Manual chip ID = 0x%02X", chip_id);
    return true;
}

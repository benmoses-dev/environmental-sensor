#pragma once

#include "config.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

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

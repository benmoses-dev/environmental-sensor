#include "bme280.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "BME280";

BME280::BME280(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr) {}

bool BME280::init() {
    if (read8(REG_ID) != 0x60) {
        ESP_LOGE(TAG, "Wrong BME280 ID!");
        return false;
    }
    write8(REG_RESET, 0xB6);
    delay_ms(10);
    while (isReadingCalibration()) {
        delay_ms(10);
    }
    readCalibration();
    setSampling();
    delay_ms(100);
    return true;
}

bool BME280::isReadingCalibration() const {
    const std::uint8_t rStatus = read8(REG_STATUS);
    return (rStatus & (1 << 0)) != 0;
}

bool BME280::readCalibration() {
    calib.dig_T1 = static_cast<std::int32_t>(read16_LE(0x88));
    calib.dig_T2 = static_cast<std::int32_t>(readS16_LE(0x8A));
    calib.dig_T3 = static_cast<std::int32_t>(readS16_LE(0x8C));

    calib.dig_P1 = static_cast<std::int64_t>(read16_LE(0x8E));
    calib.dig_P2 = static_cast<std::int64_t>(readS16_LE(0x90));
    calib.dig_P3 = static_cast<std::int64_t>(readS16_LE(0x92));
    calib.dig_P4 = static_cast<std::int64_t>(readS16_LE(0x94));
    calib.dig_P5 = static_cast<std::int64_t>(readS16_LE(0x96));
    calib.dig_P6 = static_cast<std::int64_t>(readS16_LE(0x98));
    calib.dig_P7 = static_cast<std::int64_t>(readS16_LE(0x9A));
    calib.dig_P8 = static_cast<std::int64_t>(readS16_LE(0x9C));
    calib.dig_P9 = static_cast<std::int64_t>(readS16_LE(0x9E));

    calib.dig_H1 = static_cast<std::int32_t>(read8(0xA1));
    calib.dig_H2 = static_cast<std::int32_t>(readS16_LE(0xE1));
    calib.dig_H3 = static_cast<std::int32_t>(read8(0xE3));
    calib.dig_H4 = static_cast<std::int32_t>((read8(0xE4) << 4) | (read8(0xE5) & 0xF));
    calib.dig_H5 = static_cast<std::int32_t>((read8(0xE6) << 4) | (read8(0xE5) >> 4));
    calib.dig_H6 = static_cast<std::int32_t>(read8(0xE7));

    return true;
}

void BME280::setSampling() const {
    /**
     * filter settings
     * 000 = filter off
     * 001 = 2x filter
     * 010 = 4x filter
     * 011 = 8x filter
     * 100 and above = 16x filter
     */
    const std::uint32_t filter = 0;
    /**
     * inactive duration (standby time) in normal mode
     * 000 = 0.5 ms
     * 001 = 62.5 ms
     * 010 = 125 ms
     * 011 = 250 ms
     * 100 = 500 ms
     * 101 = 1000 ms
     * 110 = 10 ms
     * 111 = 20 ms
     */
    const std::uint32_t dur = 0;
    const std::uint32_t configReg = (dur << 5) | (filter << 2) | 0;
    /**
     * temperature oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    const std::uint32_t temp = 5;
    /**
     * pressure oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    const std::uint32_t pres = 5;
    /**
     * device mode
     * 00       = sleep
     * 01 or 10 = forced
     * 11       = normal
     */
    const std::uint32_t mode = 3;
    const std::uint32_t measReg = (temp << 5) | (pres << 2) | mode;
    /**
     * humidity oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    const std::uint32_t humReg = 5;
    write8(REG_CTRL, 0b00); // Mode sleep
    write8(REG_CTRL_HUM, humReg);
    write8(REG_CONFIG, configReg);
    write8(REG_CTRL, measReg);
}

float BME280::readTemperature() {
    std::int32_t adc_T = read24(REG_TEMP_MSB) >> 4;
    std::int32_t var1 = (adc_T >> 3) - (calib.dig_T1 << 1);
    var1 = (var1 * calib.dig_T2) >> 11;
    std::int32_t var2 = (adc_T >> 4) - (calib.dig_T1);
    var2 = (((var2 * var2) >> 12) * calib.dig_T3) >> 14;
    t_fine = var1 + var2 + t_fine_adjust;
    const std::int32_t T = (t_fine * 5 + 128) >> 8;
    return static_cast<float>(T) / 100.0f;
}

float BME280::readPressure() {
    readTemperature(); // must compute t_fine
    std::int32_t adc_P = read24(REG_PRESS_MSB) >> 4;
    std::int64_t var1 = static_cast<std::int64_t>(t_fine) - 128000;
    std::int64_t var2 = var1 * var1 * calib.dig_P6;
    var2 = var2 + ((var1 * calib.dig_P5) << 17);
    var2 = var2 + ((calib.dig_P4) << 35);
    var1 = ((var1 * var1 * calib.dig_P3) >> 8) + ((var1 * calib.dig_P2) << 12);
    std::int64_t var3 = static_cast<std::int64_t>(1) << 47;
    var1 = ((var3 + var1) * calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // avoid div by zero
    }
    std::int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (calib.dig_P7 << 4);
    return static_cast<float>(p) / 256.0f;
}

float BME280::readHumidity() {
    readTemperature(); // must compute t_fine
    std::int32_t adc_H = read16(REG_HUM_MSB);
    std::int32_t var1 = t_fine - 76800;
    std::int32_t var2 = adc_H << 14;
    std::int32_t var3 = calib.dig_H4 << 20;
    std::int32_t var4 = calib.dig_H5 * var1;
    std::int32_t var5 = (var2 - var3 - var4 + 16384) >> 15;
    var2 = (var1 * calib.dig_H6) >> 10;
    var3 = (var1 * calib.dig_H3) >> 11;
    var4 = ((var2 * (var3 + 32768)) >> 10) + 2097152;
    var2 = ((var4 * calib.dig_H2) + 8192) >> 14;
    var3 = var5 + var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
    var5 = var3 - ((var4 * calib.dig_H1) >> 4);
    var5 = var5 < 0 ? 0 : var5;
    var5 = var5 > 419430400 ? 419430400 : var5;
    var5 = var5 >> 12;
    return static_cast<float>(var5) / 1024.0f;
}

void BME280::setTemperatureCompensation(const float adjustment) {
    t_fine_adjust = (static_cast<std::int32_t>(adjustment * 100) << 8) / 5;
}

float BME280::getTemperatureCompensation() const {
    return static_cast<float>((t_fine_adjust * 5) >> 8) / 100.0f;
}

std::uint8_t BME280::read8(std::uint8_t reg) const {
    std::uint8_t val;
    i2c_master_write_read_device(i2c_port, i2c_addr, &reg, 1, &val, 1,
                                 pdMS_TO_TICKS(1000));
    return val;
}

std::uint16_t BME280::read16(std::uint8_t reg) const {
    std::uint8_t buf[2];
    i2c_master_write_read_device(i2c_port, i2c_addr, &reg, 1, buf, 2,
                                 pdMS_TO_TICKS(1000));
    return (buf[0] << 8) | buf[1];
}

std::int16_t BME280::readS16(std::uint8_t reg) const {
    return static_cast<std::int16_t>(read16(reg));
}

std::uint16_t BME280::read16_LE(std::uint8_t reg) const {
    std::uint16_t val = read16(reg);
    return (val >> 8) | (val << 8);
}

std::int16_t BME280::readS16_LE(std::uint8_t reg) const {
    return static_cast<std::int16_t>(read16_LE(reg));
}

std::uint32_t BME280::read24(std::uint8_t reg) const {
    std::uint8_t buf[3];
    i2c_master_write_read_device(i2c_port, i2c_addr, &reg, 1, buf, 3,
                                 pdMS_TO_TICKS(1000));
    return (static_cast<std::uint32_t>(buf[0]) << 16) |
           (static_cast<std::uint32_t>(buf[1]) << 8) | buf[2];
}

void BME280::write8(std::uint8_t reg, std::uint8_t value) const {
    std::uint8_t buf[2] = {reg, value};
    i2c_master_write_to_device(i2c_port, i2c_addr, buf, 2, pdMS_TO_TICKS(1000));
}

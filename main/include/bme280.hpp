#pragma once

#include "driver/i2c.h"
#include <cstdint>
#include <ctime>

enum EventType {
    TEMP,
    HUM,
    PRES,
};

struct Event {
    float val;
    time_t timestamp;
    EventType type;
};

class BME280 {
  public:
    explicit BME280(const i2c_port_t port = I2C_NUM_0, const std::uint8_t addr = 0x76);
    ~BME280();

    bool init();
    float readTemperature();
    float readPressure();
    float readHumidity();
    float readAltitude(const float seaLevel);
    float seaLevelForAltitude(const float altitude, const float atmospheric);
    void setTemperatureCompensation(const float adjustment);
    float getTemperatureCompensation() const;

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    bool i2cInitialised = false;

    struct Calibration {
        std::int32_t dig_T1;
        std::int32_t dig_T2;
        std::int32_t dig_T3;

        std::int64_t dig_P1;
        std::int64_t dig_P2;
        std::int64_t dig_P3;
        std::int64_t dig_P4;
        std::int64_t dig_P5;
        std::int64_t dig_P6;
        std::int64_t dig_P7;
        std::int64_t dig_P8;
        std::int64_t dig_P9;

        std::int32_t dig_H1;
        std::int32_t dig_H2;
        std::int32_t dig_H3;
        std::int32_t dig_H4;
        std::int32_t dig_H5;
        std::int32_t dig_H6;
    } calib;

    std::int32_t t_fine = 0;
    std::int32_t t_fine_adjust = 0;

    static constexpr std::uint8_t REG_ID = 0xD0;
    static constexpr std::uint8_t REG_RESET = 0xE0;
    static constexpr std::uint8_t REG_CTRL_HUM = 0xF2;
    static constexpr std::uint8_t REG_STATUS = 0xF3;
    static constexpr std::uint8_t REG_CTRL = 0xF4;
    static constexpr std::uint8_t REG_CONFIG = 0xF5;
    static constexpr std::uint8_t REG_PRESS_MSB = 0xF7;
    static constexpr std::uint8_t REG_TEMP_MSB = 0xFA;
    static constexpr std::uint8_t REG_HUM_MSB = 0xFD;

    bool isReadingCalibration() const;
    bool readCalibration();
    void setSampling() const;
    std::uint8_t read8(std::uint8_t reg) const;
    std::uint16_t read16(std::uint8_t reg) const;
    std::int16_t readS16(std::uint8_t reg) const;
    std::uint16_t read16_LE(std::uint8_t reg) const;
    std::int16_t readS16_LE(std::uint8_t reg) const;
    std::uint32_t read24(std::uint8_t reg) const;
    void write8(std::uint8_t reg, std::uint8_t value) const;
};

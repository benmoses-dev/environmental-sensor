#pragma once

#include "config.hpp"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstdint>
#include <ctime>

namespace BME280 {

struct Reading {
    float temperature = 0.0f;
    float pressure = 0.0f;
    float humidity = 0.0f;
    time_t tval = 0;
    bool read = false;
    bool valid = false;
};

class Device : public ISensor {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = BME280_ADDR);
    ~Device();

    bool init() override;
    std::uint32_t getInitTime() override { return 130; };
    std::uint32_t getDataReadyTime() override { return READING_DURATION_MS; };
    std::uint32_t getLoopTime() override { return BME280_READ_FREQ_MS; };
    void logReadings(QueueHandle_t q) override;
    bool sleep() override;
    bool isInitialised() override;
    void start();

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    volatile bool initialised;
    bool shutdown;
    portMUX_TYPE shutdownMux = portMUX_INITIALIZER_UNLOCKED;
    SemaphoreHandle_t shutdownAck = nullptr;
    TaskHandle_t taskHandle = nullptr;
    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;

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
    static constexpr std::uint8_t STATUS_MEASURING_MASK = 0x08;
    static constexpr std::uint8_t STATUS_IM_UPDATE_MASK = 0x01;
    /**
     * device mode
     * 00       = sleep
     * 01 or 10 = forced
     * 11       = normal
     */
    static constexpr std::uint32_t MODE_SLEEP = 0;
    static constexpr std::uint32_t MODE_FORCED = 1;
    static constexpr std::uint32_t MODE_NORMAL = 3;
    // Todo: Map these to keep them together.
    /**
     * temperature oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    static constexpr std::uint32_t TEMP_OSRS = 2;
    static constexpr float osrsT = 2.0f;
    /**
     * pressure oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    static constexpr std::uint32_t PRES_OSRS = 3;
    static constexpr float osrsP = 4.0f;
    /**
     * humidity oversampling
     * 000 = skipped
     * 001 = x1
     * 010 = x2
     * 011 = x4
     * 100 = x8
     * 101 and above = x16
     */
    static constexpr std::uint32_t HUM_OSRS = 2;
    static constexpr float osrsH = 2.0f;
    /**
     * Get the measurement duration in ms based on the Bosch datasheet.
     */
    static constexpr std::uint32_t MEAS_DUR_MS = static_cast<std::uint32_t>(
        (1.25 + (2.3 * osrsT) + ((2.3 * osrsP) + 0.575) + ((2.3 * osrsH) + 0.575)) +
        0.9999f);
    static constexpr std::uint32_t READING_DURATION_MS = MEAS_DUR_MS + 5;

    bool isReadingCalibration() const;
    void readCalibration();
    bool setMode(const std::uint32_t mode) const;
    bool setSampling() const;
    bool performReading();
    bool isDataReady() const;
    float readTemperature();
    float readPressure();
    float readHumidity();
    float readAltitude(const float seaLevel);
    float seaLevelForAltitude(const float altitude, const float atmospheric);
    void setTemperatureCompensation(const float adjustment);
    float getTemperatureCompensation() const;
};

} // namespace BME280

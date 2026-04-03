#pragma once

#include "bme68x_defs.h"
#include "config.hpp"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor.hpp"
#include <cstdint>
#include <cstring>
#include <ctime>

namespace BME680 {

struct Reading {
    float temperature = 0.0f;
    float pressure = 0.0f;
    float humidity = 0.0f;
    float gasResistance = 0.0f;
    time_t tval = 0;
    bool read = false;
};

class Device : public ISensor {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = BME680_ADDR);
    ~Device();

    bool init() override;
    void logReadings(QueueHandle_t q) override;
    bool sleep() override;
    bool isInitialised() override;
    void start();

  private:
    const i2c_port_t i2c_port;
    const std::uint8_t i2c_addr;
    std::int32_t _sensorID;
    std::uint32_t measStart = 0;
    std::uint32_t measDur = 0;
    bool initialised;
    portMUX_TYPE initMux = portMUX_INITIALIZER_UNLOCKED;

    Reading reading;
    portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;

    struct bme68x_dev gas_sensor;
    struct bme68x_conf gas_conf;
    struct bme68x_heatr_conf gas_heatr_conf;

    float readAltitude(const float seaLevel);
    float seaLevelForAltitude(const float altitude, const float atmospheric);

    bool setTemperatureOversampling(std::uint8_t os);
    bool setPressureOversampling(std::uint8_t os);
    bool setHumidityOversampling(std::uint8_t os);
    bool setIIRFilterSize(std::uint8_t fs);
    bool setGasHeater(std::uint16_t heaterTemp, std::uint16_t heaterTime);
    bool setODR(std::uint8_t odr);

    std::uint32_t beginReading();
    bool endReading();
    std::int32_t remainingReadingMillis();
    bool performReading();

    static constexpr std::int32_t NOT_STARTED = -1;
    static constexpr std::int32_t COMPLETE = 0;

    static BME68X_INTF_RET_TYPE read(std::uint8_t regAddr, std::uint8_t *data,
                                     std::uint32_t len, void *interface);

    static BME68X_INTF_RET_TYPE write(std::uint8_t regAddr, const std::uint8_t *data,
                                      std::uint32_t len, void *interface);
};

} // namespace BME680

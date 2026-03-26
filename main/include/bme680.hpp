#pragma once

#include "bme68x.h"
#include "config.h"
#include "driver/i2c.h"
#include <cstdint>
#include <cstring>
#include <ctime>

namespace BME680 {

enum EventType {
    TEMP,
    HUM,
    PRES,
    GAS,
};

struct Event {
    float val;
    time_t timestamp;
    EventType type;
};

class Device {
  public:
    explicit Device(const i2c_port_t port = I2C_MASTER_NUM,
                    const std::uint8_t addr = BME680_ADDR);
    ~Device();

    bool init();
    float readAltitude(const float seaLevel);
    float seaLevelForAltitude(const float altitude, const float atmospheric);

    bool setTemperatureOversampling(std::uint8_t os);
    bool setPressureOversampling(std::uint8_t os);
    bool setHumidityOversampling(std::uint8_t os);
    bool setIIRFilterSize(std::uint8_t fs);
    bool setGasHeater(std::uint16_t heaterTemp, std::uint16_t heaterTime);
    bool setODR(std::uint8_t odr);

    bool performReading();
    std::uint32_t beginReading();
    bool endReading();
    std::int32_t remainingReadingMillis();

    static constexpr std::int32_t NOT_STARTED = -1;
    static constexpr std::int32_t COMPLETE = 0;

    float temperature;
    float pressure;
    float humidity;
    float gasResistance;

  private:
    const i2c_port_t port;
    const std::uint8_t addr;
    bool i2cInitialised = false;
    std::int32_t _sensorID;
    std::uint32_t measStart = 0;
    std::uint16_t measDur = 0;

    struct bme68x_dev gas_sensor;
    struct bme68x_conf gas_conf;
    struct bme68x_heatr_conf gas_heatr_conf;

    static BME68X_INTF_RET_TYPE read(std::uint8_t regAddr, std::uint8_t *data,
                                     std::uint32_t len, void *interface);

    static BME68X_INTF_RET_TYPE write(std::uint8_t regAddr, const std::uint8_t *data,
                                      std::uint32_t len, void *interface);
};

} // namespace BME680

#include "bme680.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

namespace BME680 {

static const char *TAG = "BME680";

static void delay_us(std::uint32_t us, void *interface) { esp_rom_delay_us(us); }

static std::uint32_t millis() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

Device::Device(const i2c_port_t i2cport, const std::uint8_t i2caddr)
    : port(i2cport), addr(i2caddr), measStart(0), measDur(0) {}

Device::~Device() {}

bool Device::init() {
    if (!i2cInitialised) {
        // Maybe replace this and call once in main
        i2c_config_t conf{};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        esp_err_t res = i2c_param_config(port, &conf);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure i2c params!");
            return false;
        }
        res = i2c_driver_install(port, conf.mode, 0, 0, 0);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install I2C driver");
            return false;
        }
        i2cInitialised = true;
    }
    gas_sensor.chip_id = addr;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = this;
    gas_sensor.read = &Device::read;
    gas_sensor.write = &Device::write;
    gas_sensor.amb_temp = 20;
    gas_sensor.delay_us = delay_us;
    std::int8_t res = bme68x_init(&gas_sensor);
    if (res != BME68X_OK) {
        return false;
    }
    res = bme68x_get_conf(&gas_conf, &gas_sensor);
    if (res != BME68X_OK) {
        return false;
    }
    /**
     * LPF for sensor readings. 3 is a nice middle ground.
     */
    if (!setIIRFilterSize(BME68X_FILTER_SIZE_3)) {
        return false;
    }
    if (!setODR(BME68X_ODR_NONE)) {
        return false;
    }
    if (!setHumidityOversampling(BME68X_OS_2X)) {
        return false;
    }
    if (!setPressureOversampling(BME68X_OS_4X)) {
        return false;
    }
    if (!setTemperatureOversampling(BME68X_OS_8X)) {
        return false;
    }
    if (!setGasHeater(320, 150)) {
        return false;
    }
    res = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
    if (res != BME68X_OK) {
        return false;
    }
    ESP_LOGI(TAG, "BME680 initialised successfully!");
    return true;
}

float Device::readAltitude(const float seaLevelPressure) {
    performReading();
    const float atmospheric = pressure / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevelPressure, 0.1903));
}

float Device::seaLevelForAltitude(const float altitude, const float atmospheric) {
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

bool Device::performReading() { return endReading(); }

std::uint32_t Device::beginReading() {
    if (measStart != 0) {
        return measStart + measDur;
    }
    std::int8_t res = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
    if (res != BME68X_OK) {
        return 0;
    }
    std::uint32_t readTime =
        bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor);
    std::uint32_t heatTime = static_cast<std::uint32_t>(gas_heatr_conf.heatr_dur) * 1000;
    std::uint32_t delay = readTime + heatTime;
    measDur = delay / 1000;
    return millis() + measDur;
}

bool Device::endReading() {
    std::uint32_t measEnd = beginReading();
    if (measEnd == 0) {
        return false;
    }
    std::int32_t remaining = remainingReadingMillis();
    if (remaining > 0) {
        vTaskDelay(pdMS_TO_TICKS(static_cast<std::uint32_t>(remaining) + 5));
    }
    measStart = 0;
    measDur = 0;
    bme68x_data data{};
    std::uint8_t n = 0;
    std::int8_t res = bme68x_get_data(BME68X_FORCED_MODE, &data, &n, &gas_sensor);
    if (res != BME68X_OK || n == 0) {
        return false;
    }
    temperature = data.temperature;
    humidity = data.humidity;
    pressure = data.pressure;
    if ((data.status & BME68X_HEAT_STAB_MSK) && (data.status & BME68X_GASM_VALID_MSK)) {
        gasResistance = data.gas_resistance;
    } else {
        gasResistance = 0.0f;
    }
    return true;
}

std::int32_t Device::remainingReadingMillis(void) {
    if (measStart != 0) {
        std::uint32_t now = millis();
        if (now < measStart) {
            return NOT_STARTED;
        }
        std::uint32_t elapsed = now - measStart;
        if (elapsed > measDur) {
            return COMPLETE;
        }
        return static_cast<int>(measDur - elapsed);
    }
    return NOT_STARTED;
}

bool Device::setGasHeater(std::uint16_t heaterTemp, std::uint16_t heaterTime) {
    if (heaterTemp == 0 || heaterTime == 0) {
        gas_heatr_conf.enable = BME68X_DISABLE;
    } else {
        gas_heatr_conf.enable = BME68X_ENABLE;
        gas_heatr_conf.heatr_temp = heaterTemp;
        gas_heatr_conf.heatr_dur = heaterTime;
    }
    std::int8_t res =
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);
    return res == 0;
}

bool Device::setODR(std::uint8_t odr) {
    if (odr > BME68X_ODR_NONE) {
        return false;
    }
    gas_conf.odr = odr;
    std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setTemperatureOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_temp = oversample;
    std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setHumidityOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_hum = oversample;
    std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setPressureOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_pres = oversample;
    std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setIIRFilterSize(std::uint8_t filtersize) {
    if (filtersize > BME68X_FILTER_SIZE_127) {
        return false;
    }
    gas_conf.filter = filtersize;
    std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

BME68X_INTF_RET_TYPE Device::read(std::uint8_t regAddr, std::uint8_t *data,
                                  std::uint32_t len, void *interface) {
    if (data == nullptr || len == 0) {
        return -1;
    }
    Device *dev = static_cast<Device *>(interface);
    esp_err_t err = i2c_master_write_read_device(dev->port, dev->addr, &regAddr, 1, data,
                                                 len, pdMS_TO_TICKS(1000));
    return (err == ESP_OK) ? BME68X_OK : -1;
}

BME68X_INTF_RET_TYPE Device::write(std::uint8_t regAddr, const std::uint8_t *data,
                                   std::uint32_t len, void *interface) {
    if (data == nullptr || len == 0 || len > 32) {
        return -1;
    }
    uint8_t buffer[33];
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, len);
    Device *dev = static_cast<Device *>(interface);
    esp_err_t err = i2c_master_write_to_device(dev->port, dev->addr, buffer, len + 1,
                                               pdMS_TO_TICKS(1000));
    return (err == ESP_OK) ? BME68X_OK : -1;
}

} // namespace BME680

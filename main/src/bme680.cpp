#include "bme680.hpp"
#include "bme68x.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.hpp"
#include <cmath>

namespace BME680 {

static const char *TAG = "BME680";

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr), measStart(0), measDur(0) {}

Device::~Device() {}

bool Device::init() {
    ESP_LOGI(TAG, "Initialising BME680...");
    if (!i2cInitialised && !initialiseI2C(i2c_port, i2c_addr)) {
        ESP_LOGE(TAG, "Failed to initialise I2C!");
        return false;
    }
    ESP_LOGI(TAG, "Starting gas sensor...");
    gas_sensor.chip_id = BME680_CHIP_ID;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = this;
    gas_sensor.read = &Device::read;
    gas_sensor.write = &Device::write;
    gas_sensor.amb_temp = 20;
    gas_sensor.delay_us = delay_us;
    std::int8_t res = bme68x_init(&gas_sensor);
    if (res != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to initialise gas sensor!");
        return false;
    }
    ESP_LOGI(TAG, "Configuring BME680...");
    res = bme68x_get_conf(&gas_conf, &gas_sensor);
    if (res != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to configure BME680!");
        return false;
    }
    /**
     * LPF for sensor readings. 3 is a nice middle ground.
     */
    if (!setIIRFilterSize(BME68X_FILTER_SIZE_3)) {
        ESP_LOGE(TAG, "Failed to set IIR filter!");
        return false;
    }
    if (!setODR(BME68X_ODR_NONE)) {
        ESP_LOGE(TAG, "Failed to set ODR!");
        return false;
    }
    if (!setHumidityOversampling(BME68X_OS_2X)) {
        ESP_LOGE(TAG, "Failed to set humidity oversampling!");
        return false;
    }
    if (!setPressureOversampling(BME68X_OS_4X)) {
        ESP_LOGE(TAG, "Failed to set pressure oversampling!");
        return false;
    }
    if (!setTemperatureOversampling(BME68X_OS_8X)) {
        ESP_LOGE(TAG, "Failed to set temperature oversampling!");
        return false;
    }
    if (!setGasHeater(320, 150)) {
        ESP_LOGE(TAG, "Failed to set gas heater config!");
        return false;
    }
    res = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
    if (res != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set operating mode!");
        return false;
    }
    ESP_LOGI(TAG, "BME680 configured successfully!");
    return true;
}

void Device::logReadings(QueueHandle_t &q, time_t t) {
    if (!performReading()) {
        return;
    }
    const Event tEvent = {temperature, t, EventType::TEMP};
    const Event hEvent = {humidity, t, EventType::HUM};
    const Event pEvent = {pressure, t, EventType::PRES};
    const Event gEvent = {gasResistance, t, EventType::GAS};
    xQueueSend(q, &tEvent, portMAX_DELAY);
    xQueueSend(q, &hEvent, portMAX_DELAY);
    xQueueSend(q, &pEvent, portMAX_DELAY);
    xQueueSend(q, &gEvent, portMAX_DELAY);
}

std::uint32_t Device::beginReading() {
    if (measStart != 0) {
        return measStart + measDur;
    }
    std::int8_t res = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
    if (res != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set operating mode while starting reading!");
        return 0;
    }
    std::uint32_t readTime =
        bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor);
    std::uint32_t heatTime = static_cast<std::uint32_t>(gas_heatr_conf.heatr_dur) * 1000;
    std::uint32_t delay = readTime + heatTime;
    measDur = delay / 1000;
    return millis() + measDur;
}

bool Device::performReading() {
    std::uint32_t measEnd = beginReading();
    if (measEnd == 0) {
        ESP_LOGE(TAG, "Failed to perform reading!");
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
        ESP_LOGE(TAG, "Failed to get data from device!");
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

float Device::readAltitude(const float seaLevelPressure) {
    performReading();
    const float atmospheric = pressure / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevelPressure, 0.1903));
}

float Device::seaLevelForAltitude(const float altitude, const float atmospheric) {
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
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
    esp_err_t err = i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr, &regAddr,
                                                 1, data, len, pdMS_TO_TICKS(1000));
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
    esp_err_t err = i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr, buffer,
                                               len + 1, pdMS_TO_TICKS(1000));
    return (err == ESP_OK) ? BME68X_OK : -1;
}

} // namespace BME680

#include "bme680.hpp"
#include "bme68x.h"
#include "config.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "i2c.hpp"
#include "portmacro.h"
#include "utils.hpp"
#include <cmath>
#include <ctime>

namespace BME680 {

static const char *TAG = "BME680";

Device::Device(Environment &e, const i2c_port_t port, const std::uint8_t addr)
    : env(e), i2c_port(port), i2c_addr(addr), measStart(0), measDur(0),
      initialised(false), shutdown(false) {}

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
}

void bme680Task(void *pvParameters) {
    Device *bme = static_cast<Device *>(pvParameters);
    bme->start();
}

bool Device::init() {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
#endif
        return false;
    }
    TickType_t startInit = xTaskGetTickCount();
#if BME680_DEBUG
    const auto startTime = millis();
    ESP_LOGI(TAG, "Initialising BME680...");
    ESP_LOGI(TAG, "Starting gas sensor...");
#endif
    gas_sensor.chip_id = BME680_CHIP_ID;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = this;
    gas_sensor.read = &Device::read;
    gas_sensor.write = &Device::write;
    gas_sensor.amb_temp = 20;
    gas_sensor.delay_us = delay_us;
    std::int8_t res = bme68x_init(&gas_sensor);
    if (res != BME68X_OK) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to initialise gas sensor!");
#endif
        return false;
    }
#if BME680_DEBUG
    ESP_LOGI(TAG, "Configuring BME680...");
#endif
    res = bme68x_get_conf(&gas_conf, &gas_sensor);
    if (res != BME68X_OK) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to configure BME680!");
#endif
        return false;
    }
    if (!setIIRFilterSize(BME68X_FILTER_OFF)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set IIR filter!");
#endif
        return false;
    }
    if (!setODR(BME68X_ODR_NONE)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set ODR!");
#endif
        return false;
    }
    if (!setHumidityOversampling(BME68X_OS_2X)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set humidity oversampling!");
#endif
        return false;
    }
    if (!setPressureOversampling(BME68X_OS_4X)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set pressure oversampling!");
#endif
        return false;
    }
    if (!setTemperatureOversampling(BME68X_OS_2X)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set temperature oversampling!");
#endif
        return false;
    }
    if (!setGasHeater(320, 150)) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set gas heater config!");
#endif
        return false;
    }
    ESP_LOGI(TAG, "BME680 configured successfully!");
    initialised = true;
#if BME680_DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u, predicted: %u", totalTime, getInitTime());
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(bme680Task, "BME680Task", 4096, this, 5, &taskHandle);
    return true;
}

bool Device::getShutdown() {
    taskENTER_CRITICAL(&shutdownMux);
    const bool shouldStop = shutdown;
    taskEXIT_CRITICAL(&shutdownMux);
    return shouldStop;
}

void Device::start() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
#if BME680_DEBUG
        const auto sTime = millis();
#endif
        if (getShutdown()) {
#if BME680_DEBUG
            ESP_LOGI(TAG, "BME680 read-loop task stopping...");
#endif
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        if (!storeReading()) {
#if BME680_DEBUG
            ESP_LOGW(TAG, "Failed to take reading in read-loop!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BME680_HEATER_FREQ_MS));
            continue;
        }
#if BME680_DEBUG
        const auto eTime = millis();
        const auto totalTime = eTime - sTime;
        ESP_LOGI(TAG, "Total reading time: %u, predicted: %u", totalTime,
                 READING_DURATION_MS);
#endif
        vTaskDelayUntil(&last, pdMS_TO_TICKS(BME680_HEATER_FREQ_MS));
    }
}

bool Device::isInitialised() { return initialised; }

Reading Device::getReading() {
    taskENTER_CRITICAL(&readingMux);
    const Reading res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    return res;
}

void Device::logReadings(QueueHandle_t q) {
    const Reading res = getReading();
    if (res.read) {
#if BME680_DEBUG
        ESP_LOGW(TAG, "Reading has already been read...");
#endif
        return;
    }
    if (!res.valid) {
#if BME680_DEBUG
        ESP_LOGW(TAG, "Reading is invalid...");
#endif
        return;
    }
    sendCombinedMetrics(res.temperature, res.humidity, res.tval, q);
    const Event pEvent = {res.pressure, res.tval, EventType::PRES};
    const Event gEvent = {res.gasResistance, res.tval, EventType::GAS};
    xQueueSend(q, &pEvent, portMAX_DELAY);
    xQueueSend(q, &gEvent, portMAX_DELAY);
}

void Device::setShutdown() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
}

bool Device::sleep() {
    setShutdown();
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
#if BME680_DEBUG
    ESP_LOGI(TAG, "Got shutdown semaphore, sleeping...");
#endif
    const std::int8_t res = bme68x_set_op_mode(BME68X_SLEEP_MODE, &gas_sensor);
    if (res != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set operating mode while sleeping!");
        return false;
    }
    return true;
}

std::uint32_t Device::beginReading() {
    if (measStart != 0) {
#if BME680_DEBUG
        ESP_LOGI(TAG, "Reading already started!");
#endif
        return measStart + measDur;
    }
    const std::int8_t res = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
    if (res != BME68X_OK) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to set operating mode while starting reading!");
#endif
        return 0;
    }
    measStart = millis();
    const std::uint32_t readTime =
        bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor);
    const std::uint32_t heatTime =
        static_cast<std::uint32_t>(gas_heatr_conf.heatr_dur) * 1000;
    const std::uint32_t delay = readTime + heatTime;
    measDur = (delay + 999) / 1000; // Round up to be safe
#if BME680_DEBUG
    ESP_LOGI(TAG, "Read time: %u", readTime);
    ESP_LOGI(TAG, "Heat time: %u", heatTime);
    ESP_LOGI(TAG, "Delay: %u", delay);
    ESP_LOGI(TAG, "measDur: %u", measDur);
    ESP_LOGI(TAG, "measStart: %u", measStart);
#endif
    return measStart + measDur;
}

bool Device::storeReading() {
    const std::uint32_t measEnd = beginReading();
    if (measEnd == 0) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to perform reading!");
#endif
        return false;
    }
    const std::int32_t remaining = remainingReadingMillis();
    if (remaining > 0) {
        delay_ms(static_cast<std::uint32_t>(remaining) + 5);
    }
    measStart = 0;
    measDur = 0;
    bme68x_data data{};
    std::uint8_t n = 0;
    const std::int8_t res = bme68x_get_data(BME68X_FORCED_MODE, &data, &n, &gas_sensor);
    if (res != BME68X_OK || n == 0) {
#if BME680_DEBUG
        ESP_LOGE(TAG, "Failed to get data from device!");
#endif
        return false;
    }
    Reading r{};
    r.temperature = data.temperature + TEMP_ADJUST;
    r.humidity = data.humidity + HUM_ADJUST;
    r.pressure = data.pressure + PRES_ADJUST;
    if ((data.status & BME68X_HEAT_STAB_MSK) && (data.status & BME68X_GASM_VALID_MSK)) {
#if BME680_DEBUG
        ESP_LOGI(TAG, "Gas and Heat ready. Heat: %d, Gas: %d, Status: %u",
                 BME68X_HEAT_STAB_MSK, BME68X_GASM_VALID_MSK, data.status);
#endif
        r.gasResistance = data.gas_resistance;
        r.valid = true;
    } else {
#if BME680_DEBUG
        ESP_LOGW(TAG, "Gas and Heat not ready! Heat: %d, Gas: %d, Status: %u",
                 BME68X_HEAT_STAB_MSK, BME68X_GASM_VALID_MSK, data.status);
#endif
        r.gasResistance = 0.0f;
        r.valid = false;
    }
    r.tval = time(NULL);
    r.read = false;
    taskENTER_CRITICAL(&readingMux);
    reading = r;
    taskEXIT_CRITICAL(&readingMux);
    env.setTemperature(r.temperature, r.tval);
    env.setHumidity(r.humidity, r.tval);
    return true;
}

float Device::readAltitude(const float seaLevelPressure) {
    storeReading();
    const float atmospheric = reading.pressure / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevelPressure, 0.1903));
}

float Device::seaLevelForAltitude(const float altitude, const float atmospheric) {
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

std::int32_t Device::remainingReadingMillis(void) {
    if (measStart != 0) {
        std::uint32_t now = millis();
        if (now < measStart) {
#if BME680_DEBUG
            ESP_LOGW(TAG, "Now is less than measStart...");
#endif
            return NOT_STARTED;
        }
        std::uint32_t elapsed = now - measStart;
        if (elapsed > measDur) {
#if BME680_DEBUG
            ESP_LOGI(TAG, "No remaining millis - complete!");
#endif
            return COMPLETE;
        }
        const std::int32_t rem = static_cast<std::int32_t>(measDur - elapsed);
#if BME680_DEBUG
        ESP_LOGI(TAG, "Remaining: %d", rem);
#endif
        return rem;
    }
#if BME680_DEBUG
    ESP_LOGW(TAG, "Measurement not started!");
#endif
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
    const std::int8_t res =
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);
    return res == 0;
}

bool Device::setODR(std::uint8_t odr) {
    if (odr > BME68X_ODR_NONE) {
        return false;
    }
    gas_conf.odr = odr;
    const std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setTemperatureOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_temp = oversample;
    const std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setHumidityOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_hum = oversample;
    const std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setPressureOversampling(std::uint8_t oversample) {
    if (oversample > BME68X_OS_16X) {
        return false;
    }
    gas_conf.os_pres = oversample;
    const std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

bool Device::setIIRFilterSize(std::uint8_t filtersize) {
    if (filtersize > BME68X_FILTER_SIZE_127) {
        return false;
    }
    gas_conf.filter = filtersize;
    const std::int8_t res = bme68x_set_conf(&gas_conf, &gas_sensor);
    return res == 0;
}

BME68X_INTF_RET_TYPE Device::read(std::uint8_t regAddr, std::uint8_t *data,
                                  std::uint32_t len, void *interface) {
    if (data == nullptr || len == 0) {
        return -1;
    }
    Device *dev = static_cast<Device *>(interface);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write8read(regAddr, data, len, dev->i2c_port, dev->i2c_addr);
    xSemaphoreGive(i2cMutex);
    return (res) ? BME68X_OK : -1;
}

BME68X_INTF_RET_TYPE Device::write(std::uint8_t regAddr, const std::uint8_t *data,
                                   std::uint32_t len, void *interface) {
    if (data == nullptr || len == 0 || len > 32) {
        return -1;
    }
    std::uint8_t buffer[33];
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, len);
    Device *dev = static_cast<Device *>(interface);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = writeBytes(buffer, len + 1, dev->i2c_port, dev->i2c_addr);
    xSemaphoreGive(i2cMutex);
    return (res) ? BME68X_OK : -1;
}

} // namespace BME680

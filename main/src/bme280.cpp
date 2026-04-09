#include "bme280.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "events.hpp"
#include "freertos/idf_additions.h"
#include "i2c.hpp"
#include "portmacro.h"
#include "utils.hpp"
#include <cmath>
#include <cstring>
#include <ctime>

namespace BME280 {

static const char *TAG = "BME280";

Device::Device(const i2c_port_t port, const std::uint8_t addr)
    : i2c_port(port), i2c_addr(addr), initialised(false), shutdown(false) {}

Device::~Device() {
    if (shutdownAck) {
        vSemaphoreDelete(shutdownAck);
        shutdownAck = nullptr;
    }
}

void bme280Task(void *pvParameters) {
    Device *bme = static_cast<Device *>(pvParameters);
    bme->start();
}

bool Device::init() {
    shutdownAck = xSemaphoreCreateBinary();
    if (!shutdownAck) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
#endif
        return false;
    }
    TickType_t startInit = xTaskGetTickCount();
#if BME280_DEBUG
    const auto startTime = millis();
#endif
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const std::uint8_t id = write8read8(REG_ID, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (id != 0x60) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Wrong BME280 ID!");
#endif
        return false;
    }
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool res = write8WithArg(REG_RESET, 0xB6, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to set reset register!");
#endif
        return false;
    }
    delay_ms(10);
    while (isReadingCalibration()) {
        delay_ms(10);
    }
    readCalibration();
    if (!setMode(MODE_SLEEP)) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to set sleep mode!");
#endif
        return false;
    }
    if (!setSampling()) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to set sampling config!");
#endif
        return false;
    }
    delay_ms(100);
    ESP_LOGI(TAG, "BME280 initialised successfully!");
    initialised = true;
#if BME280_DEBUG
    const auto endTime = millis();
    const auto totalTime = endTime - startTime;
    ESP_LOGI(TAG, "Init took: %u, predicted: %u", totalTime, getInitTime());
#endif
    vTaskDelayUntil(&startInit, pdMS_TO_TICKS(getInitTime()));
    xTaskCreate(bme280Task, "BME280Task", 4096, this, 5, &taskHandle);
    return true;
}

void Device::start() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
#if BME280_DEBUG
        const auto sTime = millis();
#endif
        bool shouldStop;
        taskENTER_CRITICAL(&shutdownMux);
        shouldStop = shutdown;
        taskEXIT_CRITICAL(&shutdownMux);
        if (shouldStop) {
#if BME280_DEBUG
            ESP_LOGI(TAG, "BME280 read-loop task stopping...");
#endif
            xSemaphoreGive(shutdownAck);
            vTaskDelete(NULL);
            return;
        }
        if (!performReading()) {
#if BME280_DEBUG
            ESP_LOGW(TAG, "Failed to take reading in read-loop!");
#endif
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BME280_READ_FREQ_MS));
            continue;
        }
#if BME280_DEBUG
        const auto eTime = millis();
        const auto totalTime = eTime - sTime;
        ESP_LOGI(TAG, "Total reading time: %u, predicted: %u", totalTime,
                 READING_DURATION_MS);
#endif
        vTaskDelayUntil(&last, pdMS_TO_TICKS(BME280_READ_FREQ_MS));
    }
}

void Device::logReadings(QueueHandle_t q) {
    Reading res{};
    taskENTER_CRITICAL(&readingMux);
    res = reading;
    reading.read = true;
    taskEXIT_CRITICAL(&readingMux);
    if (res.read) {
#if BME280_DEBUG
        ESP_LOGW(TAG, "Reading has already been read...");
#endif
        return;
    }
    if (!res.valid) {
#if BME280_DEBUG
        ESP_LOGW(TAG, "Reading is invalid...");
#endif
        return;
    }
    const Event tEvent = {res.temperature + TEMP_ADJUST, res.tval, EventType::TEMP};
    const Event hEvent = {res.humidity + HUM_ADJUST, res.tval, EventType::HUM};
    const Event pEvent = {res.pressure + PRES_ADJUST, res.tval, EventType::PRES};
    xQueueSend(q, &tEvent, portMAX_DELAY);
    xQueueSend(q, &hEvent, portMAX_DELAY);
    xQueueSend(q, &pEvent, portMAX_DELAY);
}

bool Device::sleep() {
    taskENTER_CRITICAL(&shutdownMux);
    shutdown = true;
    taskEXIT_CRITICAL(&shutdownMux);
    if (taskHandle) {
        xTaskAbortDelay(taskHandle);
    }
    xSemaphoreTake(shutdownAck, portMAX_DELAY);
#if BME280_DEBUG
    ESP_LOGI(TAG, "Got shutdown semaphore, sleeping...");
#endif
    const bool res = setMode(MODE_SLEEP);
#if BME280_DEBUG
    if (!res) {
        ESP_LOGW(TAG, "Failed to sleep device by setting sleep mode...");
    }
#endif
    return res;
}

bool Device::isDataReady() const {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const std::uint8_t status = write8read8(REG_STATUS, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    const bool measuring = (status & STATUS_MEASURING_MASK) != 0;
    const bool imUpdating = (status & STATUS_IM_UPDATE_MASK) != 0;
    return !measuring && !imUpdating;
}

bool Device::performReading() {
    const bool result = setMode(MODE_FORCED);
    if (!result) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to set forced mode!");
#endif
        return false;
    }
    delay_ms(MEAS_DUR_MS);
    std::uint32_t tries = 0;
    while (!isDataReady()) {
        if (++tries >= 10) {
#if BME280_DEBUG
            ESP_LOGW(TAG, "Waited too long, exiting!");
#endif
            return false;
        }
#if BME280_DEBUG
        ESP_LOGW(TAG, "Data not ready, waiting 5ms...");
#endif
        delay_ms(5);
    }
    Reading res{};
    res.temperature = readTemperature();
    res.humidity = readHumidity();
    res.pressure = readPressure();
    res.tval = time(NULL);
    res.valid = true;
    taskENTER_CRITICAL(&readingMux);
    reading = res;
    taskEXIT_CRITICAL(&readingMux);
    return true;
}

bool Device::isReadingCalibration() const {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const std::uint8_t rStatus = write8read8(REG_STATUS, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    return (rStatus & (1 << 0)) != 0;
}

bool Device::isInitialised() { return initialised; }

void Device::readCalibration() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    calib.dig_T1 = static_cast<std::int32_t>(write8read16LE(0x88, i2c_port, i2c_addr));
    calib.dig_T2 = static_cast<std::int32_t>(write8readS16LE(0x8A, i2c_port, i2c_addr));
    calib.dig_T3 = static_cast<std::int32_t>(write8readS16LE(0x8C, i2c_port, i2c_addr));

    calib.dig_P1 = static_cast<std::int64_t>(write8read16LE(0x8E, i2c_port, i2c_addr));
    calib.dig_P2 = static_cast<std::int64_t>(write8readS16LE(0x90, i2c_port, i2c_addr));
    calib.dig_P3 = static_cast<std::int64_t>(write8readS16LE(0x92, i2c_port, i2c_addr));
    calib.dig_P4 = static_cast<std::int64_t>(write8readS16LE(0x94, i2c_port, i2c_addr));
    calib.dig_P5 = static_cast<std::int64_t>(write8readS16LE(0x96, i2c_port, i2c_addr));
    calib.dig_P6 = static_cast<std::int64_t>(write8readS16LE(0x98, i2c_port, i2c_addr));
    calib.dig_P7 = static_cast<std::int64_t>(write8readS16LE(0x9A, i2c_port, i2c_addr));
    calib.dig_P8 = static_cast<std::int64_t>(write8readS16LE(0x9C, i2c_port, i2c_addr));
    calib.dig_P9 = static_cast<std::int64_t>(write8readS16LE(0x9E, i2c_port, i2c_addr));

    calib.dig_H1 = static_cast<std::int32_t>(write8read8(0xA1, i2c_port, i2c_addr));
    calib.dig_H2 = static_cast<std::int32_t>(write8readS16LE(0xE1, i2c_port, i2c_addr));
    calib.dig_H3 = static_cast<std::int32_t>(write8read8(0xE3, i2c_port, i2c_addr));
    calib.dig_H4 =
        static_cast<std::int32_t>((write8read8(0xE4, i2c_port, i2c_addr) << 4) |
                                  (write8read8(0xE5, i2c_port, i2c_addr) & 0xF));
    calib.dig_H5 =
        static_cast<std::int32_t>((write8read8(0xE6, i2c_port, i2c_addr) << 4) |
                                  (write8read8(0xE5, i2c_port, i2c_addr) >> 4));
    calib.dig_H6 = static_cast<std::int32_t>(write8read8(0xE7, i2c_port, i2c_addr));
    xSemaphoreGive(i2cMutex);
}

bool Device::setMode(const std::uint32_t mode) const {
#if BME280_DEBUG
    ESP_LOGI(TAG, "Setting mode to %u", mode);
#endif
    const std::uint32_t measReg = (TEMP_OSRS << 5) | (PRES_OSRS << 2) | mode;
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    const bool res = write8WithArg(REG_CTRL, measReg, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    return res;
}

bool Device::setSampling() const {
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
    const std::uint32_t dur = 5;
    const std::uint32_t configReg = (dur << 5) | (filter << 2) | 0;
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool res = write8WithArg(REG_CTRL_HUM, HUM_OSRS, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to write humidity register!");
#endif
        return false;
    }
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    res = write8WithArg(REG_CONFIG, configReg, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    if (!res) {
#if BME280_DEBUG
        ESP_LOGE(TAG, "Failed to write configuration register!");
#endif
    }
    return res;
}

float Device::readTemperature() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    std::int32_t adc_T = write8read24(REG_TEMP_MSB, i2c_port, i2c_addr) >> 4;
    xSemaphoreGive(i2cMutex);
    std::int32_t var1 = (adc_T >> 3) - (calib.dig_T1 << 1);
    var1 = (var1 * calib.dig_T2) >> 11;
    std::int32_t var2 = (adc_T >> 4) - (calib.dig_T1);
    var2 = (((var2 * var2) >> 12) * calib.dig_T3) >> 14;
    t_fine = var1 + var2 + t_fine_adjust;
    const std::int32_t T = (t_fine * 5 + 128) >> 8;
    return static_cast<float>(T) / 100.0f;
}

float Device::readPressure() {
    readTemperature(); // must compute t_fine
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    std::int32_t adc_P = write8read24(REG_PRESS_MSB, i2c_port, i2c_addr) >> 4;
    xSemaphoreGive(i2cMutex);
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

float Device::readHumidity() {
    readTemperature(); // must compute t_fine
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    std::int32_t adc_H = write8read16(REG_HUM_MSB, i2c_port, i2c_addr);
    xSemaphoreGive(i2cMutex);
    std::int32_t var1 = t_fine - 76800;
    std::int32_t var2 = adc_H << 14;
    std::int32_t var3 = calib.dig_H4 << 20;
    std::int32_t var4 = calib.dig_H5 * var1;
    std::int32_t var5 = (var2 - var3 - var4 + 16384) >> 15;
    var2 = (var1 * calib.dig_H6) >> 10;
    var3 = (var1 * calib.dig_H3) >> 11;
    var4 = ((var2 * (var3 + 32768)) >> 10) + 2097152;
    var2 = ((var4 * calib.dig_H2) + 8192) >> 14;
    var3 = var5 * var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
    var5 = var3 - ((var4 * calib.dig_H1) >> 4);
    var5 = var5 < 0 ? 0 : var5;
    var5 = var5 > 419430400 ? 419430400 : var5;
    var5 = var5 >> 12;
    return static_cast<float>(var5) / 1024.0f;
}

void Device::setTemperatureCompensation(const float adjustment) {
    t_fine_adjust = (static_cast<std::int32_t>(adjustment * 100) << 8) / 5;
}

float Device::getTemperatureCompensation() const {
    return static_cast<float>((t_fine_adjust * 5) >> 8) / 100.0f;
}

float Device::readAltitude(const float seaLevelPressure) {
    const float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevelPressure, 0.1903));
}

float Device::seaLevelForAltitude(const float altitude, const float atmospheric) {
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

} // namespace BME280

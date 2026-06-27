#pragma once

#include "logger.hpp"
#include "mqtt.hpp"
#include "sensor.hpp"
#include "wifi.hpp"
#include <cstdint>

class MainContext {
  public:
    explicit MainContext(const std::uint32_t ml, ISensor **s, const std::size_t sc,
                         const QueueHandle_t eq, const WIFI &w, MQTT &m, Logger &l);

    ~MainContext() = default; // This class does not own these resources
    MainContext(const MainContext &) = delete;
    MainContext(const MainContext &&) = delete;
    MainContext operator=(const MainContext &) = delete;
    MainContext operator=(const MainContext &&) = delete;

    std::uint32_t getWarmupTime();
    void shutdownSensors();
    void takeReadings() const;
    const std::uint32_t mainLoopMS;
    void publishReadings(const std::uint32_t portDelay) const;

  private:
    ISensor **sensors;
    const std::size_t sensorCount;
    const QueueHandle_t eventQueue;
    const WIFI &wifi;
    MQTT &mqtt;
    Logger &logger;
};
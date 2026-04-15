# Environmental Sensor

An ESP32-based environmental monitoring system that measures and publishes readings to an MQTT broker.

## Features

- SHT45, BME280, BME680, SGP40/41, SCD41, and SDS011 support
- Temperature, humidity, pressure, CO2, VOC, NOX, PM2.5, and PM10 readings
- WiFi connectivity and MQTT publishing
- Modular sensor driver implementation with multiple configurations

The device periodically samples environmental data and sends it to an MQTT broker for logging or monitoring.

Typical wiring:

| BME280 | ESP32 |
|------|------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

## MQTT Payload

Sensor readings are published as JSON to a topic for each reading type.

Example payload:

```json
{
  "time": 1773430361,
  "val": 20.48
}
```

Example topic:

```
device/1/temperature
```

## Configuration

```
cp include/config.example.hpp include/config.hpp
```

## Build

Clone the repository:

```bash
git clone https://github.com/benmoses-dev/environmental-sensor.git
cd environmental-sensor
```

Build the firmware:

```bash
idf.py build
```

Flash to the ESP32:

```bash
idf.py -p PORT flash monitor
```

# Environmental Sensor

An ESP32-based environmental monitoring device that measures **temperature, humidity, and pressure** using a BME280 sensor and publishes readings to an MQTT broker over TLS.

## Features

- BME280 environmental sensor
- Temperature, humidity, and pressure readings
- WiFi connectivity
- MQTT publishing
- ESP‑IDF based firmware
- Modular sensor driver implementation

The device periodically samples environmental data and sends it to an MQTT broker for logging, monitoring, or home‑automation integration.

## Architecture

Sensor -> ESP32 -> MQTT (TLS) -> Broker

The ESP32 reads sensor data via I2C, formats the measurements and applies calibration, and publishes them to a configured MQTT topic.

## Hardware

Required components:

- ESP32 development board
- BME280 environmental sensor (I2C)
- Breadboard / wiring

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

WiFi and MQTT settings are configured through include/config.h

```
cp include/config.example.h include/config.h
```

TLS certificates can also be configured if required by the broker, but I currently use LetsEncrypt on the server (global CA).

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

## Use Cases

- Home automation sensors
- Environmental monitoring
- MQTT telemetry testing


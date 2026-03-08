# Project Roadmap

---

**Sensors:** BME280, BME680, SCD41, SDS011, BMP388
**Platform:** ESP32

### Stage 1 - Calibration
- Implement temperature compensation and drift correction
- Cross-sensor calibration
- Noise modelling for each sensor
- Test with real environmental variations

### Stage 2 - Sensor Fusion
- Combine multiple sensor readings to estimate hidden states
  - Air Quality Index
  - Occupancy estimation
  - Ventilation efficiency
- Implement probabilistic models
  - Bayesian filtering
  - Weighted sensor fusion
- Evaluate accuracy against real measurements

### Stage 3 - Edge Anomaly Detection
- Detect sensor anomalies and environmental events on-device
  - Pollution spikes
  - Ventilation failures
  - Sensor malfunction
- Lightweight ML or statistical models suitable for ESP32
- Log anomalies for later analysis

---


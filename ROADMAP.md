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

---


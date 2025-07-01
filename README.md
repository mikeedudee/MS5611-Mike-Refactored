[![Arduino CI](https://github.com/RobTillaart/MS5611/workflows/Arduino%20CI/badge.svg)](https://github.com/marketplace/actions/arduino_ci)
[![Arduino-lint](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml)
[![JSON check](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml)
[![GitHub issues](https://img.shields.io/github/issues/mikeedudee/MS5611-Mike-Refactored.svg)](https://github.com/mikeedudee/MS5611-Mike-Refactored/issues)

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/mikeedudee/MS5611-Mike-Refactored/blob/master/LICENSE)
[![GitHub tag (latest by date)](https://img.shields.io/github/v/tag/mikeedudee/MS5611-Mike-Refactored?label=latest)](https://github.com/mikeedudee/MS5611-Mike-Refactored/tags)

# MS5611 Mike Refactored

A refactored and enhanced Arduino library for the MS5611 pressure & temperature sensor, now featuring dynamic filtering, derivative estimation, spike detection, and performance benchmarking.

```cpp
//
//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull-up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77
//
```

## Description

This library extends the original MS5611 driver by Korneliusz Jarzebski with structural refactoring, optimizations, and a suite of new features:

* **Dynamic filtering & smoothing** via median and Kalman filters
* **True derivative estimation** for vertical velocity and acceleration using external timestamps
* **Spike detection** to flag or suppress sudden outliers
* **Performance benchmarking** with `performanceRead()` timing metrics
* **Manual-mode read APIs** for last pressure and temperature values
* **Reference-based altitude calculation** overload

Fully independent and maintained; please report any issues on the [issue tracker](https://github.com/mikeedudee/MS5611-Mike-Refactored/issues).

## Compatibility

* I2C interface only
* Default addresses: `0x76` or `0x77` (CSB/CSO pin)
* Tested on Teensy 4.1; should work with any Arduino-compatible board supporting `Wire`
* Compatible with MS56xx, MS57xx, MS58xx (some require reset & pressure check fix)

## Installation

1. Clone or download this repository
2. In Arduino IDE: **Sketch** → **Include Library** → **Add .ZIP Library…**

## Usage Examples

### Basic Example

```cpp
#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ms5611.begin()) {
    Serial.println("MS5611 not found!");
    while (1);
  }
  ms5611.setOversampling(HIGH_RES);
}

void loop() {
  ms5611.readSensor();
  Serial.print("Temp (°C): ");
  Serial.println(ms5611.readTemperature());
  Serial.print("Pres (Pa): ");
  Serial.println(ms5611.readPressure());
  delay(1000);
}
```

### Advanced Example (Filters, Dynamics & Spike Detection)

```cpp
#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;
float refPressure;
unsigned long lastTime;
float lastVelocity;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  ms5611.begin(ULTRA_HIGH_RES);

  // reference for relative altitude
  ms5611.readSensor();
  refPressure = ms5611.readPressure();

  // enable dynamic filters
  ms5611.enableMedianFilter(11);
  ms5611.enableKalmanFilter(0.5, 0.5, 0.138);

  // enable spike detection
  ms5611.spikeDetection(true);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  // batch read + timing
  auto perf = ms5611.performanceRead();
  Serial.print("Read time (ms): ");
  Serial.println(perf.elapsedMs);

  float alt = ms5611.getAltitude(perf.pressure, refPressure);
  float vel = ms5611.getVelocity(alt, now);
  float acc = ms5611.getAcceleration(vel, now);

  Serial.print("Alt (m): "); Serial.println(alt);
  Serial.print("Vel (m/s): "); Serial.println(vel);
  Serial.print("Acc (m/s²): "); Serial.println(acc);

  lastVelocity = vel;
  lastTime = now;
  delay(500);
}
```

## API Reference

### Initialization & Configuration

| Function                                  | Description                                          | Sample Usage
| ----------------------------------------- | ---------------------------------------------------- |------------
| `bool begin(Oversampling osr = HIGH_RES)` | Initialize sensor with optional oversampling setting |ms5611.begin()
| `void setOversampling(Oversampling osr)`  | Change oversampling on the fly                       |
| `Oversampling getOversampling()`          | Current oversampling setting                         |
| `void setPressureOffset(float offset)`    | Apply manual pressure calibration offset             |
| `void setTemperatureOffset(float offset)` | Apply manual temperature calibration offset          |

### Standard Reads

| Function                        | Description                             |
| ------------------------------- | --------------------------------------- |
| `readSensor()`             | Trigger a new temperature+pressure read |
| `readTemperature()`      | Last measured temperature (°C)          |
| `readPressure()`         | Last measured pressure (Pa)             |
| `readRawTemperature()` | Raw temperature ADC value               |
| `readRawPressure()`    | Raw pressure ADC value                  |

### Filters & Dynamics

| Function                                                        | Description                               | Sample Usage
| --------------------------------------------------------------- | ----------------------------------------- |-----------------
| `enableMedianFilter(uint32_t windowSize)`                  | Enable median smoothing                   |
| `medianFilter(double input)`                             | Apply median filter once                  |
| `enableKalmanFilter(double e_mea, double e_est, double q)` | Enable Kalman filter                      |
| `kalmanFilter(double input)`                             | Apply Kalman filter once                  |
| `resetDynamics()`                                          | Clear internal filter & derivative states |

### Derivative Estimation

| Function                                                          | Description                  |
| ----------------------------------------------------------------- | ---------------------------- |
| `getVelocity(double altitude, unsigned long timestamp)`     | Vertical velocity (m/s)      |
| `getAcceleration(double velocity, unsigned long timestamp)` | Vertical acceleration (m/s²) |

### Spike Detection & Utilities

| Function                                                  | Description                                                             |
| --------------------------------------------------------- | ----------------------------------------------------------------------- |
| `spikeDetection(bool enable)`                        | Toggle outlier detection/suppression. Automatically resets the MS5611 sensor once detected (1 second delay).                                    |
| `performanceRead()`                                  | Batch read with timing metrics (`elapsedMs`, `pressure`, `temperature`) |
| `getPressure()`                                     | Last raw pressure reading                                               |
| `getTemperature()`                                  | Last raw temperature reading                                            |
| `getAltitude(int32_t pressure, float refPressure)` | Altitude relative to reference (m)                                      |
| `getSeaLevel()`                                    | Compute sea-level pressure (Pa)                                         |

## Note

Always refer to the [MS5611 datasheet](https://cdn-shop.adafruit.com/datasheets/MS5611.pdf) for timing, calibration, and electrical specifications.

## Contributing

Contributions, issues, and feature requests are welcome — please open an issue or submit a pull request on GitHub.

---

**License**: MIT (see [LICENSE](./LICENSE))

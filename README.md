[![Arduino CI](https://github.com/RobTillaart/MS5611/workflows/Arduino%20CI/badge.svg)](https://github.com/marketplace/actions/arduino_ci)
[![Arduino-lint](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml)
[![JSON check](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml)
[![GitHub issues](https://img.shields.io/github/issues/RobTillaart/MS5611.svg)](https://github.com/RobTillaart/MS5611/issues)

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/RobTillaart/MS5611/blob/master/LICENSE)
[![GitHub release](https://img.shields.io/github/release/mikeedudee/MS5611-Mike-Refactored.svg?maxAge=3600)](https://github.com/mikeedudee/MS5611-Mike-Refactored/releases)



# MS5611 Mike Refactored

Refactored and Enhanced version of the MS5611 Pressure & Temperature Sensor Arduino Library.

### Breakout GY-63

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
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77
//
```

## Description
This library only implements the I2C interface. This refactored and enhanced library is originally from the library of Korneliusz Jarzebski. I just decided to do some restructuring, optimization, and refactoring while adding enhancements and some features and API calls, etc. Some functions and features come from or are inspired by Rob Tillaart MS5611 library, integrated into this library with a few tweaks.

The device default address is 0x76 or 0x77, depending on the CSB/CSO pin.


Please do note that this library is maintained and solely developed independently. Bugs or incompatibility may occur at your instance; if so, please report to me immediately.

Major key optimizations and changes in the source code are listed in the [CHANGELOG.md](./CHANGELOG.md); future enhancements are still ongoing development.

### Compatibility

The library should be compatible with MS56XX, MS57xx and MS58xx devices (to be tested). 
Some device types will return only 50% of the pressure value. 

### Oversampling

- **void setOversampling(osr samplingRate)** sets the amount of oversampling. 
See table below and test example how to use.
- **osr_t getOversampling()** returns amount of oversampling.


Some numbers from datasheet, page 3 MAX column rounded up. (see #23)
(actual read time differs - see performance sketch)

|        definition       | value | oversampling ratio | resolution (mbar) | time (us) | notes  |
|:-----------------------:|:-----:|:------------------:|:-----------------:|:---------:|:------:|
| MS5611_ULTRA_HIGH_RES   |  10   |        TBT         |        TBT        |    TBT    |
| MS5611_HIGH_RES         |  5    |        TBT         |        TBT        |    TBT    |
| MS5611_STANDARD         |  3    |        TBT         |        TBT        |    TBT    |
| MS5611_LOW_POWER        |  2    |        TBT         |        TBT        |    TBT    |
| MS5611_ULTRA_LOW_POWER  |  1    |        TBT         |        TBT        |    TBT    | 
- TBT = To Be Added
- Code Example:
  ```cpp
  MS5611.setOversampling(MS5611::Oversampling::MS5611_HIGH_RES);
  // or
  ms5611.begin(MS5611::Oversampling::MS5611_ULTRA_HIGH_RES))
  ```
`In future releases, you won't need to add the "MS5611::Oversampling::"`

## Installation

You can install this library via the Arduino Library Manager by importing it manually:

1. Download or clone this repository:
   ```bash
   git clone https://github.com/mikeedudee/MS5611-Mike-Refactored.git
   ```
   
---

### 2. **Example Usage**
Provide a working usage snippet and brief explanation:

## Basic Example

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

  ms5611.setOversampling(MS5611::Oversampling::MS5611_HIGH_RES);
}

void loop() {
  ms5611.readSensor();
  Serial.print("Temperature (°C): ");
  Serial.println(ms5611.readTemperature());
  Serial.print("Pressure (mbar): ");
  Serial.println(ms5611.readPressure());
  delay(1000);
}
```

## API Reference

| Function | Description |
|---------|-------------|
| `begin()` | Initializes the sensor |
| `readTemperature()` | Returns last measured temperature (°C) |
| `readPressure()` | Returns last measured pressure (Pa) |
| `readRawPressure()` | Returns raw reading of the sensor pressure |
| `readRawTemperature()` | Return raw reading of the sensor temperature |
| `setOversampling(mode)` | Sets oversampling (see table above) |
| `getOversampling()` | Returns current oversampling setting |
| `getAltitude()` | Returns computed altitude in meters |
| `getSeaLevel()` | Computes sea level pressure in Pascal |
| `getManufacturer()` | returns manufacturer private info | 
| `getSerialCode()` | returns serialCode from the PROM\[7] minus the CRC |
| `getOSRCode()` | Return the raw OSR command code (0x00,0x02,0x04,0x06,0x08) |
| `getConvTimeMs()` | Return the conversion time (ms) for the current OSR |

## Contributing

Contributions, issues, and feature requests are welcome.  
Feel free to check the [issues page](https://github.com/mikeedudee/MS5611-Mike-Refactored/issues).

This library is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

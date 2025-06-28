[![Arduino CI](https://github.com/RobTillaart/MS5611/workflows/Arduino%20CI/badge.svg)](https://github.com/marketplace/actions/arduino_ci)
[![Arduino-lint](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/arduino-lint.yml)
[![JSON check](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml/badge.svg)](https://github.com/RobTillaart/MS5611/actions/workflows/jsoncheck.yml)
[![GitHub issues](https://img.shields.io/github/issues/RobTillaart/MS5611.svg)](https://github.com/RobTillaart/MS5611/issues)

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/RobTillaart/MS5611/blob/master/LICENSE)
[![GitHub release](https://img.shields.io/github/release/mikeedudee/MS5611-Mike-Refactored.svg?maxAge=3600)](https://github.com/mikeedudee/MS5611-Mike-Refactored/releases)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/robtillaart/library/MS5611.svg)](https://registry.platformio.org/libraries/robtillaart/MS5611)



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
The device default address is 0x76 or 0x77, depending on the CSB/CSO pin.
This library only implements the I2C interface.

Please do note that this library is maintained and solely developed independently. Bugs or incompatibility may occur at your instance; if so, please report to me immediately.

Major key optimizations and changes in the source code are listed in the CHANGELOG.md; future enhancements are still ongoing development.

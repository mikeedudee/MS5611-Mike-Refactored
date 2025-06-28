/*

The MIT License

  Copyright (c) 2014–2023 Korneliusz Jarzębski
  Copyright (c) 2023–2025 Rob Tillaart
  Copyright (c) 2025–2026 Francis “Mike” J. Camogao [Refactor]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "MS5611.h"
#include <math.h>

// MS5611 class constructor
// Default address is 0x77, uses the default Wire instance
// Custom address can be set with the second constructor
// The mathMode can be set to either Datasheet or AppNote for different scaling methods
MS5611::MS5611() : wire_(&Wire)
    , address_(MS5611_ADDRESS)
    , osr_(Oversampling::MS5611_HIGH_RES)
    , mathMode_(MathMode::Datasheet)
    , pressureOffset_(0)
    , temperatureOffset_(0)
    , deviceID_(0)
{
    memset(cal_, 0, sizeof(cal_));
}

// MS5611 class constructor with custom I2C address and Wire instance
// Allows for flexibility in using different I2C addresses and Wire ports
MS5611::MS5611(uint8_t address, TwoWire &wirePort) : wire_(&wirePort)
    , address_(address)
    , osr_(Oversampling::MS5611_HIGH_RES)
    , mathMode_(MathMode::Datasheet)
    , pressureOffset_(0)
    , temperatureOffset_(0)
    , deviceID_(0)
{
    memset(cal_, 0, sizeof(cal_));
}

// Initializes the MS5611 sensor with the specified oversampling rate and math mode
// Resets the sensor, reads calibration data, and sets the oversampling rate and math mode
bool MS5611::begin(Oversampling osr, MathMode math) {
    osr_      = osr;
    mathMode_ = math;
    wire_->begin();

    resetSensor();

    if (!readCalibration()) return false;
    return true;
}

// Reads a 16-bit register from the MS5611 sensor
// Returns UINT16_MAX on failure, otherwise returns the 16-bit value
uint16_t MS5611::readRegister16(uint8_t reg) {
    wire_->beginTransmission(address_);
    wire_->write(reg);

    if (wire_->endTransmission() != 0) return UINT16_MAX;

    wire_->requestFrom(address_, (uint8_t)2);

    if (wire_->available() < 2) {
        return UINT16_MAX;
    }

    uint16_t msb = wire_->read();
    uint16_t lsb = wire_->read();

    return (msb << 8) | lsb;
}

uint32_t MS5611::readRegister24(uint8_t reg) {
    wire_->beginTransmission(address_);
    wire_->write(reg);

    // Check if the transmission was successful
    // If not, return UINT32_MAX to indicate an error
    if (wire_->endTransmission() != 0) {
    return UINT32_MAX;
    }

    // Request 3 bytes from the sensor
    // This is necessary because the MS5611 ADC value is 24 bits (3 bytes)
    // The requestFrom function will block until the bytes are available.
    wire_->requestFrom(address_, (uint8_t)3);

    // Check if we received 3 bytes
    // If not, return UINT32_MAX to indicate an error
    if (wire_->available() < 3) {
        return UINT32_MAX;
    }

    // Read the three bytes from the sensor
    // The MS5611 ADC value is transmitted as three separate bytes
    uint32_t b1 = wire_->read();
    uint32_t b2 = wire_->read();
    uint32_t b3 = wire_->read();

    // Combine the three bytes into a 24-bit value
    // MS5611 uses big-endian format, so we shift accordingly
    // b1 is the most significant byte, followed by b2 and b3
    // The result is a 24-bit value, which is returned as a 32-bit integer
    // The the upper 8 bits of the 32-bit return value that are zero. 
    // We’re filling bits 23–0 with the three ADC bytes; bits 31–24 end up zero.
    return (b1 << 16) | (b2 << 8) | b3;
}

// Resets the MS5611 sensor by sending the reset command
// This command clears the sensor's internal state and prepares it for a fresh start
void MS5611::resetSensor() {
    wire_->beginTransmission(address_);
    wire_->write(MS5611_RESET);
    wire_->endTransmission();
    delay(3);
}

// Reads the calibration data from the MS5611 sensor's PROM
// The calibration data is used to compensate the raw ADC readings
// Returns true if calibration data was successfully read, false otherwise
bool MS5611::readCalibration() {
    uint16_t prom[8];
    deviceID_ = 0;

    // Read the PROM coefficients from the sensor
    // The MS5611 has 8 coefficients stored in its PROM
    for (uint8_t i = 0; i < 8; ++i) {
        prom[i] = readRegister16(MS5611_READ_PROM + (i << 1));

        if (prom[i] == UINT16_MAX) {
            return false;
        }

        // Calculate the device ID by XORing the PROM coefficients
        // The device ID is a unique identifier for the sensor, derived from its calibration data
        // This is done by shifting the current deviceID_ left by 4 bits and XORing it with the current PROM coefficient
        // This effectively combines the coefficients into a single unique identifier
        deviceID_ = (deviceID_ << 4) ^ prom[i];
    }

    for (uint8_t i = 0; i < 6; ++i) {
        cal_[i] = prom[i + 1];
    }
    
    return true;
}

// Reads the raw temperature from the MS5611 sensor
// This function initiates a temperature conversion and waits for the result
uint32_t MS5611::readRawTemperature() {
    wire_->beginTransmission(address_);
    wire_->write(MS5611_CONVERT_D2 | uint8_t(osr_));
    if (wire_->endTransmission() != 0) {
        return UINT32_MAX;
    }

    uint32_t start = millis();
    uint8_t  idx   = uint8_t(osr_) >> 1;

    while (millis() - start < MS5611_CONVERSION_TIMEOUT_MS[idx]) {}

    return readRegister24(MS5611_READ_ADC);
}

// Reads the raw pressure from the MS5611 sensor
// This function initiates a pressure conversion and waits for the result
uint32_t MS5611::readRawPressure() {
    wire_->beginTransmission(address_);
    wire_->write(MS5611_CONVERT_D1 | uint8_t(osr_));

    if (wire_->endTransmission() != 0) { 
        return UINT32_MAX;
    }

    uint32_t start = millis();
    uint8_t  idx   = uint8_t(osr_) >> 1;

    while (millis() - start < MS5611_CONVERSION_TIMEOUT_MS[idx]) {}

    return readRegister24(MS5611_READ_ADC);
}

// Reads the pressure from the MS5611 sensor
// This function reads the raw pressure and temperature values, then compensates them using the calibration data
// If `comp` is true, it applies second-order compensation for temperature effects
// Returns the compensated pressure in millibars (mbar) or INT32_MIN on failure
int32_t MS5611::readPressure(bool comp) const {
    uint32_t D1 = const_cast<MS5611*>(this)->readRawPressure();
    uint32_t D2 = const_cast<MS5611*>(this)->readRawTemperature();

    if (D1 == UINT32_MAX || D2 == UINT32_MAX) { 
        return INT32_MIN;
    }

    // Calculate the pressure using the calibration coefficients and raw ADC values
    // The formula is derived from the MS5611 datasheet and includes temperature compensation
    // D1 is the raw pressure value, D2 is the raw temperature value
    // cal_ contains the calibration coefficients read from the sensor's PROM
    // The calculations involve several steps:
    // 1. Calculate dT, the difference between the raw temperature and the calibration value
    // 2. Calculate OFF, the offset for pressure calculation
    // 3. Calculate SENS, the sensitivity for pressure calculation
    // 4. Calculate TEMP, the compensated temperature in hundredths of degrees Celsius
    // 5. If compensation is requested and the temperature is below 2000°C,
    //    apply second-order compensation to OFF and SENS
    // 6. Finally, calculate the pressure P in millibars (mbar)
    // The final pressure is adjusted by a user-defined offset (pressureOffset_)
    // The result is returned as an int32_t value representing the pressure in mbar

    int32_t  dT   = int32_t(D2) - (int32_t(cal_[4]) << 8);
    int64_t  OFF  = (int64_t(cal_[1]) << 16) + ((int64_t)cal_[3] * dT >> 7);
    int64_t  SENS = (int64_t(cal_[0]) << 15) + ((int64_t)cal_[2] * dT >> 8);
    int32_t  TEMP = 2000 + (int64_t(dT) * cal_[5] >> 23);

    if (comp && TEMP < 2000) {
        int64_t d2      = int64_t(TEMP - 2000);
        int64_t off2    = (5 * d2 * d2) >> 1;
        int64_t sens2   = (5 * d2 * d2) >> 2;

        if (TEMP < -1500) {
        int64_t d22 = int64_t(TEMP + 1500);
        off2  += 7 * d22 * d22;
        sens2 += (11 * d22 * d22) >> 1;
        }

        OFF  -= off2;
        SENS -= sens2;
  }
    int32_t P = int32_t(((D1 * SENS >> 21) - OFF) >> 15);
    return P + pressureOffset_;
}

// Reads the temperature from the MS5611 sensor
// This function reads the raw temperature value and compensates it using the calibration data
// If `comp` is true, it applies second-order compensation for temperature effects
// Returns the compensated temperature in degrees Celsius or NAN on failure
// The temperature is adjusted by a user-defined offset (temperatureOffset_)
// The result is returned as a double value representing the temperature in degrees Celsius
double MS5611::readTemperature(bool comp) const {
  uint32_t D2 = const_cast<MS5611*>(this)->readRawTemperature();
  if (D2 == UINT32_MAX) return NAN;

  int32_t dT   = int32_t(D2) - (int32_t(cal_[4]) << 8);
  int32_t TEMP = 2000 + (int64_t(dT) * cal_[5] >> 23);

  if (comp && TEMP < 2000) {
    int64_t t2 = (int64_t(dT) * dT) >> 31;
    TEMP     -= int32_t(t2);
  }
  return TEMP * 0.01 + temperatureOffset_;
}

// Calculates the altitude based on the pressure and sea level pressure
// Uses the barometric formula to compute altitude in meters
// The formula is derived from the International Standard Atmosphere (ISA) model
double MS5611::getAltitude(double p, double slp) {
  return 44330.0 * (1.0 - pow(p / slp, 0.1902949));
}

// Calculates the sea level pressure based on the current pressure and altitude
// Uses the barometric formula to compute sea level pressure in Pascals
double MS5611::getSeaLevel(double p, double alt) {
  return p / pow(1.0 - (alt / 44330.0), 5.255);
}

// Returns the device ID, which is a unique identifier for the MS5611 sensor
// The device ID is calculated as the XOR of the PROM coefficients
uint16_t MS5611::getManufacturer() const {
  return readRegister16(MS5611_READ_PROM + (0 << 1));
}

// Returns the serial code, which is a unique identifier for the MS5611 sensor
// The serial code is derived from the last PROM word, which contains the serial number
// The serial code is extracted by reading the last PROM word and shifting it right by 4
uint16_t MS5611::getSerialCode() const {
  return (readRegister16(MS5611_READ_PROM + (7 << 1)) >> 4) & 0x0FFF;
}

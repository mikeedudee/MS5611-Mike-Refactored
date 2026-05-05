/*

The MIT License

    Copyright (c) 2014–2023 Korneliusz Jarzębski
    Copyright (c) 2025 Francis Mike John Camogao [Refactor/Enhancements]
    Copyright (c) 2026 Francis Mike John Camogao [Added New Features, Improvements and Bug Fixes]

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



/*

___  ___ _____ _____  ____  __   __   ___  ____ _         ______      __           _                     _ 
|  \/  |/  ___|  ___|/ ___|/  | /  |  |  \/  (_) |        | ___ \    / _|         | |                   | |
| .  . |\ `--.|___ \/ /___ `| | `| |  | .  . |_| | _____  | |_/ /___| |_ __ _  ___| |_ ___  _ __ ___  __| |
| |\/| | `--. \   \ \ ___ \ | |  | |  | |\/| | | |/ / _ \ |    // _ \  _/ _` |/ __| __/ _ \| '__/ _ \/ _` |
| |  | |/\__/ /\__/ / \_/ |_| |__| |_ | |  | | |   <  __/ | |\ \  __/ || (_| | (__| || (_) | | |  __/ (_| |
\_|  |_/\____/\____/\_____/\___/\___/ \_|  |_/_|_|\_\___| \_| \_\___|_| \__,_|\___|\__\___/|_|  \___|\__,_|
                              LIBRARY VERSION: 1.2.5_exp_build_04126026                   

*/

#include "MS5611.h"
#include <math.h>
#include <Arduino.h>

// MS5611 class constructor
// Default address is 0x77, uses the default Wire instance
// Custom address can be set with the second constructor
// The mathMode can be set to either Datasheet or AppNote for different scaling methods
MS5611::MS5611() : wire_(&Wire)
    , spi_(nullptr)
    , csPin_(-1)
    , useSPI_(false)
    , address_(MS5611_ADDRESS)
    , osr_(HIGH_RES)
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
    , spi_(nullptr)
    , csPin_(-1)
    , useSPI_(false)
    , address_(address)
    , osr_(HIGH_RES)
    , mathMode_(MathMode::Datasheet)
    , pressureOffset_(0)
    , temperatureOffset_(0)
    , deviceID_(0)
{
    memset(cal_, 0, sizeof(cal_));
}

// NEW: Custom SPI Constructor for SPI communication 1.2.0_exp_build_02122026
MS5611::MS5611(int8_t csPin, SPIClass &spiPort) : wire_(nullptr)
    , spi_(&spiPort)
    , csPin_(csPin)
    , useSPI_(true)
    , address_(0) // Address not used in SPI
    , osr_(HIGH_RES)
    , mathMode_(MathMode::Datasheet)
    , pressureOffset_(0)
    , temperatureOffset_(0)
    , deviceID_(0)
{
    memset(cal_, 0, sizeof(cal_));
}

/// MODIFIED FOR 1.2.0_exp_build_02122026
// Initializes the MS5611 sensor with the specified oversampling rate and math mode
// Resets the sensor, reads calibration data, and sets the oversampling rate and math mode
bool MS5611::begin(Oversampling osr, MathMode math, int8_t sdaPin, int8_t sclPin) {
    osr_      = osr;
    mathMode_ = math;

    if (useSPI_) {
        // Initialize SPI Hardware
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH); // Deselect MS5611
        spi_->begin();
    } else {
        // Initialize I2C Hardware
        if (sdaPin >= 0 && sclPin >= 0) {
            recoverI2C(sdaPin, sclPin);
        }
        wire_->begin();
    }

    resetSensor();
    if (!readCalibration()) {
        _healthStatus |= STATUS_INIT_FAILED;
        return false;
    }
    return true;
}

void MS5611::recoverI2C(int8_t sdaPin, int8_t sclPin) {
    // Set the pin to GPIO mode (dsable hardware I2c temporarily)
    // Set the SDA to Input (High-Z) to let the line float up
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, OUTPUT);

        // Check if SDA is actually stuck LOW
        // If SDA is HIGH, the bus is likely free, but still cycle anyway to be safe
        // or we can bail out early. Fore safety, we just cycle.

    // Toggle SCL 9 times to free up the bus
    // This forces th slave to clock out any data bit it might be holding LOW 
    for (int i = 0; i < 9; ++i) {
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(10); // Short delay to ensure the line is held high
        digitalWrite(sclPin, LOW);
        delayMicroseconds(10); // Short delay to ensure the line is held low
    }    

    // Send a STOP condition manually
    // SDA starts LOW, SCL goes HIGH, then SDA goes HIGH
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(10);

    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(sdaPin, HIGH); // STOP
    delayMicroseconds(10);

        // Pins will be reconfigured by wire_->begin() immediately after this
}

// Start of 1.2.0_exp_build_02122026: SPI support and asynchronous API additions
// Reads a 16-bit register from the MS5611 sensor
// Returns UINT16_MAX on failure, otherwise returns the 16-bit value
uint16_t MS5611::readRegister16(uint8_t reg) const {
    if (useSPI_) {
        spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin_, LOW);
        spi_->transfer(reg);
        uint16_t msb = spi_->transfer(0x00);
        uint16_t lsb = spi_->transfer(0x00);
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
        return (msb << 8) | lsb;
    } else {
        wire_->beginTransmission(address_);
        wire_->write(reg);
        if (wire_->endTransmission() != 0) {
            const_cast<MS5611*>(this)->_healthStatus |= STATUS_BUS_ERROR;
            return UINT16_MAX;
        }
        wire_->requestFrom(address_, (uint8_t)2);
        if (wire_->available() < 2) {
            const_cast<MS5611*>(this)->_healthStatus |= STATUS_BUS_ERROR;
            return UINT16_MAX;
        }
        uint16_t msb = wire_->read();
        uint16_t lsb = wire_->read();
        return (msb << 8) | lsb;
    }
}

uint32_t MS5611::readRegister24(uint8_t reg) const {
    if (useSPI_) {
        spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin_, LOW);
        spi_->transfer(reg);
        uint32_t b1 = spi_->transfer(0x00);
        uint32_t b2 = spi_->transfer(0x00);
        uint32_t b3 = spi_->transfer(0x00);
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
        return (b1 << 16) | (b2 << 8) | b3;
    } else {
        wire_->beginTransmission(address_);
        wire_->write(reg);

        // Check if the transmission was successful
        // If not, return UINT32_MAX to indicate an error
        if (wire_->endTransmission() != 0) {
            const_cast<MS5611*>(this)->_healthStatus |= STATUS_BUS_ERROR;
            return UINT32_MAX;
        }

        // Request 3 bytes from the sensor
        // This is necessary because the MS5611 ADC value is 24 bits (3 bytes)
        // The requestFrom function will block until the bytes are available.
        wire_->requestFrom(address_, (uint8_t)3);

        // Check if we received 3 bytes
        // If not, return UINT32_MAX to indicate an error
        if (wire_->available() < 3) {
            const_cast<MS5611*>(this)->_healthStatus |= STATUS_BUS_ERROR;
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
    
}

/// ADDED FOR 1.2.0_exp_build_02122026
bool MS5611::validatePhysics(double temperature, float pressure) {
    // PHYSICAL LIMIT CHECKS AND TRIGGERS
    bool tempOk = (temperature >= MIN_TEMP_C && temperature <= MAX_TEMP_C);
    bool presOk = (pressure >= MIN_PRESSURE_MBAR && pressure <= MAX_PRESSURE_MBAR);

    // APPROACHING LIMIT CHECKS AND TRIGGERS
    bool approachingLimits_TEMPERATURE  = (temperature >= (-30.0f) && temperature <= (75.0f));
    bool approachingLimits_PRESSURE     = (pressure >= (100.0f) && pressure <= (1100.0f));

    if (!tempOk || !presOk) {
        _healthStatus |= STATUS_PHYSICS_VIOLATION; // Flag the error
        return false;
    }

    if (!approachingLimits_TEMPERATURE || !approachingLimits_PRESSURE) {
        _healthStatus |= STATUS_APPROACHING_LIMITS; // Flag the warning
        return false;
    }

    return true;
}
/// END OF 1.2.0_exp_build_02122026

// Resets the MS5611 sensor by sending the reset command
// This command clears the sensor's internal state and prepares it for a fresh start
void MS5611::resetSensor() { // UPDATED FOR 1.2.0_exp_build_02122026: Added SPI support
    if (useSPI_) {
        spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz
        digitalWrite(csPin_, LOW);
        spi_->transfer(MS5611_RESET);
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
    } else {
        wire_->beginTransmission(address_);
        wire_->write(MS5611_RESET);
        wire_->endTransmission();
    }
    ++_resetCount;
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

/// ADDED FOR 1.2.0_exp_build_02122026 
// Asynchronous (Non-blocking) API implementation
// SPI support
bool MS5611::startTemperature() {
    if (useSPI_) {
        spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin_, LOW);
        spi_->transfer(MS5611_CONVERT_D2 | uint8_t(osr_));
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
    } else {
        wire_->beginTransmission(address_);
        wire_->write(MS5611_CONVERT_D2 | uint8_t(osr_));
        if (wire_->endTransmission() != 0) return false;
    }

    _conversionStartTime = millis();
    _isConverting = true;
    _conversionTimeout = MS5611_CONVERSION_TIMEOUT_MS[uint8_t(osr_) >> 1];
    return true;
}

// Start a pressure conversion (D1)
// Updated for 1.2.0_exp_build_02122026: Added SPI support and non-blocking logic
bool MS5611::startPressure() {
    if (useSPI_) {
        spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin_, LOW);
        spi_->transfer(MS5611_CONVERT_D1 | uint8_t(osr_));
        digitalWrite(csPin_, HIGH);
        spi_->endTransaction();
    } else {
        wire_->beginTransmission(address_);
        wire_->write(MS5611_CONVERT_D1 | uint8_t(osr_));
        if (wire_->endTransmission() != 0) return false;
    }

    _conversionStartTime = millis();
    _isConverting = true;
    _conversionTimeout = MS5611_CONVERSION_TIMEOUT_MS[uint8_t(osr_) >> 1];
    return true;
}

// Check if the current conversion is complete (based on timeout)
bool MS5611::isConversionComplete() const {
    if (!_isConverting) {
        return false; // No conversion started
    }
    return (millis() - _conversionStartTime) >= _conversionTimeout;
}

// Retrieve the results of the latest conversion (either D1 or D2)
uint32_t MS5611::getConversionValue() {
    _isConverting = false;
    return readRegister24(MS5611_READ_ADC);
}

// Reads the raw temperature from the MS5611 sensor
// This function initiates a temperature conversion and waits for the result
uint32_t MS5611::readRawTemperature() {
    if (!startTemperature()) {
        return UINT32_MAX;
    }

    // Blocking wait using the async logic
    while (!isConversionComplete()) {
        yield(); // Allows other microcontrollers tasks to run background while waiting
    }

    return getConversionValue();
}

// Reads the raw pressure from the MS5611 sensor
// This function initiates a pressure conversion and waits for the result
uint32_t MS5611::readRawPressure() {
    if (!startPressure()) {
        return UINT32_MAX;
    }

    while (!isConversionComplete()) {
        yield();
    }

    return getConversionValue();
}
/// END OF 1.2.0_exp_build_02122026 

/// MODIFIED FOR 1.2.0_exp_build_02122026
// Reads the pressure from the MS5611 sensor
// This function reads the raw pressure and temperature values, then compensates them using the calibration data
// If `comp` is true, it applies second-order compensation for temperature effects
// Returns the compensated pressure in millibars (mbar) or INT32_MIN on failure
float MS5611::readPressure(bool comp) const {
    // 1. Get Raw Data
    uint32_t D1 = const_cast<MS5611*>(this)->readRawPressure();
    uint32_t D2 = const_cast<MS5611*>(this)->readRawTemperature();

    if (D1 == UINT32_MAX || D2 == UINT32_MAX) { 
        // _healthStatus is typically set by the low-level read, 
        // but we can enforce an error here if needed.
        return NAN;
    }

    // 2. Perform Calculation (Existing Math)
    int32_t  dT   = int32_t(D2) - (int32_t(cal_[4]) << 8);
    int64_t  OFF  = (int64_t(cal_[1]) << 16) + ((int64_t)cal_[3] * dT >> 7);
    int64_t  SENS = (int64_t(cal_[0]) << 15) + ((int64_t)cal_[2] * dT >> 8);
    int32_t  TEMP = 2000 + (int64_t(dT) * cal_[5] >> 23);

    if (comp && TEMP < 2000) {
        int64_t d2    = int64_t(TEMP - 2000);
        int64_t off2  = (5 * d2 * d2) >> 1;
        int64_t sens2 = (5 * d2 * d2) >> 2;

        if (TEMP < -1500) {
            int64_t d22   = int64_t(TEMP + 1500);
                    off2  += 7 * d22 * d22;
                    sens2 += (11 * d22 * d22) >> 1;
        }
        OFF  -= off2;
        SENS -= sens2;
    }

    int32_t P = int32_t(((D1 * SENS >> 21) - OFF) >> 15);
        float   pressure_final  = P + float(pressureOffset_);
        
        // Correctly scale to mbar BEFORE validation
        float   pressure_mbar   = pressure_final / 100.0f; // optional setting and only for validation
        
        // --- CORRECTED VALIDATION STEP ---
        if (pressure_mbar < MIN_PRESSURE_MBAR || pressure_mbar > MAX_PRESSURE_MBAR) {
            const_cast<MS5611*>(this)->_healthStatus = STATUS_PHYSICS_VIOLATION;
            return NAN; 
        }

        return pressure_final;
    }// END OF 1.2.0_exp_build_02122026

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
    int64_t t2    = (int64_t(dT) * dT) >> 31;
            TEMP  -= int32_t(t2);
  }

  double finalTemp = TEMP * 0.01 + temperatureOffset_;

  // --- NEW VALIDATION STEP ---
  if (finalTemp < MIN_TEMP_C || finalTemp > MAX_TEMP_C) {
      const_cast<MS5611*>(this)->_healthStatus = STATUS_PHYSICS_VIOLATION;
      return NAN;
  }

  // Flag the approach to the limits
  if (finalTemp < -30.0 || finalTemp > 75.0) {
      const_cast<MS5611*>(this)->_healthStatus = STATUS_PHYSICS_VIOLATION; 
  }

  return finalTemp;
}
/// END OF 1.2.0_exp_build_02122026

MS5611::Measure MS5611::performanceRead(bool comp)
{
  // grab raw ADC values
  uint32_t D1 = readRawPressure();
  uint32_t D2 = readRawTemperature();

  if (D1 == UINT32_MAX || D2 == UINT32_MAX) {
    // I2C error or timeout: return NAN / INT32_MIN as sentinel
    return { NAN, (float)INT32_MIN };
  }

  // compute dT and first-order offsets/sensitivity
  int32_t  dT   = int32_t(D2) - (int32_t(cal_[4]) << 8);
  int64_t  OFF  = (int64_t(cal_[1]) << 16) + ((int64_t)cal_[3] * dT >> 7);
  int64_t  SENS = (int64_t(cal_[0]) << 15) + ((int64_t)cal_[2] * dT >> 8);

  // calculate “raw” temperature (in 0.01°C)
  int32_t  TEMP = 2000 + (int64_t(dT) * cal_[5] >> 23);

  // optional 2nd-order compensation below 20°C
  if (comp && TEMP < 2000) {
    int64_t d2    = int64_t(TEMP - 2000);
    int64_t off2  = (5 * d2 * d2) >> 1;
    int64_t sens2 = (5 * d2 * d2) >> 2;

    if (TEMP < -1500) {
      int64_t d22     = int64_t(TEMP + 1500);
              off2    += 7 * d22 * d22;
              sens2   += (11 * d22 * d22) >> 1;
    }

    OFF  -= off2;
    SENS -= sens2;
    TEMP -= int32_t((int64_t(dT) * dT) >> 31);  // 2nd-order temp correction
  }

  // compute final pressure (mbar) and temperature (°C)
  int32_t P = int32_t(((D1 * SENS >> 21) - OFF) >> 15);

  Measure m;

  m.pressure    = float(P + pressureOffset_);
  m.temperature = double(TEMP) * 0.01 + temperatureOffset_;

  return m;
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


/// SPIKE DETECTION API ///
void MS5611::spikeDetection(bool enable,
                            uint8_t ringSize,
                            float   threshold,
                            float   temperature,
                            float   pressure,
                            uint8_t consecutiveCount)
{
    // On real enable-transition or window-size change: seed buffers & reset counters
    if ( enable && (! _spikeWasEnabled || (ringSize != _spikeRingSize)) ) {
        // clamp window
        _spikeRingSize  = constrain(ringSize, 1, SPIKE_MAX_RING);

        // update threshold & count
        if (threshold > 0.0f) _spikeThreshold = threshold * 10.0f;
        _spikeConsecNeed = max<uint8_t>(1, consecutiveCount);

        // fetch first real sample
        float p0 = isnan(pressure)    ? readPressure()    : pressure;
        float t0 = isnan(temperature) ? readTemperature() : temperature;

        // seed circular buffers
        for (uint8_t i = 0; i < _spikeRingSize; ++i) {
            _spikeBufP[i] = p0;
            _spikeBufT[i] = t0;
        }
        _spikeIdx         = 0;
        _spikeCount       = 0;
    }

    _spikeWasEnabled = enable;    // remember for next call

    // If disabled, bail out
    if (!enable) return;

    // Read or use overrides
    float p = isnan(pressure)    ? readPressure()    : pressure;
    float t = isnan(temperature) ? readTemperature() : temperature;

    // Update buffer & compute rolling mean
    _spikeBufP[_spikeIdx] = p;
    _spikeBufT[_spikeIdx] = t;
    _spikeIdx             = (_spikeIdx + 1) % _spikeRingSize;

    float sumP = 0, sumT = 0;

    for (uint8_t i = 0; i < _spikeRingSize; ++i) {
        sumP += _spikeBufP[i];
        sumT += _spikeBufT[i];
    }

    float avgP = sumP / _spikeRingSize;
    float avgT = sumT / _spikeRingSize;

    // Detect & count consecutive spikes
    bool isSpike = (fabs(p - avgP) > _spikeThreshold) || (fabs(t - avgT) > _spikeThreshold);

    if (isSpike) {
        _healthStatus |= STATUS_SPIKE_DETECTED; // Flag the anomaly
        incrementSpikeCounter();

        if (getSpikeCounter() >= _spikeConsecNeed) {
            Serial.println(F("MS5611 Spike Detected!"));
            Serial.println("Resetting sensor...");
            //delay(1000);  // give time for the user to see the message
            resetSensor();
            resetDynamics();
            resetSpikeCounter();
        }
    } else {
        resetSpikeCounter();
    }
}

void MS5611::incrementSpikeCounter() {
    // Power-of-Ten rule: tiny, self-contained, no hidden side-effects
    ++_spikeCount;
}

int MS5611::getSpikeCounter() const {
    return _spikeCount;
}

void MS5611::resetSpikeCounter() {
    _spikeCount = 0;
}

//// THIS SECTION IS EXPERIMENTAL AND MAY CHANGE IN FUTURE VERSIONS ////

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

uint16_t MS5611::readProm(uint8_t reg) {
  //  last EEPROM register is CRC - Page 13 datasheet.
  uint8_t promCRCRegister = 7;
  if (reg > promCRCRegister) return 0;

  uint8_t offset = reg * 2;

  // Check the return of the command() directly
  if (command(MS5611_READ_PROM + offset) == 0) {
    uint8_t length  = 2;
    int     bytes   = wire_->requestFrom(address_, length);

    if (bytes >= length) {
      uint16_t  value = wire_->read() * 256;
                value += wire_->read();

      return value;
    }
  }

    _healthStatus |= STATUS_BUS_ERROR; // Flag the error
    return 0;
}

uint32_t MS5611::readADC() {
  if (command(MS5611_READ_ADC) == 0) {
    uint8_t length  = 3;
    int     bytes   = wire_->requestFrom(address_, length);

    if (bytes >= length) {
      uint32_t  value = wire_->read() * 65536UL;
                value += wire_->read() * 256UL;
                value += wire_->read();

      return value;
    }
  }
  
    _healthStatus |= STATUS_BUS_ERROR; // Flag the error
    return 0UL;
}

int MS5611::command(const uint8_t command) {
  yield();
  wire_->beginTransmission(address_);
  wire_->write(command);
  
  int err = wire_->endTransmission();
  if (err != 0) {
    _healthStatus |= STATUS_BUS_ERROR; 
  }

  return err;
}

uint16_t MS5611::getProm(uint8_t index) {
  return readProm(index);
}

uint16_t MS5611::getCRC() {
  return readProm(7) & 0x0F;
}
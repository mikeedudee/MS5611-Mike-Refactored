/*

The MIT License

Copyright (c) 2014-2023 Korneliusz Jarzębski
Copyright (c) 2025 Francis Mike John Camogao [Refactor]

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

#pragma once // Ensures this header file is included only once
// Project name: MS5611 Sensor Library

#include <cstdint>
#include <Arduino.h>
#include <Wire.h>

class MS5611 { 
    public:
        // Enum for oversampling rates; these control the resolution and power consumption
        // Higher values yield better resolution but consume more power and take longer to read
        enum class Oversampling : uint8_t {
            MS5611_ULTRA_HIGH_RES   = 0x08,
            MS5611_HIGH_RES         = 0x06,
            MS5611_STANDARD         = 0x04,
            MS5611_LOW_POWER        = 0x02,
            MS5611_ULTRA_LOW_POWER  = 0x00
        };

        /// Scaling math modes
        enum class MathMode : uint8_t {
            Datasheet = 0,
            AppNote   = 1
        };

        // Default: Wire @ 0x77
        MS5611();
        // Custom I²C address / bus
        MS5611(uint8_t address, TwoWire &wirePort = Wire);

        // Reset, read PROM (+CRC), choose OSR & math mode
        bool begin(Oversampling osr   = Oversampling::MS5611_HIGH_RES,
                   MathMode     math  = MathMode::Datasheet);

        // Raw ADC reads (blocking)
        uint32_t readRawTemperature();
        uint32_t readRawPressure();

        // Calibrated values
        double  readTemperature(bool compensate = false) const;
        int32_t readPressure   (bool compensate = false) const;

        // Helpers
        static double getAltitude(double pressure, double seaLevelPressure = 101325.0);
        static double getSeaLevel(double pressure, double altitude);

        // Offsets & IDs
        void     setPressureOffset   (int32_t mbar) { pressureOffset_    = mbar; }
        void     setTemperatureOffset(double degC ) { temperatureOffset_ = degC; }
        uint32_t getDeviceID         () const       { return deviceID_; }
        uint16_t getManufacturer     () const;
        uint16_t getSerialCode       () const;

         // Oversampling introspection
        void         setOversampling(Oversampling osr) { osr_ = osr; }
        Oversampling getOversampling()        const    { return osr_; }
        uint8_t      getOSRCode()             const    { return uint8_t(osr_); }
        uint16_t     getConvTimeMs()          const    {
            return MS5611_CONVERSION_TIMEOUT_MS[uint8_t(osr_) >> 1];
        }



    private:
        // I2C command and address constants
        static constexpr uint8_t        MS5611_ADDRESS              = 0x77; // Devuce I2C Address
        static constexpr uint8_t        MS5611_RESET                = 0x1E; // Reset command code
        static constexpr uint8_t        MS5611_CONVERT_D1           = 0x40; // Base for D1 (pressure) conversion
        static constexpr uint8_t        MS5611_CONVERT_D2           = 0x50; // Base for D2 (temperature) conversion
        static constexpr uint8_t        MS5611_READ_ADC             = 0x00; // Read ADC value command
        static constexpr uint8_t        MS5611_READ_PROM            = 0xA0; // Read PROM command (add index<<1)

        // Conversion times (ms) for OSR = 256, 512, 1024, 2048, 4096
        static constexpr uint16_t MS5611_CONVERSION_TIMEOUT_MS[] = { 1, 2, 3, 5, 10 };   // Timeout for conversion in milliseconds  

        TwoWire*        wire_;                  // I2C bus
        uint8_t         address_;               // I2C address (0x76/0x77)
        Oversampling    osr_;                   // current OSR
        MathMode        mathMode_;              // datasheet vs appnote

        uint16_t        cal_[6];                // PROM coefficients
        int32_t         pressureOffset_;        // user offset in mbar
        double          temperatureOffset_;     // user offset °C
        uint32_t        deviceID_;              // XOR of PROM words

        int64_t OFF2 = 0, SENS2 = 0, TEMP2 = 0;  // second‐order terms

        // Low-level I2C communication methods
        void        resetSensor();                  // Send reset over I2C
        bool        readCalibration();              // Read PROM and verify CRC4
        uint16_t    readRegister16(uint8_t reg);    // Read 16-bit register value
        uint32_t    readRegister24(uint8_t reg);    // Read 24-bit register value
};
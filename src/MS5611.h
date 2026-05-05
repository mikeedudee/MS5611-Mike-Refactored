/*

The MIT License

    Copyright (c) 2014-2023 Korneliusz Jarzębski
    Copyright (c) 2023–2025 Rob Tillaart
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

#pragma once // Ensures this header file is included only once
// Project name: MS5611 Sensor Library

#include <cstdint>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Inlcude Filter & Velocity/Acceleration Libraries
#include "filters/median_filter.h"
#include "filters/kalman_filter.h"
#include "derivative_estimator.h"

#define MS5611_LIB_VERSION          (F("1.2.5_exp_build_04126026"))
#define MS5611_ENVIRONMENT_COMPT    (F("Universal Dual-Bus Support (SPI & I²C), compatiable with almost all boards. Please do note that this library is maintained and solely developed independently. Bugs or incompatibility may occur at your instance; if so, please report to me immediately."))
#define MS5611_AUTHOR               (F("Copyright (c) 2025 Francis Mike John Camogao [Refactor/Enhancements]"))
#define MS5611_LIBRARY_NAME         (F("MS5611-Mike-Refactored"))
#define MS5611_LIBRARY_URL          (F("https://github.com/mikeedudee/MS5611-Mike-Refactored.git"))
#define MS5611_LIBRARY_DESCRIPTION  (F("A library for the MS5611 pressure sensor, providing high-resolution temperature and pressure readings with optional compensation. This library exclusively supports the I2C interface. It is a refactored and enhanced version originally based on the library by Korneliusz Jarzebski. Structural improvements, optimizations, and additional features have been incorporated, including expanded API support. Several functions and design elements are adapted from or inspired by Rob Tillaart's MS5611 library, integrated with further modifications for improved performance and usability."))
#define MS5611_LIBRARY_LICENSE      (F("MIT License - see LICENSE file for details."))

// Sensor Limits (Datasheet values + Safety Margin)
static constexpr float MIN_TEMP_C           = -40.0f;   // Minimum temperature in °C
static constexpr float MAX_TEMP_C           = 85.0f;    // Maximum temperature in °C
static constexpr float MIN_PRESSURE_MBAR    = 10.0f;    // Minimum pressure in mbar
static constexpr float MAX_PRESSURE_MBAR    = 1200.0f;  // Maximum pressure in mbar
/// END OF 1.2.0_exp_build_02122026

        // Enum for oversampling rates; these control the resolution and power consumption
        // Higher values yield better resolution but consume more power and take longer to read
        enum Oversampling : uint8_t {
            ULTRA_HIGH_RES   = 0x08,
            HIGH_RES         = 0x06,
            STANDARD         = 0x04,
            LOW_POWER        = 0x02,
            ULTRA_LOW_POWER  = 0x00
        };

class MS5611 { 
    public:
        /// ADDED 1.2.0_exp_build_02122026 for HEALTH DIAGNOSTICS
        // -------------------------------------------------------------------------
        static constexpr uint16_t   STATUS_OK                   = 0x0001;
        static constexpr uint16_t   STATUS_BUS_ERROR            = 0x0002; // Bit 0: I2C/SPI hardware failure
        static constexpr uint16_t   STATUS_CRC_ERROR            = 0x0003; // Bit 1: PROM memory corrupted
        static constexpr uint16_t   STATUS_PHYSICS_VIOLATION    = 0x0004; // Bit 2: Temp/Pres out of bounds
        static constexpr uint16_t   STATUS_APPROACHING_LIMITS   = 0x0005; // Bit 2 + Bit 0: Values approaching physical limits
        static constexpr uint16_t   STATUS_SPIKE_DETECTED       = 0x0008; // Bit 3: Anomaly caught by filter
        static constexpr uint16_t   STATUS_INIT_FAILED          = 0x0010; // Bit 4: Sensor offline at boot
        static constexpr uint16_t   MS5611_READ_OK              = STATUS_OK;

        uint16_t                    getHealthStatus             () const { return _healthStatus; }
        void                        clearHealthStatus           ()       { _healthStatus = STATUS_OK; }
        uint16_t                    getResult                   () const { return _healthStatus; }                                    // Get result of the last operation (0 = success, non-zero = error)

        // -------------------------------------------------------------------------
        /// END OF 1.2.0_exp_build_02122026

        /// Scaling math modes
        enum class MathMode : uint8_t {
            Datasheet = 0,
            AppNote   = 1
        };

        /// Do a measurement cycle and return both values at once
        struct Measure { 
            double temperature;  // °C
            float  pressure;     // mbar
        };

        /// ----------------------- ADDED FOR 1.2.0_exp_build_02122026  
        // Asynchronous (Non-blocking) API
        // Start a temperature conversion (D2)
        // Returns true if the command was sent successfully
        bool startTemperature(); 

        // Start a pressure conversion (D1)
        // Returnes true if the command was sent successfully
        bool startPressure();

        // Check if the current conversion is complete (based on timeout)
        bool isConversionComplete() const;

        // Retrieve the results of the latest conversion (either D1 or D2)
        // Should be called only after isConversionComplete() returns true
        // Returns 24-bit ADC value or UINT32_MAX on error
        uint32_t getConversionValue();

        // Valudates if values are within realisitc physical bounds
        // Sets internal _result to MS5611_ERROR_Range if invalid
        bool validatePhysics(double temperature, float pressure);
        /// ----------------------- END OF 1.2.0_exp_build_02122026

        /// Read both temperature & pressure in one go.
        /// if true, apply 2nd-order (low-temp) compensation
        Measure performanceRead (bool compensate = false);

        /// ADDED FOR 1.2.0_exp_build_02122026
        // -------------------------------------------------------------------------
        // Constructors
        // -------------------------------------------------------------------------
        MS5611                  ();                                             // Default: Wire @ 0x77
        MS5611                  (uint8_t address, TwoWire &wirePort = Wire);    // Custom I²C address / bus
        MS5611                  (int8_t csPin, SPIClass &spiPort = SPI);        // NEW: SPI Custom Constructor

        
        
        /// MODIFIED FOR 1.2.0_exp_build_02122026
        // UPDATED begin() to accept optional pins for bus recovery
        // sdaPin / sclPin: Microcontrolelr pin numbers. If set, runs recovery before init.
        // Reset, read PROM (+CRC), choose OSR & math mode
        bool begin(Oversampling osr   = HIGH_RES,
                   MathMode     math  = MathMode::Datasheet,
                   int8_t       sdaPin = -1,
                   int8_t       sclPin = -1
                );
        
        // Manually attempt to unstick the I2C bus
        // Toggle SCL 9 times to flush any stuck bits
        void recoverI2C(int8_t sdaPin, int8_t sclPin);
        /// END OF MODIFIED FOR 1.2.0_exp_build_02122026

        // Raw ADC reads (blocking)
        uint32_t        readRawTemperature  ();
        uint32_t        readRawPressure     ();
        
        // Calibrated values
        double          readTemperature     (bool compensate = true) const;
        float           readPressure        (bool compensate = true) const;
        double          getTemperature      (bool compensate = true) { return performanceRead(compensate).temperature; }
        float           getPressure         (bool compensate = true) { return performanceRead(compensate).pressure;    }

        // Helpers
        static double   getAltitude         (double pressure, double seaLevelPressure = 101325.0);
        static double   getSeaLevel         (double pressure, double altitude);

        // Offsets & IDs
        void            setPressureOffset   (int32_t mbar)      { pressureOffset_    = mbar; }               // Set pressure offset in mbar
        void            setTemperatureOffset(double degC )      { temperatureOffset_ = degC; }               // Set temperature offset in degrees Celsius
        uint32_t        getDeviceID         () const            { return deviceID_; }                        // Get device ID (XOR of PROM words)
        uint16_t        getManufacturer     () const;                                                        // Declare getManufacturer() const; // Get manufacturer code (first PROM word)
        uint16_t        getSerialCode       () const;                                                        // Declare getSerialCode() const; // Get serial code (last PROM word shifted right by 4 bits)
        uint16_t        getAddress          () const            { return address_; }                         // Get I2C address (0x76 or 0x77)
            

        // Oversampling introspection
        void            setOversampling     (Oversampling osr)  { osr_ = osr; }                              // Set oversampling rate
        Oversampling    getOversampling     () const            { return osr_; }                             // Get current oversampling rate
        uint8_t         getOSRCode          () const            { return static_cast<uint8_t>(osr_); }       // Get OSR code (0-4) for conversion commands
        uint16_t        getConvTimeMs       () const            {                                            // Get conversion time in milliseconds based on the current OSR
            return MS5611_CONVERSION_TIMEOUT_MS[uint8_t(osr_) >> 1];
        }

        // Low-level I2C communication method
        void            resetSensor         ();                                                             // Send reset over I2C
        bool            readCalibration     ();                                                             // Read PROM and verify CRC4
        void            convert             (const uint8_t addr, uint8_t bits);                             // Start conversion with specified address and bits
        int             read                (uint8_t bits);                                                 // Read ADC value with specified oversampling bits  
        inline int      read                () { return read( (uint8_t) HIGH_RES); };                       // Default to HIGH_RES
        uint32_t        readADC             ();                                                             // Read ADC value (24-bit) from the sensor
        uint16_t        readProm            (uint8_t reg);                                                  // Read PROM register (16-bit) from the sensor
        int             command             (const uint8_t command);                                        // Send command to the sensor
        uint16_t        getProm             (uint8_t index);                                                 // Read PROM coefficient at specified index (0-6)    
        uint16_t        getCRC              ();                                                             // Calculate CRC4 checksum for the PROM coefficients

        uint16_t        getLastRead         () const { return _lastRead; }                                  // Get timestamp of the last read operation
        
        /// MEDIAN FILTER ///
        bool enableMedianFilter(uint8_t ws = 5u) {
            enabledMedian_ = median_.setWindowSize(ws);
            
            return enabledMedian_;
        }

        /// Read the latest filtered altitude
        float medianFilter(float base_value) {
            float raw = base_value;

            return enabledMedian_
                ? median_.update(raw)
                : raw;
        }

        /// KALMAN FILTER ///
        /// Enable Kalman filtering with specified uncertainties
        /// e_mea: measurement uncertainty (>0)
        /// e_est: estimation uncertainty (≥0)
        /// q:     process noise (≥0)
        /// Returns true if parameters valid
        bool enableKalmanFilter(float e_mea = 1.0f,
                                float e_est = 1.0f,
                                float q     = 0.01f) 
        {
            enabledKalman_ = kalman_.setParameters(e_mea, e_est, q);
            return enabledKalman_;
        }

        /// Read the latest filtered altitude (Kalman)
        float kalmanFilter(float base_value) {
            float raw = base_value;

            return enabledKalman_
                ? kalman_.update(raw)
                : raw;
        }

        /// NEW: Read filtered altitude with outlier rejection (Gated Kalman)
        /// sigma: Deviation threshold (default 3.0)
        /// max_rejects: Max ignored samples before forced update (default 10)
        float kalmanFilterGated(float base_value, float sigma = 3.0f, uint8_t max_rejects = 10) {
             float raw = base_value;
             return enabledKalman_ ? kalman_.updateGated(raw, sigma, max_rejects) : raw;
        }

        /// VELOCITY/ACCELERATION COMPUTE ///
        float getVelocity(float altitude, float timeSec) {
            return _velEstimator.update(altitude, timeSec);
        }

        /// Compute vertical acceleration [m/s²] from any velocity sample
        float getAcceleration(float velocity, float timeSec) {
            return _accEstimator.update(velocity, timeSec);
        }

        /// Reset both estimators (e.g. after changing filters)
        void resetDynamics() {
            _velEstimator.reset();
            _accEstimator.reset();
        }

        // ─── Unified spikeDetection API ───
        //   enable:            on/off
        //   ringSize:          1…SPIKE_MAX_RING      (default 5)
        //   threshold:         >0                    (default 1.0f)
        //   temperature:       NAN → readTemperature()
        //   pressure:          NAN → readPressure()
        //   consecutiveCount:  spikes in a row before reset (default 5)
        void spikeDetection(bool enable = false,
                            uint8_t ringSize          = 5,
                            float   threshold         = 10.0f,
                            float   temperature       = NAN,
                            float   pressure          = NAN,
                            uint8_t consecutiveCount  = 5);

        
        void incrementSpikeCounter  ();                                     // call this whenever you detect a spike     
        int  getSpikeCounter        () const;                               // get how many spikes have occurred so far  
        void resetSpikeCounter      ();                                     // reset count back to zero
        int  getResetCount          () const { return _resetCount; }        // Returns how many times resetSensor() has been called


    private:
        uint16_t                        _healthStatus               = STATUS_OK;
        
        // I2C command and address constants
        static constexpr uint8_t        MS5611_ADDRESS              = 0x77; // Device I2C Address
        static constexpr uint8_t        MS5611_RESET                = 0x1E; // Reset command code
        static constexpr uint8_t        MS5611_CONVERT_D1           = 0x40; // Base for D1 (pressure) conversion
        static constexpr uint8_t        MS5611_CONVERT_D2           = 0x50; // Base for D2 (temperature) conversion
        static constexpr uint8_t        MS5611_READ_ADC             = 0x00; // Read ADC value command
        static constexpr uint8_t        MS5611_READ_PROM            = 0xA0; // Read PROM command (add index<<1)

        // Conversion times (ms) for OSR = 256, 512, 1024, 2048, 4096
        static constexpr uint16_t MS5611_CONVERSION_TIMEOUT_MS[] = { 1, 2, 3, 5, 10 };   // Timeout for conversion in milliseconds  

        // updated for 1.2.0_exp_build_02122026: SPI support
        TwoWire*        wire_;                      // I2C bus
        SPIClass*       spi_;                       // SPI bus
        int8_t          csPin_;                     // SPI Chip Select Pin
        bool            useSPI_;                    // Flag to determine hardware interface
        // END of SPI support additions
        uint8_t         address_;                   // I2C address (0x76/0x77)
        Oversampling    osr_;                       // current OSR
        MathMode        mathMode_;                  // datasheet vs appnote

        uint16_t        cal_[6];                    // PROM coefficients
        int32_t         pressureOffset_;            // user offset in mbar
        double          temperatureOffset_;         // user offset °C
        uint32_t        deviceID_;                  // XOR of PROM words

        float           C[7];                       // Calibration coefficients for second-order compensation
        uint32_t        _lastRead;                  // Timestamp of the last read operation
        uint32_t        _deviceID;                  // Device ID calculated from the PROM coefficients

        int64_t OFF2 = 0, SENS2 = 0, TEMP2 = 0;     // second‐order terms
        
        // Low-level I2C communication method
        uint16_t    readRegister16(uint8_t reg)     const;    // Read 16-bit register value
        uint32_t    readRegister24(uint8_t reg)     const;    // Read 24-bit register value

        // Derivative estimators for velocity and acceleration
        DerivativeEstimator _velEstimator;
        DerivativeEstimator _accEstimator;

        // Filters
        MedianFilter        median_;
        KalmanFilter        kalman_;
        bool                enabledMedian_          = false;
        bool                enabledKalman_          = false;

        /// Spike‐Detection API ///  
        // compile-time maximum buffer size
        static constexpr uint8_t SPIKE_MAX_RING     = 20;

        bool                    _spikeEnabled       = false;
        bool                    _spikeWasEnabled    = false;
        uint8_t                 _spikeRingSize      = 5;
        float                   _spikeThreshold     = 1.0f;
        uint8_t                 _spikeIdx           = 0;

        float                   _spikeBufP[SPIKE_MAX_RING]{};
        float                   _spikeBufT[SPIKE_MAX_RING]{};

        /// NON-Blocking Addition 
        // ADDED FOR 1.2.0_exp_build_02122026
        uint32_t               _conversionStartTime = 0;    // Timestamp when the current conversion was started
        uint32_t               _conversionTimeout   = 0;    // Timeout for the current conversion in milliseconds
        bool                   _isConverting        = false; // Explicitly tracks if a conversion is running
        /// END OF 1.2.0_exp_build_02122026  

        // new members for N-in-a-row logic:
        uint8_t  _spikeConsecNeed  = 0;
        int      _spikeCount       = 0;
        
        // counter for resets
        int      _resetCount       = -1;  // Start at -1 to account for the first reset in begin()
};

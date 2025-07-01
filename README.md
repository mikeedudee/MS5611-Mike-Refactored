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
* **Spike detection** to flag or suppress sudden outliers—flags and resets the sensor once detected and then counts it
* **Performance mode** is useful for during memory constraints
* **Manual-mode read APIs** for last getPressure() and getTemperature() values [available only if performance mode is enabled]
* **Reference-based altitude calculation** dynamically set for user specific reading-output

Some features come from or are inspired by Rob Tillaart MS5611 library.

Fully independent and maintained; please report any issues on the [issue tracker](https://github.com/mikeedudee/MS5611-Mike-Refactored/issues).

## Compatibility

* I2C interface only
* Default addresses: `0x76` or `0x77` (CSB/CSO pin)
* Tested on Teensy 4.1; should work with any Arduino-compatible board supporting `Wire`
* Compatible with MS56xx, MS57xx, MS58xx (some require reset & pressure check fix)

## Installation

1. Clone or download this repository
2. In Arduino IDE: **Sketch** → **Include Library** → **Add .ZIP Library…**

### Oversampling

- **void setOversampling(Oversampling osr)** sets the amount of oversampling. 
See the table below and test the example of how to use.
- **Oversampling getOversampling()** returns amount of oversampling.


Some numbers from the datasheet, page 3, MAX column rounded up. (see #23)
(actual read time differs - see performance sketch)

There are 5 oversampling settings, each corresponding to a different number of milliseconds. The higher the oversampling, the more accurate the reading will be; however, the longer it will take. So it must take into account to suit your mission profile/needs.

|        definition       | value (ms) | true value (ms) | oversampling ratio | resolution (mbar) |  resolution (°C)  | Resolution | notes  |
|:-----------------------:|:----------:|:---------------:|:------------------:|:-----------------:|:-----------------:|:----------:|:-------:
| ULTRA_HIGH_RES          |     10     |      9.04       |        4096        |        0.012      |        0.002      |  HIGHEST   |
| HIGH_RES                |      5     |      4.54       |        2048        |        0.018      |        0.003      |  HIGH      | Default 
| STANDARD                |      3     |      2.28       |        1024        |        0.027      |        0.005      |  MEDIUM    |
| LOW_POWER               |      2     |      1.17       |        512         |        0.042      |        0.008      |  LOW       |
| ULTRA_LOW_POWER         |      1     |      0.60       |        256         |        0.065      |        0.012      |  LOWEST    |
- Code Example:
  ```cpp
  MS5611.setOversampling(HIGH_RES);
  // or
  ms5611.begin(ULTRA_HIGH_RES))
  ```
Setting the oversample resolution through `setOversampling()` or using it during `.begin` is usually the same,  the difference is that the `setOversampling()` API is dynamic—can be called anywhere, setting the sensors to adapt to various mission profiles or requirements.

## API Reference
**Second-Order Temperature Compensation:**
This feature "trigger" is enabled by default to automatically correct for sensor non-linearities at extreme temperatures. Compensation will automatically applied when the measured temperature falls below 20 °C or exceeds 90 °C, ensuring improved accuracy across the full operational range.


Before using the API functions, an instance of the class must be created and assigned to a variable. This instance serves as the reference through which all API functions are accessed and invoked.

Example: 
  ```cpp
  MS5611 ms5611;
  ```
- `MS5611` <-- is the class bundle
- `ms5611` <-- is the variable we're gonna assign it to

### Initialization & Configuration

| Function                                  | Description                                          | Sample Usage
| ----------------------------------------- | ---------------------------------------------------- |------------
| `begin(Oversampling osr = HIGH_RES)`      | Initialize sensor with optional oversampling setting | `ms5611.begin(resolution);` or `ms5611.begin();`
| `setOversampling(Oversampling osr)`       | Change oversampling on the fly                       | `ms5611.setOversampling(resolution);` <-- dynamic resolution changer
| `getOversampling()`                       | Returns current applied oversampling setting         | `Serial.println(ms5611.getOversampling());`
| `setPressureOffset(float offset)`         | Apply manual pressure calibration offset             | `ms5611.setPressureOffset(10000);` <-- Must be in unit of Pascal
| `setTemperatureOffset(float offset)`      | Apply manual temperature calibration offset          | `ms5611.setPressureOffset(15);` <-- Must be in degree of Celsius
| `resetSensor()`                           | This command clears the sensor's internal state and prepares it for a fresh start | `ms5611.resetSensor();`
| `resetDynamics()`                         | Clear internal filter & derivative states. (Optional) reset the velocity and acceleration computation if you reconfigure filters mid-flight | `ms5611.resetDynamics();`

### Standard Reads
The most accurate and best choice since each API function initiates its own raw conversion (either D1 for pressure or D2 for temperature), waits for the required conversion time, reads the ADC result, and performs the necessary compensation calculations—including second-order correction, which is enabled by default. This approach ensures high accuracy but comes at the cost of increased latency and memory cost, and processing overhead. If you don’t mind the slight extra latency and I²C overhead of separate calls, then readPressure()/readTemperature() alone are sufficient.
    
| Function                        | Description                             | Sample Usage
| ------------------------------- | --------------------------------------- | --------------------
| `readTemperature(bool comp)`    | get the Temperature reading             |  `Serial.println(ms5611.readTemperature());`
| `readPressure(bool comp)`       | provide the pressure reading            |  `Serial.println(ms5611.readPressure());`

### Performance Mode
In cases you have memory constraints or you prioritize minimum latency as much as possible and simultaneous reading of both temperature and pressure, the performance API's are best suited to your mission profile. However, it is not as accurate as the standard read mode. How? It only does one pair of conversions instead of kicking off two separate conversions if you were to call getTemperature(), then getPressure(). Both values come from the same “moment” in time, since D1 and D2 were measured back-to-back, which is important in high-dynamics (e.g. fast maneuvers) and you can call getPressure() and getTemperature() as often as you like, without extra I²C or math, until your next read due to cached results.

| Function                        | Description                             | Sample Usage          |
| ------------------------------- | --------------------------------------- | ----------------
| `performanceRead(bool value)`   | API call to trigger (on/off) the second-order compensation computation (default: on) | `auto ms5611.performanceRead(true/false);`
| `getPressure(bool comp)`        | Performance mode pressure       | `Serial.println(ms5611.getPressure());`
| `getTemperature(bool comp)`     | Performance mode of temperature | `Serial.println(ms5611.getTemperature());`

*This can also be achieved using the following code:*
  ```cpp
  auto performanceRead = ms5611.performanceRead();
  Serial.print("T = ");
  Serial.print(performanceRead.temperature, 2);
  Serial.print(" °C   P = ");
  Serial.print(performanceRead.pressure, 2);
  ```
*Either way works, use that suits your workflow.*

### Filters
This library provides two built-in filtering methods, each with its own distinct characteristics and advantages. I encourage you to experiment and evaluate both to determine which is best suited for their specific mission profile or application requirements.

- **Median Filter** - a non-linear digital filtering technique often used to remove spikes or "salt-and-pepper" noise from a signal while preserving sharp transitions.
- **Kalman Filter** - a recursive, optimal estimator for linear dynamical systems subject to Gaussian noise. It blends a predictive model ("what you expect") with noisy measurements ("what you observe") to produce a statistically optimal estimate of the system’s state.
    
| Function                                                        | Description                               | Sample Usage
| --------------------------------------------------------------- | ----------------------------------------- |-----------------
| `enableMedianFilter(uint32_t windowSize)`                       | Enable median smoothing                   | `ms5611.enableMedianFilter(value);`
| `medianFilter(double input)`                                    | Apply median filter                       | `float Altitude = ms5611.getAltitude(Pressure); ms5611.medianFilter(Altitude);`
| `enableKalmanFilter(double e_mea, double e_est, double q)`      | Enable Kalman filter                      | `ms5611.enableKalmanFilter(0.5, 0.5, 0.138);`
| `kalmanFilter(double input)`                                    | Apply Kalman filter                       | `float Altitude = ms5611.getAltitude(Pressure); ms5611.kalmanFilter(Altitude);`


- **Median Filter** - The single parameter to `enableMedianFilter(ws)` is the *window size* `ws`, which controls how many of the most recent altitude samples are considered when computing the median. Recommended starting point for barometric altitude at ~10 Hz sample rate, ws = 5 or 7 often works well—enough to suppress occasional spikes without too much lag. The default value for the filter is 5. **Value must be odd and max=21 & min=3**
  - default value: `enableMedianFilter(5u);`
- **Kalman Filter**
  - `e_mea = measurement uncertainty` - *Represents how noisy you believe each new barometric altitude reading is. A larger e_mea means “I don’t trust the sensor much,” so the filter leans more on its prediction and smooths out spikes.*
  - `e_est = Initial estimate uncertainty` - *The filter’s initial covariance on its own state estimate before seeing any measurements. A larger e_est says “I’m very uncertain about my starting altitude estimate,” so the first few measurements will shift the estimate more aggressively.*
  - `q = Process noise` - *Models how much the true altitude is expected to change unpredictably between samples (e.g.\ turbulence, rapid ascent/descent). A larger q makes the filter adapt more quickly to real changes, but also lets more measurement noise through.*
  - default value: `enableKalmanFilter(float e_mea = 1.0f, float e_est = 1.0f, float q = 0.01f);`


### Derivative Estimation
Using the `getVelocity()` and `getAcceleration()` API functions.

| Function                                                          | Description                  |
| ----------------------------------------------------------------- | ---------------------------- |
| `getVelocity(double altitude, unsigned long timestamp)`           | Vertical velocity (m/s)      |
| `getAcceleration(double velocity, unsigned long timestamp)`       | Vertical acceleration (m/s²) |

**Sample Usage**
  ```cpp
  void setup() {
    referencePressure = ms5611.readPressure();
    ms5611.enableMedianFilter(11u);
    ms5611.enableKalmanFilter(0.5, 0.5, 0.138);
    ms5611.resetDynamics();
  }

  void loop() {
    unsigned long now = millis()/1000;

    long realPressure = ms5611.readPressure();
    double realAltitude = ms5611.getAltitude(realPressure, referencePressure);

    double    medianAltitude    = ms5611.medianFilter(realAltitude);
    double    kalmanAltitude    = ms5611.kalmanFilter(realAltitude);

    // in this sample we'll use both filter
    float     velocityMedian    = ms5611.getVelocity(medianAltitude, now);
    float     acceleMedian      = ms5611.getAcceleration(velocityMedian, now);
  
    float     velocityKalman    = ms5611.getVelocity(medianAltitude, now);
    float     acceleKalmanedian = ms5611.getAcceleration(velocityKalman, now);
  }
  ```

### Spike Detection & Utilities

| Function                                           | Description                                                             | Sample Usage
| -------------------------------------------------- | ----------------------------------------------------------------------- | ------------
| `spikeDetection(bool enable, uint8_t ringSize, float threshold, temperature, pressure, uint8_t consecutiveCount);`                      | Toggle outlier detection/suppression. Automatically resets the MS5611 sensor once anomally is detected (1 second delay) before resets.                                    |
| `readRawTemperature()`          | Raw temperature ADC value               |  `Serial.println(ms5611.readRawTemperature());`
| `readRawPressure()`             | Raw pressure ADC value                  |  `Serial.println(ms5611.readRawPressure());`
| `getAltitude(int32_t pressure, float refPressure)` | Altitude relative to reference (m)                                      | `ms5611.getAltitude(Pressure);`
| `getSeaLevel(double p, double alt)`                | Compute sea-level pressure (Pa)                                         | `ms5611.getSeaLevel(Pressure, Current_Altitude);`

- *How to use the `spikeDetection()` API:*
  - The function requires six positional arguments, all of which must be explicitly defined for proper operation. These parameters are user-defined and should be configured to suit your mission requirements or system standards.
  - it can be called once or only 1 positional argument, e.g.: `spikeDetection(true);` this works and will load the default values for the other positional arguments
    - default values:
      - `Window = 5`
      - `Threshold` = 10` - the jump in 10kPa
      - `temperature = NAN` - reads the standard reading of temperature unless satisfied.
      - `pressure = NAN` - reads the standard reading of pressure unless satisfied.
      - `consecutiveCount = 5` - spikes in a row before reset (going lower will make it react immediately since it only needs 1 data)

**Sample  Usage**
  ```cpp
  ms5611.spikeDetection(true, 5, 10, temperature, pressure, 3);
  ```
  - Once there is a spike for about >=< 10kPa in the pressure or in the order of temperature for three cycles, it will trigger the spike guard and reset the sensor.

### Debugging (Dev-mode)
| Function                      | Description                                                             |
| ------------------------------| ----------------------------------------------------------------------- |
| `MS5611_AUTHOR`               | returns the author of the library                                       |
| `MS5611_LIBRARY_NAME`         | returns the library name
| `MS5611_LIB_VERSION`          | returns the library build version
| `MS5611_LIBRARY_LICENSE`      | returns the library license
| `MS5611_LIBRARY_DESCRIPTION`  | returns library description
| `MS5611_LIBRARY_URL`          | returns the library's GitHub repository
| `MS5611_ENVIRONMENT_COMPT`    | returns the library's tested environment
| `getManufacturer()`           | returns manufacturer private info | 
| `getSerialCode()`             | returns serialCode from the PROM\[7] minus the CRC |
| `getOSRCode()`                | return the raw OSR command code (0x00,0x02,0x04,0x06,0x08) |
| `getConvTimeMs()`             | return the conversion time (ms) for the current OSR |
| `getAddress()`                | return I2C Address |
| `getDeviceID()`               | return device ID (XOR of PROM words) |
| `getPROM()`                   | read PROM coefficient at specified index (0-6)  |
| `getCRC()`                    | calculate CRC4 checksum for the PROM coefficients |
| `getLastRead()`               | timestamp of the last read operation
| `getResult()`                 | get result of the last operation (0 = success, non-zero = error)
| `getSpikeCounter()`           | return the counting of spike/anomaly reading position 
| `getResetCount()`             | return how many times a reset occurred (dev-mode)


## Note

Always refer to the [MS5611 datasheet](https://cdn-shop.adafruit.com/datasheets/MS5611.pdf) for timing, calibration, and electrical specifications.

Major key optimizations and changes in the source code are listed in the [CHANGELOG.md](./CHANGELOG.md); future enhancements are still ongoing development.

*The features and the algorithm written for this library are all based on or are inspired by other libraries and on the MS5611 DATA SHEET*

## Usage Examples

### Basic Example

```cpp
#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!ms5611.begin()) {
    Serial.println("MS5611 not found!");
    while (1);
  }
  ms5611.setOversampling(HIGH_RES);
}

void loop() {
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
  Serial.begin(9600);
  Wire.begin();
  ms5611.begin(ULTRA_HIGH_RES);

  // reference for relative altitude
  refPressure = ms5611.getPressure();

  // enable dynamic filters
  ms5611.enableMedianFilter(11); // Format: (window size) (value must be odd and max=21, min=3, default=5)

  lastTime = millis();
}

void loop() {
  // enable spike detection
  ms5611.spikeDetection(true, 5, 10); // Format: (state, window, threshold) 

  unsigned long now = millis();

  // batch read + timing
  Serial.print("Read time (ms): ");

  float alt = ms5611.getAltitude(ms5611.getPressure(), refPressure);
  float altFilt = ms5611.medianFilter(alt);
  float vel = ms5611.getVelocity(altFilt, now);
  float acc = ms5611.getAcceleration(vel, now);

  Serial.print("Alt (m): "); Serial.println(altFilt);
  Serial.print("Vel (m/s): "); Serial.println(vel);
  Serial.print("Acc (m/s²): "); Serial.println(acc);

  lastVelocity = vel;
  lastTime = now;
  delay(500);
}
```


## Contributing

Contributions, issues, and feature requests are welcome — please open an issue or submit a pull request on GitHub.

---

**License**: MIT (see [LICENSE](./LICENSE))

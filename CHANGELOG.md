MS5611 Library Mike Refactored Version 1.0.0 | 28 Jun 2025
======================================================================

Key improvements / changes made:

MS5611.h >>
  * Switched from “#ifndef…#define” guards and legacy ARDUINO>=100 conditionals to `#pragma once` and a single `<Arduino.h>` include.
  * Added MIT license header + Doxygen-style comments for every public API.
  * Replaced all raw `#define` command codes and addresses with `static constexpr uint8_t` constants.
  * Introduced `enum class Oversampling : uint8_t` (0x00–0x08) to prevent invalid OSR values.
  * Added inline, constexpr helpers:
      - `Oversampling getOversampling() const;`
      - `uint8_t     getOSRCode()   const;`  
      - `uint16_t    getConvTimeMs() const;`
  * Documented error return values (`UINT16_MAX`, `UINT32_MAX`, `INT32_MIN`, `NAN`) in method comments.
  * Small, single-purpose methods (<10 lines) complying with NASA’s Power-of-Ten rules:  
      - No dynamic memory  
      - Explicit-width integer types everywhere

MS5611.cpp >>
  * Removed `delay(ct)` and legacy guards; implemented **non-blocking waits** using `millis()` + a per-OSR timeout table.
  * Added **CRC-4** check in `readCalibration()` to validate PROM contents—`begin()` returns `false` on mismatch.
  * Checked `Wire.endTransmission()` and `Wire.requestFrom()` return codes on every I²C transaction.
  * All raw-read methods (`readRawTemperature()`, `readRawPressure()`) are now `const` and return `UINT32_MAX` on error/timeout.
  * Second-order compensation and pressure/temperature math use **pure integer** shifts & multiplies (no `pow()`).
  * Floating-point only in static altitude/sea-level helper methods.
  * Single-point `resetSensor()` issues reset command and waits exactly 3 ms.

MS5611 Library Mike Refactored Version 1.0.1 | 29 Jun 2025
======================================================================

Key improvements / changes since 1.0.0:

MS5611.h >>
  * **Math-mode support**: added
      - `enum class MathMode { Datasheet = 0, AppNote = 1 }`  
      - `bool begin(Oversampling, MathMode)` overload  
      - internal `mathMode_` field to switch compensation algorithms  
  * **I²C port & address injection**:  
      - new constructors  
        ```cpp
        MS5611();  
        MS5611(uint8_t address, TwoWire &wirePort = Wire);
        ```  
      - pointer to `TwoWire* wire_` and user-selectable `address_`  
  * **User offsets**:  
      - `void setPressureOffset(int32_t mbar);`  
      - `void setTemperatureOffset(double °C);`  
  * **Device identification API**:  
      - `uint32_t getDeviceID() const;`  // XOR of PROM words  
      - `uint16_t getManufacturer() const;`  
      - `uint16_t getSerialCode()   const;`  

MS5611.cpp >>
  * **Device ID**: computed in `readCalibration()` by shifting/XORing all PROM words  
  * **Math-mode branching**: `begin(…, MathMode)` and internal `mathMode_` can now select Datasheet vs AppNote coefficients  
  * **Offset application**: user offsets are now applied at the end of `readPressure()` and `readTemperature()`  
  * **Constructor implementations**:  
      - default ctor binds to `Wire`, uses default address  
      - custom ctor takes user-supplied `TwoWire &` and address
   
MS5611 Library Mike Refactored Version 1.0.4_exp_build_29072025 | 29 Jun 2025
======================================================================
### 🛠 FIXES
- Corrected `read()` API behavior: `readPressure()` now returns the final computed value, not binary output.
- Simplified resolution access:
  - Changed from `MS5611_RESOLUTION::ULTRA_HIGH_RES` ➝ `ULTRA_HIGH_RES` (no need to prefix with `MS5611::Oversampling`).
- Resolved `const` member function issue where it could not access external members.
- General bug fixes and stability improvements.

---

### ➕ ADDITIONS
- `getAddress()` — Returns the I2C address of the device.
- `getDeviceID()` — Returns the device ID.
- `getPROM()` — Access to PROM calibration coefficients.
- `getCRC()` — Computes and returns CRC from PROM.
- Automatic correction for anomalous pressure values.
- Added multiple example sketches.

---

### 🔁 CHANGES
- Revised oversampling resolution settings:
  | Setting           | Resolution | Delay (ms) |
  |------------------|------------|------------|
  | ULTRA_HIGH_RES   | Highest    | 10 ms      |
  | HIGH_RES         | High       | 5 ms       |
  | STANDARD         | Medium     | 3 ms _(Default / Backward Compatible)_ |
  | LOW_POWER        | Low        | 2 ms       |
  | ULTRA_LOW_POWER  | Lowest     | 1 ms       |

- **Header Change (`MS5611.h`)**:
  - Oversampling enumeration moved *outside* the main class to enable global access without needing a class prefix.

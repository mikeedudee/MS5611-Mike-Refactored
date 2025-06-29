/*
  MS5611 Barometric Pressure & Temperature Sensor. Output for processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujnik-cisnienia-i-temperatury-ms5611.html
  GIT: https://github.com/jarzebski/Arduino-MS5611
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski

// MODIFICATION: FRANCIS MIKE JOHN CAMOGAO
//         DATE: 29 JUN 2025
//          URL: https://github.com/mikeedudee/MS5611-Mike-Refactored.git

  This code is licensed under the MIT License (MIT).
  See LICENSE file for more details.
  This example reads the MS5611 sensor and prints the temperature, pressure, and altitude.
  It reads both compensated and uncompensated values, and prints them in a formatted manner.
  The output format is "uncompensated_temp:compensated_temp | uncompensated_pressure:compensated_pressure | uncompensated_altitude:compensated_altitude".
  The code checks for errors in reading the sensor data and exits if any read fails.
  The sensor is initialized with ultra high resolution settings.
  The output is printed to the Serial Monitor at a baud rate of 115200.
  Make sure to connect the MS5611 sensor to the correct I2C pins on your Arduino board.
 */

#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void setup() 
{
  Serial.begin(115200);

  // Initialize MS5611 sensor
  // Ultra high resolution: ULTRA_HIGH_RES
  // (default) High resolution: HIGH_RES
  // Standard: STANDARD
  // Low power: LOW_POWER
  // Ultra low power: ULTRA_LOW_POWER
  while(!ms5611.begin(ULTRA_HIGH_RES))
  {
    delay(500);
  }
  
  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOSRCode());
}

void loop()
{
  // Read true temperature & Pressure (without compensation)
  double    realTemperature   = ms5611.readTemperature();
  long      realPressure      = ms5611.readPressure();
  double    realAltitude      = ms5611.getAltitude(realPressure);

  // Read true temperature & Pressure (with compensation)
  double    realTemperature2  = ms5611.readTemperature(true);
  long      realPressure2     = ms5611.readPressure(true);
  double    realAltitude2     = ms5611.getAltitude(realPressure2);

  double first[3]   = {realTemperature, realPressure, realAltitude};
  double second[3]  = {realTemperature2, realPressure2, realAltitude2}; 

  // Number of pairs
  const uint8_t PAIRS = sizeof(first) / sizeof(first[0]);

  for (int i = 0; i < 3; i++)
  {
    if (first[i] && second[i] == NAN || first[i] && second[i] == INT32_MIN)
    {
      Serial.print("Error reading sensor data at index: ");
      Serial.println(i);
      return; // Exit if any read fails
    }
  }
  
  // Print each pair in the format “first:second”, separated by “ | ”
  for (uint8_t i = 0; i < PAIRS; ++i) {
      Serial.print(first[i]);
      Serial.print(':');
      Serial.print(second[i]);
      if (i < PAIRS - 1) {
          Serial.print(" | ");
      }
  }
  Serial.println();
}


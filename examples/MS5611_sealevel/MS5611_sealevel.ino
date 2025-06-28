/*
  MS5611 Barometric Pressure & Temperature Sensor. Sea Level
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujnik-cisnienia-i-temperatury-ms5611.html
  GIT: https://github.com/jarzebski/Arduino-MS5611
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski

  (c) 2025 by Francis Mike John Camogao [Refactor]
  This code is licensed under the MIT License (MIT).
  See LICENSE file for more details.
  This example reads the MS5611 sensor and prints the temperature, pressure, and sea level
  pressure. It calculates the sea level pressure based on the current pressure and a given altitude.
  The output format is "realTemp = X *C, realPressure = Y hPa, seaLevelPressure = Z hPa".
  The code checks for errors in reading the sensor data and retries initialization if it fails.
  The sensor is initialized with ultra high resolution settings.
  The output is printed to the Serial Monitor at a baud rate of 9600.
  Make sure to connect the MS5611 sensor to the correct I2C pins on your Arduino board.
*/

#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void setup() 
{
  Serial.begin(9600);

  // Initialize MS5611
  Serial.println("Initialize MS5611 sensor...");

  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while(!ms5611.begin(MS5611::Oversampling::MS5611_ULTRA_HIGH_RES))
  {
    Serial.println("Could not find a valid MS5611, check wiring!");
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
  // Set you real altitude
  // My location: Lapu-Lapu City, Philippines, 4m above sea level
  double myRealAltitude = 4;

  // Read true temperature & Pressure
  double realTemperature  = ms5611.readTemperature();
  double realPressure     = ms5611.readPressure();

  // Calculate sealevel pressure
  double seaLevelPressure = ms5611.getSeaLevel(realPressure, myRealAltitude);

  Serial.print("realTemp = ");
  Serial.print(realTemperature);
  Serial.print(" *C");

  Serial.print(", realPressure = ");
  Serial.print(realPressure);
  Serial.print(" Pa");

  Serial.print(", seaLevelPressure = ");
  Serial.print(seaLevelPressure);
  Serial.println(" Pa");

  delay(1000);
}

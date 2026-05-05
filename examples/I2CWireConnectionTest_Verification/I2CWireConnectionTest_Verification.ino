#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

// Teensy 4.1 Default I2C Pins
// Wire  (0): SDA = 18, SCL = 19
// Wire1 (1): SDA = 17, SCL = 16
// Wire2 (2): SDA = 25, SCL = 24
const int SDA_PIN = 18;
const int SCL_PIN = 19;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // New Init: Pass resolution, Math Mode, AND pins
  // This triggers the safety recovery sequence automatically.
  if (!ms5611.begin(ULTRA_HIGH_RES, MS5611::MathMode::Datasheet, SDA_PIN, SCL_PIN)) {
    Serial.println("Sensor Failed to Init (Even after recovery attempt)");
    while(1);
  }
  
  Serial.println("Sensor Initialized & Bus Verified.");
}

void loop() {
}
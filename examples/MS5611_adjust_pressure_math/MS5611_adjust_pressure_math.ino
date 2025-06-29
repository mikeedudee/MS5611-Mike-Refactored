//
//    FILE: adjust_pressure_math.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//    DATE: 2022-10-27
//     URL: https://github.com/RobTillaart/MS5611

// MODIFICATION: FRANCIS MIKE JOHN CAMOGAO
//         DATE: 29 JUN 2025
//          URL: https://github.com/mikeedudee/MS5611-Mike-Refactored.git

#include "MS5611.h"

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


MS5611 MS5611(0x77);

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

uint32_t start, stop;


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("MS5611_LIB_VERSION: ");
  Serial.println(MS5611_LIB_VERSION);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  if (MS5611.begin() == true)
  {
    Serial.print("MS5611 found [Address]: ");
    Serial.println(MS5611.getAddress());
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
  //  use adjusted math for pressure.
  MS5611.resetSensor();

  Serial.println();
}


/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  ULTRA_HIGH_RES   -> 8.22 millis
  HIGH_RES         -> 4.11 millis
  STANDARD         -> 2.1 millis
  LOW_POWER        -> 1.1 millis
  ULTRA_LOW_POWER  -> 0.5 millis   Default = backwards compatible
*/
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(ULTRA_LOW_POWER);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(LOW_POWER);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(STANDARD);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(HIGH_RES);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(ULTRA_HIGH_RES);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  Serial.println();
}


void test()
{
  start = micros();
  int result = MS5611.read();
  stop = micros();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
  else
  {
    Serial.print("T:\t");
    Serial.print(MS5611.readTemperature(), 2);
    Serial.print("\tP:\t");
    Serial.print(MS5611.readPressure(), 2);
    Serial.print("\tt:\t");
    Serial.print(stop - start);
    Serial.println();
  }
}


//  -- END OF FILE --

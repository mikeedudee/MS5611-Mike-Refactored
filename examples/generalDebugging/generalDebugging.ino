#include "MS5611.h"

MS5611 ms5611;

void setup() {
  Serial.begin(115200);
  Serial.println();
  ms5611.resetSensor();
  Serial.println(__FILE__);
  Serial.print("MS5611_LIB_VERSION: ");
  Serial.println(MS5611_LIB_VERSION);
  Serial.print("LIBRAY AUTHOR: ");
  Serial.println(AUTHOR);
  Serial.print("LIBRAY NAME: ");
  Serial.println(LIBRARY_NAME);
  Serial.print("LIBRAY DESCRIPTION: ");
  Serial.println(LIBRARY_DESCRIPTION);
  Serial.print("LIBRAY URL: ");
  Serial.println(LIBRARY_URL);
  Serial.print("TESTED ENVIRONMENT: ");
  Serial.println(ENVIRONMENT_COMPT);
  Serial.println();
  Serial.println();
  Serial.print("Manufacturer ID: ");
  Serial.println(ms5611.getManufacturer());
  Serial.print("Serial Code: ");
  Serial.println(ms5611.getSerialCode());
  Serial.print("Device ID: ");
  Serial.println(ms5611.getDeviceID(), HEX);
  Serial.print("Sensor Address: ");
  Serial.println(ms5611.getAddress());
  Serial.print("Last Read State: ");
  Serial.println(ms5611.getLastRead());
  Serial.print("Result of the last operation (0 = success, non-zero = error): ");
  Serial.println(ms5611.getResult());

  pinMode(LED_BUILTIN, OUTPUT);

  ms5611.begin(ULTRA_HIGH_RES);
  Serial.print("Oversampling Resolutin= ");
  Serial.print("OSR Value: ");
  Serial.print(ms5611.getOSRCode());
  Serial.print(" | Time in MS Value: ");
  Serial.println(ms5611.getConvTimeMs());
  Serial.println();
  Serial.print(" | Calibration State: ");
  Serial.println(ms5611.readCalibration());
  Serial.print(" | ADC State: ");
  Serial.println(ms5611.readADC());

  Serial.println();

  digitalWrite(LED_BUILTIN, 1);

  Serial.print("Raw Temperature Reading: ");
  Serial.println(ms5611.readRawTemperature());
  Serial.print("Raw Pressure Reading: ");
  Serial.println(ms5611.readRawPressure());
  Serial.print("Calibrated Temperature Reading: ");
  Serial.println(ms5611.readTemperature(true));
  Serial.print("Calibrated Pressure Reading: ");
  double Pressure = ms5611.readPressure(true);
  Serial.println(Pressure);
  Serial.print("Calculated Altitude: ");
  double Altitude = ms5611.getAltitude(Pressure, 101325); // 101325 Standard sea level atmospheric pressure
  Serial.println(Altitude); // in meters
  Serial.print("Sea Level Pressure: ");
  Serial.println(ms5611.getSeaLevel(Pressure, (4 + Altitude))); // added 4 meters since the location I live is 4 meters above the sea level [Lapu-Lapu City, Philippines] 
}

void loop() {
  // put your main code here, to run repeatedly:

}

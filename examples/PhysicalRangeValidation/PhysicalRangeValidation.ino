#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    if (!ms5611.begin()) {
        Serial.println("Sensor not found");
        while(1);
    }
}

void loop() {
    // Read Temperature
    double temp = ms5611.readTemperature();
    
    // Check for Validity
    if (isnan(temp)) {
        // If it's NAN, check WHY.
        if (ms5611.getResult() == MS5611_ERROR_RANGE) {
            Serial.println("CRITICAL: Temperature Sensor Physics Violation! (Out of Range)");
            // Trigger emergency recovery or safing mode
        } else {
            Serial.println("Error: Sensor Read Failed (I2C/Timeout)");
        }
    } else {
        Serial.print("Valid Temp: ");
        Serial.println(temp);
    }
    
    delay(500);
}
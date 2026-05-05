#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

// -------------------------------------------------------------------------
// State Machine Definitions
// -------------------------------------------------------------------------
enum SensorState : uint8_t {
    STATE_IDLE,         // Ready to start a new cycle
    STATE_WAIT_TEMP,    // Waiting for Temperature Conversion (D2)
    STATE_WAIT_PRESS,   // Waiting for Pressure Conversion (D1)
    STATE_DATA_READY    // Data collected, ready to calculate
};

SensorState currentState = STATE_IDLE;

// Variables to hold raw results temporarily
uint32_t rawTemp = 0;
uint32_t rawPress = 0;

// Timer for our simulated background task
uint32_t lastTaskTime = 0;
uint32_t taskCounter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println(F("\n[AETHER] System Initialized: Non-Blocking Mode Test"));
    Serial.println(F("-----------------------------------------------------"));

    Wire.begin();
    
    // Initialize MS5611 with ULTRA_HIGH_RES
    // This setting has the longest wait time (~9ms), making it perfect 
    // to demonstrate the non-blocking capability.
    if (!ms5611.begin(ULTRA_HIGH_RES)) {
        Serial.println(F("Error: MS5611 not found. Halting."));
        while (1);
    }
    
    Serial.println(F("Sensor Detected. Starting State Machine..."));
}

void loop() {
    // ---------------------------------------------------------------------
    // TASK 1: The Sensor State Machine (Non-Blocking)
    // ---------------------------------------------------------------------
    runSensorStateMachine();

    // ---------------------------------------------------------------------
    // TASK 2: Simulated Critical Flight Task (e.g., Stabilization Loop)
    // ---------------------------------------------------------------------
    // This runs continuously. If we used the old blocking read(), 
    // this counter would freeze for 9ms every time we read the sensor.
    runCriticalTask();
}

void runSensorStateMachine() {
    switch (currentState) {
        // Step 1: Trigger Temperature Conversion
        case STATE_IDLE:
            if (ms5611.startTemperature()) {
                currentState = STATE_WAIT_TEMP;
                // Serial.println(F("-> Temp Conversion Started"));
            } else {
                Serial.println(F("I2C Error: Could not start Temp"));
            }
            break;

        // Step 2: Check Temperature Status
        case STATE_WAIT_TEMP:
            if (ms5611.isConversionComplete()) {
                rawTemp = ms5611.getConversionValue();
                
                // Immediately start pressure to save time
                if (ms5611.startPressure()) {
                    currentState = STATE_WAIT_PRESS;
                } else {
                    currentState = STATE_IDLE; // Retry next loop on error
                }
            }
            break;

        // Step 3: Check Pressure Status
        case STATE_WAIT_PRESS:
            if (ms5611.isConversionComplete()) {
                rawPress = ms5611.getConversionValue();
                currentState = STATE_DATA_READY;
            }
            break;

        // Step 4: Process Data (Math happens here)
        case STATE_DATA_READY:
            {
                // We now have both raw values. We can use the library's
                // internal math functions if we expose them, or simply use 
                // the standard high-level read() which will now be instant
                // because the cache is fresh (if implemented), or manually calculate.
                
                // For this example, let's just use the simpler approach:
                // We actually have the raw data. 
                // Note: To use the library's compensation logic properly with 
                // async raw data, we typically need to modify the library to 
                // accept raw inputs for calculation. 
                
                // However, since we added "readRawTemperature" overrides in the 
                // previous step, calling the standard .readTemperature() 
                // right now would trigger *another* blocking read. 
                
                // *OPTIMIZATION*: For this demo to be mathematically useful immediately,
                // let's just print the RAW values to prove the acquisition works.
                // In a production version, we would split the Calculate() logic 
                // from the Read() logic.
                
                Serial.print(F("[SENSOR] Raw T: "));
                Serial.print(rawTemp);
                Serial.print(F(" | Raw P: "));
                Serial.println(rawPress);

                // Reset state to start over
                currentState = STATE_IDLE; 
            }
            break;
    }
}

void runCriticalTask() {
    // Run this task every 1ms
    if (millis() - lastTaskTime >= 1) {
        lastTaskTime = millis();
        taskCounter++;

        // Every 100ms, print a dot to visualize "Aliveness"
        if (taskCounter % 100 == 0) {
            Serial.print("."); 
        }
        
        // Every 5000ms (5 seconds), print the task count stats
        // This proves we didn't miss cycles waiting for the sensor.
        if (taskCounter % 5000 == 0) {
            Serial.println();
            Serial.print(F("[SYSTEM] Heartbeat. Tasks executed: "));
            Serial.println(taskCounter);
        }
    }
}
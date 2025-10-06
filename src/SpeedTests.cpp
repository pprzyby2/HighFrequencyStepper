#include "SpeedTests.h"
#include "TestUtils.h"

void testHighSpeedAcceleration(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== High Speed Acceleration Test (up to 200kHz) ===");
    
    counter.resetPosition();
    stepper.enable();
    stepper.setDirection(true);
    
    // Test speeds from 1kHz to 400kHz
    uint32_t testSpeeds[] = {1000, 5000, 10000, 25000, 50000, 75000, 100000, 150000, 200000, 250000, 300000, 350000, 400000};
    int numSpeeds = sizeof(testSpeeds) / sizeof(testSpeeds[0]);
    
    Serial.println("Testing acceleration profile...");
    
    int passedSpeeds = 0;
    for (int i = 0; i < numSpeeds; i++) {
        uint32_t speed = testSpeeds[i];
        Serial.print("Setting speed to: "); Serial.print(speed); Serial.println(" Hz");
        
        int32_t startPos = counter.getPosition();
        uint32_t startTime = millis();        
        
        stepper.startPWM(speed);
        delay(1000); // Run for 1 second at each speed
        stepper.stopPWM();
        
        uint32_t endTime = millis();
        int32_t endPos = counter.getPosition();
        
        uint32_t duration = endTime - startTime;
        int32_t stepsCounted = endPos - startPos;
        uint32_t expectedSteps = (speed * duration) / 1000;
        float accuracy = (float)stepsCounted / expectedSteps * 100.0;
        
        Serial.print("  Duration: "); Serial.print(duration); Serial.print("ms");
        Serial.print(" | Expected steps: "); Serial.print(expectedSteps);
        Serial.print(" | Counted: "); Serial.print(stepsCounted);
        Serial.print(" | Accuracy: "); Serial.print(accuracy, 1); Serial.println("%");
        
        bool speedTestOK = (accuracy >= 90.0);
        if (speedTestOK) passedSpeeds++;
        
        String speedName = "Speed " + String(speed/1000) + "kHz";
        addTestResult(speedName, speedTestOK, 
                      "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                      accuracy);
        
        if (accuracy < 90.0) {
            Serial.println("  WARNING: Low accuracy detected!");
        }
        
        delay(200); // Brief pause between speeds
    }
    
    stepper.stopPWM();

    // Overall high speed test result
    bool overallHighSpeedOK = (passedSpeeds >= 7); // At least 7/13 speeds must pass
    addTestResult("High Speed Overall", overallHighSpeedOK, 
                  String(passedSpeeds) + "/" + String(numSpeeds) + " speeds passed");
    
    Serial.println("High speed test completed!");
    Serial.print("Total steps moved: "); Serial.println(counter.getPosition());
}

void testLowSpeedPrecision(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== Low Speed Precision Test (Timer Mode) ===");
    
    counter.resetPosition();
    stepper.enable();
    stepper.setDirection(true);
    
    // Test very low frequencies (should use Timer mode)
    float testFreqs[] = {1.0, 5.0, 10.0, 50.0, 100.0, 200.0, 500.0};
    int numFreqs = sizeof(testFreqs) / sizeof(testFreqs[0]);
    
    Serial.println("Testing low speed precision...");
    
    int passedLowSpeeds = 0;
    for (int i = 0; i < numFreqs; i++) {
        float freq = testFreqs[i];
        Serial.print("Testing low speed: "); Serial.print(freq, 1); Serial.println(" Hz");
        
        uint32_t freqHz = (uint32_t)freq;
        int32_t startPos = counter.getPosition();
        
        stepper.startPWM(freqHz);
        delay(5000); // Run for 5 seconds
        stepper.stopPWM();
        
        int32_t endPos = counter.getPosition();
        int32_t stepsCounted = endPos - startPos;
        uint32_t expectedSteps = freqHz * 5; // 5 seconds
        float accuracy = (float)stepsCounted / expectedSteps * 100.0;
        bool testPassed = (accuracy >= 95.0); // Higher accuracy expected for low speeds
        
        Serial.print("  Expected: "); Serial.print(expectedSteps);
        Serial.print(" steps, Counted: "); Serial.println(stepsCounted);
        Serial.print("  Accuracy: "); Serial.print(accuracy, 1); Serial.println("%");
        Serial.print("  Mode: "); Serial.println(stepper.isInLEDCMode() ? "LEDC" : "Timer");
        
        String freqName = "Low speed " + String(freq, 1) + "Hz";
        addTestResult(freqName, testPassed, 
                      "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                      accuracy);
        
        if (testPassed) passedLowSpeeds++;
        delay(1000);
    }
    
    // Overall low speed test result
    bool overallLowSpeedOK = (passedLowSpeeds >= 5); // At least 5/7 speeds must pass
    addTestResult("Low Speed Overall", overallLowSpeedOK, 
                  String(passedLowSpeeds) + "/" + String(numFreqs) + " speeds passed");
    
    Serial.println("Low speed test completed!");
    Serial.print("Total position: "); Serial.println(counter.getPosition());
}

void demonstrateSpeedMeasurement(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== Speed Measurement Demo ===");
    
    stepper.enable();
    
    // Test different speeds
    uint32_t testSpeeds[] = {200, 500, 1000, 1500, 2000};
    
    for (int i = 0; i < 5; i++) {
        uint32_t setSpeed = testSpeeds[i];
        Serial.print("Setting speed to: "); Serial.print(setSpeed); Serial.println(" Hz");
        
        stepper.setDirection(i % 2 == 0); // Alternate direction
        stepper.startPWM(setSpeed);
        
        delay(1000); // Let it stabilize
        
        // Measure speed over 2 seconds
        for (int j = 0; j < 10; j++) {
            int32_t measuredSpeed = counter.getStepsPerSecond(200);
            if (measuredSpeed != 0) {
                Serial.printf("Steps: %d Measured: %d Hz | Error: %d Hz\n", measuredSpeed, abs(measuredSpeed), abs(measuredSpeed) - setSpeed);
            }
            delay(200);
        }
        
        stepper.stopPWM();
        delay(500);
    }
}
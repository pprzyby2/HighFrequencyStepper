#include "SpeedTests.h"
#include "TestUtils.h"
#include "HighFrequencyStepper.h"

void testHighSpeedAcceleration(HighFrequencyStepper& controller, uint8_t index) {
    Serial.printf("\n=== High Speed Acceleration Test (up to %fkHz) ===\n", controller.getMaxFrequency(index) / 1000);
    
    controller.setPosition(index, 0);
    controller.enableStepper(index);
    bool dir = true;
    
    // Test speeds from 1kHz to 400kHz
    
    Serial.println("Testing acceleration profile...");
    
    int passedSpeeds = 0;
    int numSpeeds = 0;
    for (int i = 5; i < 100; i += 5) {
        numSpeeds++;
        double speed = controller.getMaxFrequency(index) * i / 100.0; // From 5% to 100%
        Serial.printf("Setting speed to: %.2f Hz\n", speed);


        controller.accelerateToFrequency(index, speed, dir, true);
        int32_t startPos = controller.getPosition(index);
        uint32_t startTime = millis();
        delay(1000); // Run for 1 second at each speed

        uint32_t endTime = millis();
        int32_t endPos = controller.getPosition(index);
        
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

    controller.stop(index);

    // Overall high speed test result
    bool overallHighSpeedOK = (passedSpeeds >= 7); // At least 7/13 speeds must pass
    addTestResult("High Speed Overall", overallHighSpeedOK, 
                  String(passedSpeeds) + "/" + String(numSpeeds) + " speeds passed");
    
    Serial.println("High speed test completed!");
    Serial.print("Total steps moved: "); Serial.println(controller.getPosition(index));
}

void testHighSpeedAcceleration(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        testHighSpeedAcceleration(controller, i);
    }
}

void testLowSpeedPrecision(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Low Speed Precision Test (Timer Mode) ===");

    controller.setPosition(index, 0);
    controller.enableStepper(index);
    bool dir = true;
    
    // Test very low frequencies (should use Timer mode)
    float testFreqs[] = {1.0, 5.0, 10.0, 50.0, 100.0, 200.0, 500.0};
    int numFreqs = sizeof(testFreqs) / sizeof(testFreqs[0]);
    
    Serial.println("Testing low speed precision...");
    
    int passedLowSpeeds = 0;
    for (int i = 0; i < numFreqs; i++) {
        float freq = testFreqs[i];
        Serial.print("Testing low speed: "); Serial.print(freq, 1); Serial.println(" Hz");
        
        uint32_t freqHz = (uint32_t)freq;
        int32_t startPos = controller.getPosition(index);

        controller.startContinuous(index, freqHz, dir);
        delay(5000); // Run for 5 seconds
        controller.stop(index);

        int32_t endPos = controller.getPosition(index);
        int32_t stepsCounted = endPos - startPos;
        uint32_t expectedSteps = freqHz * 5; // 5 seconds
        float accuracy = (float)stepsCounted / expectedSteps * 100.0;
        bool testPassed = (accuracy >= 95.0); // Higher accuracy expected for low speeds
        
        Serial.print("  Expected: "); Serial.print(expectedSteps);
        Serial.print(" steps, Counted: "); Serial.println(stepsCounted);
        Serial.print("  Accuracy: "); Serial.print(accuracy, 1); Serial.println("%");
        Serial.print("  Mode: "); Serial.println(controller.isInLEDCMode(index) ? "LEDC" : "Timer");
        
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
    Serial.print("Total position: "); Serial.println(controller.getPosition(index));
}

void testLowSpeedPrecision(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        testLowSpeedPrecision(controller, i);
    }
}

void demonstrateSpeedMeasurement(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Speed Measurement Demo ===");

    controller.enableStepper(index);

    // Test different speeds
    uint32_t testSpeeds[] = {200, 500, 1000, 1500, 2000};
    
    for (int i = 0; i < 5; i++) {
        uint32_t setSpeed = testSpeeds[i];
        Serial.print("Setting speed to: "); Serial.print(setSpeed); Serial.println(" Hz");
        
        controller.startContinuous(index, setSpeed, i % 2 == 0);
        
        delay(1000); // Let it stabilize
        
        // Measure speed over 2 seconds
        for (int j = 0; j < 10; j++) {
            int64_t startTime = millis();
            int32_t startPos = controller.getPosition(index);
            delay(200);
            int32_t endPos = controller.getPosition(index);
            int64_t endTime = millis();
            int32_t measuredSpeed = (int32_t)((endPos - startPos) * 1000 / (endTime - startTime));
            if (measuredSpeed != 0) {
                Serial.printf("Steps: %d Measured: %d Hz | Error: %d Hz\n", measuredSpeed, abs(measuredSpeed), abs(measuredSpeed) - setSpeed);
            }
            delay(200);
        }

        controller.stop(index);
        delay(500);
    }
}

void demonstrateSpeedMeasurement(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        demonstrateSpeedMeasurement(controller, i);
    }
}

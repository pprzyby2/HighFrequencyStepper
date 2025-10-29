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
        addTestResult(controller.getName(index), speedName, speedTestOK, 
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
    addTestResult(controller.getName(index), "High Speed Overall", overallHighSpeedOK, 
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
    int toleranceSteps = 2 * controller.getConfig(index).encoderToMicrostepRatio; // Allowable error within one encoder count
    
    // Test very low frequencies (should use Timer mode)
    float testFreqs[] = {2.0, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 200.0, 500.0};
    int testTimes[] = {30, 30, 30, 20, 10, 5, 5, 5, 5}; // seconds for each frequency
    int numFreqs = sizeof(testFreqs) / sizeof(testFreqs[0]);
    
    Serial.println("Testing low speed precision...");
    
    int passedLowSpeeds = 0;
    for (int i = 0; i < numFreqs; i++) {
        float freq = testFreqs[i];
        int testTime = testTimes[i];
        Serial.printf("Testing low speed: %.1f Hz for %d seconds\n", freq, testTime);
        
        //controller.moveAtFrequency(index, freqHz, dir);
        controller.accelerateToFrequency(index, freq, dir, true);
        delay(5000); // Allow time to stabilize encoder reading
        uint32_t startTime = millis();
        int32_t startPos = controller.getPosition(index);
        for (int t = 0; t < testTime; t++) {
            delay(1000);
            Serial.printf("  Time: %d ms, Current frequency: %.2f Hz\n", (millis() - startTime), (controller.getPosition(index) - startPos) * 1000.0 / (1 + millis() - startTime));
            //controller.printStatus(index);
        }
        int32_t endPos = controller.getPosition(index);
        uint32_t endTime = millis();
        controller.stop(index);

        int32_t stepsCounted = abs(endPos - startPos);
        float expectedSteps = freq * (endTime - startTime) / 1000.0; // Expected steps for the test duration
        int32_t error = abs(expectedSteps - stepsCounted);
        bool testPassed = (error <= toleranceSteps) || (error / expectedSteps < 0.05); // Allowable error within one encoder count
        float accuracy = (float)stepsCounted / expectedSteps * 100.0;

        Serial.printf("  Expected:  %.2f steps, Counted: %d steps\n", expectedSteps, stepsCounted);
        Serial.printf("  Error: %d steps with tolerance: %d steps\n", error, max(toleranceSteps, (int)(expectedSteps * 0.05)));
        Serial.printf("  Mode: %s\n", controller.isInLEDCMode(index) ? "LEDC" : "Timer");

        String freqName = "Low speed " + String(freq, 1) + "Hz";
        addTestResult(controller.getName(index), freqName, testPassed, 
                      "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                      accuracy);
        
        if (testPassed) passedLowSpeeds++;
        delay(1000);
    }
    
    // Overall low speed test result
    bool overallLowSpeedOK = (passedLowSpeeds >= 5); // At least 5/7 speeds must pass
    addTestResult(controller.getName(index), "Low Speed Overall", overallLowSpeedOK, 
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
        
        controller.moveAtFrequency(index, setSpeed, i % 2 == 0);
        
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

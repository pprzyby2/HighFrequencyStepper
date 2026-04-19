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


        controller.accelerateToFrequency(index, speed * (dir ? 1 : -1), true);
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
    
    cleanupAfterTest(controller);
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
        controller.accelerateToFrequency(index, freq * (dir ? 1 : -1), true);
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
    
    cleanupAfterTest(controller);
}

void demonstrateSpeedMeasurement(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Speed Measurement Demo ===");

    controller.enableStepper(index);

    // Test different speeds
    uint32_t testSpeeds[] = {200, 500, 1000, 1500, 2000};
    
    for (int i = 0; i < 5; i++) {
        double setSpeed = testSpeeds[i];
        Serial.printf("Setting speed to: %.2f Hz\n", setSpeed);
        
        controller.moveAtFrequency(index, setSpeed * (i % 2 == 0 ? 1 : -1));
        
        delay(1000); // Let it stabilize
        
        // Measure speed over 2 seconds
        for (int j = 0; j < 10; j++) {
            int64_t startTime = millis();
            int32_t startPos = controller.getPosition(index);
            delay(200);
            int32_t endPos = controller.getPosition(index);
            int64_t endTime = millis();
            double measuredSpeed = (double)(endPos - startPos) * 1000.0 / (endTime - startTime);
            if (measuredSpeed != 0) {
                Serial.printf("Steps: %d Measured: %.2f Hz | Error: %.2f Hz\n", measuredSpeed, abs(measuredSpeed), abs(measuredSpeed) - setSpeed);
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
    
    cleanupAfterTest(controller);
}

/**
 * Test that multiple motors can run simultaneously at different speeds.
 * This verifies that different LEDC timers are used for each motor.
 * 
 * If all motors share the same timer, changing one motor's speed would
 * affect the other motors' speeds as well.
 */
void testMultiMotorIndependentSpeeds(HighFrequencyStepper& controller) {
    Serial.println("\n=== Multi-Motor Independent Speed Test (LEDC Timer Test) ===");
    
    uint8_t stepperCount = controller.getStepperCount();
    if (stepperCount < 2) {
        Serial.println("ERROR: This test requires at least 2 steppers.");
        addTestResult("System", "Multi-Motor Speed Test", false, "Requires at least 2 steppers");
        return;
    }
    
    Serial.printf("Testing %d motors running at different speeds simultaneously...\n", stepperCount);
    
    // Define different speeds for each motor (in Hz)
    double testSpeeds[] = {1000.0, 2000.0, 3000.0, 4000.0};  // Different speeds for up to 4 motors
    double measuredSpeeds[MAX_STEPPERS] = {0};
    int32_t startPositions[MAX_STEPPERS] = {0};
    int32_t endPositions[MAX_STEPPERS] = {0};
    
    // Reset positions and enable all steppers
    for (uint8_t i = 0; i < stepperCount; i++) {
        controller.setPosition(i, 0);
        controller.enableStepper(i);
    }
    
    // Start all motors at different speeds
    Serial.println("\nStarting motors at different frequencies:");
    for (uint8_t i = 0; i < stepperCount; i++) {
        double targetSpeed = testSpeeds[i % 4];
        // Clamp to max frequency if needed
        if (targetSpeed > controller.getMaxFrequency(i)) {
            targetSpeed = controller.getMaxFrequency(i) * 0.5;
        }
        testSpeeds[i] = targetSpeed;
        
        Serial.printf("  Motor %d (%s): %.0f Hz\n", i, controller.getName(i).c_str(), targetSpeed);
        controller.moveAtFrequency(i, targetSpeed);
    }
    
    // Let motors stabilize
    Serial.println("\nWaiting for motors to stabilize...");
    delay(2000);
    
    // Record start positions
    for (uint8_t i = 0; i < stepperCount; i++) {
        startPositions[i] = controller.getPosition(i);
    }
    
    // Run for test duration
    int testDurationMs = 5000;
    Serial.printf("\nRunning for %d ms...\n", testDurationMs);
    
    uint32_t startTime = millis();
    
    // Periodically print status during test
    for (int t = 0; t < testDurationMs / 1000; t++) {
        delay(1000);
        Serial.printf("  Time: %ds - ", t + 1);
        for (uint8_t i = 0; i < stepperCount; i++) {
            int32_t pos = controller.getPosition(i);
            Serial.printf("M%d: %d ", i, pos);
        }
        Serial.println();
    }
    
    uint32_t endTime = millis();
    uint32_t actualDuration = endTime - startTime;
    
    // Record end positions
    for (uint8_t i = 0; i < stepperCount; i++) {
        endPositions[i] = controller.getPosition(i);
    }
    
    // Stop all motors
    controller.stopAll();
    
    // Calculate and verify measured speeds
    Serial.println("\n=== Results ===");
    Serial.printf("Actual test duration: %d ms\n\n", actualDuration);
    
    bool allPassed = true;
    int passedCount = 0;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        int32_t stepsMoved = abs(endPositions[i] - startPositions[i]);
        measuredSpeeds[i] = (double)stepsMoved * 1000.0 / actualDuration;
        
        double expectedSpeed = testSpeeds[i];
        double speedError = abs(measuredSpeeds[i] - expectedSpeed);
        double errorPercent = (speedError / expectedSpeed) * 100.0;
        
        // Allow 10% error tolerance
        bool passed = (errorPercent <= 10.0);
        
        Serial.printf("Motor %d (%s):\n", i, controller.getName(i).c_str());
        Serial.printf("  Target speed:   %.0f Hz\n", expectedSpeed);
        Serial.printf("  Measured speed: %.0f Hz\n", measuredSpeeds[i]);
        Serial.printf("  Error:          %.1f%% %s\n", errorPercent, passed ? "PASS" : "FAIL");
        Serial.printf("  Steps moved:    %d\n\n", stepsMoved);
        
        if (passed) passedCount++;
        if (!passed) allPassed = false;
        
        String testName = "Motor " + String(i) + " Independent Speed";
        addTestResult(controller.getName(i), testName, passed,
                      "Target: " + String(expectedSpeed, 0) + " Hz, Measured: " + String(measuredSpeeds[i], 0) + " Hz",
                      100.0 - errorPercent);
    }
    
    // Check speed ratios between motors to verify independence
    Serial.println("=== Speed Ratio Verification ===");
    bool ratiosCorrect = true;
    
    for (uint8_t i = 0; i < stepperCount - 1; i++) {
        for (uint8_t j = i + 1; j < stepperCount; j++) {
            double expectedRatio = testSpeeds[i] / testSpeeds[j];
            double measuredRatio = measuredSpeeds[i] / measuredSpeeds[j];
            double ratioError = abs(measuredRatio - expectedRatio) / expectedRatio * 100.0;
            
            bool ratioOk = (ratioError <= 15.0);  // 15% tolerance for ratio
            
            Serial.printf("  M%d/M%d ratio - Expected: %.2f, Measured: %.2f, Error: %.1f%% %s\n",
                          i, j, expectedRatio, measuredRatio, ratioError, ratioOk ? "OK" : "FAIL");
            
            if (!ratioOk) ratiosCorrect = false;
        }
    }
    
    // Overall test result
    bool overallPassed = allPassed && ratiosCorrect;
    
    Serial.println("\n=== Summary ===");
    Serial.printf("Motors tested: %d\n", stepperCount);
    Serial.printf("Speed tests passed: %d/%d\n", passedCount, stepperCount);
    Serial.printf("Speed ratios correct: %s\n", ratiosCorrect ? "YES" : "NO");
    Serial.printf("Overall result: %s\n", overallPassed ? "PASS" : "FAIL");
    
    if (!overallPassed) {
        Serial.println("\nWARNING: If motors have similar measured speeds regardless of target,");
        Serial.println("         it may indicate they are sharing the same LEDC timer.");
    }
    
    addTestResult("System", "Multi-Motor Independent Speeds Overall", overallPassed,
                  String(passedCount) + "/" + String(stepperCount) + " motors passed, ratios " + (ratiosCorrect ? "OK" : "FAIL"));
    
    cleanupAfterTest(controller);
}

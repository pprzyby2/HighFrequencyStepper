#include "RapidTargetChangeTests.h"
#include "TestUtils.h"

/**
 * @brief Test rapid target position changes while motor is moving
 * 
 * This test starts moving to a position, then rapidly changes the target
 * multiple times to test the stepper's responsiveness and stability.
 */
void testRapidTargetChanges(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Rapid Target Position Change Test ===");
    Serial.println("Testing stepper response to rapid target position updates...\n");

    for (int index = 0; index < stepper.getStepperCount(); index++) {
        String stepperName = stepper.getName(index);
        Serial.printf("\n--- Testing %s (Index %d) ---\n", stepperName.c_str(), index);
        
        // Reset position and enable stepper
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        delay(100);
        
        int32_t stepsPerRev = stepper.getMicrostepsPerRevolution(index);
        double maxFreq = stepper.getMaxFrequency(index);
        double testFreq = maxFreq; // Use max frequency for testing
        
        Serial.printf("Steps per revolution: %d\n", stepsPerRev);
        Serial.printf("Test frequency: %.2f Hz\n", testFreq);
        
        // ===========================================
        // Test 1: Rapid forward target changes
        // ===========================================
        Serial.println("\n[Test 1] Rapid Forward Target Changes");
        Serial.println("Starting move to position and updating target rapidly...");
        
        int32_t initialTarget = 10 * stepsPerRev ;  // 10 revolutions
        int32_t finalTarget = 5 * stepsPerRev;    // 5 revolutions
        int numChanges = 10;
        int changeInterval = 500;  // ms between target changes
        
        // Start moving (non-blocking)
        stepper.moveToPosition(index, initialTarget, testFreq, false, false);
        
        // Rapidly update target position while moving
        uint32_t startTime = millis();
        for (int i = 1; i <= numChanges; i++) {
            delay(changeInterval);
            int32_t intermediateTarget = initialTarget + (finalTarget - initialTarget) * i / numChanges;
            stepper.moveToPosition(index, intermediateTarget, testFreq, false, false);
            Serial.printf("  [%d ms] Target changed to: %d\n", millis() - startTime, intermediateTarget);
        }
        
        // Wait for motor to reach final position
        uint32_t timeout = 10000;
        uint32_t waitStart = millis();
        while (stepper.isMoving(index) && (millis() - waitStart < timeout)) {
            delay(10);
        }
        
        int32_t finalPos = stepper.getPosition(index);
        int32_t error = abs(finalTarget - finalPos);
        float accuracy = 100.0 * (1.0 - (float)error / finalTarget);
        bool passed = error <= stepsPerRev / 100; // Within 1% tolerance (3.6 degrees)
        
        Serial.printf("Final target: %d, Actual position: %d, Error: %d steps\n", finalTarget, finalPos, error);
        Serial.printf("Accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Rapid Forward Target Changes", passed,
                     "Target: " + String(finalTarget) + ", Actual: " + String(finalPos) + ", Error: " + String(error), accuracy);
        
        delay(500);
        
        // ===========================================
        // Test 2: Rapid direction reversals
        // ===========================================
        Serial.println("\n[Test 2] Rapid Direction Reversals");
        Serial.println("Changing direction rapidly while moving...");
        
        stepper.setPosition(index, 0);
        delay(100);
        
        int32_t pos1 = stepsPerRev / 4;   // +90 degrees
        int32_t pos2 = -stepsPerRev / 4;  // -90 degrees
        int reversalCount = 5;
        
        startTime = millis();
        
        for (int i = 0; i < reversalCount; i++) {
            // Move forward
            stepper.moveToPosition(index, pos1, testFreq, false, false);
            Serial.printf("  [%d ms] Target: +%d (forward)\n", millis() - startTime, pos1);
            delay(100);  // Brief delay before reversal
            
            // Move backward
            stepper.moveToPosition(index, pos2, testFreq, false, false);
            Serial.printf("  [%d ms] Target: %d (backward)\n", millis() - startTime, pos2);
            delay(100);
        }
        
        // Final target is pos2 (-90 degrees), wait for completion
        int32_t reversalFinalTarget = pos2;
        stepper.moveToPosition(index, reversalFinalTarget, testFreq, true, false);
        
        finalPos = stepper.getPosition(index);
        error = abs(reversalFinalTarget - finalPos);
        accuracy = 100.0 * (1.0 - (float)abs(error) / abs(reversalFinalTarget));
        passed = error <= stepsPerRev / 50; // Within 2% tolerance
        
        Serial.printf("Final target: %d, Actual position: %d, Error: %d steps\n", reversalFinalTarget, finalPos, error);
        Serial.printf("Accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Rapid Direction Reversals", passed,
                     "Target: " + String(reversalFinalTarget) + ", Actual: " + String(finalPos), accuracy);
        
        delay(500);
        
        // ===========================================
        // Test 3: Very rapid (burst) target changes
        // ===========================================
        Serial.println("\n[Test 3] Burst Target Changes (no delay between updates)");
        Serial.println("Sending multiple target updates in rapid succession...");
        
        stepper.setPosition(index, 0);
        delay(100);
        
        int32_t burstFinalTarget = stepsPerRev;  // Full revolution
        int burstCount = 20;
        
        startTime = millis();
        
        // Send rapid burst of position updates with no delay
        for (int i = 1; i <= burstCount; i++) {
            int32_t burstTarget = (burstFinalTarget * i) / burstCount;
            stepper.moveToPosition(index, burstTarget, testFreq, false, false);
        }
        
        uint32_t burstTime = millis() - startTime;
        Serial.printf("  Sent %d target updates in %d ms\n", burstCount, burstTime);
        
        // Wait for completion
        stepper.moveToPosition(index, burstFinalTarget, testFreq, true, false);
        
        finalPos = stepper.getPosition(index);
        error = abs(burstFinalTarget - finalPos);
        accuracy = 100.0 * (1.0 - (float)error / burstFinalTarget);
        passed = error <= stepsPerRev / 50; // Within 2% tolerance
        
        Serial.printf("Final target: %d, Actual position: %d, Error: %d steps\n", burstFinalTarget, finalPos, error);
        Serial.printf("Accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Burst Target Changes", passed,
                     "Target: " + String(burstFinalTarget) + ", Actual: " + String(finalPos), accuracy);
        
        // Reset position
        stepper.stop(index);
        stepper.setPosition(index, 0);
        delay(500);
    }
    
    Serial.println("\n=== Rapid Target Change Test Complete ===\n");
}

/**
 * @brief Test rapid back-and-forth oscillation
 */
void testRapidOscillation(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Rapid Oscillation Test ===");
    Serial.println("Testing rapid back-and-forth movement with decreasing intervals...\n");
    
    for (int index = 0; index < stepper.getStepperCount(); index++) {
        String stepperName = stepper.getName(index);
        Serial.printf("\n--- Testing %s (Index %d) ---\n", stepperName.c_str(), index);
        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        delay(100);
        
        int32_t stepsPerRev = stepper.getMicrostepsPerRevolution(index);
        double testFreq = stepper.getMaxFrequency(index) / 2;
        
        int32_t amplitude = stepsPerRev / 8;  // 45 degrees oscillation
        int oscillations = 8;
        int initialInterval = 500;  // Start with 500ms intervals
        int minInterval = 50;       // Decrease to 50ms intervals
        
        Serial.printf("Oscillation amplitude: %d steps (45 degrees)\n", amplitude);
        Serial.printf("Oscillations: %d, Interval: %d -> %d ms\n", oscillations, initialInterval, minInterval);
        
        uint32_t startTime = millis();
        int totalMoves = 0;
        
        for (int i = 0; i < oscillations; i++) {
            // Calculate decreasing interval
            int interval = initialInterval - (initialInterval - minInterval) * i / (oscillations - 1);
            
            // Move to positive amplitude
            stepper.moveToPosition(index, amplitude, testFreq, false, false);
            totalMoves++;
            delay(interval);
            
            // Move to negative amplitude
            stepper.moveToPosition(index, -amplitude, testFreq, false, false);
            totalMoves++;
            delay(interval);
            
            Serial.printf("  Oscillation %d/%d, interval: %d ms, current pos: %d\n", 
                         i + 1, oscillations, interval, stepper.getPosition(index));
        }
        
        // Return to center
        stepper.moveToPosition(index, 0, testFreq, true, false);
        
        uint32_t totalTime = millis() - startTime;
        int32_t finalPos = stepper.getPosition(index);
        int32_t error = abs(finalPos);
        
        bool passed = error <= stepsPerRev / 100; // Within 1% (should return to 0)
        float accuracy = 100.0 - (float)error / amplitude * 100.0;
        
        Serial.printf("Total time: %d ms, Total moves: %d\n", totalTime, totalMoves);
        Serial.printf("Final position (should be 0): %d, Error: %d steps\n", finalPos, error);
        Serial.printf("Return accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Rapid Oscillation", passed,
                     "Final pos: " + String(finalPos) + ", Error: " + String(error), accuracy);
        
        stepper.stop(index);
        delay(500);
    }
    
    Serial.println("\n=== Rapid Oscillation Test Complete ===\n");
}

/**
 * @brief Test chasing a continuously changing target
 */
void testChasingTarget(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Chasing Target Test ===");
    Serial.println("Simulating a continuously moving target that the stepper must track...\n");
    
    for (int index = 0; index < stepper.getStepperCount(); index++) {
        String stepperName = stepper.getName(index);
        Serial.printf("\n--- Testing %s (Index %d) ---\n", stepperName.c_str(), index);
        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        delay(100);
        
        int32_t stepsPerRev = stepper.getMicrostepsPerRevolution(index);
        double testFreq = stepper.getMaxFrequency(index);
        
        // Simulate chasing a target that moves in a sine wave pattern
        int32_t amplitude = stepsPerRev / 4;  // 90 degree amplitude
        int updateInterval = 20;              // Update target every 20ms
        int testDuration = 3000;              // 3 second test
        float frequency = 0.5;                // 0.5 Hz sine wave
        
        Serial.printf("Target motion: Sine wave, amplitude: %d steps, frequency: %.2f Hz\n", amplitude, frequency);
        Serial.printf("Update interval: %d ms, Test duration: %d ms\n", updateInterval, testDuration);
        
        uint32_t startTime = millis();
        int32_t maxError = 0;
        int64_t sumError = 0;
        int updateCount = 0;
        
        while (millis() - startTime < testDuration) {
            // Calculate target position based on sine wave
            float elapsed = (millis() - startTime) / 1000.0;  // Time in seconds
            int32_t targetPos = (int32_t)(amplitude * sin(2.0 * PI * frequency * elapsed));
            
            // Update target position
            stepper.moveToPosition(index, targetPos, testFreq, false, false);
            
            // Track error
            int32_t currentPos = stepper.getPosition(index);
            int32_t error = abs(targetPos - currentPos);
            if (error > maxError) maxError = error;
            sumError += error;
            updateCount++;
            
            delay(updateInterval);
        }
        
        // Stop and measure final position
        stepper.stop(index);
        
        // Calculate expected final position (sine at testDuration)
        float finalTime = testDuration / 1000.0;
        int32_t expectedFinalPos = (int32_t)(amplitude * sin(2.0 * PI * frequency * finalTime));
        int32_t finalPos = stepper.getPosition(index);
        int32_t finalError = abs(expectedFinalPos - finalPos);
        
        float avgError = (float)sumError / updateCount;
        float maxErrorDegrees = (float)maxError / stepsPerRev * 360.0;
        float avgErrorDegrees = avgError / stepsPerRev * 360.0;
        
        Serial.printf("Updates sent: %d\n", updateCount);
        Serial.printf("Max tracking error: %d steps (%.2f degrees)\n", maxError, maxErrorDegrees);
        Serial.printf("Average tracking error: %.1f steps (%.2f degrees)\n", avgError, avgErrorDegrees);
        Serial.printf("Final position error: %d steps\n", finalError);
        
        // Pass if average error is less than 5 degrees
        bool passed = avgErrorDegrees < 5.0;
        float accuracy = 100.0 - avgErrorDegrees / 90.0 * 100.0; // Relative to 90 degree amplitude
        
        Serial.printf("Tracking performance: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Chasing Target (Sine Wave)", passed,
                     "Avg error: " + String(avgErrorDegrees, 2) + " deg, Max: " + String(maxErrorDegrees, 2) + " deg", accuracy);
        
        // Reset position
        stepper.setPosition(index, 0);
        delay(500);
    }
    
    Serial.println("\n=== Chasing Target Test Complete ===\n");
}

void testLongRun(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Long Run Stability Test ===");
    Serial.println("Running the stepper continuously for an extended period to check for stability and overheating...\n");
    
    for (int index = 0; index < stepper.getStepperCount(); index++) {
        String stepperName = stepper.getName(index);
        Serial.printf("\n--- Testing %s (Index %d) ---\n", stepperName.c_str(), index);
        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        delay(100);
        
        int32_t stepsPerRev = stepper.getMicrostepsPerRevolution(index);
        double testFreq = stepper.getMaxFrequency(index) / 2;
        
        int32_t targetPos = 2312696; 
        uint32_t runDuration = 90000; // Run for 90 seconds
        
        Serial.printf("Target position: %d steps (%d revolutions)\n", targetPos, targetPos / stepsPerRev);
        Serial.printf("Test duration: %d ms\n", runDuration);
        
        uint32_t startTime = millis();
        
        // Start moving to target position
        stepper.moveToPosition(index, targetPos, testFreq, false, false);
        
        // Monitor during run
        int stage = 0;
        while (millis() - startTime < runDuration && stepper.isMoving(index)) {
            int32_t currentPos = stepper.getPosition(index);
            Serial.printf("  Time: %d s, Current position: %d\n", (millis() - startTime) / 1000, currentPos);
            
            if (stage == 0 && currentPos > targetPos * 0.7) { // Once we are 70% of the way there, change target to 30% to force a rapid change in direction and test stability
                stepper.moveToPosition(index, targetPos * 0.3, testFreq, false, false);
                stage = 1;
            } else if (stage == 1 && currentPos < targetPos * 0.4) {
                stepper.moveToPosition(index, targetPos, testFreq, false, false);
                stage = 2;                
            }
            delay(500); // Log every 0.5 seconds
        }
        
        // Stop and check final position
        stepper.stop(index);
        
        int32_t finalPos = stepper.getPosition(index);
        int32_t error = abs(targetPos - finalPos);
        
        bool passed = error <= stepsPerRev / 50; // Within 2% tolerance
        float accuracy = 100.0 * (1.0 - (float)error / targetPos);
        
        Serial.printf("Final target: %d, Actual position: %d, Error: %d steps\n", targetPos, finalPos, error);
        Serial.printf("Long run accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Long Run Stability", passed,
                     "Target: " + String(targetPos) + ", Actual: " + String(finalPos) + ", Error: " + String(error), accuracy);
        
        Serial.printf("Target position: %d steps (%d revolutions)\n", targetPos, targetPos / stepsPerRev);
        Serial.printf("Test duration: %d ms\n", runDuration);
        
        startTime = millis();
        
        // Start moving to target position
        stepper.moveToPosition(index, 0, testFreq, false, false);
        
        // Monitor during run
        while (millis() - startTime < runDuration && stepper.isMoving(index)) {
            int32_t currentPos = stepper.getPosition(index);
            Serial.printf("  Time: %d s, Current position: %d\n", (millis() - startTime) / 1000, currentPos);
            stepper.moveToPosition(index, 0, testFreq, false, false);
            delay(500); // Log every 0.5 seconds
        }
        
        // Stop and check final position
        stepper.stop(index);
        
        finalPos = stepper.getPosition(index);
        error = abs(0 - finalPos);
        
        passed = error <= stepsPerRev / 50; // Within 2% tolerance
        accuracy = 100.0 * (1.0 - (float)error / stepsPerRev);
        
        Serial.printf("Final target: %d, Actual position: %d, Error: %d steps\n", 0, finalPos, error);
        Serial.printf("Long run accuracy: %.2f%% - %s\n", accuracy, passed ? "PASSED" : "FAILED");
        
        addTestResult(stepperName, "Long Run Stability", passed,
                     "Target: " + String(0) + ", Actual: " + String(finalPos) + ", Error: " + String(error), accuracy);

    }
}
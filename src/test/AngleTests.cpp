#include "AngleTests.h"
#include "TestUtils.h"


void testAnglePrecision(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Angle Precision Test ===");

    Serial.printf("Forward Rotation Test:\n");
    for (int index = 0; index < stepper.getStepperCount(); index++) { // Test all steppers        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        
        int totalSteps = stepper.getMicrostepsPerRevolution(index); // Full revolution in microsteps;
        int testFreq = stepper.getMaxFrequency(index) / 4; // Limit to quarter max for reliability

        Serial.printf("Testing full revolution (%d steps) at %d Hz...\n", totalSteps, testFreq);

        stepper.moveToPosition(index, totalSteps, testFreq); // Move full revolution at quarter max frequency

        int32_t finalPos = stepper.getPosition(index);

        Serial.printf("Expected: ~%d, Actual: %d\n", totalSteps, finalPos);

        // Evaluate test
        bool testPassed = (finalPos >= totalSteps * 0.95 && finalPos <= totalSteps * 1.05); // ±5% tolerance
        float accuracy = (float)(finalPos) / totalSteps * 100.0;
        addTestResult("Angle Precision", testPassed, 
                    "Expected: " + String(totalSteps) + ", Got: " + String(finalPos), accuracy);

        delay(1000);
    }
    Serial.printf("Backward Rotation Test:\n");
    for (int index = 0; index < stepper.getStepperCount(); index++) { // Test all steppers        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);
        
        int totalSteps = -stepper.getMicrostepsPerRevolution(index); // Full revolution in microsteps;
        int testFreq = stepper.getMaxFrequency(index) / 4; // Limit to quarter max for reliability

        Serial.printf("Testing full revolution (%d steps) at %d Hz...\n", totalSteps, testFreq);

        stepper.moveToPosition(index, totalSteps, testFreq); // Move full revolution at quarter max frequency

        int32_t finalPos = stepper.getPosition(index);

        Serial.printf("Expected: ~%d, Actual: %d\n", totalSteps, finalPos);

        // Evaluate test
        bool testPassed = (finalPos >= totalSteps * 0.95 && finalPos <= totalSteps * 1.05); // ±5% tolerance
        float accuracy = (float)(finalPos) / totalSteps * 100.0;
        addTestResult("Angle Precision", testPassed, 
                    "Expected: " + String(totalSteps) + ", Got: " + String(finalPos), accuracy);

        delay(1000);
    }    
}
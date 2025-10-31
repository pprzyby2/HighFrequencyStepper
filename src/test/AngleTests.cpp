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

        stepper.moveToAngle(index, 360, testFreq, true); // Move full revolution at quarter max frequency

        int32_t finalPos = stepper.getPosition(index);

        Serial.printf("Expected: ~%d, Actual: %d\n", totalSteps, finalPos);

        // Evaluate test
        bool testPassed = (finalPos / totalSteps > 0.95 && finalPos / totalSteps < 1.05); // ±5% tolerance
        float accuracy = (float)(finalPos) / totalSteps * 100.0;
        addTestResult(stepper.getName(index), "Angle Precision (Forward)", testPassed,
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

        stepper.moveToAngleRelative(index, -360, testFreq, true); // Move full revolution at quarter max frequency

        int32_t finalPos = stepper.getPosition(index);

        Serial.printf("Expected: ~%d, Actual: %d\n", totalSteps, finalPos);

        // Evaluate test
        float accuracy = (float)(finalPos) / totalSteps * 100.0;
        bool testPassed = (accuracy > 95.0 && accuracy < 105.0); // ±5% tolerance
        addTestResult(stepper.getName(index), "Angle Precision (Backward)", testPassed,
                    "Expected: " + String(totalSteps) + ", Got: " + String(finalPos), accuracy);

        delay(1000);
    }

    Serial.printf("Forward Angle Test:\n");
    for (int index = 0; index < stepper.getStepperCount(); index++) { // Test all steppers        
        stepper.setPosition(index, 0);
        stepper.enableStepper(index);

        Serial.printf("Testing two revolutions (%f degrees) at %d Hz...\n", 720.0, stepper.getMaxFrequency(index));

        stepper.moveToAngle(index, 720, stepper.getMaxFrequency(index), true); // Move full revolution at quarter max frequency

        double finalPos = stepper.getAngle(index);

        Serial.printf("Expected: ~%f, Actual: %f\n", 720.0, finalPos);

        // Evaluate test
        bool testPassed = (finalPos / 720.0 > 0.95 && finalPos / 720.0 < 1.05); // ±5% tolerance
        float accuracy = (float)(finalPos) / 720.0 * 100.0;
        addTestResult(stepper.getName(index), "Angle Precision (Degrees)", testPassed,
                    "Expected: " + String(720.0) + ", Got: " + String(finalPos), accuracy);

        delay(1000);
    }    
}
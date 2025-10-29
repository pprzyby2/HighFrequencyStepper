#include "DirectionTests.h"
#include "TestUtils.h"
#include "HighFrequencyStepper.h"

void testDirectionChanges(HighFrequencyStepper& stepper, int index) {
    Serial.println("\n=== Direction Change Test ===");

    stepper.setPosition(index, 0);
    stepper.enableStepper(index);
    
    // Test 1: Forward direction
    int fullCircleSteps = stepper.getMicrostepsPerRevolution(index);
    int testFreq = stepper.getMaxFrequency(index) / 4; // Limit to quarter max for reliability
    Serial.printf("Testing FORWARD direction, full circle (%d steps) at %d Hz...", fullCircleSteps, testFreq);

    stepper.moveToPosition(index, fullCircleSteps, testFreq); // Move full circle steps forward at quarter max frequency

    int32_t forwardPos = stepper.getPosition(index);

    Serial.printf("Expected: ~%d, Actual: %d\n", fullCircleSteps, forwardPos);
    Serial.printf("Direction: %s\n", forwardPos > 0 ? "Forward" : "Reverse");

    // Evaluate forward test
    bool forwardTest = (forwardPos >= fullCircleSteps * 0.95 && forwardPos <= fullCircleSteps * 1.05); // ±5% tolerance
    float forwardAccuracy = (float)(forwardPos) / fullCircleSteps * 100.0;
    addTestResult(stepper.getName(index), "Forward Direction", forwardTest, 
                  "Expected: " + String(fullCircleSteps) + ", Got: " + String(forwardPos), forwardAccuracy);

    delay(1000);
    
    // Test 2: Reverse direction
    stepper.setPosition(index, 0);
    Serial.printf("Testing REVERSE direction, full circle (-%d steps) at %d Hz...", fullCircleSteps, testFreq);
    stepper.moveToPosition(index, -fullCircleSteps, testFreq); // Move full circle steps backward at quarter max frequency

    int32_t reversePos = stepper.getPosition(index);

    Serial.printf("Expected: ~%d, Actual: %d\n", -fullCircleSteps, reversePos);
    Serial.printf("Direction detection: %s\n", reversePos < 0 ? "Forward" : "Reverse");

    // Evaluate reverse test
    bool reverseTest = (reversePos <= -fullCircleSteps * 0.95 && reversePos >= -fullCircleSteps * 1.05); // ±5% tolerance
    float reverseAccuracy = (float)abs(reversePos) / abs(-fullCircleSteps) * 100.0;
    addTestResult(stepper.getName(index), "Reverse Direction", reverseTest, 
                  "Expected: " + String(-fullCircleSteps) + ", Got: " + String(reversePos), reverseAccuracy);

    delay(1000);
    // Test 3: Multiple direction changes
    stepper.setPosition(index, 0);
    Serial.println("Testing rapid direction changes...");
    int passedChanges = 0;
    for (int i = 0; i < 5; i++) {
        bool dir = (i % 2 == 0);
        int startPos = stepper.getPosition(index);
        stepper.moveRelative(index, dir ? 200 : -200, testFreq); // Move 200 steps in chosen direction at testFreq
        delay(200); // 200 steps

        int32_t steps = stepper.getPosition(index) - startPos;
        int32_t expectedSteps = dir ? 200 : -200;
        bool changeOK = (abs(abs(steps) - 200) <= 20); // ±10% tolerance
        
        if (changeOK) passedChanges++;
        
        Serial.print("Change "); Serial.print(i+1); 
        Serial.print(" ("); Serial.print(dir ? "FWD" : "REV"); 
        Serial.print("): "); Serial.print(steps);
        Serial.println(changeOK ? " ✓" : " ✗");
        
        delay(100);
    }
    
    bool rapidChangesTest = (passedChanges >= 4); // At least 4/5 must pass
    addTestResult(stepper.getName(index), "Rapid Direction Changes", rapidChangesTest,
                  String(passedChanges) + "/5 changes successful");
    
    Serial.print("Final position after direction test: "); 
    Serial.println(stepper.getPosition(index));
}

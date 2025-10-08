#include "DirectionTests.h"
#include "TestUtils.h"
#include "HighFrequencyStepper.h"

void testDirectionChanges(HighFrequencyStepper& stepper, int index) {
    Serial.println("\n=== Direction Change Test ===");

    stepper.setPosition(index, 0);
    stepper.enableStepper(index);
    
    // Test 1: Forward direction
    Serial.println("Testing FORWARD direction (5000 steps)...");
    
    stepper.moveToPosition(index, 5000, 1500); // Move 5000 steps forward at 1500 Hz

    int32_t forwardPos = stepper.getPosition(index);

    Serial.printf("Expected: ~5000, Actual: %d\n", forwardPos);
    Serial.printf("Direction: %s\n", forwardPos > 0 ? "Forward" : "Reverse");

    // Evaluate forward test
    bool forwardTest = (forwardPos >= 4750 && forwardPos <= 5250); // ±5% tolerance
    float forwardAccuracy = (float)(forwardPos) / 5000.0 * 100.0;
    addTestResult("Forward Direction", forwardTest, 
                  "Expected: 5000, Got: " + String(forwardPos), forwardAccuracy);

    delay(1000);
    
    // Test 2: Reverse direction
    stepper.setPosition(index, 0);
    Serial.println("Testing REVERSE direction (-5000 steps)...");
    stepper.moveToPosition(index, -5000, 1500); // Move 5000 steps backward at 1500 Hz

    int32_t reversePos = stepper.getPosition(index);

    Serial.printf("Expected: ~-5000, Actual: %d\n", reversePos);
    Serial.printf("Direction detection: %s\n", reversePos < 0 ? "Forward" : "Reverse");

    // Evaluate reverse test
    bool reverseTest = (reversePos <= -4750 && reversePos >= -5250); // ±5% tolerance
    float reverseAccuracy = (float)abs(reversePos) / 5000.0 * 100.0;
    addTestResult("Reverse Direction", reverseTest, 
                  "Expected: -5000, Got: " + String(reversePos), reverseAccuracy);
    
    // Test 3: Multiple direction changes
    stepper.setPosition(index, 0);
    Serial.println("Testing rapid direction changes...");
    int passedChanges = 0;
    for (int i = 0; i < 5; i++) {
        bool dir = (i % 2 == 0);
        int startPos = stepper.getPosition(index);
        stepper.moveRelative(index, dir ? 200 : -200, 1000); // Move 200 steps in chosen direction at 1000 Hz
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
    addTestResult("Rapid Direction Changes", rapidChangesTest, 
                  String(passedChanges) + "/5 changes successful");
    
    Serial.print("Final position after direction test: "); 
    Serial.println(stepper.getPosition(index));
}

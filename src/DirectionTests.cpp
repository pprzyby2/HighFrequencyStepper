#include "DirectionTests.h"
#include "TestUtils.h"

void testDirectionChanges(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== Direction Change Test ===");
    
    counter.resetPosition();
    stepper.enable();
    
    // Test 1: Forward direction
    Serial.println("Testing FORWARD direction (7500 steps)...");
    stepper.setDirection(true);
    
    int32_t startPos = counter.getPosition();
    stepper.startPWM(1500);
    delay(5000); // 7500 steps at 1500 Hz = 5 seconds
    stepper.stopPWM();
    
    int32_t forwardPos = counter.getPosition();
    int32_t forwardSteps = forwardPos - startPos;

    Serial.printf("Forward steps counted: %d\n", forwardSteps);
    Serial.printf("Expected: ~5000, Actual: %d\n", forwardSteps);
    Serial.printf("Direction detection: %s\n", counter.getDirection() ? "Forward" : "Reverse");

    // Evaluate forward test
    bool forwardTest = (forwardSteps >= 4750 && forwardSteps <= 5250); // ±5% tolerance
    float forwardAccuracy = (float)forwardSteps / 5000.0 * 100.0;
    addTestResult("Forward Direction", forwardTest, 
                  "Expected: 5000, Got: " + String(forwardSteps), forwardAccuracy);

    delay(1000);
    
    // Test 2: Reverse direction
    Serial.println("Testing REVERSE direction (7500 steps)...");
    stepper.setDirection(false);
    
    startPos = counter.getPosition();
    stepper.startPWM(1500);
    delay(5000); // 7500 steps at 1500 Hz = 5 seconds
    stepper.stopPWM();
    
    int32_t reversePos = counter.getPosition();
    int32_t reverseSteps = reversePos - startPos;

    Serial.printf("Reverse steps counted: %d\n", reverseSteps);
    Serial.printf("Expected: ~-5000, Actual: %d\n", reverseSteps);
    Serial.printf("Direction detection: %s\n", counter.getDirection() ? "Forward" : "Reverse");

    // Evaluate reverse test
    bool reverseTest = (reverseSteps <= -4750 && reverseSteps >= -5250); // ±5% tolerance
    float reverseAccuracy = (float)abs(reverseSteps) / 5000.0 * 100.0;
    addTestResult("Reverse Direction", reverseTest, 
                  "Expected: -5000, Got: " + String(reverseSteps), reverseAccuracy);
    
    // Test 3: Multiple direction changes
    Serial.println("Testing rapid direction changes...");
    int passedChanges = 0;
    for (int i = 0; i < 5; i++) {
        bool dir = (i % 2 == 0);
        stepper.setDirection(dir);
        delay(50);
        
        startPos = counter.getPosition();
        stepper.startPWM(1000);
        delay(200); // 200 steps
        stepper.stopPWM();
        
        int32_t steps = counter.getPosition() - startPos;
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
    Serial.println(counter.getPosition());
}
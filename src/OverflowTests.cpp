#include "OverflowTests.h"
#include "TestUtils.h"

void testCounterOverflow(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== Counter Overflow Test ===");
    Serial.println("Testing position tracking through 16-bit counter overflow...");
    
    counter.resetPosition();
    stepper.enable();
    stepper.setDirection(true);
    
    // First, get close to overflow (16-bit counter has values -32768 to +32767)
    // Move to position close to positive overflow
    Serial.println("Moving close to positive overflow point...");
    
    int targetSteps = 32000;
    int32_t startPosition = counter.getPosition();
    
    stepper.startPWM(10000); // 10kHz
    
    while (counter.getPosition() < targetSteps) {
        if (millis() % 2000 == 0) {
            Serial.print("Current position: "); 
            Serial.println(counter.getPosition());
            delay(1); // Prevent multiple prints in same millisecond
        }
    }
    
    stepper.stopPWM();
    
    int32_t nearOverflowPosition = counter.getPosition();
    Serial.print("Position before overflow test: "); 
    Serial.println(nearOverflowPosition);
    
    // Now move enough to cause overflow
    Serial.println("Triggering positive overflow...");
    
    int32_t preOverflowPos = counter.getPosition();
    stepper.startPWM(5000);
    delay(2000); // Should add ~10,000 steps, crossing overflow
    stepper.stopPWM();
    
    int32_t postOverflowPos = counter.getPosition();
    int32_t stepsThroughOverflow = postOverflowPos - preOverflowPos;
    
    Serial.print("Position after overflow: "); Serial.println(postOverflowPos);
    Serial.print("Steps through overflow: "); Serial.println(stepsThroughOverflow);
    
    // Check if overflow handling worked (position should continue counting up)
    bool positiveOverflowOK = (stepsThroughOverflow > 8000 && stepsThroughOverflow < 12000);
    addTestResult("Positive Overflow", positiveOverflowOK, 
                  "Steps through overflow: " + String(stepsThroughOverflow));
    
    // Test negative overflow
    Serial.println("\nTesting negative overflow...");
    stepper.setDirection(false); // Reverse
    
    // Move way past the negative overflow point
    int32_t negativeTarget = -50000;
    
    stepper.startPWM(15000);
    
    while (counter.getPosition() > negativeTarget) {
        if (millis() % 2000 == 0) {
            Serial.print("Current position: "); 
            Serial.println(counter.getPosition());
            delay(1);
        }
    }
    
    stepper.stopPWM();
    
    int32_t finalPosition = counter.getPosition();
    Serial.print("Final position after negative overflow: "); 
    Serial.println(finalPosition);
    
    // Check if negative overflow handling worked
    bool negativeOverflowOK = (finalPosition < -40000 && finalPosition > -60000);
    addTestResult("Negative Overflow", negativeOverflowOK, 
                  "Final position: " + String(finalPosition));
    
    // Test total range
    int32_t totalMovement = abs(finalPosition - startPosition);
    bool rangeTestOK = (totalMovement > 80000); // Should have moved >80k steps total
    addTestResult("Extended Range", rangeTestOK, 
                  "Total movement: " + String(totalMovement) + " steps");
    
    Serial.print("Total movement through test: "); 
    Serial.print(totalMovement); Serial.println(" steps");
    Serial.println("Overflow test completed!");
}
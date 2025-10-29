#include "OverflowTests.h"
#include "TestUtils.h"


void testCounterOverflow(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Counter Overflow Test ===");
    Serial.println("Testing position tracking through 16-bit counter overflow...");

    controller.setPosition(index, 0);
    controller.enableStepper(index);
    bool dir = true;

    // First, get close to overflow (16-bit counter has values -32768 to +32767)
    // Move to position close to positive overflow
    Serial.println("Moving close to positive overflow point...");
    
    int targetSteps = 50000;
    controller.moveToPosition(index, targetSteps, 10000, true); // Move to 32k steps at 10kHz
    controller.stop(index);
    int32_t postOverflowPos = controller.getPosition(index);
    
    Serial.print("Position after overflow: "); Serial.println(postOverflowPos);
    
    // Check if overflow handling worked (position should continue counting up)
    bool positiveOverflowOK = (postOverflowPos > 40000 && postOverflowPos < 60000);
    addTestResult(controller.getName(index), "Positive Overflow", positiveOverflowOK, 
                  "Position after overflow: " + String(postOverflowPos));

    // Test negative overflow
    Serial.println("\nTesting negative overflow...");
    dir = false; // Reverse
    
    // Move way past the negative overflow point
    int32_t negativeTarget = -50000;

    controller.moveToPosition(index, negativeTarget, dir);
    controller.stop(index);

    int32_t finalPosition = controller.getPosition(index);
    Serial.print("Final position after negative overflow: "); 
    Serial.println(finalPosition);
    
    // Check if negative overflow handling worked
    bool negativeOverflowOK = (finalPosition < -40000 && finalPosition > -60000);
    addTestResult(controller.getName(index), "Negative Overflow", negativeOverflowOK, 
                  "Final position: " + String(finalPosition));
    
    // Test total range
    int32_t totalMovement = abs(finalPosition - postOverflowPos);
    bool rangeTestOK = (totalMovement > 80000); // Should have moved >80k steps total
    addTestResult(controller.getName(index), "Extended Range", rangeTestOK, 
                  "Total movement: " + String(totalMovement) + " steps");
    
    Serial.print("Total movement through test: "); 
    Serial.print(totalMovement); Serial.println(" steps");
    Serial.println("Overflow test completed!");
}

void testCounterOverflow(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        testCounterOverflow(controller, i);
    }
}
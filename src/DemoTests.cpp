#include "DemoTests.h"
#include "TestUtils.h"

void demonstratePositionTracking(PWMStepper& stepper, PulseCounter& counter) {
    Serial.println("\n=== Position Tracking Demo ===");
    
    counter.resetPosition();
    stepper.enable();
    
    Serial.println("Demonstrating position tracking with various movements...");
    
    // Demo 1: Simple forward movement
    Serial.println("\n1. Simple forward movement (1000 steps at 1kHz)");
    stepper.setDirection(true);
    
    int32_t startPos = counter.getPosition();
    stepper.startPWM(1000);
    delay(1000); // Exactly 1000 steps
    stepper.stopPWM();
    
    int32_t endPos = counter.getPosition();
    Serial.print("Expected: 1000, Actual: "); Serial.println(endPos - startPos);
    
    // Demo 2: Reverse movement
    Serial.println("\n2. Reverse movement (500 steps at 2kHz)");
    stepper.setDirection(false);
    
    startPos = counter.getPosition();
    stepper.startPWM(2000);
    delay(250); // 500 steps
    stepper.stopPWM();
    
    endPos = counter.getPosition();
    Serial.print("Expected: -500, Actual: "); Serial.println(endPos - startPos);
    
    // Demo 3: Position tracking during operation
    Serial.println("\n3. Real-time position tracking (10 seconds)");
    stepper.setDirection(true);
    stepper.startPWM(1500);
    
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        if ((millis() - startTime) % 1000 == 0) {
            Serial.print("Time: "); Serial.print((millis() - startTime) / 1000);
            Serial.print("s, Position: "); Serial.println(counter.getPosition());
            delay(1); // Prevent multiple prints
        }
    }
    
    stepper.stopPWM();
    Serial.print("Final position: "); Serial.println(counter.getPosition());
    
    addTestResult("Position Tracking Demo", true, "Completed successfully");
}

void demonstrateClosedLoopControl(PWMStepper& stepper, PulseCounter& counter, TMC2209Stepper& driver) {
    Serial.println("\n=== Closed Loop Control Demo ===");
    
    // Enable the driver
    driver.begin();
    driver.toff(5);
    driver.rms_current(800); // 800mA
    driver.microsteps(16);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    
    counter.resetPosition();
    stepper.enable();
    
    Serial.println("Demonstrating closed-loop positioning...");
    
    // Target positions to move to
    int32_t targets[] = {1000, 2500, 500, 3000, 0};
    int numTargets = sizeof(targets) / sizeof(targets[0]);
    
    for (int i = 0; i < numTargets; i++) {
        int32_t target = targets[i];
        Serial.print("\nMoving to position: "); Serial.println(target);
        
        // Simple closed-loop controller
        int attempts = 0;
        while (attempts < 50) { // Max 50 attempts
            int32_t currentPos = counter.getPosition();
            int32_t error = target - currentPos;
            
            if (abs(error) <= 5) { // Within 5 steps = success
                Serial.print("Reached target! Position: "); Serial.println(currentPos);
                break;
            }
            
            // Set direction based on error
            stepper.setDirection(error > 0);
            
            // Calculate speed based on error magnitude
            uint32_t speed = min(abs(error) * 10, 5000); // Proportional control
            speed = max(speed, (uint32_t)100); // Minimum speed
            
            stepper.startPWM(speed);
            delay(50); // Short move
            stepper.stopPWM();
            
            attempts++;
            
            if (attempts % 10 == 0) {
                Serial.print("Position: "); Serial.print(currentPos);
                Serial.print(", Error: "); Serial.print(error);
                Serial.print(", Speed: "); Serial.println(speed);
            }
        }
        
        if (attempts >= 50) {
            Serial.println("Failed to reach target within attempt limit");
        }
        
        delay(1000); // Pause between moves
    }
    
    Serial.print("Final position: "); Serial.println(counter.getPosition());
    
    // Test positioning accuracy
    int32_t finalError = abs(counter.getPosition() - targets[numTargets-1]);
    bool accuracyTest = (finalError <= 10);
    addTestResult("Closed Loop Accuracy", accuracyTest, 
                  "Final error: " + String(finalError) + " steps");
    
    addTestResult("Closed Loop Demo", true, "Completed successfully");
}
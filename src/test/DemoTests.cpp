#include "DemoTests.h"
#include "TestUtils.h"

void demonstratePositionTracking(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Position Tracking Demo ===");

    controller.setPosition(index, 0);
    controller.enableStepper(index);
    
    Serial.println("Demonstrating position tracking with various movements...");
    
    // Demo 1: Simple forward movement
    Serial.println("\n1. Simple forward movement (1000 steps at 1kHz)");
    bool dir = true;

    int32_t startPos = controller.getPosition(index);
    controller.startContinuous(index, 1000, dir);
    delay(1000); // Exactly 1000 steps
    controller.stop(index);

    int32_t endPos = controller.getPosition(index);
    Serial.print("Expected: 1000, Actual: "); Serial.println(endPos - startPos);
    
    // Demo 2: Reverse movement
    Serial.println("\n2. Reverse movement (500 steps at 2kHz)");
    dir = false;

    startPos = controller.getPosition(index);
    controller.startContinuous(index, 2000, dir);
    delay(250); // 500 steps
    controller.stop(index);

    endPos = controller.getPosition(index);
    Serial.print("Expected: -500, Actual: "); Serial.println(endPos - startPos);
    
    // Demo 3: Position tracking during operation
    Serial.println("\n3. Real-time position tracking (10 seconds)");
    dir = true;
    controller.startContinuous(index, 1500, dir);
    
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        if ((millis() - startTime) % 1000 == 0) {
            Serial.print("Time: "); Serial.print((millis() - startTime) / 1000);
            Serial.print("s, Position: "); Serial.println(controller.getPosition(index));
            delay(1); // Prevent multiple prints
        }
    }

    controller.stop(index);
    Serial.print("Final position: "); Serial.println(controller.getPosition(index));

    addTestResult("Position Tracking Demo", true, "Completed successfully");
}

void demonstratePositionTracking(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        Serial.printf("\n--- Demonstration for Stepper %d ---\n", i);
        demonstratePositionTracking(controller, i);
    }
}

void demonstrateClosedLoopControl(HighFrequencyStepper& controller, uint8_t index) {
    Serial.println("\n=== Closed Loop Control Demo ===");
    
    // Enable the driver
    controller.setPosition(index,0);
    controller.enableStepper(index);
    
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
            int32_t currentPos = controller.getPosition(index);
            int32_t error = target - currentPos;
            
            if (abs(error) <= 5) { // Within 5 steps = success
                Serial.print("Reached target! Position: "); Serial.println(currentPos);
                break;
            }
            
            // Set direction based on error
            bool dir = error > 0;
            
            // Calculate speed based on error magnitude
            uint32_t speed = min(abs(error) * 10, 5000); // Proportional control
            speed = max(speed, (uint32_t)100); // Minimum speed

            controller.startContinuous(index, speed, dir);
            delay(50); // Short move
            controller.stop(index);

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

    Serial.print("Final position: "); Serial.println(controller.getPosition(index));

    // Test positioning accuracy
    int32_t finalError = abs(controller.getPosition(index) - targets[numTargets-1]);
    bool accuracyTest = (finalError <= 10);
    addTestResult("Closed Loop Accuracy", accuracyTest, 
                  "Final error: " + String(finalError) + " steps");
    
    addTestResult("Closed Loop Demo", true, "Completed successfully");
}

void demonstrateClosedLoopControl(HighFrequencyStepper& controller) {
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        Serial.printf("\n--- Closed Loop Demo for Stepper %d ---\n", i);
        demonstrateClosedLoopControl(controller, i);
    }
}

void testAsyncMovement(HighFrequencyStepper& stepper) {
    Serial.println("\n=== Asynchronous Movement Demo ===");
    
    // Enable all steppers
    stepper.enableAll();
    
    
    // Define target positions for each stepper
    int numSteppers = stepper.getStepperCount();
    
    // Start asynchronous movements
    for (uint8_t i = 0; i < numSteppers; i++) {
        stepper.setPosition(i, 0); // Reset position
        int maxFreq = stepper.getMaxFrequency(i); // Just to ensure it's configured
        int fullCircle = stepper.getMicrostepsPerRevolution(i);
        printf("Stepper %d max frequency: %d Hz, steps/rev: %d, target 50 circles: %d\n", i, maxFreq, fullCircle, fullCircle * 50);
        stepper.moveToPositionAsync(i, fullCircle * 50, maxFreq); // Move at max frequency
    }
    
    // Monitor progress
    bool allDone = false;
    while (!allDone) {
        allDone = true;
        for (uint8_t i = 0; i < numSteppers; i++) {
            if (stepper.isMoving(i)) {
                allDone = false;
                Serial.printf("Stepper %d current position: %d, target position: %d, current frequency: %f\n", i, stepper.getPosition(i), stepper.getTargetPosition(i), stepper.getCurrentFrequency(i));
            } else {
                Serial.printf("Stepper %d reached target position: %d\n", i, stepper.getPosition(i));
            }
        }
        delay(500); // Update every 500ms
    }
    
    Serial.println("All steppers have reached their target positions.");
    
    addTestResult("Asynchronous Movement Demo", true, "Completed successfully");
}
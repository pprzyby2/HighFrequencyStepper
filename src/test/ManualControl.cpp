#include "ManualControl.h"

// Manual control state variables
bool manualControlMode = false;
uint8_t manualControlStepperIndex = 0;
double manualControlSpeed = 100.0;  // Hz
bool manualControlDirection = true;  // true = forward

// Forward declaration for printTestMenu (defined in main.cpp)
extern void printTestMenu();

void printManualControlMenu() {
    Serial.println("\n=== MANUAL MOTOR CONTROL ===");
    Serial.printf("Stepper: %s (Index %d)\n", stepperController.getName(manualControlStepperIndex).c_str(), manualControlStepperIndex);
    
    // Check if TMC2209 driver
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    bool isTMC2209 = (config.driverSettings.driverType == TMC2209_DRIVER);
    
    Serial.printf("Speed: %.2f Hz (%.2f RPM)\n", stepperController.getCurrentFrequency(manualControlStepperIndex), 
        stepperController.getCurrentFrequency(manualControlStepperIndex) / stepperController.getMicrostepsPerRevolution(manualControlStepperIndex) * 60.0);
    Serial.printf("Direction: %s\n", manualControlDirection ? "Forward (CW)" : "Reverse (CCW)");
    Serial.printf("Status: %s\n", stepperController.isEnabled(manualControlStepperIndex) ? "ENABLED" : "DISABLED");
    Serial.printf("Moving: %s\n", stepperController.isMoving(manualControlStepperIndex) ? "YES" : "NO");
    Serial.printf("Position: %d (%.2f degrees)\n", stepperController.getPosition(manualControlStepperIndex), stepperController.getAngle(manualControlStepperIndex));
    
    if (isTMC2209) {
        Serial.printf("Driver: TMC2209 (RMS Current: %d mA)\n", stepperController.getRMSCurrent(manualControlStepperIndex));
        Serial.printf("SpreadCycle: %s\n", stepperController.isSpreadCycleEnabled(manualControlStepperIndex) ? "ENABLED" : "DISABLED");
        Serial.printf("Microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    }
    
    Serial.println("\n--- Commands ---");
    Serial.println("e - Enable stepper");
    Serial.println("d - Disable stepper");
    Serial.println("s - Start/Resume movement");
    Serial.println("x - Stop movement");
    Serial.println("r - Reverse direction");
    Serial.println("+ - Increase speed (10 Hz)");
    Serial.println("- - Decrease speed (10 Hz)");
    Serial.println("* - Increase speed (100 Hz)");
    Serial.println("/ - Decrease speed (100 Hz)");
    Serial.println("n - Select next stepper");
    Serial.println("p - Show position/status");
    Serial.println("z - Zero position");
    
    if (isTMC2209) {
        Serial.println("\n--- TMC2209 Driver Controls ---");
        Serial.println("c - Increase RMS current (+50 mA)");
        Serial.println("C - Decrease RMS current (-50 mA)");
        Serial.println("t - Toggle StealthChop/SpreadCycle");
        Serial.println("g - Set StallGuard threshold");
        Serial.println("m - Change microsteps");
        Serial.println("i - Show TMC2209 info");
    }
    
    Serial.println("\nq - Quit manual control");
    Serial.println("\nEnter command: ");
}

void manualControlEnableStepper() {
    if (stepperController.enableStepper(manualControlStepperIndex)) {
        Serial.printf("✓ Stepper %s ENABLED\n", stepperController.getName(manualControlStepperIndex).c_str());
        ledStatus.setStatus(LED_IDLE);
    } else {
        Serial.println("✗ Failed to enable stepper");
        ledStatus.setStatus(LED_ERROR);
    }
}

void manualControlDisableStepper() {
    if (stepperController.disableStepper(manualControlStepperIndex)) {
        Serial.printf("✓ Stepper %s DISABLED\n", stepperController.getName(manualControlStepperIndex).c_str());
        ledStatus.setStatus(LED_DISABLED);
    } else {
        Serial.println("✗ Failed to disable stepper");
    }
}

void manualControlStart() {
    if (!stepperController.isEnabled(manualControlStepperIndex)) {
        Serial.println("✗ Stepper is disabled. Enable it first (press 'e')");
        return;
    }
    
    if (stepperController.moveAtFrequency(manualControlStepperIndex, manualControlSpeed * (manualControlDirection ? 1 : -1))) {
        Serial.printf("✓ Moving at %.2f Hz %s\n", manualControlSpeed, manualControlDirection ? "Forward" : "Reverse");
        ledStatus.setStatus(LED_MOVING);
    } else {
        Serial.println("✗ Failed to start movement");
        ledStatus.setStatus(LED_ERROR);
    }
}

void manualControlStop() {
    if (stepperController.stop(manualControlStepperIndex)) {
        Serial.println("✓ Motor STOPPED");
        ledStatus.setStatus(LED_IDLE);
    } else {
        Serial.println("✗ Failed to stop motor");
    }
}

void manualControlReverseDirection() {
    manualControlDirection = !manualControlDirection;
    Serial.printf("✓ Direction changed to %s\n", manualControlDirection ? "Forward (CW)" : "Reverse (CCW)");
    
    // If motor is moving, update movement with new direction
    if (stepperController.isMoving(manualControlStepperIndex)) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
        manualControlStart();
    }
}

void manualControlIncreaseSpeed(double increment) {
    double maxFreq = stepperController.getMaxFrequency(manualControlStepperIndex);
    manualControlSpeed += increment;
    
    if (manualControlSpeed > maxFreq) {
        manualControlSpeed = maxFreq;
        Serial.printf("⚠ Speed limited to maximum: %.2f Hz\n", maxFreq);
    } else {
        Serial.printf("✓ Speed increased to %.2f Hz\n", manualControlSpeed);
    }
    
    // If motor is moving, update speed
    if (stepperController.isMoving(manualControlStepperIndex)) {
        manualControlStart();
    }
}

void manualControlDecreaseSpeed(double decrement) {
    manualControlSpeed -= decrement;
    
    if (manualControlSpeed < 1.0) {
        manualControlSpeed = 1.0;
        Serial.println("⚠ Speed limited to minimum: 1.0 Hz");
    } else {
        Serial.printf("✓ Speed decreased to %.2f Hz\n", manualControlSpeed);
    }
    
    // If motor is moving, update speed
    if (stepperController.isMoving(manualControlStepperIndex)) {
        manualControlStart();
    }
}

void manualControlSelectNextStepper() {
    // Move to next stepper
    manualControlStepperIndex++;
    if (manualControlStepperIndex >= stepperController.getStepperCount()) {
        manualControlStepperIndex = 0;
    }
    
    Serial.printf("✓ Selected stepper %s (Index %d)\n", 
                  stepperController.getName(manualControlStepperIndex).c_str(), 
                  manualControlStepperIndex);
}

void manualControlShowStatus() {
    Serial.println("\n--- Stepper Status ---");
    stepperController.printStatus(manualControlStepperIndex);
}

void manualControlZeroPosition() {
    stepperController.setPosition(manualControlStepperIndex, 0);
    Serial.printf("✓ Position reset to 0 for stepper %s\n", 
                  stepperController.getName(manualControlStepperIndex).c_str());
}

void manualControlIncreaseRMSCurrent() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    uint16_t currentRMS = stepperController.getRMSCurrent(manualControlStepperIndex);
    uint16_t newRMS = currentRMS + 50;
    
    if (newRMS > 2000) {
        Serial.println("⚠ Current limited to 2000 mA (safety limit)");
        newRMS = 2000;
    }
    
    if (stepperController.setRMSCurrent(manualControlStepperIndex, newRMS)) {
        Serial.printf("✓ RMS Current increased to %d mA\n", newRMS);
    } else {
        Serial.println("✗ Failed to set RMS current");
    }
}

void manualControlDecreaseRMSCurrent() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    uint16_t currentRMS = stepperController.getRMSCurrent(manualControlStepperIndex);
    
    if (currentRMS <= 100) {
        Serial.println("⚠ Current at minimum (100 mA)");
        return;
    }
    
    uint16_t newRMS = currentRMS - 50;
    if (newRMS < 100) {
        newRMS = 100;
    }
    
    if (stepperController.setRMSCurrent(manualControlStepperIndex, newRMS)) {
        Serial.printf("✓ RMS Current decreased to %d mA\n", newRMS);
    } else {
        Serial.println("✗ Failed to set RMS current");
    }
}

void manualControlToggleStealthChop() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    // Stop motor first for mode change
    bool wasMoving = stepperController.isMoving(manualControlStepperIndex);
    if (wasMoving) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
    }
    
    // Toggle: SpreadCycle = false means StealthChop is active
    // We'll use a static variable to track current state
    static bool spreadCycleEnabled = stepperController.isSpreadCycleEnabled(manualControlStepperIndex);
    spreadCycleEnabled = !spreadCycleEnabled;
    
    if (stepperController.setSpreadCycle(manualControlStepperIndex, spreadCycleEnabled)) {
        Serial.printf("✓ Switched to %s mode\n", spreadCycleEnabled ? "SpreadCycle (high speed)" : "StealthChop (quiet)");
    } else {
        Serial.println("✗ Failed to change mode");
    }
    
    // Resume if it was moving
    if (wasMoving) {
        delay(100);
        manualControlStart();
    }
}

void manualControlSetStallGuardThreshold() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.println("\nEnter StallGuard threshold (0-255, higher = less sensitive):");
    Serial.println("Typical values: 0-10 for very sensitive, 50-100 for moderate, 200+ for low sensitivity");
    Serial.print("Current value (-1 = disabled): ");
    
    // Wait for input with timeout
    unsigned long startTime = millis();
    while (!Serial.available() && (millis() - startTime < 10000)) {
        delay(10);
    }
    
    if (!Serial.available()) {
        Serial.println("\n⚠ Timeout - cancelled");
        return;
    }
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    int threshold = input.toInt();
    
    if (threshold < -1 || threshold > 255) {
        Serial.println("✗ Invalid threshold (must be -1 to 255)");
        return;
    }
    
    if (stepperController.setStallGuardThreshold(manualControlStepperIndex, threshold)) {
        if (threshold == -1) {
            Serial.println("✓ StallGuard disabled");
        } else {
            Serial.printf("✓ StallGuard threshold set to %d\n", threshold);
        }
    } else {
        Serial.println("✗ Failed to set StallGuard threshold");
    }
}

void manualControlChangeMicrosteps() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.printf("\nCurrent microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    Serial.println("Enter new microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256):");
    
    // Wait for input with timeout
    unsigned long startTime = millis();
    while (!Serial.available() && (millis() - startTime < 10000)) {
        delay(10);
    }
    
    if (!Serial.available()) {
        Serial.println("\n⚠ Timeout - cancelled");
        return;
    }
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    uint16_t microsteps = input.toInt();
    
    // Validate power of 2 and in range
    bool valid = false;
    uint16_t validValues[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    for (int i = 0; i < 9; i++) {
        if (microsteps == validValues[i]) {
            valid = true;
            break;
        }
    }
    
    if (!valid) {
        Serial.println("✗ Invalid microsteps value");
        return;
    }
    
    // Stop motor before changing microsteps
    bool wasMoving = stepperController.isMoving(manualControlStepperIndex);
    if (wasMoving) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
    }
    
    if (stepperController.setMicrosteps(manualControlStepperIndex, microsteps)) {
        Serial.printf("✓ Microsteps changed to %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
        Serial.printf("   Steps per revolution: %d\n", stepperController.getMicrostepsPerRevolution(manualControlStepperIndex));
        Serial.printf("   Max frequency: %.2f Hz\n", stepperController.getMaxFrequency(manualControlStepperIndex));
    } else {
        Serial.println("✗ Failed to set microsteps");
    }
    
    // Resume if it was moving
    if (wasMoving) {
        delay(100);
        manualControlStart();
    }
}

void manualControlShowTMCInfo() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.println("\n=== TMC2209 Driver Information ===");
    Serial.printf("Stepper: %s\n", stepperController.getName(manualControlStepperIndex).c_str());
    Serial.printf("RMS Current: %d mA\n", stepperController.getRMSCurrent(manualControlStepperIndex));
    Serial.printf("Microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    Serial.printf("R_Sense: %.3f Ω\n", stepperController.getRSense(manualControlStepperIndex));
    Serial.printf("Driver Address: 0x%02X\n", stepperController.getDriverAddress(manualControlStepperIndex));
    Serial.printf("UART: %s\n", stepperController.getUART(manualControlStepperIndex) == &Serial2 ? "Serial2" : "Serial1");
    Serial.printf("Stall Detection: %s\n", stepperController.isStallDetected(manualControlStepperIndex) ? "ACTIVE" : "Inactive");
    Serial.println("================================\n");
}

void processManualControlInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) {
            return;
        }
        
        char cmd = input.charAt(0);
        
        switch (cmd) {
            case 'e':
            case 'E':
                manualControlEnableStepper();
                break;
                
            case 'd':
            case 'D':
                manualControlDisableStepper();
                break;
                
            case 's':
            case 'S':
                manualControlStart();
                break;
                
            case 'x':
            case 'X':
                manualControlStop();
                break;
                
            case 'r':
            case 'R':
                manualControlReverseDirection();
                break;
                
            case '+':
                manualControlIncreaseSpeed(10.0);
                break;
                
            case '-':
                manualControlDecreaseSpeed(10.0);
                break;
                
            case '*':
                manualControlIncreaseSpeed(100.0);
                break;
                
            case '/':
                manualControlDecreaseSpeed(100.0);
                break;
                
            case 'n':
            case 'N':
                manualControlSelectNextStepper();
                break;
                
            case 'p':
            case 'P':
                manualControlShowStatus();
                break;
                
            case 'z':
            case 'Z':
                manualControlZeroPosition();
                break;
                
            case 'c':
                manualControlIncreaseRMSCurrent();
                break;
                
            case 'C':
                manualControlDecreaseRMSCurrent();
                break;
                
            case 't':
            case 'T':
                manualControlToggleStealthChop();
                break;
                
            case 'g':
            case 'G':
                manualControlSetStallGuardThreshold();
                break;
                
            case 'm':
            case 'M':
                manualControlChangeMicrosteps();
                break;
                
            case 'i':
            case 'I':
                manualControlShowTMCInfo();
                break;
                
            case 'q':
            case 'Q':
                manualControlStop();
                manualControlMode = false;
                Serial.println("✓ Exiting manual control mode");
                ledStatus.setStatus(LED_IDLE);
                printTestMenu();
                return;
                
            case 'h':
            case 'H':
            case '?':
                printManualControlMenu();
                return;
                
            default:
                Serial.println("✗ Invalid command. Press 'h' for help.");
                break;
        }
        
        printManualControlMenu();
    }
}

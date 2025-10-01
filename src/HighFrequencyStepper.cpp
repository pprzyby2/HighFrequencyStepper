#include "HighFrequencyStepper.h"

// Constructor
HighFrequencyStepper::HighFrequencyStepper() {
    stepperCount = 0;
    globalEnable = false;
    
    // Initialize arrays to nullptr
    for (uint8_t i = 0; i < MAX_STEPPERS; i++) {
        pwmSteppers[i] = nullptr;
        pulseCounters[i] = nullptr;
        tmcDrivers[i] = nullptr;
        uartPorts[i] = nullptr;
    }
}

// Destructor
HighFrequencyStepper::~HighFrequencyStepper() {
    // Clean up allocated instances
    for (uint8_t i = 0; i < MAX_STEPPERS; i++) {
        if (pwmSteppers[i]) {
            delete pwmSteppers[i];
            pwmSteppers[i] = nullptr;
        }
        if (pulseCounters[i]) {
            delete pulseCounters[i];
            pulseCounters[i] = nullptr;
        }
        if (tmcDrivers[i]) {
            delete tmcDrivers[i];
            tmcDrivers[i] = nullptr;
        }
        // Note: uartPorts[i] points to HardwareSerial instances, don't delete
    }
}

// Helper method to validate stepper index
bool HighFrequencyStepper::validateStepperIndex(uint8_t index) const {
    return (index < MAX_STEPPERS && index < stepperCount && pwmSteppers[index] != nullptr);
}

// Add a stepper configuration
bool HighFrequencyStepper::addStepper(uint8_t index, const StepperConfig& config) {
    if (index >= MAX_STEPPERS) {
        Serial.println("ERROR: Stepper index out of range");
        return false;
    }
    
    // Store configuration
    configs[index] = config;
    
    // Create PWMStepper instance
    pwmSteppers[index] = new PWMStepper(config.stepPin, config.dirPin, config.enablePin, config.ledcChannel);
    if (!pwmSteppers[index]) {
        Serial.println("ERROR: Failed to create PWMStepper instance");
        return false;
    }
    
    // Create PulseCounter instance
    pulseCounters[index] = new PulseCounter(config.pcntUnit, config.stepCountPin, config.dirPin);
    if (!pulseCounters[index]) {
        Serial.println("ERROR: Failed to create PulseCounter instance");
        delete pwmSteppers[index];
        pwmSteppers[index] = nullptr;
        return false;
    }
    
    // Configure UART port
    configureUART(index);
    
    // Create TMC2209 instance
    tmcDrivers[index] = new TMC2209Stepper(uartPorts[index], config.rSense, config.driverAddress);
    if (!tmcDrivers[index]) {
        Serial.println("ERROR: Failed to create TMC2209Stepper instance");
        delete pwmSteppers[index];
        delete pulseCounters[index];
        pwmSteppers[index] = nullptr;
        pulseCounters[index] = nullptr;
        return false;
    }
    
    // Update stepper count
    if (index >= stepperCount) {
        stepperCount = index + 1;
    }
    
    Serial.print("Added stepper ");
    Serial.print(index);
    Serial.println(" successfully");
    
    return true;
}

// Configure UART for TMC communication
void HighFrequencyStepper::configureUART(uint8_t index) {
    // Map UART ports based on index
    switch (index) {
        case 0:
            uartPorts[index] = &Serial1;
            Serial1.begin(115200, SERIAL_8N1, configs[index].rxPin, configs[index].txPin);
            break;
        case 1:
            uartPorts[index] = &Serial2;
            Serial2.begin(115200, SERIAL_8N1, configs[index].rxPin, configs[index].txPin);
            break;
        case 2:
            // For ESP32, we might need to use SoftwareSerial or additional hardware serial
            uartPorts[index] = &Serial1; // Share Serial1 with different address
            break;
        case 3:
            uartPorts[index] = &Serial2; // Share Serial2 with different address
            break;
        default:
            uartPorts[index] = &Serial2; // Fallback
            break;
    }
}

// Initialize a specific stepper
bool HighFrequencyStepper::initializeStepper(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("ERROR: Invalid stepper index for initialization");
        return false;
    }
    
    // Initialize PWMStepper
    pwmSteppers[index]->begin();
    
    // Initialize PulseCounter
    if (!pulseCounters[index]->begin()) {
        Serial.println("ERROR: Failed to initialize PulseCounter");
        return false;
    }
    
    // Initialize TMC2209
    tmcDrivers[index]->begin();
    tmcDrivers[index]->toff(5);                    // Enable driver
    tmcDrivers[index]->rms_current(configs[index].rmsCurrent);
    tmcDrivers[index]->microsteps(configs[index].microsteps);
    tmcDrivers[index]->pwm_autoscale(true);        // Enable automatic current scaling
    tmcDrivers[index]->en_spreadCycle(false);      // Use StealthChop by default
    
    // Initialize status
    status[index].isInitialized = true;
    status[index].currentPosition = 0;
    status[index].targetPosition = 0;
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.println(" initialized successfully");
    
    return true;
}

// Initialize all steppers
bool HighFrequencyStepper::initializeAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!initializeStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// Set microsteps
bool HighFrequencyStepper::setMicrosteps(uint8_t index, uint16_t microsteps) {
    if (!validateStepperIndex(index)) return false;
    
    // Validate microsteps value
    if (microsteps != 1 && microsteps != 2 && microsteps != 4 && microsteps != 8 && 
        microsteps != 16 && microsteps != 32 && microsteps != 64 && microsteps != 128 && microsteps != 256) {
        Serial.println("ERROR: Invalid microsteps value");
        return false;
    }
    
    configs[index].microsteps = microsteps;
    tmcDrivers[index]->microsteps(microsteps);
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.print(" microsteps set to ");
    Serial.println(microsteps);
    
    return true;
}

// Set RMS current
bool HighFrequencyStepper::setRMSCurrent(uint8_t index, uint16_t currentMA) {
    if (!validateStepperIndex(index)) return false;
    
    configs[index].rmsCurrent = currentMA;
    tmcDrivers[index]->rms_current(currentMA);
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.print(" RMS current set to ");
    Serial.print(currentMA);
    Serial.println(" mA");
    
    return true;
}

// Set maximum frequency
bool HighFrequencyStepper::setMaxFrequency(uint8_t index, double frequency) {
    if (!validateStepperIndex(index)) return false;
    
    configs[index].maxFrequency = frequency;
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.print(" max frequency set to ");
    Serial.print(frequency);
    Serial.println(" Hz");
    
    return true;
}

// Move to absolute position
bool HighFrequencyStepper::moveToPosition(uint8_t index, int32_t position, double frequency) {
    if (!validateStepperIndex(index)) return false;
    
    if (frequency == 0) frequency = configs[index].maxFrequency;
    
    int32_t currentPos = getPosition(index);
    int32_t steps = position - currentPos;
    
    return moveRelative(index, steps, frequency);
}

// Move relative steps
bool HighFrequencyStepper::moveRelative(uint8_t index, int32_t steps, double frequency) {
    if (!validateStepperIndex(index)) return false;
    if (steps == 0) return true;
    
    if (frequency == 0) frequency = configs[index].maxFrequency;
    if (frequency > configs[index].maxFrequency) frequency = configs[index].maxFrequency;
    
    bool direction = steps > 0;
    uint32_t absSteps = abs(steps);
    
    // Update target position
    status[index].targetPosition = status[index].currentPosition + steps;
    status[index].isMoving = true;
    status[index].targetFrequency = frequency;
    
    // Execute movement
    pwmSteppers[index]->step(absSteps, frequency, direction);
    
    // Update position tracking
    updatePosition(index);
    
    return true;
}

// Start continuous movement
bool HighFrequencyStepper::startContinuous(uint8_t index, double frequency, bool direction) {
    if (!validateStepperIndex(index)) return false;
    
    if (frequency > configs[index].maxFrequency) frequency = configs[index].maxFrequency;
    
    status[index].isMoving = true;
    status[index].currentFrequency = frequency;
    
    pwmSteppers[index]->setDirection(direction);
    pwmSteppers[index]->startPWM(frequency);
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.print(" started continuous movement at ");
    Serial.print(frequency);
    Serial.println(" Hz");
    
    return true;
}

// Stop specific stepper
bool HighFrequencyStepper::stop(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->stopPWM();
    status[index].isMoving = false;
    status[index].currentFrequency = 0;
    
    updatePosition(index);
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.println(" stopped");
    
    return true;
}

// Stop all steppers
bool HighFrequencyStepper::stopAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!stop(i)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// Emergency stop - immediate stop of all steppers
bool HighFrequencyStepper::emergencyStop() {
    Serial.println("EMERGENCY STOP ACTIVATED!");
    
    // Disable all steppers immediately
    disableAll();
    stopAll();
    
    return true;
}

// Get current position
int32_t HighFrequencyStepper::getPosition(uint8_t index) {
    if (!validateStepperIndex(index)) return 0;
    
    // Get position from pulse counter
    int32_t pulseCount = pulseCounters[index]->getPosition();
    
    // Update internal position tracking
    status[index].currentPosition = pulseCount;
    
    return pulseCount;
}

// Update position tracking
void HighFrequencyStepper::updatePosition(uint8_t index) {
    if (!validateStepperIndex(index)) return;
    
    // Update current position from pulse counter
    status[index].currentPosition = getPosition(index);
    
    // Check if movement is complete
    if (abs(status[index].currentPosition - status[index].targetPosition) <= 1) {
        status[index].isMoving = false;
        status[index].currentFrequency = 0;
    }
}

// Enable stepper
bool HighFrequencyStepper::enableStepper(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->enable();
    status[index].isEnabled = true;
    
    return true;
}

// Disable stepper
bool HighFrequencyStepper::disableStepper(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->disable();
    status[index].isEnabled = false;
    status[index].isMoving = false;
    
    return true;
}

// Enable all steppers
bool HighFrequencyStepper::enableAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!enableStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    globalEnable = allSuccess;
    return allSuccess;
}

// Disable all steppers
bool HighFrequencyStepper::disableAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!disableStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    globalEnable = false;
    return allSuccess;
}

// Check if stepper is enabled
bool HighFrequencyStepper::isEnabled(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    return status[index].isEnabled;
}

// Set position (zero or calibrate)
bool HighFrequencyStepper::setPosition(uint8_t index, int32_t position) {
    if (!validateStepperIndex(index)) return false;
    
    pulseCounters[index]->setPosition(position);
    status[index].currentPosition = position;
    status[index].targetPosition = position;
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.print(" position set to ");
    Serial.println(position);
    
    return true;
}

// Zero position
bool HighFrequencyStepper::zeroPosition(uint8_t index) {
    return setPosition(index, 0);
}

// Get stepper status
StepperStatus HighFrequencyStepper::getStatus(uint8_t index) {
    if (!validateStepperIndex(index)) {
        return StepperStatus(); // Return default status
    }
    
    // Update position before returning status
    updatePosition(index);
    
    // Update additional status information
    status[index].temperature = getTemperature(index);
    status[index].stallGuard = isStallDetected(index);
    
    return status[index];
}

// Print status for one stepper
void HighFrequencyStepper::printStatus(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("Invalid stepper index");
        return;
    }
    
    StepperStatus stat = getStatus(index);
    
    Serial.println("=== Stepper " + String(index) + " Status ===");
    Serial.println("Initialized: " + String(stat.isInitialized ? "YES" : "NO"));
    Serial.println("Enabled: " + String(stat.isEnabled ? "YES" : "NO"));
    Serial.println("Moving: " + String(stat.isMoving ? "YES" : "NO"));
    Serial.println("Position: " + String(stat.currentPosition));
    Serial.println("Target: " + String(stat.targetPosition));
    Serial.println("Frequency: " + String(stat.currentFrequency) + " Hz");
    Serial.println("Temperature: " + String(stat.temperature) + "°C");
    Serial.println("StallGuard: " + String(stat.stallGuard ? "DETECTED" : "OK"));
    Serial.println("Microsteps: " + String(configs[index].microsteps));
    Serial.println("RMS Current: " + String(configs[index].rmsCurrent) + " mA");
    Serial.println("Mode: " + String(pwmSteppers[index]->isInLEDCMode() ? "LEDC" : "Timer"));
    Serial.println("==========================");
}

// Print status for all steppers
void HighFrequencyStepper::printAllStatus() {
    Serial.println("\n=== HIGH FREQUENCY STEPPER STATUS ===");
    Serial.println("Stepper Count: " + String(stepperCount));
    Serial.println("Global Enable: " + String(globalEnable ? "ON" : "OFF"));
    Serial.println("");
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            printStatus(i);
            Serial.println("");
        }
    }
}

// Get temperature from TMC driver
float HighFrequencyStepper::getTemperature(uint8_t index) {
    if (!validateStepperIndex(index)) return 0.0;
    
    // Note: TMC2209 doesn't have direct temperature readout
    // This is a placeholder for future implementation
    return 25.0; // Return room temperature as default
}

// Check if stall is detected
bool HighFrequencyStepper::isStallDetected(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    // Check TMC StallGuard status
    uint32_t drv_status = tmcDrivers[index]->DRV_STATUS();
    return (drv_status & 0x1000000) != 0; // StallGuard flag
}

// Self test for one stepper
bool HighFrequencyStepper::selfTest(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("Self-test failed: Invalid stepper index");
        return false;
    }
    
    Serial.print("Self-test for stepper ");
    Serial.print(index);
    Serial.println("...");
    
    // Test 1: Communication with TMC driver
    uint32_t version = tmcDrivers[index]->version();
    if (version == 0 || version == 0xFFFFFFFF) {
        Serial.println("FAIL: TMC communication error");
        return false;
    }
    
    // Test 2: Enable/disable functionality
    enableStepper(index);
    if (!isEnabled(index)) {
        Serial.println("FAIL: Enable function");
        return false;
    }
    
    // Test 3: Small movement test
    int32_t startPos = getPosition(index);
    moveRelative(index, 100, 1000); // Move 100 steps at 1kHz
    delay(200);
    int32_t endPos = getPosition(index);
    
    if (abs(endPos - startPos - 100) > 5) { // Allow 5 step tolerance
        Serial.println("FAIL: Movement accuracy");
        return false;
    }
    
    // Return to start position
    moveToPosition(index, startPos, 1000);
    delay(200);
    
    Serial.println("PASS: Self-test completed successfully");
    return true;
}

// Self test for all steppers
bool HighFrequencyStepper::selfTestAll() {
    bool allPass = true;
    
    Serial.println("Starting self-test for all steppers...");
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!selfTest(i)) {
                allPass = false;
            }
        }
    }
    
    if (allPass) {
        Serial.println("All steppers passed self-test!");
    } else {
        Serial.println("Some steppers failed self-test!");
    }
    
    return allPass;
}

// Wait for movement completion
bool HighFrequencyStepper::waitForCompletion(uint8_t index, uint32_t timeoutMS) {
    if (!validateStepperIndex(index)) return false;
    
    uint32_t startTime = millis();
    
    while (status[index].isMoving) {
        updatePosition(index);
        
        if (millis() - startTime > timeoutMS) {
            Serial.println("Timeout waiting for movement completion");
            return false;
        }
        
        delay(10);
    }
    
    return true;
}

// Additional methods for completeness...
bool HighFrequencyStepper::isMoving(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    updatePosition(index);
    return status[index].isMoving;
}

int32_t HighFrequencyStepper::getTargetPosition(uint8_t index) {
    if (!validateStepperIndex(index)) return 0;
    return status[index].targetPosition;
}

double HighFrequencyStepper::getCurrentFrequency(uint8_t index) {
    if (!validateStepperIndex(index)) return 0.0;
    return status[index].currentFrequency;
}

bool HighFrequencyStepper::isAtPosition(uint8_t index, int32_t tolerance) {
    if (!validateStepperIndex(index)) return false;
    updatePosition(index);
    return abs(status[index].currentPosition - status[index].targetPosition) <= tolerance;
}

StepperConfig HighFrequencyStepper::getConfig(uint8_t index) const {
    if (index < MAX_STEPPERS) {
        return configs[index];
    }
    return StepperConfig(); // Return default config for invalid index
}

// Move all steppers to specified positions
bool HighFrequencyStepper::moveAllToPosition(const int32_t positions[], double frequency) {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!moveToPosition(i, positions[i], frequency)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// Move all steppers relative amounts
bool HighFrequencyStepper::moveAllRelative(const int32_t steps[], double frequency) {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!moveRelative(i, steps[i], frequency)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// Wait for all steppers to complete movement
bool HighFrequencyStepper::waitForAllCompletion(uint32_t timeoutMS) {
    uint32_t startTime = millis();
    
    while (true) {
        bool anyMoving = false;
        
        for (uint8_t i = 0; i < stepperCount; i++) {
            if (pwmSteppers[i] != nullptr) {
                updatePosition(i);
                if (status[i].isMoving) {
                    anyMoving = true;
                }
            }
        }
        
        if (!anyMoving) {
            return true; // All completed
        }
        
        if (millis() - startTime > timeoutMS) {
            Serial.println("Timeout waiting for all movements to complete");
            return false;
        }
        
        delay(10);
    }
}